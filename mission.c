#include "mission.h"
#include "mavlink_tx.h"
#include "scheduler.h"
#include "uart.h"
#include "c_library_v2/mavlink_sha256.h"

/* ── mission storage ── */
waypoint_t mission[MAX_WAYPOINTS];
uint16_t   mission_count = 0;
uint16_t   mission_rx_idx = 0;

/* ── position state ── */
int32_t  current_lat = 130827000;
int32_t  current_lon = 802707000;
int32_t  current_alt_mm = 10000;
uint16_t current_heading = 0;

uint16_t current_waypoint_idx = 0;
uint8_t mission_loaded = 0;

/* ── mission integrity hash ── */
uint8_t  mission_hash_valid = 0;        /* 1 = mission integrity verified */
static uint8_t stored_mission_hash[MISSION_HASH_SIZE]; /* copy of hash from NVM */
static uint8_t current_mission_hash[MISSION_HASH_SIZE]; /* recomputed current mission hash */

/* ── mission auth metadata ── */
uint32_t current_mission_id = 0;
uint32_t current_mission_ver = 0;
static uint32_t pending_mission_id = 0;
static uint32_t pending_mission_ver = 0;
uint32_t current_mission_challenge = 0;
static uint32_t mission_challenge_seed = 0xC0FF01D0;
static uint8_t  mission_upload_pending = 0;
static uint8_t  mission_upload_authorized_flag = 0;

/* ── configurable parameters ── */
uint32_t wp_radius_m = 33;              /* default 33m acceptance radius */

/* ── GUIDED mode target (set by RX handler) ── */
int32_t  guided_target_lat = 0;
int32_t  guided_target_lon = 0;
int32_t  guided_target_alt = 0;
uint8_t  guided_target_set = 0;
static uint8_t mission_active = 0;
/* RTL state: 0=inactive, 1=in progress, 2=complete */
static uint8_t rtl_active = 0;

/* ── RTL experiment measurement ── */
static uint32_t rtl_start_tick = 0;
static uint32_t rtl_end_tick = 0;
static uint32_t rtl_start_distance_cm = 0;  /* RTL start distance in cm (for XX.XX m display) */
static uint8_t  rtl_measurement_active = 0;

/* ── battery state (starts at 100%, drains dynamically) ── */
uint16_t battery_voltage_mv   = 12600;  /* 12.6V fully charged */
int16_t  battery_current_ma   = 0;
uint8_t  battery_remaining_pct = 100;

/* ── GPS / sensor state ── */
uint8_t  gps_fix_type   = 3;           /* 3 = 3D fix */
uint8_t  gps_satellites = 8;

/* ── failsafe state ── */
uint8_t  failsafe_battery = 0;
uint8_t  failsafe_gps     = 0;

/* constants */
#define BATTERY_CRITICAL_PCT          5
#define BATTERY_DRAIN_IDLE_PCT        1    /* % per second when armed but not moving */
#define BATTERY_DRAIN_MOVE_PCT        1    /* % per second when moving in AUTO/GUIDED/RTL */
#define BATTERY_DRAIN_INTERVAL_TICKS 10    /* 10 × 100ms = 1 second */
#define BATTERY_RECOVERY_INTERVAL_TICKS 10 /* 10 × 100ms = 1 second */
#define GPS_LOSS_TIMEOUT_TICKS        50   /* 5 seconds (50 × 100ms) */
#define BATTERY_RECOVERY_PCT          25

static uint16_t gps_loss_counter = 0;
static uint8_t  gps_was_lost     = 0;
static uint8_t  battery_tick_counter = 0;

/* ── periodic debug STATUSTEXT counter ── */
static uint8_t mission_debug_counter = 0;
/* ── helper: simulated NVM (from linker script .nvm section) ── */
extern uint8_t _nvm_start[];
extern uint8_t _nvm_end[];
#define NVM_BASE  ((volatile uint8_t *)_nvm_start)

/* ── SHA-256 full final (produce 32 bytes, not just 6) ── */
static void mavlink_sha256_final_full(mavlink_sha256_ctx *m, uint8_t result[MISSION_HASH_SIZE])
{
    unsigned char zeros[72];
    unsigned offset = (m->sz[0] / 8) % 64;
    unsigned int dstart = (120 - offset - 1) % 64 + 1;
    uint8_t *p = (uint8_t *)&m->counter[0];
    
    *zeros = 0x80;
    memset(zeros + 1, 0, sizeof(zeros) - 1);
    zeros[dstart+7] = (m->sz[0] >> 0) & 0xff;
    zeros[dstart+6] = (m->sz[0] >> 8) & 0xff;
    zeros[dstart+5] = (m->sz[0] >> 16) & 0xff;
    zeros[dstart+4] = (m->sz[0] >> 24) & 0xff;
    zeros[dstart+3] = (m->sz[1] >> 0) & 0xff;
    zeros[dstart+2] = (m->sz[1] >> 8) & 0xff;
    zeros[dstart+1] = (m->sz[1] >> 16) & 0xff;
    zeros[dstart+0] = (m->sz[1] >> 24) & 0xff;

    mavlink_sha256_update(m, zeros, dstart + 8);

    /* extract all 8 counter values as big-endian bytes for full 32-byte hash */
    for (int i = 0; i < 8; i++) {
        result[i*4 + 0] = p[3 + i*4];
        result[i*4 + 1] = p[2 + i*4];
        result[i*4 + 2] = p[1 + i*4];
        result[i*4 + 3] = p[0 + i*4];
    }
}

/* ── compute SHA-256 hash of mission waypoint data ── */
static void compute_mission_hash(const waypoint_t *wp, uint16_t count, uint8_t hash[MISSION_HASH_SIZE])
{
    mavlink_sha256_ctx ctx;
    uint8_t buf[16];  /* 16 bytes per waypoint: 4+4+4+2+2 padding, zero-filled */
    mavlink_sha256_init(&ctx);
    for (uint16_t i = 0; i < count; i++) {
        memcpy(buf,       &wp[i].lat,     4);  /* int32_t lat */
        memcpy(buf + 4,   &wp[i].lon,     4);  /* int32_t lon */
        memcpy(buf + 8,   &wp[i].alt_raw, 4);  /* uint32_t alt_raw */
        memcpy(buf + 12,  &wp[i].command, 2);  /* uint16_t command */
        buf[14] = 0;  /* explicit zero padding */
        buf[15] = 0;
        mavlink_sha256_update(&ctx, buf, 16);
    }
    mavlink_sha256_final_full(&ctx, hash);
}

static uint8_t verify_mission_hash(void)
{
    if (mission_count == 0)
        return 0;

    compute_mission_hash(mission, mission_count, current_mission_hash);

    for (uint16_t i = 0; i < MISSION_HASH_SIZE; i++) {
        if (current_mission_hash[i] != stored_mission_hash[i])
            return 0;
    }
    return 1;
}

static void debug_uart_message(const char *label, uint32_t value)
{
    uart_print(label);
    uart_print_int(value);
    uart_putchar('\n');
}

/* ── verify mission integrity ── */
/* returns 1 if hash matches stored hash, 0 if tampered */
static uint32_t read_uint32_param(uint16_t index)
{
    union { float f; uint32_t u; } conv;
    conv.f = get_param_value(index);
    return conv.u;
}

static uint8_t load_pending_mission_metadata(void)
{
    int16_t idx_id = find_param_index("MISSION_ID");
    int16_t idx_ver = find_param_index("MISSION_VER");
    if (idx_id < 0 || idx_ver < 0) {
        uart_print("DBG: META IDX FAIL\n");
        send_statustext(MAV_SEVERITY_WARNING, "MISSION META PARAM MISSING");
        return 0;
    }

    uint32_t id = read_uint32_param((uint16_t)idx_id);
    uint32_t ver = read_uint32_param((uint16_t)idx_ver);

    if (id == 0 || ver == 0) {
        uart_print("DBG: META ZERO\n");
        send_statustext(MAV_SEVERITY_WARNING, "MISSION META ZERO");
        return 0;
    }

    if (current_mission_id != 0 && id == current_mission_id && ver <= current_mission_ver) {
        uart_print("DBG: META STALE\n");
        send_statustext(MAV_SEVERITY_WARNING, "MISSION META STALE");
        return 0;
    }

    pending_mission_id = id;
    pending_mission_ver = ver;
    debug_uart_message("DBG META ID=", id);
    debug_uart_message("DBG META VER=", ver);
    return 1;
}

static uint32_t generate_mission_challenge(void)
{
    mission_challenge_seed = mission_challenge_seed * 1664525UL + 1013904223UL;
    uint32_t payload = mission_challenge_seed & 0x007FFFFFUL;
    if (payload == 0)
        payload = 1;
    return payload;  /* integer challenge value */
}

static uint32_t read_mission_challenge_response(void)
{
    int16_t idx = find_param_index("MISSION_CHAL_RESP");
    if (idx < 0)
        return 0;

    float value = get_param_value((uint16_t)idx);
    uint32_t response = (uint32_t)value;
    if ((float)response != value)
        return 0;

    return response;
}

static void try_authorize_challenge_response(void)
{
    if (mission_upload_authorized_flag || current_mission_challenge == 0)
        return;

    uint32_t response = read_mission_challenge_response();
    debug_uart_message("DBG TRY_AUTH RSP=", response);
    debug_uart_message("DBG TRY_AUTH CHAL=", current_mission_challenge);
    /* user-facing UART logs for experiment */
    uart_print("\nMISSION CHALLENGE RECEIVED\n\n");
    if (response != 0 && response == current_mission_challenge) {
        uart_print("MISSION RESPONSE VERIFIED\n\n");
        uart_print("MISSION AUTHORIZED\n\n");
        mission_upload_authorized_flag = 1;
        send_statustext(MAV_SEVERITY_INFO, "MISSION AUTHORIZED");
    } else if (response != 0) {
        uart_print("MISSION CHALLENGE EXPECTED : ");
        uart_print_int(current_mission_challenge);
        uart_print("\n\n");
        uart_print("MISSION RESPONSE INVALID\n\n");
        uart_print("MISSION AUTHORIZATION FAILED\n\n");
        send_statustext(MAV_SEVERITY_WARNING, "MISSION AUTH RESP MISMATCH");
    }
}

static void publish_mission_challenge(void)
{
    union { float f; uint32_t u; } conv;
    conv.f = (float)current_mission_challenge;
    uart_print("DEBUG PUBLISH_MISSION_CHALLENGE current_mission_challenge = ");
    uart_print_int(current_mission_challenge);
    uart_putchar('\n');
    set_param_value(PARAM_MISSION_CHALLENGE, conv.f);
    send_param_value(PARAM_MISSION_CHALLENGE);
    debug_uart_message("DBG CHAL=", current_mission_challenge);
}

uint8_t mission_upload_authorized(void)
{
    return mission_upload_authorized_flag;
}

uint8_t mission_begin_upload(uint16_t count)
{
    if (count == 0 || count > MAX_WAYPOINTS)
        return 0;

    if (!load_pending_mission_metadata())
        return 0;

    mission_rx_idx = 0;
    mission_count = count;
    mission_loaded = 0;
    mission_reset();
    mission_upload_pending = 1;
    mission_upload_authorized_flag = 0;

    try_authorize_challenge_response();

    union { float f; uint32_t u; } conv;
    conv.f = (float)current_mission_challenge;
    set_param_value(PARAM_MISSION_CHALLENGE, conv.f);
    send_param_value(PARAM_MISSION_CHALLENGE);

    conv.u = pending_mission_id;
    set_param_value(PARAM_MISSION_ID, conv.f);
    send_param_value(PARAM_MISSION_ID);

    conv.u = pending_mission_ver;
    set_param_value(PARAM_MISSION_VER, conv.f);
    send_param_value(PARAM_MISSION_VER);

    debug_uart_message("DBG UPLOAD_CHAL=", current_mission_challenge);
    debug_uart_message("DBG AUTH_FLG=", mission_upload_authorized_flag);
    return 1;
}

static float float_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

uint8_t mission_submit_upload_response(float response)
{
    if (current_mission_challenge == 0)
        return 0;

    uint32_t response_int = (uint32_t)response;
    if ((float)response_int != response)
        return 0;

    /* Experiment UART logs: received, verified/invalid, authorization result */
    uart_print("\nMISSION CHALLENGE RECEIVED\n\n");
    if (response_int == current_mission_challenge) {
        uart_print("MISSION RESPONSE VERIFIED\n\n");
        uart_print("MISSION AUTHORIZED\n\n");
        mission_upload_authorized_flag = 1;
        debug_uart_message("DBG AUTH OK=", response_int);
        return 1;
    }

    uart_print("MISSION CHALLENGE EXPECTED : ");
    uart_print_int(current_mission_challenge);
    uart_print("\n\n");
    uart_print("MISSION RESPONSE INVALID\n\n");
    uart_print("MISSION AUTHORIZATION FAILED\n\n");
    debug_uart_message("DBG AUTH BAD=", response_int);
    return 0;
}

static void nvm_write(const void *src, unsigned int len)
{
    const uint8_t *s = (const uint8_t *)src;
    for (unsigned int i = 0; i < len; i++)
        NVM_BASE[i] = s[i];
}

static void nvm_read(void *dst, unsigned int len)
{
    uint8_t *d = (uint8_t *)dst;
    for (unsigned int i = 0; i < len; i++)
        d[i] = NVM_BASE[i];
}

/* ── helpers ── */
static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t alt_bits_to_mm(uint32_t alt_raw)
{
    union { float f; uint32_t u; } alt;
    alt.u = alt_raw;
    return (int32_t)(alt.f * 1000.0f);
}

/* ── waypoint command helpers ── */
static const char *waypoint_action_str(uint16_t command)
{
    switch (command) {
    case 16:  return "WP NAV";
    case 21:  return "WP LAND";
    case 20:  return "WP RTL";
    case 115: return "WP LOITER_TIME";
    case 177: return "WP DO_CHANGE_SPEED";
    default:  return "WP";
    }
}

/* ── mission reset ── */
void mission_reset(void)
{
    current_waypoint_idx = 0;
    mission_active = 0;
    rtl_active = 0;
    mission_loaded = 0;
    current_heading = 0;
    mission_hash_valid = 0;
    mission_upload_pending = 0;
    mission_upload_authorized_flag = 0;

    publish_mission_challenge();
}

/* ── battery simulation ── */
void sim_battery_update(void)
{
    battery_tick_counter++;
    uint8_t battery_update = 0;
    if (battery_tick_counter >= BATTERY_DRAIN_INTERVAL_TICKS) {
        battery_tick_counter = 0;
        battery_update = 1;
    }

    if (battery_update) {
        /* discharge when armed */
        if (armed_state) {
            /* higher discharge when moving (AUTO/GUIDED/RTL) */
            uint8_t drain = BATTERY_DRAIN_IDLE_PCT;
            if (custom_mode == 3 || custom_mode == 4 || custom_mode == 6)
                drain = BATTERY_DRAIN_MOVE_PCT;

            if (battery_remaining_pct > drain)
                battery_remaining_pct -= drain;
            else
                battery_remaining_pct = 0;

            /* voltage follows remaining % linearly: 12.6V @ 100% → 10.8V @ 0% */
            uint32_t raw = (uint32_t)battery_remaining_pct * 1800UL;
            battery_voltage_mv = 10800 + (uint16_t)(raw / 100);
            battery_current_ma = (int16_t)(drain * 10);
        } else {
            /* trickle charge when disarmed */
            if (battery_remaining_pct < 100) {
                battery_remaining_pct += 1;
                if (battery_remaining_pct > 100)
                    battery_remaining_pct = 100;
            }
            uint32_t raw = (uint32_t)battery_remaining_pct * 1800UL;
            battery_voltage_mv = 10800 + (uint16_t)(raw / 100);
            battery_current_ma = 0;
        }
    }

    /* simulate GPS fix dropping below 6 satellites occasionally */
    if (gps_satellites > 6) {
        if ((sys_tick % 137) == 0)
            gps_satellites--;
    } else if (gps_satellites < 6 && (sys_tick % 251) == 0) {
        gps_satellites++;
    }
    gps_fix_type = (gps_satellites >= 4) ? 3 : 2;
}

/* ── failsafe logic ── */
void failsafe_check(void)
{
    /* ── Low Battery → RTL ── */
    if (battery_remaining_pct <= BATTERY_CRITICAL_PCT && armed_state) {
        if (!failsafe_battery) {
            failsafe_battery = 1;
            failsafe_gps = 0;  /* clear GPS failsafe, RTL takes priority */
            send_statustext(MAV_SEVERITY_CRITICAL, "FS:BAT LOW → RTL");
            /* force RTL if not already in RTL */
            if (custom_mode != 6) {
                custom_mode = 6;
                send_statustext(MAV_SEVERITY_INFO, "MODE RTL");
            }
        }
    } else if (battery_remaining_pct >= BATTERY_RECOVERY_PCT) {
        failsafe_battery = 0;
    }

    /* ── GPS Loss → LOITER ── */
    if (gps_fix_type < 3 && armed_state) {
        gps_loss_counter++;
        if (gps_loss_counter >= GPS_LOSS_TIMEOUT_TICKS && !failsafe_gps) {
            failsafe_gps = 1;
            send_statustext(MAV_SEVERITY_CRITICAL, "FS:GPS LOST→LOITER");
            /* force LOITER if not already in a failsafe or RTL */
            if (!failsafe_battery && custom_mode != 5) {
                custom_mode = 5;
                send_statustext(MAV_SEVERITY_INFO, "MODE LOITER");
            }
        }
    } else {
        gps_loss_counter = 0;
        if (gps_fix_type >= 3)
            failsafe_gps = 0;
    }
}

/* ── mission persistence ── */
void save_mission_to_nvm(void)
{
    if (mission_count == 0)
        return;

    nvm_mission_t data;
    unsigned int i;

    data.signature = NVM_SIGNATURE;
    data.count = mission_count;
    data.mission_id = current_mission_id;
    data.mission_ver = current_mission_ver;
    data.mission_challenge = 0;

    for (i = 0; i < mission_count; i++) {
        data.waypoints[i].lat     = mission[i].lat;
        data.waypoints[i].lon     = mission[i].lon;
        data.waypoints[i].alt_raw = mission[i].alt_raw;
        data.waypoints[i].command = mission[i].command;
    }

    current_mission_id = pending_mission_id;
    current_mission_ver = pending_mission_ver;
    pending_mission_id = 0;
    pending_mission_ver = 0;
    mission_upload_pending = 0;
    mission_upload_authorized_flag = 0;

    /* compute SHA-256 hash of waypoint data before saving */
    compute_mission_hash(data.waypoints, data.count, data.hash);
    memcpy(stored_mission_hash, data.hash, MISSION_HASH_SIZE);
    memcpy(current_mission_hash, data.hash, MISSION_HASH_SIZE);
    mission_hash_valid = 1;
    uart_print("MISSION HASH GENERATED\n");
    uart_print("MISSION HASH STORED\n");

    nvm_write(&data, sizeof(nvm_mission_t));
    send_statustext(MAV_SEVERITY_INFO, "MISSION VERIFIED AND SAVED");
}

uint8_t load_mission_from_nvm(void)
{
    nvm_mission_t data;
    unsigned int i;

    nvm_read(&data, sizeof(nvm_mission_t));

    if (data.signature != NVM_SIGNATURE)
        return 0;

    mission_count = data.count;
    if (mission_count > MAX_WAYPOINTS)
        mission_count = MAX_WAYPOINTS;

    for (i = 0; i < mission_count; i++) {
        mission[i].lat     = data.waypoints[i].lat;
        mission[i].lon     = data.waypoints[i].lon;
        mission[i].alt_raw = data.waypoints[i].alt_raw;
        mission[i].command = data.waypoints[i].command;
    }

    current_mission_id = data.mission_id;
    current_mission_ver = data.mission_ver;
    /* keep the session challenge generated at boot; do not reset it here */

    /* store the hash separately for verification later */
    memcpy(stored_mission_hash, data.hash, MISSION_HASH_SIZE);

    mission_loaded = 1;
    current_waypoint_idx = 0;
    mission_hash_valid = 1;

    /* verify integrity immediately on load */
    if (!verify_mission_hash()) {
        mission_hash_valid = 0;
        mission_loaded = 0;
        send_statustext(MAV_SEVERITY_CRITICAL, "NVM HASH MISMATCH");
        return 0;
    }

    uart_print("DEBUG LOAD_MISSION_FROM_NVM current_mission_challenge = ");
    uart_print_int(current_mission_challenge);
    uart_putchar('\n');

    publish_mission_challenge();
    send_statustext(MAV_SEVERITY_INFO, "MISSION RESTORED");
    return 1;
}

/* ── approximate atan2 in degrees × 100 (no FPU needed) ── */
/* returns heading in centidegrees (0-36000) given delta lat/lon */
uint16_t approx_heading_centideg(int32_t dlat, int32_t dlon)
{
    if (dlat == 0 && dlon == 0)
        return (uint16_t)current_heading * 100;

    /* use the larger axis to normalise */
    int32_t adlat = abs_i32(dlat);
    int32_t adlon = abs_i32(dlon);
    int32_t ratio;  /* ratio × 1000 */

    if (adlat >= adlon) {
        if (adlat == 0) return 0;
        ratio = (adlon * 1000) / adlat;
    } else {
        ratio = (adlat * 1000) / adlon;
    }

    /* approximate angle = 45° × ratio/1000, then map to correct quadrant */
    uint16_t angle_cd = (uint16_t)((45UL * (uint32_t)ratio) / 1000) * 100;

    if (angle_cd > 4500) angle_cd = 4500;  /* clamp to 45° */

    /* map to correct quadrant */
    if (dlon >= 0 && dlat >= 0) return angle_cd;                    /* Q1: NE */
    if (dlon >= 0 && dlat < 0)  return 18000 - angle_cd;            /* Q2: SE */
    if (dlon < 0  && dlat < 0)  return 18000 + angle_cd;            /* Q3: SW */
    return 36000 - angle_cd;                                         /* Q4: NW */
}

/* ── move toward target with proportional control ── */
/* returns 1 if arrived, 0 if still moving */
static uint8_t move_toward(int32_t *current, int32_t target,
                           int32_t max_step, int32_t threshold)
{
    int32_t diff = target - *current;
    int32_t adiff = abs_i32(diff);

    if (adiff <= threshold) {
        *current = target;
        return 1;
    }

    /* proportional step: at least 20% of max_step, up to max_step */
    int32_t step = (adiff * 3) / 10;  /* 30% of remaining distance */
    if (step < 20)  step = 20;        /* minimum movement */
    if (step > max_step) step = max_step;

    *current += (diff > 0) ? step : -step;
    return 0;
}

/* ── move toward target with bearing-based distribution ── */
/* distributes the step between lat and lon based on the bearing */
static void move_toward_2d(int32_t *lat, int32_t *lon,
                           int32_t target_lat, int32_t target_lon,
                           int32_t max_step, int32_t threshold)
{
    int32_t dlat = target_lat - *lat;
    int32_t dlon = target_lon - *lon;
    int32_t adlat = abs_i32(dlat);
    int32_t adlon = abs_i32(dlon);

    /* if within threshold on both axes, snap */
    if (adlat <= threshold && adlon <= threshold) {
        *lat = target_lat;
        *lon = target_lon;
        return;
    }

    /* compute the total "distance" for proportional distribution */
    int32_t total = adlat + adlon;
    if (total == 0) return;

    /* proportional step: 30% of remaining distance, clamped */
    int32_t step = (total * 3) / 10;
    if (step < 20)  step = 20;
    if (step > max_step) step = max_step;

    /* distribute step proportionally between lat and lon */
    int32_t lat_step = (step * adlat) / total;
    int32_t lon_step = step - lat_step;  /* ensures we move exactly `step` total */

    /* ensure minimum movement on non-zero axes, then reclamp total */
    if (adlat > 0 && lat_step < 20) lat_step = 20;
    if (adlon > 0 && lon_step < 20) lon_step = 20;

    /* reclamp total step to max_step after minimum adjustments */
    int32_t total_step = lat_step + lon_step;
    if (total_step > max_step) {
        /* scale both proportionally down */
        lat_step = (lat_step * max_step) / total_step;
        lon_step = (lon_step * max_step) / total_step;
    }

    /* don't overshoot */
    if (lat_step > adlat) lat_step = adlat;
    if (lon_step > adlon) lon_step = adlon;

    *lat += (dlat > 0) ? lat_step : -lat_step;
    *lon += (dlon > 0) ? lon_step : -lon_step;
}

/* ── RTL experiment: print measurement results ── */
static void rtl_print_experiment_result(void)
{
    if (!rtl_measurement_active)
        return;
    rtl_end_tick = sys_tick;
    rtl_measurement_active = 0;
    uint32_t rtl_time_ms = rtl_end_tick - rtl_start_tick;
    uart_print("\n========================================\n");
    uart_print("RTL EXPERIMENT\n");
    uart_print("========================================\n");
    uart_print("RTL Start Distance     : ");
    /* Print distance in XX.XX m format from rtl_start_distance_cm */
    uart_print_int(rtl_start_distance_cm / 100);
    uart_print(".");
    uint16_t rtl_dist_frac = rtl_start_distance_cm % 100;
    uart_print_int(rtl_dist_frac / 10);
    uart_print_int(rtl_dist_frac % 10);
    uart_print(" m\n");
    uart_print("RTL Start Tick         : ");
    uart_print_int(rtl_start_tick);
    uart_print("\n");
    uart_print("RTL End Tick           : ");
    uart_print_int(rtl_end_tick);
    uart_print("\n");
    uart_print("RTL Completion Time    : ");
    uart_print_int(rtl_time_ms);
    uart_print(" ms\n");
    uart_print("RTL Completion Time    : ");
    uart_print_int(rtl_time_ms / 1000);
    uart_print(".");
    uint16_t rtl_frac = rtl_time_ms % 1000;
    uart_print_int(rtl_frac / 100);
    uart_print_int((rtl_frac / 10) % 10);
    uart_print_int(rtl_frac % 10);
    uart_print(" s\n");
    uart_print("========================================\n\n");
}

/* ── compute waypoint acceptance threshold in degE7 from configurable radius (m) ── */
/* 1 degE7 ≈ 0.1113 m at equator, so 1 m ≈ 9 degE7 */
static int32_t wp_threshold_from_radius(void)
{
    uint32_t r = wp_radius_m;
    if (r < 3)  r = 3;    /* minimum 3m */
    if (r > 500) r = 500; /* maximum 500m */
    return (int32_t)(r * 9);  /* radius in degE7 */
}

/* ── mission update (main navigation loop) ── */
void mission_update(void)
{
    const int32_t position_step = 1000;      /* max step per tick (degE7) */
    const int32_t altitude_step = 200;       /* max altitude step per tick (mm) */
    const int32_t altitude_threshold = 500;  /* acceptance altitude diff (mm) */
    int32_t position_threshold = wp_threshold_from_radius();

    /* ── RTL mode ── */
    if (custom_mode == 6) {
        int32_t dlat = home_lat - current_lat;
        int32_t dlon = home_lon - current_lon;
        int32_t dalt = home_alt - current_alt_mm;

        current_heading = approx_heading_centideg(dlat, dlon) / 100;

        if (!rtl_active) {
            mission_active = 0;
            if (abs_i32(dlat) <= position_threshold &&
                abs_i32(dlon) <= position_threshold &&
                abs_i32(dalt) <= altitude_threshold) {
                rtl_active = 2;
                current_lat = home_lat;
                current_lon = home_lon;
                current_alt_mm = home_alt;
                send_statustext(MAV_SEVERITY_INFO, "RTL COMPLETE");
                rtl_print_experiment_result();
                return;
            }

            rtl_active = 1;
            send_statustext(MAV_SEVERITY_INFO, "RTL START");

            /* ── RTL experiment: record start tick and compute actual distance to home ── */
            rtl_start_tick = sys_tick;
            rtl_measurement_active = 1;
            {
                int32_t rtl_dlat = home_lat - current_lat;
                int32_t rtl_dlon = home_lon - current_lon;
                uint32_t rtl_adlat = (rtl_dlat < 0) ? (uint32_t)(-rtl_dlat) : (uint32_t)rtl_dlat;
                uint32_t rtl_adlon = (rtl_dlon < 0) ? (uint32_t)(-rtl_dlon) : (uint32_t)rtl_dlon;
                uint32_t rtl_approx = (rtl_adlat >= rtl_adlon) ? rtl_adlat + (rtl_adlon / 2) : rtl_adlon + (rtl_adlat / 2);
                /* Corrected degE7-to-cm conversion:
                   1 degE7 = 1e-7 degrees
                   1 degree ≈ 111,320 m = 11,132,000 cm
                   So 1 degE7 ≈ 11,132,000 * 1e-7 = 1.1132 cm
                   → cm = approx * 1.1132 = (approx * 11132) / 10000
                   Simplified: (approx * 111) / 100 gives cm with <10% error
                   (the remaining error is from the Manhattan distance approximation) */
                rtl_start_distance_cm = (rtl_approx * 111UL) / 100UL;

                /* DEBUG: print all coordinate values to verify distance calculation */
                uart_print("\n========================================\n");
                uart_print("RTL DEBUG\n");
                uart_print("========================================\n");
                uart_print("Current Lat : ");
                uart_print_int(current_lat);
                uart_print("\n");
                uart_print("Current Lon : ");
                uart_print_int(current_lon);
                uart_print("\n");
                uart_print("RTL Target Lat : ");
                uart_print_int(home_lat);
                uart_print("\n");
                uart_print("RTL Target Lon : ");
                uart_print_int(home_lon);
                uart_print("\n");
                uart_print("Delta Lat : ");
                uart_print_int(rtl_dlat);
                uart_print("\n");
                uart_print("Delta Lon : ");
                uart_print_int(rtl_dlon);
                uart_print("\n");
                uart_print("Approx (degE7) : ");
                uart_print_int(rtl_approx);
                uart_print("\n");
                uart_print("Computed Distance (cm) : ");
                uart_print_int(rtl_start_distance_cm);
                uart_print("\n");
                uart_print("Computed Distance (m)  : ");
                uart_print_int(rtl_start_distance_cm / 100);
                uart_print(".");
                uart_print_int((rtl_start_distance_cm / 10) % 10);
                uart_print_int(rtl_start_distance_cm % 10);
                uart_print("\n");
                uart_print("========================================\n\n");
            }
        }

        /* check arrival */
        if (abs_i32(dlat) <= position_threshold &&
            abs_i32(dlon) <= position_threshold &&
            abs_i32(dalt) <= altitude_threshold) {
            current_lat = home_lat;
            current_lon = home_lon;
            current_alt_mm = home_alt;
            if (rtl_active == 1) {
                rtl_active = 2;
                send_statustext(MAV_SEVERITY_INFO, "RTL COMPLETE");
                rtl_print_experiment_result();
            }
            return;
        }

        /* move toward home with bearing-based 2D navigation */
        move_toward_2d(&current_lat, &current_lon, home_lat, home_lon,
                       position_step, position_threshold);
        move_toward(&current_alt_mm, home_alt, altitude_step, altitude_threshold);
        return;
    }

    rtl_active = 0;

    /* ── GUIDED mode click-to-fly ── */
    if (custom_mode == 4 && guided_target_set) {
        int32_t dlat = guided_target_lat - current_lat;
        int32_t dlon = guided_target_lon - current_lon;
        int32_t dalt = guided_target_alt - current_alt_mm;

        current_heading = approx_heading_centideg(dlat, dlon) / 100;

        /* arrived at guided target */
        if (abs_i32(dlat) <= position_threshold &&
            abs_i32(dlon) <= position_threshold &&
            abs_i32(dalt) <= altitude_threshold) {
            current_lat = guided_target_lat;
            current_lon = guided_target_lon;
            current_alt_mm = guided_target_alt;
            guided_target_set = 0;
            send_statustext(MAV_SEVERITY_INFO, "GUIDED TARGET REACHED");
            return;
        }

        /* move toward guided target */
        move_toward_2d(&current_lat, &current_lon, guided_target_lat, guided_target_lon,
                       position_step, position_threshold);
        move_toward(&current_alt_mm, guided_target_alt, altitude_step, altitude_threshold);
        return;
    }

    /* ── AUTO mode: waypoint mission ── */
    if (custom_mode == 3) {
        /* DEBUG: trace entry into AUTO block */
        uart_print("DBG AUTO: entered custom_mode=3 block\n");
        uart_print("DBG AUTO: mission_count=");
        uart_print_int(mission_count);
        uart_print(" mission_loaded=");
        uart_print_int(mission_loaded);
        uart_print(" mission_hash_valid=");
        uart_print_int(mission_hash_valid);
        uart_print(" mission_active=");
        uart_print_int(mission_active);
        uart_print("\n");

        /* no mission loaded */
        if (mission_count == 0 || !mission_loaded) {
            uart_print("DBG AUTO: no mission loaded, returning\n");
            mission_active = 0;
            return;
        }

        /* DEBUG: check verification gate condition */
        uart_print("DBG AUTO: checking verify gate (hash_valid && !active) = ");
        uart_print_int(mission_hash_valid && !mission_active);
        uart_print("\n");

        /* verify mission integrity on every AUTO mode entry (before mission_active) */
        if (mission_hash_valid && !mission_active) {
            uart_print("DBG AUTO: calling verify_mission_hash()...\n");
            uint8_t verify_result = verify_mission_hash();
            uart_print("DBG AUTO: verify_mission_hash returned ");
            uart_print_int(verify_result);
            uart_print("\n");
            if (verify_result) {
                uart_print("MISSION HASH VERIFIED\n");
                uart_print("MISSION INTEGRITY VERIFIED\n");
                uart_print("AUTO MODE ENABLED\n");
                send_statustext(MAV_SEVERITY_INFO, "MISSION HASH VERIFIED");
            } else {
                uart_print("MISSION HASH MISMATCH\n");
                uart_print("MISSION TAMPERED\n");
                uart_print("AUTO MODE BLOCKED\n");
                send_statustext(MAV_SEVERITY_CRITICAL, "MISSION TAMPERED");
                if (custom_mode == 3) {
                    custom_mode = 0;
                    send_statustext(MAV_SEVERITY_INFO, "MODE STABILIZE");
                }
                return;
            }
        }

        if (!mission_active) {
            mission_active = 1;
            send_statustext(MAV_SEVERITY_INFO, "MISSION VALID");
            send_statustext(MAV_SEVERITY_INFO, "MISSION START");
        }

        if (current_waypoint_idx < mission_count)
            send_mission_current(current_waypoint_idx);

        waypoint_t *waypoint = &mission[current_waypoint_idx];
        int32_t target_alt_mm = alt_bits_to_mm(waypoint->alt_raw);

        /* compute heading toward waypoint */
        int32_t dlat = waypoint->lat - current_lat;
        int32_t dlon = waypoint->lon - current_lon;
        current_heading = approx_heading_centideg(dlat, dlon) / 100;

        /* debug waypoint navigation state */
        uart_print("AUTO NAV IDX: ");
        uart_print_int(current_waypoint_idx);
        uart_print("\nCUR LAT: ");
        uart_print_int(current_lat);
        uart_print(" LON: ");
        uart_print_int(current_lon);
        uart_print(" ALT: ");
        uart_print_int(current_alt_mm);
        uart_print("\nTGT LAT: ");
        uart_print_int(waypoint->lat);
        uart_print(" LON: ");
        uart_print_int(waypoint->lon);
        uart_print(" ALT: ");
        uart_print_int(target_alt_mm);
        uart_print("\n");

        int32_t dist_lat = abs_i32(dlat);
        int32_t dist_lon = abs_i32(dlon);
        int32_t dist_approx = (dist_lat >= dist_lon) ? dist_lat + (dist_lon / 2) : dist_lon + (dist_lat / 2);
        uart_print("DIST APPROX: ");
        uart_print_int(dist_approx);
        uart_print(" THR: ");
        uart_print_int(position_threshold);
        uart_print("\n");

        /* periodic STATUSTEXT debug every 50 ticks (5 seconds) */
        mission_debug_counter++;
        if (mission_debug_counter >= 50) {
            mission_debug_counter = 0;
            /* send wp_idx, distance, battery */
            {
                uint32_t dist_m = (uint32_t)dist_approx * 111UL / 1000UL;
                char dbg_buf[50];
                uint8_t dbg_i = 0;
                /* build "WP:0 D:1234 B:95" manually */
                dbg_buf[0] = 'W'; dbg_buf[1] = 'P'; dbg_buf[2] = ':';
                uint16_t idx = current_waypoint_idx;
                dbg_buf[3] = '0' + (idx / 10);
                dbg_buf[4] = '0' + (idx % 10);
                dbg_buf[5] = ' '; dbg_buf[6] = 'D'; dbg_buf[7] = ':';
                uint16_t d1 = dist_m / 1000;
                uint16_t d2 = dist_m % 1000;
                dbg_buf[8] = '0' + d1;
                dbg_buf[9] = '0' + (d2 / 100);
                dbg_buf[10] = '0' + ((d2 / 10) % 10);
                dbg_buf[11] = '0' + (d2 % 10);
                dbg_buf[12] = ' '; dbg_buf[13] = 'B'; dbg_buf[14] = ':';
                uint8_t bat = battery_remaining_pct;
                dbg_buf[15] = '0' + (bat / 100);
                dbg_buf[16] = '0' + ((bat / 10) % 10);
                dbg_buf[17] = '0' + (bat % 10);
                dbg_buf[18] = '\0';
                send_statustext(MAV_SEVERITY_INFO, dbg_buf);
            }
        }

        /* check arrival at waypoint */
        if (abs_i32(dlat) <= position_threshold &&
            abs_i32(dlon) <= position_threshold &&
            abs_i32(target_alt_mm - current_alt_mm) <= altitude_threshold) {
            current_lat = waypoint->lat;
            current_lon = waypoint->lon;
            current_alt_mm = target_alt_mm;
            send_mission_item_reached(current_waypoint_idx);
            send_statustext(MAV_SEVERITY_INFO, waypoint_action_str(waypoint->command));
            uart_print("WAYPOINT REACHED: ");
            uart_print_int(current_waypoint_idx);
            uart_print("\n");
            current_waypoint_idx++;

            if (current_waypoint_idx >= mission_count) {
                send_statustext(MAV_SEVERITY_INFO, "MISSION COMPLETE");
                uart_print("MISSION COMPLETE\n");
                mission_active = 0;
                save_mission_to_nvm();
            } else {
                send_mission_current(current_waypoint_idx);
            }
            return;
        }

        move_toward_2d(&current_lat, &current_lon, waypoint->lat, waypoint->lon,
                       position_step, position_threshold);
        move_toward(&current_alt_mm, target_alt_mm, altitude_step, altitude_threshold);
    }
}
