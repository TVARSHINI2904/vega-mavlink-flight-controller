#include "mission.h"
#include "mavlink_tx.h"
#include "scheduler.h"

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
#define BATTERY_CRITICAL_PCT   15
#define BATTERY_DRAIN_PER_TICK 1        /* % per 100ms when armed + moving */
#define GPS_LOSS_TIMEOUT_TICKS 50       /* 5 seconds (50 × 100ms) */
#define BATTERY_RECOVERY_PCT   25

static uint16_t gps_loss_counter = 0;
static uint8_t  gps_was_lost     = 0;
/* ── helper: simulated NVM (from linker script .nvm section) ── */
extern uint8_t _nvm_start[];
extern uint8_t _nvm_end[];
#define NVM_BASE  ((volatile uint8_t *)_nvm_start)

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
}

/* ── battery simulation ── */
void sim_battery_update(void)
{
    /* discharge when armed */
    if (armed_state) {
        /* higher discharge when moving (AUTO/GUIDED/RTL) */
        uint8_t drain = BATTERY_DRAIN_PER_TICK;
        if (custom_mode == 3 || custom_mode == 4 || custom_mode == 6)
            drain = BATTERY_DRAIN_PER_TICK * 2;

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
    for (i = 0; i < mission_count; i++) {
        data.waypoints[i].lat     = mission[i].lat;
        data.waypoints[i].lon     = mission[i].lon;
        data.waypoints[i].alt_raw = mission[i].alt_raw;
        data.waypoints[i].command = mission[i].command;
    }

    nvm_write(&data, sizeof(nvm_mission_t));
    send_statustext(MAV_SEVERITY_INFO, "MISSION SAVED");
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

    mission_loaded = 1;
    current_waypoint_idx = 0;
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

    /* ensure minimum movement on non-zero axes */
    if (adlat > 0 && lat_step < 20) lat_step = 20;
    if (adlon > 0 && lon_step < 20) lon_step = 20;

    /* don't overshoot */
    if (lat_step > adlat) lat_step = adlat;
    if (lon_step > adlon) lon_step = adlon;

    *lat += (dlat > 0) ? lat_step : -lat_step;
    *lon += (dlon > 0) ? lon_step : -lon_step;
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
    const int32_t position_step = 500;      /* max step per tick (degE7) */
    const int32_t altitude_step = 100;      /* max altitude step per tick (mm) */
    const int32_t altitude_threshold = 500; /* acceptance altitude diff (mm) */
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
                return;
            }

            rtl_active = 1;
            send_statustext(MAV_SEVERITY_INFO, "RTL START");
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
        /* no mission loaded */
        if (mission_count == 0 || !mission_loaded) {
            mission_active = 0;
            return;
        }

        if (current_waypoint_idx >= mission_count)
            return;

        if (!mission_active) {
            mission_active = 1;
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

        /* check arrival at waypoint */
        if (abs_i32(dlat) <= position_threshold &&
            abs_i32(dlon) <= position_threshold &&
            abs_i32(target_alt_mm - current_alt_mm) <= altitude_threshold) {
            current_lat = waypoint->lat;
            current_lon = waypoint->lon;
            current_alt_mm = target_alt_mm;
            send_mission_item_reached(current_waypoint_idx);
            send_statustext(MAV_SEVERITY_INFO, waypoint_action_str(waypoint->command));
            current_waypoint_idx++;

            if (current_waypoint_idx >= mission_count) {
                send_statustext(MAV_SEVERITY_INFO, "MISSION COMPLETE");
                mission_active = 0;
                save_mission_to_nvm();
            } else {
                send_mission_current(current_waypoint_idx);
            }
            return;
        }

        /* move toward waypoint with bearing-based 2D navigation */
        move_toward_2d(&current_lat, &current_lon, waypoint->lat, waypoint->lon,
                       position_step, position_threshold);
        move_toward(&current_alt_mm, target_alt_mm, altitude_step, altitude_threshold);
    }
}
