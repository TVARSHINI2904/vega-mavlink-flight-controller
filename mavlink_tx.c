#include "mavlink_tx.h"
#include "uart.h"
#include "mission.h"

uint8_t armed_state = 0;
uint32_t custom_mode = 0;  /* 0=STABILIZE, 3=AUTO, 4=GUIDED, 5=LOITER, 6=RTL */
int32_t home_lat = 130827000;
int32_t home_lon = 802707000;
int32_t home_alt = 10000;
static float   home_x = 0.0f;
static float   home_y = 0.0f;
static float   home_z = 0.0f;
static float   home_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float   home_approach_x = 0.0f;
static float   home_approach_y = 0.0f;
static float   home_approach_z = 0.0f;

typedef struct {
    char id[16];
    float value;
    uint8_t type;
} param_entry_t;

static param_entry_t params[] = {
    {"SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_REAL32},
    {"SYSID_MYGCS", 255.0f, MAV_PARAM_TYPE_REAL32},
    {"ARMING_CHECK", 0.0f, MAV_PARAM_TYPE_REAL32},
    {"FRAME_CLASS", 1.0f, MAV_PARAM_TYPE_REAL32},
    {"SERIAL0_BAUD", 115.0f, MAV_PARAM_TYPE_REAL32},
};

#define PARAM_COUNT ((uint16_t)(sizeof(params) / sizeof(params[0])))

static uint8_t heartbeat_base_mode(void)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    if (armed_state)
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

    if (custom_mode == 3)
        base_mode |= MAV_MODE_FLAG_AUTO_ENABLED |
                     MAV_MODE_FLAG_GUIDED_ENABLED |
                     MAV_MODE_FLAG_STABILIZE_ENABLED;
    else if (custom_mode == 4)
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED |
                     MAV_MODE_FLAG_STABILIZE_ENABLED;
    else
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;

    return base_mode;
}

static const char *mode_name(uint32_t mode)
{
    switch (mode) {
    case 0: return "MODE STABILIZE";
    case 3: return "MODE AUTO";
    case 4: return "MODE GUIDED";
    case 5: return "MODE LOITER";
    case 6: return "MODE RTL";
    default: return "MODE CUSTOM";
    }
}

void set_flight_mode(uint32_t mode)
{
    custom_mode = mode;
    send_statustext(MAV_SEVERITY_INFO, mode_name(mode));
}

void update_home_position(int32_t latitude, int32_t longitude, int32_t altitude,
                          float x, float y, float z,
                          const float q[4],
                          float approach_x, float approach_y, float approach_z)
{
    home_lat = latitude;
    home_lon = longitude;
    home_alt = altitude;
    home_x = x;
    home_y = y;
    home_z = z;
    home_q[0] = q[0];
    home_q[1] = q[1];
    home_q[2] = q[2];
    home_q[3] = q[3];
    home_approach_x = approach_x;
    home_approach_y = approach_y;
    home_approach_z = approach_z;
    send_statustext(MAV_SEVERITY_INFO, "HOME UPDATED");
    send_gps_global_origin();
    send_home_position();
}

/* simulated sensor state */
static int32_t sim_pitch_mrad = 0;
static int32_t sim_roll_mrad  = 0;
static int32_t sim_alt_mm     = 0;

/* ── core send helper ── */
void mav_send_message(mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    for (uint16_t i = 0; i < len; i++)
        uart_putchar(buf[i]);
}

/* ── HEARTBEAT ── */
void send_heartbeat(void)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        heartbeat_base_mode(),
        custom_mode,
        MAV_STATE_ACTIVE
    );
    mav_send_message(&msg);
}

void send_statustext(uint8_t severity, const char *text)
{
    mavlink_message_t msg;
    char buf[50];
    uint8_t i = 0;

    for (; i < sizeof(buf); i++)
        buf[i] = '\0';
    for (i = 0; i < (sizeof(buf) - 1) && text[i] != '\0'; i++)
        buf[i] = text[i];

    mavlink_msg_statustext_pack(
        1, 1, &msg,
        severity,
        buf,
        0,
        0
    );
    mav_send_message(&msg);
}

/* ── SYS_STATUS ── */
void send_sys_status(void)
{
    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(
        1, 1, &msg,
        0x1F, 0x1F, 0x1F,  /* sensors present/enabled/health */
        300,                 /* load 30% */
        12600,               /* voltage_battery mV */
        150,                 /* current_battery 1.5A */
        100,                 /* battery_remaining % */
        0, 0, 0, 0, 0, 0,   /* drop_rate, errors x4 */
        0, 0, 0              /* extended sensor fields */
    );
    mav_send_message(&msg);
}

/* ── ATTITUDE ── */
void send_attitude(void)
{
    mavlink_message_t msg;

    sim_pitch_mrad += 10;
    if (sim_pitch_mrad > 500) sim_pitch_mrad = -500;
    sim_roll_mrad += 20;
    if (sim_roll_mrad > 800) sim_roll_mrad = -800;

    /* integer milliradians → IEEE754 float via union, no FPU */
    union { float f; uint32_t u; } pitch, roll, zero, half;
    zero.u = 0x00000000;  /* 0.0f */
    half.u = 0x3F000000;  /* 0.5f */
    roll.u  = (sim_roll_mrad  < 0) ? (0x80000000 | 0x3C23D70A) : 0x3C23D70A;
    pitch.u = (sim_pitch_mrad < 0) ? (0x80000000 | 0x3C23D70A) : 0x3C23D70A;

    mavlink_msg_attitude_pack(
        1, 1, &msg,
        0,          /* time_boot_ms */
        roll.f,     /* roll */
        pitch.f,    /* pitch */
        zero.f,     /* yaw */
        zero.f, zero.f, zero.f
    );
    mav_send_message(&msg);
}

/* ── VFR_HUD ── */
void send_vfr_hud(void)
{
    mavlink_message_t msg;

    /* encode current simulated altitude as IEEE754 float without FPU */
    union { uint32_t u; float f; } alt_f;
    uint32_t alt_m = (uint32_t)(current_alt_mm / 1000);
    /* simple LUT: 0..100m range */
    if      (alt_m == 0)   alt_f.u = 0x00000000;
    else if (alt_m < 5)    alt_f.u = 0x40800000;  /* ~4.0  */
    else if (alt_m < 15)   alt_f.u = 0x41200000;  /* ~10.0 */
    else if (alt_m < 25)   alt_f.u = 0x41C00000;  /* ~24.0 */
    else if (alt_m < 50)   alt_f.u = 0x41F00000;  /* ~30.0 */
    else if (alt_m < 75)   alt_f.u = 0x42480000;  /* ~50.0 */
    else                   alt_f.u = 0x42C80000;  /* ~100.0*/

    mavlink_msg_vfr_hud_pack(
        1, 1, &msg,
        15.0f,                          /* airspeed */
        14.5f,                          /* groundspeed */
        current_heading,                /* heading */
        50,                             /* throttle */
        alt_f.f,                        /* alt */
        0.5f                            /* climb */
    );
    mav_send_message(&msg);
}

/* ── GPS_RAW_INT ── */
void send_gps_raw_int(void)
{
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack(
        1, 1, &msg,
        0,                         /* time_usec */
        3,                         /* fix_type: 3D fix */
        current_lat,               /* lat degE7 */
        current_lon,               /* lon degE7 */
        current_alt_mm,            /* alt mm */
        100,                       /* eph */
        150,                       /* epv */
        1000,                      /* vel cm/s */
        current_heading * 100,     /* cog 90.00 deg */
        8,                         /* satellites_visible */
        0,                         /* alt_ellipsoid */
        0, 0, 0, 0,                /* h_acc, v_acc, vel_acc, hdg_acc */
        current_heading            /* yaw */
    );
    mav_send_message(&msg);
}

/* ── GLOBAL_POSITION_INT ── */
void send_global_position_int(void)
{
    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(
        1, 1, &msg,
        0,                        /* time_boot_ms */
        current_lat,              /* lat degE7 */
        current_lon,              /* lon degE7 */
        current_alt_mm,           /* alt MSL mm */
        current_alt_mm,           /* relative alt mm */
        0, 0, 0,                  /* vx vy vz */
        current_heading * 100     /* hdg 90.00 deg */
    );
    mav_send_message(&msg);
}

/* ── HOME_POSITION ── */
void send_home_position(void)
{
    mavlink_message_t msg;
    mavlink_msg_home_position_pack(
        1, 1, &msg,
        home_lat,
        home_lon,
        home_alt,
        home_x, home_y, home_z,
        home_q,
        home_approach_x, home_approach_y, home_approach_z,
        0                   /* time_usec */
    );
    mav_send_message(&msg);
}

void send_gps_global_origin(void)
{
    mavlink_message_t msg;
    mavlink_msg_gps_global_origin_pack(
        1, 1, &msg,
        home_lat,
        home_lon,
        home_alt,
        0
    );
    mav_send_message(&msg);
}

/* ── COMMAND_ACK ── */
void send_command_ack(uint16_t command, uint8_t result)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(
        1, 1, &msg,
        command, result,
        0, 0,
        255, 190
    );
    mav_send_message(&msg);
}

static uint8_t param_id_equal(const char *a, const char *b)
{
    for (uint8_t i = 0; i < 16; i++) {
        if (a[i] != b[i])
            return 0;
        if (a[i] == '\0')
            return 1;
    }
    return 1;
}

int16_t find_param_index(const char *param_id)
{
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        if (param_id_equal(params[i].id, param_id))
            return (int16_t)i;
    }
    return -1;
}

void set_param_value(uint16_t index, float value)
{
    if (index < PARAM_COUNT)
        params[index].value = value;
}

/* ── PARAM_VALUE ── */
void send_param_value(uint16_t index)
{
    mavlink_message_t msg;

    if (index >= PARAM_COUNT)
        return;

    mavlink_msg_param_value_pack(
        1, 1, &msg,
        params[index].id,
        params[index].value,
        params[index].type,
        PARAM_COUNT,
        index
    );
    mav_send_message(&msg);
}

void send_all_params(void)
{
    for (uint16_t i = 0; i < PARAM_COUNT; i++)
        send_param_value(i);
}

/* ── MISSION_REQUEST_INT ── */
void send_mission_request_int(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_int_pack(
        1, 1, &msg,
        255, 190,      /* reply to GCS (sys 255, comp 190) */
        seq,
        MAV_MISSION_TYPE_MISSION
    );
    mav_send_message(&msg);
}

void send_mission_request_int_to(uint16_t seq, uint8_t target_sys, uint8_t target_comp)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_int_pack(
        1, 1, &msg,
        target_sys, target_comp,
        seq,
        MAV_MISSION_TYPE_MISSION
    );
    mav_send_message(&msg);
}

/* ── MISSION_REQUEST (non-INT, msgid 40) ── */
void send_mission_request(uint16_t seq)
{
    send_mission_request_to(seq, 255, 190);
}

void send_mission_request_to(uint16_t seq, uint8_t target_system, uint8_t target_component)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_pack(
        1, 1, &msg,
        target_system, target_component,
        seq,
        MAV_MISSION_TYPE_MISSION
    );
    mav_send_message(&msg);
}

/* ── MISSION_ACK ── */
void send_mission_ack(uint8_t type)
{
    mavlink_message_t msg;
    mavlink_msg_mission_ack_pack(
        1, 1, &msg,
        255, 190,
        type,
        MAV_MISSION_TYPE_MISSION,
        0            /* opaque_id */
    );
    mav_send_message(&msg);
}

void send_mission_ack_to(uint8_t type, uint8_t target_sys, uint8_t target_comp)
{
    mavlink_message_t msg;
    mavlink_msg_mission_ack_pack(
        1, 1, &msg,
        target_sys, target_comp,
        type,
        MAV_MISSION_TYPE_MISSION,
        0            /* opaque_id */
    );
    mav_send_message(&msg);
}

void send_mission_current(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_msg_mission_current_pack(
        1, 1, &msg,
        seq,
        mission_count,
        0,  /* mission_state */
        0,  /* mission_mode */
        0,  /* mission_id */
        0,  /* fence_id */
        0   /* rally_points_id */
    );
    mav_send_message(&msg);
}

void send_mission_item_reached(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_msg_mission_item_reached_pack(
        1, 1, &msg,
        seq
    );
    mav_send_message(&msg);
}

void send_mission_item_int_to(uint16_t seq, uint8_t target_sys, uint8_t target_comp)
{
    if (seq >= mission_count)
        return;

    mavlink_message_t msg;
    union { float f; uint32_t u; } alt;
    alt.u = mission[seq].alt_raw;

    mavlink_msg_mission_item_int_pack(
        1, 1, &msg,
        target_sys, target_comp,
        seq,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mission[seq].command,
        (seq == current_waypoint_idx) ? 1 : 0,
        1,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        mission[seq].lat,
        mission[seq].lon,
        alt.f,
        MAV_MISSION_TYPE_MISSION
    );
    mav_send_message(&msg);
}

void send_mission_item_to(uint16_t seq, uint8_t target_sys, uint8_t target_comp)
{
    if (seq >= mission_count)
        return;

    mavlink_message_t msg;
    union { float f; uint32_t u; } alt;
    alt.u = mission[seq].alt_raw;

    mavlink_msg_mission_item_pack(
        1, 1, &msg,
        target_sys, target_comp,
        seq,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mission[seq].command,
        (seq == current_waypoint_idx) ? 1 : 0,
        1,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        (float)mission[seq].lat / 10000000.0f,
        (float)mission[seq].lon / 10000000.0f,
        alt.f,
        MAV_MISSION_TYPE_MISSION
    );
    mav_send_message(&msg);
}
