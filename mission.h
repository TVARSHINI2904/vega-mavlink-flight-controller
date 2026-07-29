#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>

#define MAX_WAYPOINTS 20
#define MISSION_HASH_SIZE 32   /* SHA-256 hash size in bytes */

/* ── waypoint ── */
typedef struct {
    int32_t  lat;
    int32_t  lon;
    uint32_t alt_raw;
    uint16_t command;
} waypoint_t;

extern waypoint_t mission[MAX_WAYPOINTS];
extern uint16_t   mission_count;
extern uint16_t   mission_rx_idx;
extern uint8_t    mission_loaded;
extern uint16_t   current_waypoint_idx;
extern uint32_t   current_mission_id;
extern uint32_t   current_mission_ver;
extern uint32_t   current_mission_challenge;

/* ── position ── */
extern int32_t  current_lat;
extern int32_t  current_lon;
extern int32_t  current_alt_mm;
extern uint16_t current_heading;

/* ── home ── */
extern int32_t  home_lat;
extern int32_t  home_lon;
extern int32_t  home_alt;

/* ── battery state ── */
extern uint16_t battery_voltage_mv;   /* mV */
extern int16_t  battery_current_ma;   /* mA  */
extern uint8_t  battery_remaining_pct; /* 0..100 */

/* ── GPS / sensor state ── */
extern uint8_t  gps_fix_type;         /* 0=no GPS, 3=3D fix */
extern uint8_t  gps_satellites;       /* visible count */

/* ── failsafe state ── */
extern uint8_t  failsafe_battery;     /* 1 = low-battery RTL triggered */
extern uint8_t  failsafe_gps;         /* 1 = GPS-loss LOITER triggered */

/* ── mission persistence ── */
#define NVM_SIGNATURE 0xA5A5          /* magic to detect valid NVM data */

typedef struct {
    uint16_t signature;
    uint16_t count;
    waypoint_t waypoints[MAX_WAYPOINTS];
    uint8_t  hash[MISSION_HASH_SIZE];  /* SHA-256 hash of waypoint data for integrity check */
    uint32_t mission_id;
    uint32_t mission_ver;
    uint32_t mission_challenge;
} nvm_mission_t;

/* ── externally settable parameters ── */
extern uint32_t wp_radius_m;            /* waypoint acceptance radius in meters */
extern uint8_t  mission_hash_valid;     /* 1 = mission integrity verified */

/* ── GUIDED mode target (set by RX handler) ── */
extern int32_t  guided_target_lat;
extern int32_t  guided_target_lon;
extern int32_t  guided_target_alt;
extern uint8_t  guided_target_set;

/* ── functions ── */
void mission_update(void);
void mission_reset(void);
uint8_t mission_begin_upload(uint16_t count);
uint8_t mission_submit_upload_response(float response);
uint8_t mission_upload_authorized(void);
void sim_battery_update(void);
void failsafe_check(void);
void save_mission_to_nvm(void);
uint8_t load_mission_from_nvm(void);
uint16_t approx_heading_centideg(int32_t dlat, int32_t dlon);

#endif