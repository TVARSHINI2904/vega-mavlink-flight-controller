#ifndef MAVLINK_TX_H
#define MAVLINK_TX_H

#include <stdint.h>
#include "c_library_v2/common/mavlink.h"

/* armed state - readable by heartbeat and RX handler */
extern uint8_t armed_state;
extern uint32_t custom_mode;

/* core send helper */
void mav_send_message(mavlink_message_t *msg);

/* telemetry */
void send_heartbeat(void);
void send_sys_status(void);
void send_attitude(void);
void send_vfr_hud(void);
void send_battery_status(void);
void send_gps_raw_int(void);
void send_global_position_int(void);
void send_home_position(void);
void send_gps_global_origin(void);
void update_home_position(int32_t latitude, int32_t longitude, int32_t altitude,
                          float x, float y, float z,
                          const float q[4],
                          float approach_x, float approach_y, float approach_z);
void send_statustext(uint8_t severity, const char *text);
void set_flight_mode(uint32_t mode);

/* protocol responses */
#define PARAM_WP_RADIUS 5
#define PARAM_CMD_AUTH   6
#define PARAM_MISSION_ID 7
#define PARAM_MISSION_VER 8
#define PARAM_MISSION_CHALLENGE 9
#define PARAM_MISSION_CHAL_RESP 10

/* session-specific command auth token (regenerated on each boot) */
extern uint32_t current_cmd_auth_token;

void send_command_ack(uint16_t command, uint8_t result);
void send_param_value(uint16_t index);
void send_all_params(void);
int16_t find_param_index(const char *param_id);
void set_param_value(uint16_t index, float value);
uint8_t secure_command_allowed(void);
float get_param_value(uint16_t index);
void send_mission_request_int(uint16_t seq);
void send_mission_request_int_to(uint16_t seq, uint8_t target_sys, uint8_t target_comp);
void send_mission_request(uint16_t seq);
void send_mission_request_to(uint16_t seq, uint8_t target_system, uint8_t target_component);
void send_mission_ack(uint8_t type);
void send_mission_ack_to(uint8_t type, uint8_t target_sys, uint8_t target_comp);
void send_mission_current(uint16_t seq);
void send_mission_item_reached(uint16_t seq);
void send_mission_item_int_to(uint16_t seq, uint8_t target_sys, uint8_t target_comp);
void send_mission_item_to(uint16_t seq, uint8_t target_sys, uint8_t target_comp);
void send_nav_controller_output(void);

/* ── CMD_AUTH authentication timing ── */
extern uint32_t auth_start_time;
extern uint32_t auth_end_time;
extern uint32_t auth_latency;

#endif
