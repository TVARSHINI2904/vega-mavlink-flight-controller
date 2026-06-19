#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>

#define MAX_WAYPOINTS 20

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

extern int32_t  current_lat;
extern int32_t  current_lon;
extern int32_t  current_alt_mm;
extern uint16_t current_heading;

extern int32_t  home_lat;
extern int32_t  home_lon;
extern int32_t  home_alt;

void mission_update(void);
void mission_reset(void);

#endif
