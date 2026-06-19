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

#endif
