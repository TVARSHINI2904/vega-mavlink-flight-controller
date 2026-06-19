#include "mission.h"
#include "mavlink_tx.h"

waypoint_t mission[MAX_WAYPOINTS];
uint16_t   mission_count = 0;
uint16_t   mission_rx_idx = 0;

int32_t  current_lat = 130827000;
int32_t  current_lon = 802707000;
int32_t  current_alt_mm = 10000;
uint16_t current_heading = 0;

uint16_t current_waypoint_idx = 0;
uint8_t mission_loaded = 0;
static uint8_t mission_active = 0;
static uint8_t rtl_active = 0;

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

static uint16_t compute_heading(int32_t dlat, int32_t dlon)
{
    if (dlat == 0 && dlon == 0)
        return current_heading;

    if (abs_i32(dlon) > abs_i32(dlat))
        return (dlon > 0) ? 90 : 270;
    return (dlat > 0) ? 0 : 180;
}

void mission_reset(void)
{
    current_waypoint_idx = 0;
    mission_active = 0;
    rtl_active = 0;
    mission_loaded = 0;
    current_heading = 0;
}

void mission_update(void)
{
    const int32_t position_step = 500;
    const int32_t altitude_step = 100;
    const int32_t position_threshold = 300;
    const int32_t altitude_threshold = 500;

    if (custom_mode == 6) {
        if (!rtl_active) {
            rtl_active = 1;
            mission_active = 0;
            send_statustext(MAV_SEVERITY_INFO, "RTL START");
        }

        int32_t dlat = home_lat - current_lat;
        int32_t dlon = home_lon - current_lon;
        int32_t dalt = home_alt - current_alt_mm;

        current_heading = compute_heading(dlat, dlon);

        if (abs_i32(dlat) <= position_threshold &&
            abs_i32(dlon) <= position_threshold &&
            abs_i32(dalt) <= altitude_threshold) {
            current_lat = home_lat;
            current_lon = home_lon;
            current_alt_mm = home_alt;
            rtl_active = 0;
            send_statustext(MAV_SEVERITY_INFO, "RTL COMPLETE");
            return;
        }

        if (abs_i32(dlat) > position_step) {
            current_lat += (dlat > 0) ? position_step : -position_step;
        } else {
            current_lat = home_lat;
        }

        if (abs_i32(dlon) > position_step) {
            current_lon += (dlon > 0) ? position_step : -position_step;
        } else {
            current_lon = home_lon;
        }

        if (abs_i32(dalt) > altitude_step) {
            current_alt_mm += (dalt > 0) ? altitude_step : -altitude_step;
        } else {
            current_alt_mm = home_alt;
        }
        return;
    }

    rtl_active = 0;

    if (mission_count == 0 || !mission_loaded) {
        mission_active = 0;
        return;
    }

    if (custom_mode != 3 && custom_mode != 4) {
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
    int32_t dlat = waypoint->lat - current_lat;
    int32_t dlon = waypoint->lon - current_lon;
    int32_t target_alt_mm = alt_bits_to_mm(waypoint->alt_raw);
    int32_t dalt = target_alt_mm - current_alt_mm;

    current_heading = compute_heading(dlat, dlon);

    if (abs_i32(dlat) <= position_threshold &&
        abs_i32(dlon) <= position_threshold &&
        abs_i32(dalt) <= altitude_threshold) {
        current_lat = waypoint->lat;
        current_lon = waypoint->lon;
        current_alt_mm = target_alt_mm;
        send_mission_item_reached(current_waypoint_idx);
        current_waypoint_idx++;
        send_statustext(MAV_SEVERITY_INFO, "WAYPOINT REACHED");
        if (current_waypoint_idx >= mission_count) {
            send_statustext(MAV_SEVERITY_INFO, "MISSION COMPLETE");
            mission_active = 0;
        } else {
            send_mission_current(current_waypoint_idx);
        }
        return;
    }

    if (abs_i32(dlat) > position_step) {
        current_lat += (dlat > 0) ? position_step : -position_step;
    } else {
        current_lat = waypoint->lat;
    }

    if (abs_i32(dlon) > position_step) {
        current_lon += (dlon > 0) ? position_step : -position_step;
    } else {
        current_lon = waypoint->lon;
    }

    if (abs_i32(dalt) > altitude_step) {
        current_alt_mm += (dalt > 0) ? altitude_step : -altitude_step;
    } else {
        current_alt_mm = target_alt_mm;
    }
}
