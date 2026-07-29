#include "mavlink_rx.h"
#include "mavlink_tx.h"
#include "mission.h"
#include "uart.h"
#include "scheduler.h"

/* ── mission upload timing ── */
uint32_t upload_start_time = 0;
uint32_t upload_end_time = 0;

static void print_auth_result(uint8_t allowed)
{
    uint32_t received = (uint32_t)get_param_value(PARAM_CMD_AUTH);

    uart_print("\n");
    uart_print("CMD_AUTH RECEIVED : ");
    uart_print_int(received);
    uart_print("\n\n");

    if (allowed) {
        uart_print("AUTHENTICATION SUCCESS\n\n");
    } else {
        uart_print("EXPECTED TOKEN : ");
        uart_print_int(current_cmd_auth_token);
        uart_print("\n\n");
        uart_print("AUTHENTICATION FAILED\n\n");
    }

    uart_print("Authentication Latency : ");
    uart_print_int(auth_latency);
    uart_print(" ms\n\n");
    uart_print("-------------------------------------\n\n");
}

static void debug_uart_message(const char *label, uint32_t value)
{
    uart_print(label);
    uart_print_int(value);
    uart_putchar('\n');
}

static mavlink_message_t rx_msg;
static mavlink_status_t  rx_status;

static uint8_t is_for_this_vehicle(uint8_t target_system, uint8_t target_component)
{
    return (target_system == 0 || target_system == 1) &&
           (target_component == 0 || target_component == 1);
}

static uint32_t command_param_to_mode(uint32_t bits)
{
    switch (bits) {
    case 0x00000000UL: return 0;  /* 0.0f */
    case 0x40400000UL: return 3;  /* 3.0f */
    case 0x40800000UL: return 4;  /* 4.0f */
    case 0x40A00000UL: return 5;  /* 5.0f */
    case 0x40C00000UL: return 6;  /* 6.0f */
    default: return custom_mode;
    }
}

static int32_t float_bits_to_scaled_i32(uint32_t bits, uint32_t scale)
{
    uint32_t sign = bits >> 31;
    uint32_t exp = (bits >> 23) & 0xFF;
    uint32_t mant = bits & 0x7FFFFFUL;
    uint64_t value;
    int32_t shift;

    if (exp == 0)
        return 0;
    if (exp == 255)
        return 0;

    mant |= 0x800000UL;
    value = (uint64_t)mant * (uint64_t)scale;
    shift = (int32_t)exp - 127 - 23;

    if (shift >= 0)
        value <<= shift;
    else
        value >>= -shift;

    if (sign)
        return -(int32_t)value;
    return (int32_t)value;
}

/* message handlers */
static void handle_command_long(const mavlink_message_t *msg)
{
    mavlink_command_long_t cmd;
    union { float f; uint32_t u; } param1;
    union { float f; uint32_t u; } param2;
    union { float f; uint32_t u; } param5;
    union { float f; uint32_t u; } param6;
    union { float f; uint32_t u; } param7;

    mavlink_msg_command_long_decode(msg, &cmd);
    if (!is_for_this_vehicle(cmd.target_system, cmd.target_component))
        return;

    param1.f = cmd.param1;
    param2.f = cmd.param2;
    param5.f = cmd.param5;
    param6.f = cmd.param6;
    param7.f = cmd.param7;

    if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
        if (param1.u == 0x3F800000UL) {
            uint8_t auth_ok = secure_command_allowed();
            print_auth_result(auth_ok);
            if (!auth_ok) {
                send_command_ack(cmd.command, MAV_RESULT_DENIED);
                send_statustext(MAV_SEVERITY_WARNING, "CMD AUTH FAIL");
                return;
            }
            armed_state = 1;
            send_statustext(MAV_SEVERITY_INFO, "ARMED");
        }
        else if (param1.u == 0x00000000UL) {
            armed_state = 0;
            send_statustext(MAV_SEVERITY_INFO, "DISARMED");
        }
        send_command_ack(cmd.command, MAV_RESULT_ACCEPTED);
    }
    else if (cmd.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) {
        send_command_ack(cmd.command, MAV_RESULT_ACCEPTED);
    }
    else if (cmd.command == MAV_CMD_DO_SET_MODE) {
        uint32_t requested_mode = command_param_to_mode(param2.u);
        if ((requested_mode == 3 || requested_mode == 4 || requested_mode == 6)) {
            uint8_t auth_ok = secure_command_allowed();
            print_auth_result(auth_ok);
            if (!auth_ok) {
                send_command_ack(cmd.command, MAV_RESULT_DENIED);
                send_statustext(MAV_SEVERITY_WARNING, "CMD AUTH FAIL");
                return;
            }
        }
        set_flight_mode(requested_mode);
        send_command_ack(cmd.command, MAV_RESULT_ACCEPTED);
    }
    else if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
        send_command_ack(cmd.command, MAV_RESULT_ACCEPTED);
        if (param1.u == 0x43720000UL)  /* 242.0f: HOME_POSITION */
            send_home_position();
    }
    else if (cmd.command == MAV_CMD_DO_SET_HOME) {
        uint8_t auth_ok = secure_command_allowed();
        print_auth_result(auth_ok);
        if (!auth_ok) {
            send_command_ack(cmd.command, MAV_RESULT_DENIED);
            send_statustext(MAV_SEVERITY_WARNING, "CMD AUTH FAIL");
            return;
        }
        float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        if (param1.u == 0x3F800000UL) {
            update_home_position(130827000, 802707000, 10000,
                                 0.0f, 0.0f, 0.0f,
                                 q, 0.0f, 0.0f, 0.0f);
        } else {
            update_home_position(float_bits_to_scaled_i32(param5.u, 10000000UL),
                                 float_bits_to_scaled_i32(param6.u, 10000000UL),
                                 float_bits_to_scaled_i32(param7.u, 1000UL),
                                 0.0f, 0.0f, 0.0f,
                                 q, 0.0f, 0.0f, 0.0f);
        }
        send_command_ack(cmd.command, MAV_RESULT_ACCEPTED);
    }
    else if (cmd.command == MAV_CMD_GET_HOME_POSITION) {
        send_command_ack(cmd.command, MAV_RESULT_ACCEPTED);
        send_home_position();
    }
    else {
        send_command_ack(cmd.command, MAV_RESULT_UNSUPPORTED);
    }
}

static void handle_set_mode(const mavlink_message_t *msg)
{
    mavlink_set_mode_t mode;

    mavlink_msg_set_mode_decode(msg, &mode);
    if (mode.target_system == 0 || mode.target_system == 1)
        set_flight_mode(mode.custom_mode);
}

static void handle_set_home_position(const mavlink_message_t *msg)
{
    mavlink_set_home_position_t home;
    float q[4];

    mavlink_msg_set_home_position_decode(msg, &home);
    if (!is_for_this_vehicle(home.target_system, 0))
        return;

    q[0] = home.q[0];
    q[1] = home.q[1];
    q[2] = home.q[2];
    q[3] = home.q[3];

    update_home_position(home.latitude, home.longitude, home.altitude,
                         home.x, home.y, home.z,
                         q,
                         home.approach_x, home.approach_y, home.approach_z);
}

static void handle_param_request_list(const mavlink_message_t *msg)
{
    mavlink_param_request_list_t req;

    mavlink_msg_param_request_list_decode(msg, &req);
    if (is_for_this_vehicle(req.target_system, req.target_component))
        send_all_params();
}

static void handle_param_request_read(const mavlink_message_t *msg)
{
    mavlink_param_request_read_t req;
    int16_t index;

    mavlink_msg_param_request_read_decode(msg, &req);
    if (!is_for_this_vehicle(req.target_system, req.target_component))
        return;

    index = req.param_index;
    if (index < 0)
        index = find_param_index(req.param_id);

    if (index >= 0)
        send_param_value((uint16_t)index);
}

static void handle_param_set(const mavlink_message_t *msg)
{
    mavlink_param_set_t req;
    int16_t index;

    mavlink_msg_param_set_decode(msg, &req);
    if (!is_for_this_vehicle(req.target_system, req.target_component))
        return;

    index = find_param_index(req.param_id);
    if (index >= 0) {
        set_param_value((uint16_t)index, req.param_value);
        send_param_value((uint16_t)index);

        if (index == PARAM_MISSION_CHAL_RESP) {
            uint32_t raw_val = (uint32_t)req.param_value;
            debug_uart_message("DBG CHAL_RESP=", raw_val);
            if (mission_submit_upload_response(req.param_value)) {
                debug_uart_message("DBG AUTH FLG=", mission_upload_authorized());
                send_statustext(MAV_SEVERITY_INFO, "MISSION chal response OK");
                send_mission_request_int_to(mission_rx_idx, msg->sysid, msg->compid);
            } else {
                debug_uart_message("DBG AUTH FLG=", mission_upload_authorized());
                send_statustext(MAV_SEVERITY_WARNING, "MISSION chal response BAD");
            }
        }
    }
}

static void handle_mission_count(const mavlink_message_t *msg)
{
    mavlink_mission_count_t count;

    mavlink_msg_mission_count_decode(msg, &count);

    uint8_t mc_auth = secure_command_allowed();
    print_auth_result(mc_auth);
    if (!mc_auth) {
        send_mission_ack_to(MAV_MISSION_DENIED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_WARNING, "MISSION AUTH FAIL");
        return;
    }

    if (count.count > MAX_WAYPOINTS) {
        send_mission_ack_to(MAV_MISSION_DENIED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_WARNING, "MISSION COUNT TOO LARGE");
        return;
    }

    if (!mission_begin_upload(count.count)) {
        send_mission_ack_to(MAV_MISSION_DENIED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_WARNING, "MISSION METADATA MISSING");
        return;
    }

    /* start mission upload timer */
    upload_start_time = sys_tick;

    debug_uart_message("DBG MCOUNT=", count.count);
    debug_uart_message("DBG AUTH FLG=", mission_upload_authorized());
    send_statustext(MAV_SEVERITY_INFO, "MISSION_COUNT RXD");

    if (mission_count > 0) {
        /* require challenge response before accepting mission items */
        send_statustext(MAV_SEVERITY_INFO, "MISSION UPLOAD CHALLENGE SENT");
        send_mission_request_int_to(0, msg->sysid, msg->compid);
    } else {
        send_mission_ack_to(MAV_MISSION_ACCEPTED, msg->sysid, msg->compid);
    }
}

static void handle_mission_item_int(const mavlink_message_t *msg)
{
    mavlink_mission_item_int_t item;
    union { float f; uint32_t u; } alt;

    uint8_t mii_auth = secure_command_allowed();
    print_auth_result(mii_auth);
    if (!mii_auth) {
        send_mission_ack_to(MAV_MISSION_DENIED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_WARNING, "MISSION AUTH FAIL");
        return;
    }

    if (!mission_upload_authorized()) {
        debug_uart_message("DBG AUTH FLG=", mission_upload_authorized());
        send_mission_ack_to(MAV_MISSION_DENIED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_WARNING, "MISSION UPLOAD UNAUTHORIZED");
        return;
    }

    mavlink_msg_mission_item_int_decode(msg, &item);
    alt.f = item.z;

    if (item.seq < MAX_WAYPOINTS) {
        mission[item.seq].lat     = item.x;
        mission[item.seq].lon     = item.y;
        mission[item.seq].alt_raw = alt.u;
        mission[item.seq].command = item.command;
    }

    mission_rx_idx++;
    if (mission_rx_idx >= mission_count)
        mission_loaded = 1;

    if (mission_rx_idx < mission_count)
        send_mission_request_int_to(mission_rx_idx, msg->sysid, msg->compid);
    else {
        send_statustext(MAV_SEVERITY_INFO, "MISSION VERIFIED");
        send_mission_ack_to(MAV_MISSION_ACCEPTED, msg->sysid, msg->compid);
        save_mission_to_nvm();   /* persist uploaded mission */
        /* stop timer and print upload time */
        upload_end_time = sys_tick;
        uart_print("MISSION UPLOAD COMPLETE\n");
        uart_print("Mission Upload Time : ");
        uart_print_int(upload_end_time - upload_start_time);
        uart_print(" ms\n");
    }
}

/* ── MISSION_ITEM (float lat/lon version) ── */
static void handle_mission_item(const mavlink_message_t *msg)
{
    mavlink_mission_item_t item;
    union { float f; uint32_t u; } alt;

    uint8_t mi_auth = secure_command_allowed();
    print_auth_result(mi_auth);
    if (!mi_auth) {
        send_mission_ack_to(MAV_MISSION_DENIED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_WARNING, "MISSION AUTH FAIL");
        return;
    }

    if (!mission_upload_authorized()) {
        debug_uart_message("DBG AUTH FLG=", mission_upload_authorized());
        send_mission_ack_to(MAV_MISSION_DENIED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_WARNING, "MISSION UPLOAD UNAUTHORIZED");
        return;
    }

    mavlink_msg_mission_item_decode(msg, &item);
    alt.f = item.z;

    if (item.seq < MAX_WAYPOINTS) {
        /* MISSION_ITEM uses float degrees, convert to degE7 (int32_t) */
        mission[item.seq].lat     = (int32_t)(item.x * 10000000.0f);
        mission[item.seq].lon     = (int32_t)(item.y * 10000000.0f);
        mission[item.seq].alt_raw = alt.u;
        mission[item.seq].command = item.command;
    }

    mission_rx_idx++;
    if (mission_rx_idx >= mission_count)
        mission_loaded = 1;

    if (mission_rx_idx < mission_count)
        send_mission_request_int_to(mission_rx_idx, msg->sysid, msg->compid);
    else {
        send_statustext(MAV_SEVERITY_INFO, "MISSION VERIFIED");
        send_mission_ack_to(MAV_MISSION_ACCEPTED, msg->sysid, msg->compid);
        save_mission_to_nvm();   /* persist uploaded mission */
        /* stop timer and print upload time */
        upload_end_time = sys_tick;
        uart_print("MISSION UPLOAD COMPLETE\n");
        uart_print("Mission Upload Time : ");
        uart_print_int(upload_end_time - upload_start_time);
        uart_print(" ms\n");
    }
}

static void handle_set_position_target_global_int(const mavlink_message_t *msg)
{
    mavlink_set_position_target_global_int_t target;

    mavlink_msg_set_position_target_global_int_decode(msg, &target);
    if (!is_for_this_vehicle(target.target_system, target.target_component))
        return;

    /* only accept position targets in GUIDED mode */
    if (custom_mode != 4)
        return;

    /* ignore if position bits (0-3) are masked out */
    uint16_t mask = target.type_mask;
    if (!(mask & 0x01) && !(mask & 0x02) && !(mask & 0x04)) {
        guided_target_lat = target.lat_int;
        guided_target_lon = target.lon_int;
        guided_target_alt = (int32_t)(target.alt * 1000.0f); /* m → mm */
        guided_target_set = 1;
        send_statustext(MAV_SEVERITY_INFO, "GUIDED TARGET SET");
    }
}

/* ── MISSION_CLEAR_ALL ── */
static void handle_mission_clear_all(const mavlink_message_t *msg)
{
    mavlink_mission_clear_all_t clear;
    mavlink_msg_mission_clear_all_decode(msg, &clear);

    uint8_t clr_auth = secure_command_allowed();
    print_auth_result(clr_auth);
    if (!clr_auth) {
        send_mission_ack_to(MAV_MISSION_DENIED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_WARNING, "MISSION AUTH FAIL");
        return;
    }

    if (clear.target_system == 1 || clear.target_system == 0) {
        mission_reset();
        mission_count = 0;
        mission_loaded = 0;
        guided_target_set = 0;
        send_mission_ack_to(MAV_MISSION_ACCEPTED, msg->sysid, msg->compid);
        send_statustext(MAV_SEVERITY_INFO, "MISSION CLEARED");
    }
}

/* ── MISSION_REQUEST_LIST ── */
static void handle_mission_request_list(const mavlink_message_t *msg)
{
    mavlink_mission_request_list_t req;
    mavlink_msg_mission_request_list_decode(msg, &req);

    if (req.target_system == 1 || req.target_system == 0) {
        mavlink_message_t out;
        mavlink_msg_mission_count_pack(
            1, 1, &out,
            msg->sysid, msg->compid,
            mission_count,
            MAV_MISSION_TYPE_MISSION,
            0                    /* opaque_id */
        );
        mav_send_message(&out);
    }
}

static void handle_mission_request_int(const mavlink_message_t *msg)
{
    mavlink_mission_request_int_t req;

    mavlink_msg_mission_request_int_decode(msg, &req);
    if (req.target_system == 1 || req.target_system == 0)
        send_mission_item_int_to(req.seq, msg->sysid, msg->compid);
}

static void handle_mission_request(const mavlink_message_t *msg)
{
    mavlink_mission_request_t req;

    mavlink_msg_mission_request_decode(msg, &req);
    if (req.target_system == 1 || req.target_system == 0)
        send_mission_item_to(req.seq, msg->sysid, msg->compid);
}

/* poll UART and parse MAVLink v1/v2 */
void mavlink_rx_poll(void)
{
    while (UART_RX_READY) {
        uint8_t b = UART_RHR;

        if (mavlink_parse_char(MAVLINK_COMM_0, b, &rx_msg, &rx_status)) {
            switch (rx_msg.msgid) {
            case MAVLINK_MSG_ID_COMMAND_LONG:
                handle_command_long(&rx_msg);
                break;

            case MAVLINK_MSG_ID_SET_MODE:
                handle_set_mode(&rx_msg);
                break;

            case MAVLINK_MSG_ID_SET_HOME_POSITION:
                handle_set_home_position(&rx_msg);
                break;

            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                handle_param_request_list(&rx_msg);
                break;

            case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                handle_param_request_read(&rx_msg);
                break;

            case MAVLINK_MSG_ID_PARAM_SET:
                handle_param_set(&rx_msg);
                break;

            case MAVLINK_MSG_ID_MISSION_COUNT:
                handle_mission_count(&rx_msg);
                break;

            case MAVLINK_MSG_ID_MISSION_ITEM_INT:
                handle_mission_item_int(&rx_msg);
                break;

            case MAVLINK_MSG_ID_MISSION_ACK:
                /* echo ACK back to sender */
                break;

            case MAVLINK_MSG_ID_MISSION_ITEM:
                handle_mission_item(&rx_msg);
                break;

            case MAVLINK_MSG_ID_MISSION_REQUEST:
                handle_mission_request(&rx_msg);
                break;

            case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
                handle_mission_request_int(&rx_msg);
                break;

            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                handle_mission_request_list(&rx_msg);
                break;

            case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
                handle_set_position_target_global_int(&rx_msg);
                break;

            case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                handle_mission_clear_all(&rx_msg);
                break;

            default:
                break;
            }
        }
    }
}