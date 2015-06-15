// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Common GCS MAVLink functions for all vehicle types

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <GCS_handle.h>
#include <AP_AHRS.h>

extern const AP_HAL::HAL& hal;

uint32_t GCS_handle::last_radio_status_remrssi_ms;
uint8_t GCS_handle::mavlink_active = 0;
// mask of serial ports disabled to allow for SERIAL_CONTROL
static uint8_t mavlink_locked_mask;

/*
const AP_Param::GroupInfo GCS_handle::var_info[] PROGMEM = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_handle, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_handle, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_handle, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_handle, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_handle, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_handle, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_handle, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_handle, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_handle, streamRates[8],  0),
    AP_GROUPEND
};
*/

GCS_handle::GCS_handle() : waypoint_receive_timeout(5000)
{
}

 void GCS_handle::set_snoop(void (*_msg_snoop)(const mavlink_message_t* msg) ) {
    msg_snoop = _msg_snoop;
}

void
GCS_handle::init(AP_HAL::UARTDriver *port, mavlink_channel_t mav_chan)
{
    _port = port;
    chan = mav_chan;

    switch (chan) {
        case MAVLINK_COMM_0:
            mavlink_comm_0_port = _port;
            initialised = true;
            break;
        case MAVLINK_COMM_1:
            mavlink_comm_1_port = _port;
            initialised = true;
            break;
        case MAVLINK_COMM_2:
#if MAVLINK_COMM_NUM_BUFFERS > 2
            mavlink_comm_2_port = _port;
            initialised = true;
            break;
#endif
        default:
            // do nothing for unsupport mavlink channels
            break;
    }
    _queued_parameter = NULL;
    reset_cli_timeout();
}

/*
  lock a channel, preventing use by MAVLink
 */
void GCS_handle::lock_channel(mavlink_channel_t _chan, bool lock)
{
    if (_chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return;
    }
    if (lock) {
        mavlink_locked_mask |= (1U<<(unsigned)_chan);
    } else {
        mavlink_locked_mask &= ~(1U<<(unsigned)_chan);
    }
}

/*
  setup a UART, handling begin() and init()
 */
void GCS_handle::setup_uart(const AP_SerialManager& serial_manager, AP_SerialManager::SerialProtocol protocol, uint8_t instance)
{
    // search for serial port

    AP_HAL::UARTDriver *uart;
    uart = serial_manager.find_serial(protocol, instance);
    if (uart == NULL) {
        // return immediately if not found
        return;
    }

    // get associated mavlink channel
    mavlink_channel_t mav_chan;
    if (!serial_manager.get_mavlink_channel(protocol, instance, mav_chan)) {
        // return immediately in unlikely case mavlink channel cannot be found
        return;
    }

    /*
      Now try to cope with SiK radios that may be stuck in bootloader
      mode because CTS was held while powering on. This tells the
      bootloader to wait for a firmware. It affects any SiK radio with
      CTS connected that is externally powered. To cope we send 0x30
      0x20 at 115200 on startup, which tells the bootloader to reset
      and boot normally
     */
    uart->begin(115200);
    AP_HAL::UARTDriver::flow_control old_flow_control = uart->get_flow_control();
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    for (uint8_t i=0; i<3; i++) {
        hal.scheduler->delay(1);
        uart->write(0x30);
        uart->write(0x20);
    }
    uart->set_flow_control(old_flow_control);

    // now change back to desired baudrate
    uart->begin(serial_manager.find_baudrate(protocol, instance));

    // and init the gcs instance
    init(uart, mav_chan);
}

uint16_t
GCS_handle::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Param  *vp;
        AP_Param::ParamToken token;

        vp = AP_Param::first(&token, NULL);
        do {
            _parameter_count++;
        } while (NULL != (vp = AP_Param::next_scalar(&token, NULL)));
    }
    return _parameter_count;
}

/**
 * @brief Send the next pending parameter, called from deferred message
 * handling code
 */
void
GCS_handle::queued_param_send()
{
    if (!initialised || _queued_parameter == NULL) {
        return;
    }

    uint16_t bytes_allowed;
    uint8_t count;
    uint32_t tnow = hal.scheduler->millis();

    // use at most 30% of bandwidth on parameters. The constant 26 is
    // 1/(1000 * 1/8 * 0.001 * 0.3)
    bytes_allowed = 57 * (tnow - _queued_parameter_send_time_ms) * 26;
    if (bytes_allowed > comm_get_txspace(chan)) {
        bytes_allowed = comm_get_txspace(chan);
    }
    count = bytes_allowed / (MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);

    // when we don't have flow control we really need to keep the
    // param download very slow, or it tends to stall
    if (!have_flow_control() && count > 5) {
        count = 5;
    }

    while (_queued_parameter != NULL && count--) {
        AP_Param      *vp;
        float value;

        // copy the current parameter and prepare to move to the next
        vp = _queued_parameter;

        // if the parameter can be cast to float, report it here and break out of the loop
        value = vp->cast_to_float(_queued_parameter_type);

        char param_name[AP_MAX_NAME_SIZE];
        vp->copy_name_token(_queued_parameter_token, param_name, sizeof(param_name), true);

        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(_queued_parameter_type),
            _queued_parameter_count,
            _queued_parameter_index);

        _queued_parameter = AP_Param::next_scalar(&_queued_parameter_token, &_queued_parameter_type);
        _queued_parameter_index++;
    }
    _queued_parameter_send_time_ms = tnow;
}

/**
 * @brief Send the next pending waypoint, called from deferred message
 * handling code
 */
void
GCS_handle::queued_waypoint_send()
{
    if (initialised &&
        waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last) {
        mavlink_msg_mission_request_send(
            chan,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i);
    }
}

void GCS_handle::reset_cli_timeout() {
      _cli_timeout = hal.scheduler->millis();
}

/*
  handle a MISSION_REQUEST_LIST mavlink packet
 */
void GCS_handle::handle_mission_request_list(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_request_list_t packet;
    mavlink_msg_mission_request_list_decode(msg, &packet);

    // reply with number of commands in the mission.  The GCS will then request each command separately
    mavlink_msg_mission_count_send(chan,msg->sysid, msg->compid, mission.num_commands());

    // set variables to help handle the expected sending of commands to the GCS
    waypoint_receiving = false;             // record that we are sending commands (i.e. not receiving)
    waypoint_dest_sysid = msg->sysid;       // record system id of GCS who has requested the commands
    waypoint_dest_compid = msg->compid;     // record component id of GCS who has requested the commands
}

/*
  handle a MISSION_REQUEST mavlink packet
 */
void GCS_handle::handle_mission_request(AP_Mission &mission, mavlink_message_t *msg)
{
    AP_Mission::Mission_Command cmd;
    // decode
    mavlink_mission_request_t packet;
    mavlink_msg_mission_request_decode(msg, &packet);

    // retrieve mission from eeprom
    if (!mission.read_cmd_from_storage(packet.seq, cmd)) {
        goto mission_item_send_failed;
    }

    // convert mission command to mavlink mission item packet
    mavlink_mission_item_t ret_packet;
    memset(&ret_packet, 0, sizeof(ret_packet));
    if (!AP_Mission::mission_cmd_to_mavlink(cmd, ret_packet)) {
        goto mission_item_send_failed;
    }

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
        ret_packet.current = 1;
    } else {
        ret_packet.current = 0;
    }

    // set auto continue to 1
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

    /*
      avoid the _send() function to save memory on APM2, as it avoids
      the stack usage of the _send() function by using the already
      declared ret_packet above
     */
    ret_packet.target_system = msg->sysid;
    ret_packet.target_component = msg->compid;
    ret_packet.seq = packet.seq;
    ret_packet.command = cmd.id;

    _mav_finalize_message_chan_send(chan, 
                                    MAVLINK_MSG_ID_MISSION_ITEM,
                                    (const char *)&ret_packet,
                                    MAVLINK_MSG_ID_MISSION_ITEM_LEN,
                                    MAVLINK_MSG_ID_MISSION_ITEM_CRC);
    return;

mission_item_send_failed:
    // send failure message
    mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ERROR);
}

/*
  handle a MISSION_SET_CURRENT mavlink packet
 */
void GCS_handle::handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_set_current_t packet;
    mavlink_msg_mission_set_current_decode(msg, &packet);

    // set current command
    if (mission.set_current_cmd(packet.seq)) {
        mavlink_msg_mission_current_send(chan, mission.get_current_nav_cmd().index);
    }
}

/*
  handle a MISSION_COUNT mavlink packet
 */
void GCS_handle::handle_mission_count(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_count_t packet;
    mavlink_msg_mission_count_decode(msg, &packet);

    // start waypoint receiving
    if (packet.count > mission.num_commands_max()) {
        // send NAK
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_NO_SPACE);
        return;
    }

    // new mission arriving, truncate mission to be the same length
    mission.truncate(packet.count);

    // set variables to help handle the expected receiving of commands from the GCS
    waypoint_timelast_receive = hal.scheduler->millis();    // set time we last received commands to now
    waypoint_receiving = true;              // record that we expect to receive commands
    waypoint_request_i = 0;                 // reset the next expected command number to zero
    waypoint_request_last = packet.count;   // record how many commands we expect to receive
    waypoint_timelast_request = 0;          // set time we last requested commands to zero
}

/*
  handle a MISSION_CLEAR_ALL mavlink packet
 */
void GCS_handle::handle_mission_clear_all(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_clear_all_t packet;
    mavlink_msg_mission_clear_all_decode(msg, &packet);

    // clear all waypoints
    if (mission.clear()) {
        // send ack
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_RESULT_ACCEPTED);
    }else{
        // send nack
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, 1);
    }
}

/*
  handle a MISSION_WRITE_PARTIAL_LIST mavlink packet
 */
void GCS_handle::handle_mission_write_partial_list(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_write_partial_list_t packet;
    mavlink_msg_mission_write_partial_list_decode(msg, &packet);

    // start waypoint receiving
    if ((unsigned)packet.start_index > mission.num_commands() ||
        (unsigned)packet.end_index > mission.num_commands() ||
        packet.end_index < packet.start_index) {
        send_text_P(SEVERITY_LOW,PSTR("flight plan update rejected"));
        return;
    }

    waypoint_timelast_receive = hal.scheduler->millis();
    waypoint_timelast_request = 0;
    waypoint_receiving   = true;
    waypoint_request_i   = packet.start_index;
    waypoint_request_last= packet.end_index;
}


/*
  handle a GIMBAL_REPORT mavlink packet
 */
void GCS_handle::handle_gimbal_report(AP_Mount &mount, mavlink_message_t *msg) const
{
    mount.handle_gimbal_report(chan, msg);
}


/*
  return true if a channel has flow control
 */
bool GCS_handle::have_flow_control(void)
{
    switch (chan) {
    case MAVLINK_COMM_0:
        if (mavlink_comm_0_port == NULL) {
            return false;
        } else {
            // assume USB has flow control
            return hal.gpio->usb_connected() || mavlink_comm_0_port->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
        }
        break;

    case MAVLINK_COMM_1:
        if (mavlink_comm_1_port == NULL) {
            return false;
        } else {
            return mavlink_comm_1_port->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
        }
        break;

    case MAVLINK_COMM_2:
#if MAVLINK_COMM_NUM_BUFFERS > 2
        if (mavlink_comm_2_port == NULL) {
            return false;
        } else {
            return mavlink_comm_2_port != NULL && mavlink_comm_2_port->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
        }
        break;
#endif

    default:
        break;
    }
    return false;
}


/*
  handle a request to change stream rate. Note that copter passes in
  save==false, as sending mavlink messages on copter on APM2 costs
  enough that it can cause flight issues, so we don't want the save to
  happen when the user connects the ground station.
 */
void GCS_handle::handle_request_data_stream(mavlink_message_t *msg, bool save)
{
    mavlink_request_data_stream_t packet;
    mavlink_msg_request_data_stream_decode(msg, &packet);

    int16_t freq = 0;     // packet frequency

    if (packet.start_stop == 0)
        freq = 0;                     // stop sending
    else if (packet.start_stop == 1)
        freq = packet.req_message_rate;                     // start sending
    else
        return;

    AP_Int16 *rate = NULL;
    switch (packet.req_stream_id) {
    case MAV_DATA_STREAM_ALL:
        // note that we don't set STREAM_PARAMS - that is internal only
        for (uint8_t i=0; i<STREAM_PARAMS; i++) {
            if (save) {
                streamRates[i].set_and_save_ifchanged(freq);
            } else {
                streamRates[i].set(freq);
            }
        }
        break;
    case MAV_DATA_STREAM_RAW_SENSORS:
        rate = &streamRates[STREAM_RAW_SENSORS];
        break;
    case MAV_DATA_STREAM_EXTENDED_STATUS:
        rate = &streamRates[STREAM_EXTENDED_STATUS];
        break;
    case MAV_DATA_STREAM_RC_CHANNELS:
        rate = &streamRates[STREAM_RC_CHANNELS];
        break;
    case MAV_DATA_STREAM_RAW_CONTROLLER:
        rate = &streamRates[STREAM_RAW_CONTROLLER];
        break;
    case MAV_DATA_STREAM_POSITION:
        rate = &streamRates[STREAM_POSITION];
        break;
    case MAV_DATA_STREAM_EXTRA1:
        rate = &streamRates[STREAM_EXTRA1];
        break;
    case MAV_DATA_STREAM_EXTRA2:
        rate = &streamRates[STREAM_EXTRA2];
        break;
    case MAV_DATA_STREAM_EXTRA3:
        rate = &streamRates[STREAM_EXTRA3];
        break;
    }

    if (rate != NULL) {
        if (save) {
            rate->set_and_save_ifchanged(freq);
        } else {
            rate->set(freq);
        }
    }
}

void GCS_handle::handle_param_request_list(mavlink_message_t *msg)
{
    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(msg, &packet);

#if CONFIG_HAL_BOARD != HAL_BOARD_APM1 && CONFIG_HAL_BOARD != HAL_BOARD_APM2
    // send system ID if we can
    char sysid[40];
    if (hal.util->get_system_id(sysid)) {
        send_text(SEVERITY_LOW, sysid);
    }
#endif

    // Start sending parameters - next call to ::update will kick the first one out
    _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index = 0;
    _queued_parameter_count = _count_parameters();
}

void GCS_handle::handle_param_request_read(mavlink_message_t *msg)
{
    mavlink_param_request_read_t packet;
    mavlink_msg_param_request_read_decode(msg, &packet);

    enum ap_var_type p_type;
    AP_Param *vp;
    char param_name[AP_MAX_NAME_SIZE+1];
    if (packet.param_index != -1) {
        AP_Param::ParamToken token;
        vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
        if (vp == NULL) {
            return;
        }
        vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
        param_name[AP_MAX_NAME_SIZE] = 0;
    } else {
        strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
        param_name[AP_MAX_NAME_SIZE] = 0;
        vp = AP_Param::find(param_name, &p_type);
        if (vp == NULL) {
            return;
        }
    }
    
    float value = vp->cast_to_float(p_type);
    mavlink_msg_param_value_send_buf(
        msg,
        chan,
        param_name,
        value,
        mav_var_type(p_type),
        _count_parameters(),
        packet.param_index);
}

void GCS_handle::handle_param_set(mavlink_message_t *msg, DataFlash_Class *DataFlash)
{
    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(msg, &packet);
    enum ap_var_type var_type;

    // set parameter
    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    vp = AP_Param::set_param_by_name(key, packet.param_value, &var_type);
    if (vp == NULL) {
        return;
    }

    // save the change
    vp->save();

    // Report back the new value if we accepted the change
    // we send the value we actually set, which could be
    // different from the value sent, in case someone sent
    // a fractional value to an integer type
    mavlink_msg_param_value_send_buf(
        msg,
        chan,
        key,
        vp->cast_to_float(var_type),
        mav_var_type(var_type),
        _count_parameters(),
        -1);     // XXX we don't actually know what its index is...

    if (DataFlash != NULL) {
        DataFlash->Log_Write_Parameter(key, vp->cast_to_float(var_type));
    }
}


void
GCS_handle::send_text(gcs_severity severity, const char *str)
{
    if (severity != SEVERITY_LOW && 
        comm_get_txspace(chan) >= 
        MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_STATUSTEXT_LEN) {
        // send immediately
        mavlink_msg_statustext_send(chan, severity, str);
    } else {
        // send via the deferred queuing system
        mavlink_statustext_t *s = &pending_status;
        s->severity = (uint8_t)severity;
        strncpy((char *)s->text, str, sizeof(s->text));
        send_message(MSG_STATUSTEXT);
    }
}

void
GCS_handle::send_text_P(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    memset(m.text, 0, sizeof(m.text));
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
        if (m.text[i] == '\0') {
            break;
        }
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    send_text(severity, (const char *)m.text);
}


void GCS_handle::handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio)
{
    mavlink_radio_t packet;
    mavlink_msg_radio_decode(msg, &packet);

    // record if the GCS has been receiving radio messages from
    // the aircraft
    if (packet.remrssi != 0) {
        last_radio_status_remrssi_ms = hal.scheduler->millis();
    }

    // use the state of the transmit buffer in the radio to
    // control the stream rate, giving us adaptive software
    // flow control
    if (packet.txbuf < 20 && stream_slowdown < 100) {
        // we are very low on space - slow down a lot
        stream_slowdown += 3;
    } else if (packet.txbuf < 50 && stream_slowdown < 100) {
        // we are a bit low on space, slow down slightly
        stream_slowdown += 1;
    } else if (packet.txbuf > 95 && stream_slowdown > 10) {
        // the buffer has plenty of space, speed up a lot
        stream_slowdown -= 2;
    } else if (packet.txbuf > 90 && stream_slowdown != 0) {
        // the buffer has enough space, speed up a bit
        stream_slowdown--;
    }

    //log rssi, noise, etc if logging Performance monitoring data
    if (log_radio) {
        dataflash.Log_Write_Radio(packet);
    }
}


/*
  handle an incoming mission item
 */
void GCS_handle::handle_mission_item(mavlink_message_t *msg, AP_Mission &mission)
{
    mavlink_mission_item_t packet;
    uint8_t result = MAV_MISSION_ACCEPTED;
    struct AP_Mission::Mission_Command cmd = {};

    mavlink_msg_mission_item_decode(msg, &packet);

    // convert mavlink packet to mission command
    if (!AP_Mission::mavlink_to_mission_cmd(packet, cmd)) {
        result = MAV_MISSION_INVALID;
        goto mission_ack;
    }

    if (packet.current == 2) {                                               
        // current = 2 is a flag to tell us this is a "guided mode"
        // waypoint and not for the mission
        handle_guided_request(cmd);

        // verify we received the command
        result = 0;
        goto mission_ack;
    }

    if (packet.current == 3) {
        //current = 3 is a flag to tell us this is a alt change only
        // add home alt if needed
        handle_change_alt_request(cmd);

        // verify we recevied the command
        result = 0;
        goto mission_ack;
    }

    // Check if receiving waypoints (mission upload expected)
    if (!waypoint_receiving) {
        result = MAV_MISSION_ERROR;
        goto mission_ack;
    }

    // check if this is the requested waypoint
    if (packet.seq != waypoint_request_i) {
        result = MAV_MISSION_INVALID_SEQUENCE;
        goto mission_ack;
    }
    
    // if command index is within the existing list, replace the command
    if (packet.seq < mission.num_commands()) {
        if (mission.replace_cmd(packet.seq,cmd)) {
            result = MAV_MISSION_ACCEPTED;
        }else{
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
        // if command is at the end of command list, add the command
    } else if (packet.seq == mission.num_commands()) {
        if (mission.add_cmd(cmd)) {
            result = MAV_MISSION_ACCEPTED;
        }else{
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
        // if beyond the end of the command list, return an error
    } else {
        result = MAV_MISSION_ERROR;
        goto mission_ack;
    }
    
    // update waypoint receiving state machine
    waypoint_timelast_receive = hal.scheduler->millis();
    waypoint_request_i++;
    
    if (waypoint_request_i >= waypoint_request_last) {
        mavlink_msg_mission_ack_send_buf(
            msg,
            chan,
            msg->sysid,
            msg->compid,
            MAV_MISSION_ACCEPTED);
        
        send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
        waypoint_receiving = false;
        // XXX ignores waypoint radius for individual waypoints, can
        // only set WP_RADIUS parameter
    } else {
        waypoint_timelast_request = hal.scheduler->millis();
        // if we have enough space, then send the next WP immediately
        if (comm_get_txspace(chan) >= 
            MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_MISSION_ITEM_LEN) {
            queued_waypoint_send();
        } else {
            send_message(MSG_NEXT_WAYPOINT);
        }
    }
    return;

mission_ack:
    // we are rejecting the mission/waypoint
    mavlink_msg_mission_ack_send_buf(
        msg,
        chan,
        msg->sysid,
        msg->compid,
        result);
}

// send a message using mavlink, handling message queueing
void GCS_handle::send_message(enum ap_message id)
{
    uint8_t i, nextid;

    // see if we can send the deferred messages, if any
    while (num_deferred_messages != 0) {
        if (!try_send_message(deferred_messages[next_deferred_message])) {
            break;
        }
        next_deferred_message++;
        if (next_deferred_message == MSG_RETRY_DEFERRED) {
            next_deferred_message = 0;
        }
        num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = next_deferred_message; i < num_deferred_messages; i++) {
        if (deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MSG_RETRY_DEFERRED) {
            nextid = 0;
        }
    }

    if (num_deferred_messages != 0 ||
        !try_send_message(id)) {
        // can't send it now, so defer it
        if (num_deferred_messages == MSG_RETRY_DEFERRED) {
            // the defer buffer is full, discard
            return;
        }
        nextid = next_deferred_message + num_deferred_messages;
        if (nextid >= MSG_RETRY_DEFERRED) {
            nextid -= MSG_RETRY_DEFERRED;
        }
        deferred_messages[nextid] = id;
        num_deferred_messages++;
    }
}

void
GCS_handle::update(void (*run_cli)(AP_HAL::UARTDriver *))
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = comm_receive_ch(chan);

        if (run_cli != NULL) {
            /* allow CLI to be started by hitting enter 3 times, if no
             *  heartbeat packets have been received */
            if (mavlink_active == 0 && (hal.scheduler->millis() - _cli_timeout) < 20000 && 
                comm_is_idle(chan) ) {
                if (c == '\n' || c == '\r') {
                    crlf_count++;
                } else {
                    crlf_count = 0;
                }
                if (crlf_count == 3) {
                    run_cli(_port);
                }
            }
        }

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            // we exclude radio packets to make it possible to use the
            // CLI over the radio
            if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
                mavlink_active |= (1U<<(chan-MAVLINK_COMM_0));
            }
            // if a snoop handler has been setup then use it
            if (msg_snoop != NULL) {
                msg_snoop(&msg);
            }
            if (routing.check_and_forward(chan, &msg)) {
                handleMessage(&msg);
            }
        }
    }

    if (!waypoint_receiving) {
        return;
    }

    uint32_t tnow = hal.scheduler->millis();
    uint32_t wp_recv_time = 1000U + (stream_slowdown*20);

    if (waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last &&
        tnow - waypoint_timelast_request > wp_recv_time) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (tnow - waypoint_timelast_receive) > wp_recv_time+waypoint_receive_timeout) {
        waypoint_receiving = false;
    }
}



/*
  handle a SET_MODE MAVLink message
 */
void GCS_handle::handle_set_mode(mavlink_message_t* msg, bool (*set_mode)(uint8_t mode))
{
    uint8_t result = MAV_RESULT_FAILED;
    mavlink_set_mode_t packet;
    mavlink_msg_set_mode_decode(msg, &packet);

    // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
    if (packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        if (set_mode(packet.custom_mode)) {
            result = MAV_RESULT_ACCEPTED;
        }
    } else if (packet.base_mode == MAV_MODE_FLAG_DECODE_POSITION_SAFETY) {
        // set the safety switch position. Must be in a command by itself
        if (packet.custom_mode == 0) {
            // turn safety off (pwm outputs flow to the motors)
            hal.rcout->force_safety_off();
            result = MAV_RESULT_ACCEPTED;
        } else if (packet.custom_mode == 1) {
            // turn safety on (no pwm outputs to the motors)
            if (hal.rcout->force_safety_on()) {
                result = MAV_RESULT_ACCEPTED;
            }
        }
    }

    // send ACK or NAK
    mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_SET_MODE, result);
}

