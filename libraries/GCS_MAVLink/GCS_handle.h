// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System
// protocols.

#ifndef __GCS_PR_H
#define __GCS_PR_H

#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <DataFlash.h>
#include <AP_Mission.h>
#include "../AP_BattMonitor/AP_BattMonitor.h"
#include <stdint.h>
#include <MAVLink_routing.h>
#include "../AP_SerialManager/AP_SerialManager.h"
#include "../AP_Mount/AP_Mount.h"


///
/// @class	GCS_MAVLINK
/// @brief	MAVLink transport control class
///
class GCS_handle {
public:
    GCS_handle();
    void        update(void (*run_cli)(AP_HAL::UARTDriver *));
    void        init(AP_HAL::UARTDriver *port, mavlink_channel_t mav_chan);
    void        setup_uart(const AP_SerialManager& serial_manager, AP_SerialManager::SerialProtocol protocol, uint8_t instance);
    void        set_snoop(void (*_msg_snoop)(const mavlink_message_t* msg) );

    // accessor for uart
    AP_HAL::UARTDriver *get_uart() { return _port; }


    // NOTE! The streams enum below and the
    // set of AP_Int16 stream rates _must_ be
    // kept in the same order
    enum streams {STREAM_RAW_SENSORS,
                  STREAM_EXTENDED_STATUS,
                  STREAM_RC_CHANNELS,
                  STREAM_RAW_CONTROLLER,
                  STREAM_POSITION,
                  STREAM_EXTRA1,
                  STREAM_EXTRA2,
                  STREAM_EXTRA3,
                  STREAM_PARAMS,
                  NUM_STREAMS};

    // this costs us 51 bytes per instance, but means that low priority
    // messages don't block the CPU
    mavlink_statustext_t pending_status;

    // call to reset the timeout window for entering the cli
    void reset_cli_timeout();

    uint32_t        last_heartbeat_time; // milliseconds

    // last time we got a non-zero RSSI from RADIO_STATUS
    static uint32_t last_radio_status_remrssi_ms;

    // bitmask of what mavlink channels are active
    static uint8_t mavlink_active;
    
    // saveable rate of each stream
    AP_Int16        streamRates[NUM_STREAMS];
    // set to true if this GCS link is active
    bool            initialised;
    
    void        send_message(enum ap_message id);
    void        send_text(gcs_severity severity, const char *str);
    void        send_text_P(gcs_severity severity, const prog_char_t *str);
    void        queued_param_send();
    void        queued_waypoint_send();
    
    /*
     * Overload me please
     */
    virtual void data_stream_send() {};
    
protected: /*functions*/    
    /*
     * Overload me please
     */
    virtual void handleMessage(mavlink_message_t *msg) {}
    virtual bool try_send_message(enum ap_message id) { return false; }
    virtual void handle_guided_request(AP_Mission::Mission_Command &cmd) {}
    virtual void handle_change_alt_request(AP_Mission::Mission_Command &cmd) {}
    // see if we should send a stream now. Called at 50Hz
    virtual bool stream_trigger(enum streams stream_num) { return false; }
    
    // a vehicle can optionally snoop on messages for other systems
    static void (*msg_snoop)(const mavlink_message_t* msg);
    
    /*
     * Handles
     */
    void handle_log_request_list(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_data(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_erase(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_end(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_message(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_send(DataFlash_Class &dataflash);
    void handle_log_send_listing(DataFlash_Class &dataflash);
    bool handle_log_send_data(DataFlash_Class &dataflash);

    void handle_mission_request_list(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_request(AP_Mission &mission, mavlink_message_t *msg);

    void handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_count(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_clear_all(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_write_partial_list(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_item(mavlink_message_t *msg, AP_Mission &mission);

    void handle_request_data_stream(mavlink_message_t *msg, bool save);
    void handle_param_request_list(mavlink_message_t *msg);
    void handle_param_request_read(mavlink_message_t *msg);
    void handle_param_set(mavlink_message_t *msg, DataFlash_Class *DataFlash);
    void handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio);
    void handle_serial_control(mavlink_message_t *msg, AP_GPS &gps);
    void lock_channel(mavlink_channel_t chan, bool lock);
    void handle_set_mode(mavlink_message_t* msg, bool (*set_mode)(uint8_t mode));
    void handle_gimbal_report(AP_Mount &mount, mavlink_message_t *msg) const;

    // return true if this channel has hardware flow control
    bool have_flow_control(void);
    
protected: /*variables*/    
    mavlink_channel_t chan;
    
    /// The stream we are communicating over
    AP_HAL::UARTDriver *_port;

    /// Perform queued sending operations
    ///
    AP_Param *                  _queued_parameter;      ///< next parameter to
                                                        // be sent in queue
    enum ap_var_type            _queued_parameter_type; ///< type of the next
                                                        // parameter
    AP_Param::ParamToken        _queued_parameter_token; ///AP_Param token for
                                                         // next() call
    uint16_t                    _queued_parameter_index; ///< next queued
                                                         // parameter's index
    uint16_t                    _queued_parameter_count; ///< saved count of
                                                         // parameters for
                                                         // queued send
    uint32_t                    _queued_parameter_send_time_ms;

    /// Count the number of reportable parameters.
    ///
    /// Not all parameters can be reported via MAVlink.  We count the number
    // that are
    /// so that we can report to a GCS the number of parameters it should
    // expect when it
    /// requests the full set.
    ///
    /// @return         The number of reportable parameters.
    ///
    uint16_t                    _count_parameters(); ///< count reportable
                                                     // parameters

    uint16_t                    _parameter_count;   ///< cache of reportable
                                                    // parameters
    uint16_t                    packet_drops;

    // this allows us to detect the user wanting the CLI to start
    uint8_t        crlf_count;

    // waypoints
    uint16_t        waypoint_request_i; // request index
    uint16_t        waypoint_request_last; // last request index
    uint16_t        waypoint_dest_sysid; // where to send requests
    uint16_t        waypoint_dest_compid; // "
    bool            waypoint_receiving; // currently receiving
    uint16_t        waypoint_count;
    uint32_t        waypoint_timelast_receive; // milliseconds
    uint32_t        waypoint_timelast_request; // milliseconds
    const uint16_t  waypoint_receive_timeout; // milliseconds

    // number of 50Hz ticks until we next send this stream
    uint8_t         stream_ticks[NUM_STREAMS];

    // number of extra ticks to add to slow things down for the radio
    uint8_t         stream_slowdown;

    // millis value to calculate cli timeout relative to.
    // exists so we can separate the cli entry time from the system start time
    uint32_t _cli_timeout;

    uint8_t  _log_listing:1; // sending log list
    uint8_t  _log_sending:1; // sending log data

    // next log list entry to send
    uint16_t _log_next_list_entry;

    // last log list entry to send
    uint16_t _log_last_list_entry;

    // number of log files
    uint16_t _log_num_logs;

    // log number for data send
    uint16_t _log_num_data;

    // offset in log
    uint32_t _log_data_offset;

    // size of log file
    uint32_t _log_data_size;

    // number of bytes left to send
    uint32_t _log_data_remaining;

    // start page of log data
    uint16_t _log_data_page;

    // deferred message handling
    enum ap_message deferred_messages[MSG_RETRY_DEFERRED];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;

    // mavlink routing object
    static MAVLink_routing routing;
};

#endif // __GCS_PR_H
