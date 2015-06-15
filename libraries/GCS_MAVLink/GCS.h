// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System
// protocols.

#ifndef __GCS_H
#define __GCS_H

#include <AP_Common.h>
#include <GCS_handle.h>
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
class GCS_MAVLINK : public GCS_handle {
public:
    GCS_MAVLINK();

    // common send functions
    void send_meminfo(void);
    void send_power_status(void);
    void send_ahrs2(AP_AHRS &ahrs);
    bool send_gps_raw(AP_GPS &gps);
    void send_system_time(AP_GPS &gps);
    void send_radio_in(uint8_t receiver_rssi);
    void send_raw_imu(const AP_InertialSensor &ins, const Compass &compass);
    void send_scaled_pressure(AP_Baro &barometer);
    void send_sensor_offsets(const AP_InertialSensor &ins, const Compass &compass, AP_Baro &barometer);
    void send_ahrs(AP_AHRS &ahrs);
    void send_battery2(const AP_BattMonitor &battery);
#if AP_AHRS_NAVEKF_AVAILABLE
    void send_opticalflow(AP_AHRS_NavEKF &ahrs, const OpticalFlow &optflow);
#endif
    void send_autopilot_version(void) const;
    void send_local_position(const AP_AHRS &ahrs) const;
    /*
      send a statustext message to all active MAVLink
      connections. This function is static so it can be called from
      any library
    */
    static void send_statustext_all(const prog_char_t *msg);
    /*
      send a MAVLink message to all components with this vehicle's system id
      This is a no-op if no routes to components have been learned
    */
    static void send_to_components(const mavlink_message_t* msg) { routing.send_to_components(msg); }

protected:
    // Init parameters: var_info and maybe other things the programmer needs
    virtual void setup_parameters() {};
};

#endif // __GCS_H
