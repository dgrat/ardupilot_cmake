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

#include <GCS.h>
#include <GCS_handle.h>
#include <AP_AHRS.h>


extern const AP_HAL::HAL& hal;


GCS_MAVLINK::GCS_MAVLINK() : GCS_handle() {
    //AP_Param::setup_object_defaults(this, var_info); // There must be an initialization of "var_info" at compile time: like in the above example
    // .. use a virtual init function which must be implemented together with var_info in the firmwares
    setup_parameters();
}

void GCS_MAVLINK::send_meminfo(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
    extern unsigned __brkval;
#else
    unsigned __brkval = 0;
#endif
    mavlink_msg_meminfo_send(chan, __brkval, hal.util->available_memory());
}

// report power supply status
void GCS_MAVLINK::send_power_status(void)
{
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
    mavlink_msg_power_status_send(chan,
                                  hal.analogin->board_voltage() * 1000,
                                  hal.analogin->servorail_voltage() * 1000,
                                  hal.analogin->power_status_flags());
#endif
}
  
/*
  send raw GPS position information (GPS_RAW_INT, GPS2_RAW, GPS_RTK and GPS2_RTK).
  returns true if messages fit into transmit buffer, false otherwise.
 */
bool GCS_MAVLINK::send_gps_raw(AP_GPS &gps)
{
    if (comm_get_txspace(chan) >= 
        MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
        gps.send_mavlink_gps_raw(chan);
    } else {
        return false;
    }

#if GPS_RTK_AVAILABLE
    if (gps.highest_supported_status(0) > AP_GPS::GPS_OK_FIX_3D) {
        if (comm_get_txspace(chan) >= MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_GPS_RTK_LEN) {
            gps.send_mavlink_gps_rtk(chan);
        }

    }
#endif

#if GPS_MAX_INSTANCES > 1

    if (gps.num_sensors() > 1 && gps.status(1) > AP_GPS::NO_GPS) {

        if (comm_get_txspace(chan) >= MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_GPS2_RAW_LEN) {
            gps.send_mavlink_gps2_raw(chan);
        }

#if GPS_RTK_AVAILABLE
        if (gps.highest_supported_status(1) > AP_GPS::GPS_OK_FIX_3D) {
            if (comm_get_txspace(chan) >= 
                MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_GPS2_RTK_LEN) {
                gps.send_mavlink_gps2_rtk(chan);
            }
        }
#endif
    }
#endif

    //TODO: Should check what else managed to get through...
    return true;

}
  
// report AHRS2 state
void GCS_MAVLINK::send_ahrs2(AP_AHRS &ahrs)
{
#if AP_AHRS_NAVEKF_AVAILABLE
    Vector3f euler;
    struct Location loc;
    if (ahrs.get_secondary_attitude(euler) && ahrs.get_secondary_position(loc)) {
        mavlink_msg_ahrs2_send(chan,
                               euler.x,
                               euler.y,
                               euler.z,
                               loc.alt*1.0e-2f,
                               loc.lat,
                               loc.lng);
    }
#endif
}



/*
  send the SYSTEM_TIME message
 */
void GCS_MAVLINK::send_system_time(AP_GPS &gps)
{
    mavlink_msg_system_time_send(
        chan,
        gps.time_epoch_usec(),
        hal.scheduler->millis());
}


/*
  send RC_CHANNELS_RAW, and RC_CHANNELS messages
 */
void GCS_MAVLINK::send_radio_in(uint8_t receiver_rssi)
{
    uint32_t now = hal.scheduler->millis();

    uint16_t values[8];
    memset(values, 0, sizeof(values));
    hal.rcin->read(values, 8);

    mavlink_msg_rc_channels_raw_send(
        chan,
        now,
        0, // port
        values[0],
        values[1],
        values[2],
        values[3],
        values[4],
        values[5],
        values[6],
        values[7],
        receiver_rssi);

    if (hal.rcin->num_channels() > 8 && 
        comm_get_txspace(chan) >= MAVLINK_MSG_ID_RC_CHANNELS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
        mavlink_msg_rc_channels_send(
            chan,
            now,
            hal.rcin->num_channels(),
            hal.rcin->read(CH_1),
            hal.rcin->read(CH_2),
            hal.rcin->read(CH_3),
            hal.rcin->read(CH_4),
            hal.rcin->read(CH_5),
            hal.rcin->read(CH_6),
            hal.rcin->read(CH_7),
            hal.rcin->read(CH_8),
            hal.rcin->read(CH_9),
            hal.rcin->read(CH_10),
            hal.rcin->read(CH_11),
            hal.rcin->read(CH_12),
            hal.rcin->read(CH_13),
            hal.rcin->read(CH_14),
            hal.rcin->read(CH_15),
            hal.rcin->read(CH_16),
            hal.rcin->read(CH_17),
            hal.rcin->read(CH_18),
            receiver_rssi);        
    }
}

void GCS_MAVLINK::send_raw_imu(const AP_InertialSensor &ins, const Compass &compass)
{
    const Vector3f &accel = ins.get_accel(0);
    const Vector3f &gyro = ins.get_gyro(0);
    const Vector3f &mag = compass.get_field(0);

    mavlink_msg_raw_imu_send(
        chan,
        hal.scheduler->micros(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z);
#if INS_MAX_INSTANCES > 1
    if (ins.get_gyro_count() <= 1 &&
        ins.get_accel_count() <= 1 &&
        compass.get_count() <= 1) {
        return;
    }
    const Vector3f &accel2 = ins.get_accel(1);
    const Vector3f &gyro2 = ins.get_gyro(1);
    const Vector3f &mag2 = compass.get_field(1);
    mavlink_msg_scaled_imu2_send(
        chan,
        hal.scheduler->millis(),
        accel2.x * 1000.0f / GRAVITY_MSS,
        accel2.y * 1000.0f / GRAVITY_MSS,
        accel2.z * 1000.0f / GRAVITY_MSS,
        gyro2.x * 1000.0f,
        gyro2.y * 1000.0f,
        gyro2.z * 1000.0f,
        mag2.x,
        mag2.y,
        mag2.z);        
#endif
}

void GCS_MAVLINK::send_scaled_pressure(AP_Baro &barometer)
{
    uint32_t now = hal.scheduler->millis();
    float pressure = barometer.get_pressure(0);
    mavlink_msg_scaled_pressure_send(
        chan,
        now,
        pressure*0.01f, // hectopascal
        (pressure - barometer.get_ground_pressure(0))*0.01f, // hectopascal
        barometer.get_temperature(0)*100); // 0.01 degrees C
#if BARO_MAX_INSTANCES > 1
    if (barometer.num_instances() > 1) {
        pressure = barometer.get_pressure(1);
        mavlink_msg_scaled_pressure2_send(
            chan,
            now,
            pressure*0.01f, // hectopascal
            (pressure - barometer.get_ground_pressure(1))*0.01f, // hectopascal
            barometer.get_temperature(1)*100); // 0.01 degrees C        
    }
#endif
}

void GCS_MAVLINK::send_sensor_offsets(const AP_InertialSensor &ins, const Compass &compass, AP_Baro &barometer)
{
    // run this message at a much lower rate - otherwise it
    // pointlessly wastes quite a lot of bandwidth
    static uint8_t counter;
    if (counter++ < 10) {
        return;
    }
    counter = 0;

    const Vector3f &mag_offsets = compass.get_offsets(0);
    const Vector3f &accel_offsets = ins.get_accel_offsets(0);
    const Vector3f &gyro_offsets = ins.get_gyro_offsets(0);

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_pressure(),
                                    barometer.get_temperature()*100,
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
}

void GCS_MAVLINK::send_ahrs(AP_AHRS &ahrs)
{
    const Vector3f &omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        0,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}

/*
  send a statustext message to all active MAVLink connections
 */
void GCS_MAVLINK::send_statustext_all(const prog_char_t *msg)
{
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if ((1U<<i) & mavlink_active) {
            mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0+i);
            if (comm_get_txspace(chan) >= MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_STATUSTEXT_LEN) {
                char msg2[50];
                strncpy_P(msg2, msg, sizeof(msg2));
                mavlink_msg_statustext_send(chan,
                                            SEVERITY_HIGH,
                                            msg2);
            }
        }
    }
}

// report battery2 state
void GCS_MAVLINK::send_battery2(const AP_BattMonitor &battery)
{
    if (battery.num_instances() > 1) {
        mavlink_msg_battery2_send(chan, battery.voltage2()*1000, -1);
    }
}

#if AP_AHRS_NAVEKF_AVAILABLE
/*
  send OPTICAL_FLOW message
 */
void GCS_MAVLINK::send_opticalflow(AP_AHRS_NavEKF &ahrs, const OpticalFlow &optflow)
{
    // exit immediately if no optical flow sensor or not healthy
    if (!optflow.healthy()) {
        return;
    }

    // get rates from sensor
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    float hagl = 0;

    if (ahrs.have_inertial_nav()) {
        ahrs.get_NavEKF().getHAGL(hagl);
    }

    // populate and send message
    mavlink_msg_optical_flow_send(
        chan,
        hal.scheduler->millis(),
        0, // sensor id is zero
        flowRate.x,
        flowRate.y,
        bodyRate.x,
        bodyRate.y,
        optflow.quality(),
        hagl); // ground distance (in meters) set to zero
}
#endif

/*
  send AUTOPILOT_VERSION packet
 */
void GCS_MAVLINK::send_autopilot_version(void) const
{
    uint16_t capabilities = 0;
    uint32_t flight_sw_version = 0;
    uint32_t middleware_sw_version = 0;
    uint32_t os_sw_version = 0;
    uint32_t board_version = 0;
    uint8_t flight_custom_version[8];
    uint8_t middleware_custom_version[8];
    uint8_t os_custom_version[8];
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    uint64_t uid = 0;
    
#if defined(GIT_VERSION)
    strncpy((char *)flight_custom_version, GIT_VERSION, 8);
#else
    memset(middleware_custom_version,0,8);
#endif

#if defined(PX4_GIT_VERSION)
    strncpy((char *)middleware_custom_version, PX4_GIT_VERSION, 8);
#else
    memset(middleware_custom_version,0,8);
#endif
    
#if defined(NUTTX_GIT_VERSION)
    strncpy((char *)os_custom_version, NUTTX_GIT_VERSION, 8);
#else
    memset(os_custom_version,0,8);
#endif
    
    mavlink_msg_autopilot_version_send(
        chan,
        capabilities,
        flight_sw_version,
        middleware_sw_version,
        os_sw_version,
        board_version,
        flight_custom_version,
        middleware_custom_version,
        os_custom_version,
        vendor_id,
        product_id,
        uid
    );
}

/*
  send LOCAL_POSITION_NED message
 */
void GCS_MAVLINK::send_local_position(const AP_AHRS &ahrs) const
{
    Vector3f local_position, velocity;
    if (!ahrs.get_relative_position_NED(local_position) ||
        !ahrs.get_velocity_NED(velocity)) {
        // we don't know the position and velocity
        return;
    }

    mavlink_msg_local_position_ned_send(
        chan,
        hal.scheduler->millis(),
        local_position.x,
        local_position.y,
        local_position.z,
        velocity.x,
        velocity.y,
        velocity.z);
}
