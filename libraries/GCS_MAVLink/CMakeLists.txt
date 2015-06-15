cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( GCS_MAVLink
  GCS
  GCS_handle
  GCS_log
  GCS_MAVLink
  GCS_serial_control
  MAVLink_routing
)

ADD_LIBRARY ( GCS_MAVLink STATIC ${GCS_MAVLink} )