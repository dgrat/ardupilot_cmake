# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#

cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

#TODO: find_package(i2c-tools)

# ***************************************
# Declare the library module list
# ***************************************
SET( AP_List
  AC_AttitudeControl
  AC_Fence
  AC_PID
  AC_Sprayer
  AC_WPNav
  AP_ADC
  AP_ADC_AnalogSource
  AP_AHRS
  AP_Airspeed
  AP_Arming
  AP_Baro
  AP_BattMonitor
  AP_BoardConfig
  AP_Camera
  AP_Common
  AP_Compass
  AP_Curve
  AP_Declination
  AP_EPM
  AP_Frsky_Telem
  AP_GPS
  AP_HAL
  AP_HAL_AVR
  AP_HAL_Empty
  AP_HAL_FLYMAPLE
  AP_HAL_PX4
  AP_HAL_VRBRAIN
  AP_InertialNav
  AP_InertialSensor
  AP_L1_Control
  AP_LandingGear
  AP_Math
  AP_Menu
  AP_Mission
  AP_Motors
  AP_Mount
  AP_NavEKF
  AP_Notify
  AP_OpticalFlow
  AP_Parachute
  AP_Param
  AP_PerfMon
  AP_Progmem
  AP_Rally
  AP_RangeFinder
  AP_RCMapper
  AP_Relay
  AP_Scheduler
  AP_SerialManager
  AP_ServoRelayEvents
  AP_TECS
  AP_Terrain
  APM_Control
  APM_OBC
  APM_PI
  DataFlash
  Filter
  GCS_MAVLink
  RC_Channel
  SITL
  StorageManager
)

SET( AP_List_Linux
  AP_HAL_Linux
)

SET( AP_Headers
  AP_Buffer
  AP_Navigation
  AP_SpdHgtControl
  AP_Vehicle
  #GCS_MAVLink
)

SET( AP_Firmwares
  #tinycopter
)

# ***************************************
# Generate build flags and config files
# .. if required ..
# ***************************************
message( STATUS "Include cmake modules" )
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")
include(CMakeParseArguments)
include(APMOption)

message( STATUS "Include options file" )
include(options.cmake)

message( STATUS "Add proper definitions" )
apm_option_genflags(BUILD_FLAGS APM_BUILD_FLAGS)
apm_option_genhfile(FILE "config.h")
add_definitions(${APM_BUILD_FLAGS})
message( STATUS "APM_BUILD_FLAGS are: " ${APM_BUILD_FLAGS} )

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# ***************************************
# Compiler and linker settings
# ***************************************
#set(DBG_FLAGS "-g -ggdb -O0")
set(REQ_FLAGS "-std=gnu++11 -Wno-reorder") # necessary for build
set(OPT_FLAGS "-Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2")
set(WRN_FLAGS "-Wno-unused-function -Wno-unused-parameter")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPT_FLAGS} ${WRN_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${REQ_FLAGS} ${OPT_FLAGS} ${WRN_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--relax")

# ***************************************
# Build the library
# ***************************************
message( STATUS "Walk through the sub directories of the library" )
add_subdirectory( libraries )

# ***************************************
# Build the firmwares
# ***************************************
message( STATUS "RPiCopter: Check the build target" )
foreach( TARG ${AP_Firmwares} )
	include_directories ( ${TARG} )
	add_subdirectory( ${TARG} )
endforeach()