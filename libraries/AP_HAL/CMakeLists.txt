cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL
  UARTDriver
  Util
)

include_directories ( utility )
add_subdirectory ( utility ) 

ADD_LIBRARY ( AP_HAL STATIC ${AP_HAL} )
TARGET_LINK_LIBRARIES ( AP_HAL ap_hal_utility ) 