cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL_FLYMAPLE
  AnalogIn
  AnalogSource
  GPIO
  HAL_FLYMAPLE_Class
  I2CDriver
  RCInput
  RCOutput
  Scheduler
  Semaphores
  SPIDriver
  Storage
  UARTDriver
)

include_directories ( utility )
add_subdirectory ( utility ) 

ADD_LIBRARY ( AP_HAL_FLYMAPLE STATIC ${AP_HAL_FLYMAPLE} )
TARGET_LINK_LIBRARIES ( AP_HAL_FLYMAPLE flymaple_utility ) 