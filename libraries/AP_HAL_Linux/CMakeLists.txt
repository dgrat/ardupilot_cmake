cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL_Linux
  AnalogIn
  AnalogIn_Navio
  dsm
  GPIO
  GPIO_BBB
  GPIO_RPI
  HAL_Linux_Class
  I2CDriver
  RCInput
  RCInput_AioPRU
  RCInput_Navio
  RCInput_PRU
  RCInput_ZYNQ
  RCOutput_AioPRU
  RCOutput_Navio
  RCOutput_PRU
  RCOutput_ZYNQ
  sbus
  Scheduler
  Semaphores
  SPIDriver
  SPIUARTDriver
  Storage
  Storage_FRAM
  ToneAlarmDriver
  UARTDriver
  Util
)

ADD_LIBRARY ( AP_HAL_Linux STATIC ${AP_HAL_Linux} )
TARGET_LINK_LIBRARIES ( AP_HAL_Linux pthread ) 