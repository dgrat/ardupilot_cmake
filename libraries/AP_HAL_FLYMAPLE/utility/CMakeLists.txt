cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( flymaple_utility
  EEPROM
  flash_stm32
)

ADD_LIBRARY ( flymaple_utility STATIC ${flymaple_utility} )