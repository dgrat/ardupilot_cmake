cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( avr_utility
  ISRRegistry
  pins_arduino_mega
)

ADD_LIBRARY ( avr_utility STATIC ${avr_utility} )