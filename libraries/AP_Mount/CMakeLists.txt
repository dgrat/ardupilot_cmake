cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Mount
  AP_Gimbal
  AP_Mount
  AP_Mount_Alexmos
  AP_Mount_Backend
  AP_Mount_MAVLink
  AP_Mount_Servo
  AP_Mount_SToRM32
  AP_Mount_SToRM32_serial
)

ADD_LIBRARY ( AP_Mount STATIC ${AP_Mount} )