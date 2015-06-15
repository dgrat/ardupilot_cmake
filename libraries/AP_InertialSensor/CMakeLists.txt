cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( LSM9DS0
  LSM9DS0
)

SET( AP_InertialSensor
  AP_InertialSensor
  AP_InertialSensor_Backend
  AP_InertialSensor_Flymaple
  AP_InertialSensor_HIL
  AP_InertialSensor_L3GD20
  AP_InertialSensor_LSM303D
  AP_InertialSensor_MPU6000
  AP_InertialSensor_MPU9150
  AP_InertialSensor_MPU9250
  AP_InertialSensor_Oilpan
  AP_InertialSensor_PX4
  AP_InertialSensor_UserInteract_MAVLink
  AP_InertialSensor_UserInteract_Stream
)

SET( AP_InertialSensor_Linux
  AP_InertialSensor_L3G4200D
)

include_directories( ${LSM9DS0} )
#add_subdirectory( ${LSM9DS0} )

if( ${UNIX} )
	message( STATUS "AP_InertialSensor: Unix like environment detected. Include dependent source files into build process." )
	ADD_LIBRARY( AP_InertialSensor STATIC ${AP_InertialSensor} ${AP_InertialSensor_Linux} )
else()
	ADD_LIBRARY( AP_InertialSensor STATIC ${AP_InertialSensor} )
endif()

#TARGET_LINK_LIBRARIES( AP_InertialSensor AP_InertialSensor_LSM9DS0 )