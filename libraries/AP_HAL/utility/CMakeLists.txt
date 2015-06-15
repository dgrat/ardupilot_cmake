cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( ap_hal_utility
  ftoa_engine
  Print
  print_vprintf
  utoa_invert
)

ADD_LIBRARY ( ap_hal_utility STATIC ${ap_hal_utility} )