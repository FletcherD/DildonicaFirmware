#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

if(DEFINED CUSTOM_BOILERPLATE)
    include(${CUSTOM_BOILERPLATE})
    return()
endif()

set(COMMON_PATH ${CMAKE_CURRENT_LIST_DIR}/common)
include(${COMMON_PATH}/common.cmake)

GET_DEVICE_CONFIG_FILES(${BOARD} boards)
list(APPEND DTC_OVERLAY_FILE ${BOARD}.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dildonica)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/dildonica.cpp
  #src/main.cpp
)

#target_include_directories(app PRIVATE common)

# NORDIC SDK APP END
