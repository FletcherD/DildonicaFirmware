#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(peripheral_lbs)

set(OVERLAY_FILE adafruit_itsybitsy_nrf52840.overlay)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/dildonica.cpp
  #src/main.cpp
)

# NORDIC SDK APP END
