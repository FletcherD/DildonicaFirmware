#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dildonica)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/bluetooth_peripheral_gatt.cpp
  src/bluetooth_dildonica.cpp
  src/dildonica.cpp
)

#target_include_directories(app PRIVATE common)

# NORDIC SDK APP END
