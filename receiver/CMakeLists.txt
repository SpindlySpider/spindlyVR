# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(transmitter)


zephyr_include_directories(../common/include/)
zephyr_include_directories(src)

target_sources(app PRIVATE
  src/main.c
  # src/adc.c
  src/receive_data.c
)

target_sources(app PRIVATE
  ../common/src/MadgwickAHRS.c
  ../common/src/imu.c
  ../common/src/qmc5883p.c
  ../common/src/orientation_handle.c
  ../common/src/data_handle.c
)

