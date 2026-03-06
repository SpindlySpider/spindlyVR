# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(transmitter)

target_sources(app PRIVATE
  src/main.c
  src/qmc5883p.c
  src/pwm.c
  src/imu.c
  src/adc.c
  src/orientation_handle.c
  src/data_handle.c
  src/transmit_data.c
  src/receive_data.c
  src/lib/MadgwickAHRS.c
)
