# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(RubbishBahFlighController)

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)

target_include_directories(app 
	PRIVATE 
		src
		ext/flysky_fs16x/src
		ext/ZephyrSDCard/src
		ext/ZephyrMPU6050/src
)

if(CONFIG_TEENSY41)
	target_sources(app 
		PRIVATE 
		ext/ZephyrMPU6050/src/console.cpp
	)
endif()


target_sources(app 
	PRIVATE 
		src/blinky.c
		#src/rbpid.cpp
		ext/flysky_fs16x/src/flysky_fs16x.cpp
		#ext/ZephyrSDCard/src/zsdcard.cpp
		ext/ZephyrMPU6050/src/mpu6050.cpp
		src/rbfc.cpp
		src/main.cpp
)
