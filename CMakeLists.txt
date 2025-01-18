# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(RubbishBahFlighController)

include_directories(
)

target_include_directories(app 
	PRIVATE 
		src
		ext/flysky_fs16x/src
)

target_sources(app 
	PRIVATE 
		src/blinky.c
		#src/rbpid.cpp
		ext/flysky_fs16x/src/flysky_fs16x.cpp
		src/mpu6050.cpp
		src/rbfc.cpp
		src/main.cpp

)
