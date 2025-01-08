/*
 * Copyright (c) 2024 Open Pixel Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "blinky.h"
#include "rbfc.h"

int app_init(void) {
	//gpio_pin_configure_dt(&receiver0, GPIO_INPUT);	
	startBlinkyThread();
	
	return 0;
}

int main(void) {
	app_init();

	k_msleep(2000);

	RbosDrone rbosDrone = RbosDrone();

	while(1) {
//		printf("|");
//		rbosDrone.printImuData();
		rbosDrone.do_things();
		
		k_msleep(20);
	}

	return 0;
}
