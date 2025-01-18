/*
 * Copyright (c) 2024 Open Pixel Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "blinky.h"
#include "rbfc.h"

int app_init(void) {
	startBlinkyThread();
	
	return 0;
}

int main(void) {
	app_init();

	RbosDrone rbosDrone = RbosDrone();

	while(1) {
		rbosDrone.do_things();
		
		k_msleep(50);
	}

	return 0;
}
