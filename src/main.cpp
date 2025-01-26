/*
 * Copyright (c) 2024 Open Pixel Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "blinky.h"
#include "rbfc.h"

#ifdef CONFIG_TEENSY41
#include "console.h"
#endif

int app_init(void)
{

#ifdef CONFIG_TEENSY41
	startConsole();
#endif

	startBlinkyThread();
	
	return 0;
}

int main(void)
{
	app_init();

	RbosDrone rbosDrone = RbosDrone();

	while(1)
	{
		//printf(".");		
		k_msleep(1000);
	}

	return 0;
}
