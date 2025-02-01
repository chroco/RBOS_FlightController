/*
 *  It is not so very important for a person to learn facts. 
 *  For that he does not really need a college. He can learn 
 *  them from books. The value of an education ... is not the 
 *  learning of many facts, but the training of the mind to think 
 *  something that cannot be learned from textbooks.”
 *
 *                                -Einstein
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
