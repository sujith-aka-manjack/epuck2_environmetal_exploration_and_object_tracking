#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "motors.h"
#include "sensors/proximity.h"
#include "selector.h"
#include "sensors/VL53L0X/VL53L0X.h"

#define obj_true 450  // value if obstacle detected - to be measured

// defining inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int prox_cal[8]={0,0,0,0,0,0,0,0};
int i=0;

int obs_status;
int obs_side;
int obs_value_right;
int obs_value_left;

int main(void)
{
	// Primary initialisations
    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //initialising motors
    motors_init();

    // start proximity measurement module
    proximity_start();

    // Calibrate proximity sensors
    calibrate_ir();

    /* Infinite loop. */
    while (1)
    {
    	//waits 1 second
        chThdSleepMilliseconds(250);
    	obs_status=0;
    	obs_side=0;
    	obs_value_right=0;
    	obs_value_left=0;

    	for(i=0;i<8;++i)
        	{ prox_cal[i]=get_calibrated_prox(i); }

       	//left_motor_set_speed(300);
        //right_motor_set_speed(300);

        for(i=0;i<=1;++i)
        {
            obs_value_right+=prox_cal[i];
        	if((prox_cal[i]>obj_true)&&(obs_status==0))
           		obs_status=1;
        }
        for(i=6;i<=7;++i)
        {
          obs_value_left+=prox_cal[i];
          if((prox_cal[i]>obj_true)&&(obs_status==0))
          	obs_status=1;
        }
        if (obs_status == 1)
        {
        	if(obs_value_right>=obs_value_left)
           		obs_side = 1;
           	else
           		obs_side = -1;
           	if(obs_side == 1)
           	{
           		left_motor_set_speed(-250);
           		right_motor_set_speed(250);
           		//chThdSleepMilliseconds(2500);
           	}
           	else
           	{
           		left_motor_set_speed(250);
           		right_motor_set_speed(-250);
           		//chThdSleepMilliseconds(2500);
           	}
        }
        else
        {
          left_motor_set_speed(600);
          right_motor_set_speed(600);
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
