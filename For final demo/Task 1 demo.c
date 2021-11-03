/* Code made by Sujith Kurian James for epuck lab Task 1
   cop20skj
   Group 2 Team A */

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

#define obj_true 500  // value if obstacle detected - to be measured
#define wait 100		// duration for which to pause the robot

// defining inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int prox_cal[8]={0,0,0,0,0,0,0,0};	//For storing proximity sensor values
int i=0;

int obs_status;		//Variable to state the presence of an obstacle
int obs_side;		//Variable to state the side of an obstacle
int obs_value_right;	//Variables to state the closeness of an obstacle
int obs_value_left;
int count_left=0;		//Variables to count the continuos rotation in one direction
int count_right=0;

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

    // Initialising LEDs
    clear_leds();
    spi_comm_start();

    /* Infinite loop. */
    while (1)
    {
    	//Initialisation
        chThdSleepMilliseconds(wait);
    	obs_status=0;
    	obs_side=0;
    	obs_value_right=0;
    	obs_value_left=0;

    	set_led(LED3,0);			//Turns OFF the side LEDs
    	set_led(LED7,0);
    	set_rgb_led(LED2,0,0,0);	//Turns ON all RGB LEDs green while robot is moving forward
    	set_rgb_led(LED4,0,0,0);
    	set_rgb_led(LED6,0,0,0);
    	set_rgb_led(LED8,0,0,0);

    	// main code
    	for(i=0;i<8;++i)							//Getting the proximity sensor values
        	{ prox_cal[i]=get_calibrated_prox(i); }

        for(i=0;i<=1;++i)							//Checking for the presence of obstacles in right side
        {
            obs_value_right+=prox_cal[i];
        	if((prox_cal[i]>obj_true)&&(obs_status==0))
           		obs_status=1;
        }
        for(i=6;i<=7;++i)							//Checking for the presence of obstacles in left side
        {
        	obs_value_left+=prox_cal[i];
        	if((prox_cal[i]>obj_true)&&(obs_status==0))
        		obs_status=1;
        }
        if (obs_status == 1)	//Checking at which side obstacle is closer. if right side = 1, if left side = -1
        {
        	if(obs_value_right>=obs_value_left)
        	{	obs_side = 1;
        		count_left=count_left+1;
        		count_right=0;
        	}
           	else
           	{	obs_side = -1;
           		count_right=count_right+1;
           		count_left=0;
           	}
        	if(obs_side == 1)	//If obstacle is at right, turn left till the right prox doesnt sense any obstacle
           	{					//and vice versa
           		//count_right=0;
        		while(prox_cal[1]>obj_true)
           		{
           			set_led(LED7,2);	//Flashing left side LED
           			left_motor_set_speed(-250);
           			right_motor_set_speed(250);
           			//prox_cal[0] = get_calibrated_prox(0);
           			prox_cal[1] = get_calibrated_prox(1);
           			chThdSleepMilliseconds(wait);
           		}
           	}
           	else
           	{
           		//count_left=0;
           		while(prox_cal[6]>obj_true)
           		{
           			set_led(LED3,2);	//Flashing right side LED
           			left_motor_set_speed(250);
           			right_motor_set_speed(-250);
           			prox_cal[6] = get_calibrated_prox(6);
           			//prox_cal[7] = get_calibrated_prox(7);
           			chThdSleepMilliseconds(wait);
           		}
           	}
        }
        else		// If no obstacle then continue in straight line
        {
          set_rgb_led(LED2,10,0,0);
          set_rgb_led(LED8,10,0,0);
          left_motor_set_speed(480);
          right_motor_set_speed(480);
        }
        //To prevent only wall tracing, if robot makes 4 consecutive turns in the same dir, then rotate robot
        if(count_left>10)
        {
        	set_rgb_led(LED2,0,0,0);
        	set_rgb_led(LED4,0,0,0);
        	set_rgb_led(LED6,0,0,10);
        	set_rgb_led(LED8,0,0,10);
        	left_motor_set_speed(-250);
        	right_motor_set_speed(250);
        	chThdSleepMilliseconds(2000);
        	count_left = 0;
        }
        if(count_right>10)
        {
        	set_rgb_led(LED2,0,0,10);
        	set_rgb_led(LED4,0,0,10);
        	set_rgb_led(LED6,0,0,0);
        	set_rgb_led(LED8,0,0,0);
        	left_motor_set_speed(250);
           	right_motor_set_speed(-250);
           	chThdSleepMilliseconds(2000);
           	count_right = 0;
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
