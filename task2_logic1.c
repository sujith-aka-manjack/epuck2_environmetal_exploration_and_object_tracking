/* Code made by Sujith Kurian James for epuck lab Task 2
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

#define obj_far 200  	//value if obj is considered as far
#define obj_near 400	//value if obj is considered as close
#define obj_max 30		//value if obj is farther than detectable range by prox sensor
#define obj_min 1000	//value if obj is closer than detectable range by prox sensor
#define wait 50			//duration for which to pause the robot
#define dist_min 15		//value if obj is closer than detectable range by distance sensor
#define dist_max 500	//value if obj is farther than detectable range by distance sensor

// defining inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int prox_cal[8]={0,0,0,0,0,0,0,0};	//For storing proximity sensor values
int i=0;
int prox_max=0;			//For dectecting prox sensor with max value
int prox_loc;			//Prox sensor with the max value i.e the sensor to which the obj is closest
//int prox_avg;
int dist;				//Distance measured by the distance sensor
int flag=0;				//If obj is detected by distance sensor
int count=0;			//To prevent continous rotation of robot when no obj is present

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

    //Initialising distance sensor module
    VL53L0X_start();

    // Calibrate proximity sensors
    calibrate_ir();

    // Initialising LEDs
    clear_leds();
    spi_comm_start();

    /* Infinite loop. */
    while (1)
    {
    	chThdSleepMilliseconds(wait);

        //Getting the proximity sensor values
        for(i=0;i<8;++i)
        {
        	prox_cal[i]=get_calibrated_prox(i);
        	if (prox_cal[i]>prox_max)
        	{
        		prox_max = prox_cal[i];
        		prox_loc = i;
        	}
        }
        //prox_avg = (prox_cal[0]+prox_cal[7])/2);
        // If obj is within detectable limit and not at front of robot, rotate robot till obj is at front
        if(((prox_max>obj_max)&&(prox_max<obj_min))&&((prox_loc!=0)&&(prox_loc!=7)));
        {
        	count=0;
        	if(prox_loc<=3)						//If obj at right, rotate right
        	{
        		left_motor_set_speed(250);
        		right_motor_set_speed(-250);
        	}
        	else								//If obj at left, rotate left
        	{
        		left_motor_set_speed(-250);
        		right_motor_set_speed(250);
        	}
        }
        //If obj at front and at close distance, then robot moves back
        if((prox_max>obj_near)&&((prox_loc==0)||(prox_loc==7)))
        {
        	count=0;
        	left_motor_set_speed(-300);
        	right_motor_set_speed(-300);
        }
        //If obj at front and at far distance, then robot moves front
        else if ((prox_max<obj_far)&&((prox_loc==0)||(prox_loc==7)))
        {
        	count=0;
        	left_motor_set_speed(300);
        	right_motor_set_speed(300);
        }
        //If obj is not at detectable range i.e too close or too far, robot uses distance sensor
        else if((prox<obj_max)&&(count<1))
        {
        	i = 0;
        	//rotate robot for approx 15sec to detect obj using distance sensor
        	while((i<300)&&(flag==0))
        	{
        		left_motor_set_speed(250);
        		right_motor_set_speed(-250);
        		dist = VL53L0X_get_dist_mm();
        		if((dist>dist_min)&&(dist<dis_max))
        			flag=1;
        		chThdSleepMilliseconds(wait);
        	}
        	count+=1;
        	if (flag==1)
        	{
        		left_motor_set_speed(500);
        		right_motor_set_speed(500);
        	}
        }
        else
        {
        	left_motor_set_speed(0);
        	right_motor_set_speed(0);
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
