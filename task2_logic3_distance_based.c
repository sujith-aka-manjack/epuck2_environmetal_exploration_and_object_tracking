/* Code made by Sujith Kurian James for epuck lab Task 2
   cop20skj
   Group 2 Team A 
   Logic to make epuck2 robot follow an object while maintaining a constant distance with the
   Uses both proximity sensor and IR distance sensor to detect object and IR sensor alone to maintain distance and uses IR distance sensor to detect object if its not detected by promixity sensors */

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

#define obj_far 100  	//value if obj is considered as far
#define obj_near 250	//value if obj is considered as close
#define obj_max 30		//value if obj is farther than detectable range by prox sensor
#define obj_min 3250	//value if obj is closer than detectable range by prox sensor
#define wait 50			//duration for which to pause the robot
#define dist_min 20		//value if obj is closer than detectable range by distance sensor
#define dist_max 400	//value if obj is farther than detectable range by distance sensor

// defining inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int prox_cal[8]={0,0,0,0,0,0,0,0};	//For storing proximity sensor values
int i,j;
int prox_max;			//For dectecting prox sensor with max value
int prox_loc;			//Prox sensor with the max value i.e the sensor to which the obj is closest
//int prox_avg;
int dist;				//Distance measured by the distance sensor
int flag=0;				//If obj is detected by distance sensor
int count=0;			//To prevent continous rotation of robot when no obj is present
int prox_avg=0;
int dist_avg=0;

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
    	prox_max = 0;
    	prox_avg = 0;
        //Getting the proximity sensor values
        for(i=0;i<8;++i)
        {
        	prox_avg=0;
        	for(j=0;j<50;++j)		//Taking 50 measurements and taking average to reduce error
        	{
        		prox_avg = prox_avg+get_calibrated_prox(i);
        		//chThdSleepMilliseconds(1);
        	}
        	prox_cal[i]=prox_avg/50;
        	//prox_cal[i]=get_calibrated_prox(i);
        	if (prox_cal[i]>prox_max)
        	{
        		prox_max = prox_cal[i];
        		prox_loc = i;
        	}
        }
        for(i=0;i<50;++i)		//Taking 50 measurements and taking average to reduce error
        {
        	dist_avg=dist_avg+VL53L0X_get_dist_mm();
        	//chThdSleepMilliseconds(1);
        }
        dist=dist_avg/50;
        // If obj is within detectable limit and not at front of robot, rotate robot till obj is at front
        // LHS or RHS LED will blink red
        if((prox_max>obj_max)&&(dist>100))
        {
        	count=0;
        	if(prox_loc<=3)						//If obj at right, rotate right
        	{
        		clear_leds();
        		set_led(LED3,2);
        		left_motor_set_speed(300);
        		right_motor_set_speed(-300);
        	}
        	else								//If obj at left, rotate left
        	{
        		clear_leds();
        		set_led(LED7,2);
        		left_motor_set_speed(-300);
        		right_motor_set_speed(300);
        	}
        }
        //If obj at front and at close distance, then the robot moves back
        //RGB LEDs at back shows red
        else if(dist<30)
        {
        	count=0;
        	clear_leds();
        	set_rgb_led(LED4,10,0,0);
        	set_rgb_led(LED6,10,0,0);
        	left_motor_set_speed(-500);
        	right_motor_set_speed(-500);
        }
        //If obj at front and at far distance, then the robot moves front
        //RGB LEDs at front shows red
        else if (dist>40)&&(dist<dist_max)
        {
        	count=0;
        	clear_leds();
        	set_rgb_led(LED2,10,0,0);
        	set_rgb_led(LED8,10,0,0);
        	left_motor_set_speed(500);
        	right_motor_set_speed(500);
        }
        //If obj is not at detectable range i.e too far, robot uses distance sensor
        else if((prox_max<obj_max)&&(count<1))
        {
        	//rotate robot for approx 15sec to detect obj using distance sensor
        	//All RGB LEDs show blue
        	for(i=0;((i<300)&&(flag==0));++i)
        	{
        		clear_leds();
        		set_rgb_led(LED2,0,0,10);
        		set_rgb_led(LED4,0,0,10);
        		set_rgb_led(LED6,0,0,10);
        		set_rgb_led(LED8,0,0,10);
        		left_motor_set_speed(250);
        		right_motor_set_speed(-250);
        		dist = VL53L0X_get_dist_mm();
        		chThdSleepMilliseconds(wait);
        		if((dist>dist_min)&&(dist<dist_max))
        		{	flag=1;	}
        	}
        	if(flag==0)
        		count=count+1;
        	//If robot detects obj and while its in range,  robot moves forward
        	//All RGB LEDs shows green
        	else
        	{
        		while((dist>dist_min)&&(dist<dist_max))
        		{
        			clear_leds();
        			set_rgb_led(LED2,0,10,0);
        			set_rgb_led(LED4,0,10,0);
        			set_rgb_led(LED6,0,10,0);
        			set_rgb_led(LED8,0,10,0);
        			left_motor_set_speed(500);
        			right_motor_set_speed(500);
        			dist = VL53L0X_get_dist_mm();
        			chThdSleepMilliseconds(wait);
        		}
        	}
        }
        //If obj is in the proper position, robot stays still
        //All LEDs OFF
        else
        {
        	clear_leds();
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
