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
#include "chprintf.h"
#include "usbcfg.h"

#define obj_true 100  // value if obstacle detected - to be measured
//#define obj_false 10  // value if no obstacle detected - to be measured
//#define obj_mid 55 // value if obstacle is at small distance - to be measured

// defining inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int prox_cal[8]={0,0,0,0,0,0,0,0};

void read_prox()
{
	for(i=0;i<8;++i)
		prox_cal[i]=get_calibrated_prox(i);
	/*prox_cal0 = get_calibrated_prox(0);
	prox_cal1 = get_calibrated_prox(1);
	prox_cal2 = get_calibrated_prox(2);
	prox_cal3 = get_calibrated_prox(3);
	prox_cal4 = get_calibrated_prox(4);
	prox_cal5 = get_calibrated_prox(5);
	prox_cal6 = get_calibrated_prox(6);
	prox_cal7 = get_calibrated_prox(7);	*/
}


void task1()
{
	//int dir;
	int obs_status=0;
	int obs_side=0;
	int obs_value_right=0;
	int obs_value_left=0;

	left_motor_set_speed(300);
	right_motor_set_speed(300);

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
		if(obs_value_right>obs_value_left)
			obs_side = 1;
		else
			obs_side = -1;
		if(obs_side = 1)
		{
			left_motor_set_speed(-150);
			right_motor_set_speed(150);
			//chThdSleepMilliseconds(2500);
		}
		else
		{
			left_motor_set_speed(150);
			right_motor_set_speed(-150);
			//chThdSleepMilliseconds(2500);
		}
	}

}


int main(void)
{
	// Primary initialisations
    halInit();
    chSysInit();
    mpu_init();

    // Initiating inter process communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //clearing leds
    clear_leds();
    spi_comm_start();

    //initialising motors
    motors_init();

    // start proximity measurement module
    proximity_start();

    // Calibrate proximity sensors
    calibrate_ir();

    //initialising didtance sensor module
    VL53L0X_start();

    // initialising USB communication peripheral
    usb_start();

    // initialising variables
    int prox0,prox1,prox2,prox3,prox4,prox5,prox6,prox7;
    int prox_cal0,prox_cal1,prox_cal2,prox_cal3,prox_cal4,prox_cal5,prox_cal6,prox_cal7;
    int prox_light0,prox_light1,prox_light2,prox_light3,prox_light4,prox_light5,prox_light6,prox_light7;
    int dist;

    /* Infinite loop. */
    while (1)
    {
    	//waits 1 second
        //chThdSleepMilliseconds(1000);
        
        if (get_selector() == 0)
        	;
        else if (get_selector() == 1)	// for proximity values
        {
        	prox0 = get_prox(0);
        	prox_cal0 = get_calibrated_prox(0);
        	prox_light0 = get_ambient_light(0);
        	dist = VL53L0X_get_dist_mm();
        	printf("\n\n\n Prox 0 reading: &d",prox0);
        	printf("\n Prox 0 calibrated reading: &d",prox_cal0);
        	printf("\n Prox 0 light reading: &d",prox_light0);
        	printf("\n Distance: %d mm",dist);

        /*	if (SDU1.config->usbp->state == USB_ACTIVE)
        	{
        		chprintf((BaseSequentialStream *)&SDU1, "\n%4d,", prox_values[0]);
        	}	*/
        }
        else if (get_selector() == 2)
        {
        	read_prox();
        	task1();
        }
        else()
        	;
        
		
     /*   set_led(LED1, 1);
        set_led(LED2, 1);
        set_led(LED3, 0);
        set_led(LED4, 0);
        left_motor_set_speed(500);
        right_motor_set_speed(-500);
        int dist = get_prox(0);	*/
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
