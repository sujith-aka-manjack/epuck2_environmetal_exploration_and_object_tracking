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
#include "epuck1x/uart/e_uart_char.h"
#include "serial_comm.h"

// defining inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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

    //initialising distance sensor module
    VL53L0X_start();

    // initialising Bluetooth communication peripheral
    serial_start();

    // initialising variables
    int prox0,prox1,prox6,prox7;
    int prox_cal0,prox_cal1,prox_cal6,prox_cal7;
    int prox_light0,prox_light1,prox_light6,prox_light7;
    int dist;

    /* Infinite loop. */
    while (1)
    {
    	//waits 1 second
        //chThdSleepMilliseconds(1000);
        

        	prox0 = get_prox(0);
        	prox_cal0 = get_calibrated_prox(0);
        	prox_light0 = get_ambient_light(0);
        	dist = VL53L0X_get_dist_mm();

        	char str[100];
        	int ln1,ln2,ln3,ln4,ln5,selec;
        	ln1=sprintf(str,"prox0 value: %d\n",prox0);
        	e_send_uart_char(str,ln1);
        	ln2=sprintf(str,"prox_cal0 value: %d\n",prox_cal0);
        	e_send_uart_char(str,ln2);
        	ln3=sprintf(str,"prox_light0 value: %d\n",prox_light0);
        	e_send_uart_char(str,ln3);
        	ln1=sprintf(str,"dist value: %d\n",dist);
        	e_send_uart_char(str,ln4);
        	selec = get_selector();
        	ln5=sprintf(str,"Selec value: %d\n",selec);
        	e_send_uart_char(str,ln5);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
