#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>
#include <motor.h>
#include <selector.h>

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

// Init function required by __libc_init_array
void _init(void) {}

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}


int main(void)
{
	// Enable GPIOD and GPIOE peripheral clock
	    //RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
	RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    SystemClock_Config();

    /*timer4_start();
    gpio_config_output_af_pushpull(GPIOD, 14);
    GPIO_TypeDef* port= GPIOD;
    int pin=14;
    port->MODER  =(port->MODER & ~(3 << (pin * 2))) | (2 << (pin * 2));

   port->AFR[1]= (port->AFR[1]&~(15 << 24)) | (2 << 24);
   */

    motor_init1();
    while (1) {
        
    }
}

