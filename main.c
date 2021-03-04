#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>

// Init function required by __libc_init_array
void _init(void) {}

int main(void)
{
    SystemClock_Config();

    // Enable GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN;// @suppress("Field cannot be resolved")

    // LED used init
    gpio_config_output_opendrain(LED_USED);
    gpio_clear(LED_USED);
    gpio_config_output_opendrain(LED1);
    gpio_clear(LED1);
    gpio_config_output_opendrain(LED3);
    gpio_clear(LED3);
    gpio_config_output_opendrain(LED5);
    gpio_clear(LED5);
    gpio_config_output_opendrain(LED7);
    gpio_clear(LED7);

    //gpio_config_input(GPIOD, 4);


    timer7_start( );
    while (1){}

    /*while (1) {
    	/*gpio_toggle(LED1);

    	for(uint32_t i = 0;i<1000000; ++i){}
    	gpio_toggle(LED3);

    	for(uint32_t i = 0;i<1000000; ++i){}
    	gpio_toggle(LED5);

    	for(uint32_t i = 0;i<1000000; ++i){}
    	gpio_toggle(LED7);

    	gpio_toggle(LED1);
    	gpio_toggle(LED3);
    	gpio_toggle(LED5);
		gpio_toggle(LED7);

    	if (gpio_read_input(GPIOD, 4) == 1)
    	{
    		gpio_toggle(LED1);
    		gpio_toggle(LED3);
    		gpio_toggle(LED5);
    		gpio_toggle(LED7);
    	}
    	for(uint32_t i = 0;i<1000000; ++i){}



    }*/
}
