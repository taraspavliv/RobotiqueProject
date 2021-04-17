#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include "camera/po8030.h"
#include <leds.h> //no RGB leds included here

#include <process_image.h>

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 20000);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
    po8030_config(FORMAT_RGB565, SIZE_QVGA);
	po8030_advanced_config(FORMAT_RGB565, 0, 10, 50, 50, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	set_led(3,1);
	systime_t time = 0;
    while(1){
		set_led(3,2);
		chprintf((BaseSequentialStream *)&SD3, "time: %d", chVTGetSystemTime()-time);
		time = chVTGetSystemTime();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 19200);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t main_lines[LINES_TO_ANALYSE][IMAGE_WIDTH] = {0};

	set_led(1, 0);

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		/*for(uint8_t i=0;i<LINES_TO_ANALYSE; ++i){
			for(uint8_t j=0; j < IMAGE_WIDTH; ++j){
				main_lines[i][j] = img_buff_ptr[(150+20*i)*IMAGE_WIDTH + j]>>8; //TODO:check formula
			}
		}*/
		set_led(1, 2);

    }
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
