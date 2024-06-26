//#Configure the following LVGL items for improving frame rate (LVGL v8.3):
//#define LV_MEM_CUSTOM 1 //or CONFIG_LV_MEM_CUSTOM=y
//#define LV_MEMCPY_MEMSET_STD 1 //or CONFIG_LV_MEMCPY_MEMSET_STD=y
//#define LV_ATTRIBUTE_FAST_MEM IRAM_ATTR //or CONFIG_LV_ATTRIBUTE_FAST_MEM=y

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lvgl.h"
#include "TFT_eSPI.h"
#include "lv_conf.h"
#include "examples/lv_examples.h"
#include "examples/get_started/lv_example_get_started.h"
#include "ESP_Panel_Board_Internal.h"
//#include "ESP32_IO_Expander"
#if LV_BUILD_EXAMPLES && LV_USE_BTN

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}

/**
 * Create a button with a label and react on click event.
 */
void lv_example_get_started_1(void)
{
    lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/
    lv_obj_center(label);
}

#endif

//#include "examples/get_started/lv_example_get_started.h"
extern "C"
{
  void app_main();
}

void app_main() 
{
    lv_init();
     /* Initialize SPI or I2C bus used by the drivers */
    //lvgl_driver_init();
    vTaskDelay(1); // give the watchdogtimer a chance to reset
    printf("HELLO WORLD\n");
    lv_example_get_started_1();
  while (true)
  {  
    vTaskDelay(240);
  }

} /*End Main loop*/