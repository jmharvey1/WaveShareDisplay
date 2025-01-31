/**
 * main_20240626.cpp
 * # LVGL Porting Example
 *
 * SPDX-License-Identifier: Apache-2.0
 * JMH 20240626
 * This file was modified to the demo button example code & a version of demo widgets
 * demo that ships with the Waveshare board.
 * 
 * LVGL porting example start
 * Initialize panel device
 * Initialize LVGL
 * Create UI
 * LVGL porting example end
 * IDLE loop
 * IDLE loop
 * ...
 * ```
 *
 * ## Troubleshooting
 *
 * Please check the [FAQ](https://github.com/esp-arduino-libs/ESP32_Display_Panel#faq) first to see if the same question exists. If not, please create a [Github issue](https://github.com/esp-arduino-libs/ESP32_Display_Panel/issues). We will get back to you as soon as possible.
 *
 */

#include "ESP_Panel_Library.h"
#include "lvgl.h"
#include "lv_conf.h"
#include "lv_port_disp.h"//#include <lvgl_port_v8.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
//#include "lv_demo_widgets.h"
#include "lv_example_widgets.h"  //needed to run button code
#include "demos/widgets/lv_demo_widgets.h"


// esp_err_t i2c_acquire_bus_handle(int port_num, void *i2c_new_bus, int mode)
// {
//     printf("i2c_acquire_bus info: port_num %d", port_num);
//     return ESP_OK;   
// }
extern "C"
{
  void app_main();
}

void app_main() 
{
    std::string title = "LVGL porting example";
    vTaskDelay(pdMS_TO_TICKS(3000));
    printf("LVGL porting example start\n");
    //printf("Initialize panel device\n");
    //ESP_Panel *panel = new ESP_Panel();
    //panel->init();
#if LVGL_PORT_AVOID_TEAR
    // When avoid tearing function is enabled, configure the RGB bus according to the LVGL configuration
    ESP_PanelBus_RGB *rgb_bus = static_cast<ESP_PanelBus_RGB *>(panel->getLcd()->getBus());
    rgb_bus->configRgbFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
    rgb_bus->configRgbBounceBufferSize(LVGL_PORT_RGB_BOUNCE_BUFFER_SIZE);
#endif
    /*Start LVGL v9.2 */
    printf("Start/Initialize LVGL v9.2\n");
    lv_port_disp_init();
    printf("app_main: LVGL task running\n");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);
    /* NOTE: Uncomment only one example at a time */

    /* Example 1 - Create a simple label 
    *  to enable this example uncomment the next 3 lines.
    */
    // lv_obj_t *label = lv_label_create(lv_scr_act());
    // lv_label_set_text(label, title.c_str());
    // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    /**
     * Example 2 - create two button types/behaviors
     * Try an example. Don't forget to uncomment header. lv_conf.h / LV_BUILD_EXAMPLES 1
     * See all the examples online: https://docs.lvgl.io/master/examples.html
     * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples
     */
    //lv_example_btn_1();

    /**
     * Or try out a demo.
     * Don't forget to uncomment header and enable the demos in `lv_conf.h`. E.g. `LV_USE_DEMO_WIDGETS`
     * again, uncomment 1 demo at a time
     */
    lv_demo_widgets();// demo 1
    //lv_demo_benchmark();
    // lv_demo_music();
    // lv_demo_stress();
    //Serial.println(title + " end");
    printf("\napp_main: Create UI Complete\n\n");
    /* Release the mutex */
    lvgl_port_unlock();
    int cnt = 0;
    while(0)
    {   
        printf("IDLE loop\n");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}


