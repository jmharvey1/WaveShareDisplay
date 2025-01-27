/**
 * main_20240626.cpp
 * # LVGL Porting Example
 *
 * SPDX-License-Identifier: Apache-2.0
 * JMH 20240626
 * This file was modified to the demo button example code & a version of demo widgets
 * demo that ships with the Waveshare board.
 * 
 * The example demonstrates how to port LVGL (v8.3.x). And for RGB LCD, it can enable the avoid tearing fucntion.
 *
 * important note: in lv_conf.h set #define LV_TICK_CUSTOM 1
 * 
 * ## How to Use
 *
 * To use this example, please firstly install the following dependent libraries:
 *
 * - lvgl (v8.3.x)
 *
 * Then follow the steps below to configure:
 *
 * 1. For **ESP32_Display_Panel**:
 *
 *    - [Configure drivers](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configuring-drivers) if needed.
 *    - If using a supported development board, follow the [steps](https://github.com/esp-arduino-libs/ESP32_Display_Panel#using-supported-development-boards) to configure it.
 *    - If using a custom board, follow the [steps](https://github.com/esp-arduino-libs/ESP32_Display_Panel#using-custom-development-boards) to configure it.
 *
 * 2. Follow the [steps](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configuring-lvgl) to configure the **lvgl**.
 * 3. Modify the macros in the [lvgl_port_v8.h](./lvgl_port_v8.h) file to configure the LVGL porting parameters.
 * 4. Navigate to the `Tools` menu in the Arduino IDE to choose a ESP board and configure its parameters, please refter to [Configuring Supported Development Boards](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configuring-supported-development-boards)
 * 5. Verify and upload the example to your ESP board.
 *
 * ## Serial Output
 *
 * ```bash
 * ...
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

#include <ESP_Panel_Library.h>
#include <lvgl.h>
#include <lv_conf.h>
#include <lvgl_port_v8.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lv_demo_widgets.h"
#include "lv_example_widgets.h"  //needed to run button code



esp_err_t i2c_acquire_bus_handle(int port_num, void *i2c_new_bus, int mode)
{
    printf("i2c_acquire_bus info: port_num %d", port_num);
    return ESP_OK;   
}
extern "C"
{
  void app_main();
}

void app_main() 
{
    std::string title = "LVGL porting example";
    printf("LVGL porting example start\n");
    printf("Initialize panel device\n");
    ESP_Panel *panel = new ESP_Panel();
    panel->init();
#if LVGL_PORT_AVOID_TEAR
    // When avoid tearing function is enabled, configure the RGB bus according to the LVGL configuration
    ESP_PanelBus_RGB *rgb_bus = static_cast<ESP_PanelBus_RGB *>(panel->getLcd()->getBus());
    rgb_bus->configRgbFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
    rgb_bus->configRgbBounceBufferSize(LVGL_PORT_RGB_BOUNCE_BUFFER_SIZE);
#endif
    panel->begin();

    printf("Initialize LVGL\n");
    lvgl_port_init(panel->getLcd(), panel->getTouch());

    printf("Create UI\n");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    /* Create a simple label */
    //lv_obj_t *label = lv_label_create(lv_scr_act());
    //lv_label_set_text(label, title.c_str());
    //lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    /**
     * Try an example. Don't forget to uncomment header.
     * See all the examples online: https://docs.lvgl.io/master/examples.html
     * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples
     */
    // lv_example_btn_1();

    /**
     * Or try out a demo.
     * Don't forget to uncomment header and enable the demos in `lv_conf.h`. E.g. `LV_USE_DEMOS_WIDGETS`
     */
    lv_demo_widgets();
    //lv_demo_benchmark();
    // lv_demo_music();
    // lv_demo_stress();
    //Serial.println(title + " end");
    printf("\nLVGL porting example Setup Complete\n\n");
    /* Release the mutex */
    lvgl_port_unlock();
    int cnt = 0;
    while(0)
    {   
        printf("IDLE loop\n");
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    vTaskDelete(NULL);
}


