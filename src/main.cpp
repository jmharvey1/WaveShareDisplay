/** JMH 20240626
 * This code/file is an adaptation of the rgb_lcd_example_main.c found in the
 * ESP32-S3-Touch-LCD-7_Code.zip file @
 * /ESP32-S3-Touch-LCD-7_Code/ESP-IDF-5.3.0/lvgl_Porting/main/
 * The main idea behind this example is to strip the "setup" down to just the basics
 * needed to get the Waveshare display (ST7262 & GT911)running on ESPIDF 5.2.1 using the lvlg 8.3.8
 * library
 * note: in lv_conf.h set #define LV_TICK_CUSTOM 0
 */

#include <lvgl.h>
#include "lv_conf.h"
#include <lvgl_port_v8.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lv_demo_widgets.h"
#include "lv_example_widgets.h"

#include <stdio.h>
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "lvgl.h"
#include "hal/lv_hal_tick.h"
#include "demos/lv_demos.h"
// #include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "touch/base/esp_lcd_touch_gt911.h"

#define LV_TICK_PERIOD_MS (2)

#define CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM 1

#define I2C_MASTER_SCL_IO 9 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 8 /*!< GPIO number used for I2C master data  */

static const char *TAG = "example";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (18 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT -1
#define EXAMPLE_PIN_NUM_HSYNC 46
#define EXAMPLE_PIN_NUM_VSYNC 3
#define EXAMPLE_PIN_NUM_DE 5
#define EXAMPLE_PIN_NUM_PCLK 7
#define EXAMPLE_PIN_NUM_DATA0 14  // B3
#define EXAMPLE_PIN_NUM_DATA1 38  // B4
#define EXAMPLE_PIN_NUM_DATA2 18  // B5
#define EXAMPLE_PIN_NUM_DATA3 17  // B6
#define EXAMPLE_PIN_NUM_DATA4 10  // B7
#define EXAMPLE_PIN_NUM_DATA5 39  // G2
#define EXAMPLE_PIN_NUM_DATA6 0   // G3
#define EXAMPLE_PIN_NUM_DATA7 45  // G4
#define EXAMPLE_PIN_NUM_DATA8 48  // G5
#define EXAMPLE_PIN_NUM_DATA9 47  // G6
#define EXAMPLE_PIN_NUM_DATA10 21 // G7
#define EXAMPLE_PIN_NUM_DATA11 1  // R3
#define EXAMPLE_PIN_NUM_DATA12 2  // R4
#define EXAMPLE_PIN_NUM_DATA13 42 // R5
#define EXAMPLE_PIN_NUM_DATA14 41 // R6
#define EXAMPLE_PIN_NUM_DATA15 40 // R7
#define EXAMPLE_PIN_NUM_DISP_EN -1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 800
#define EXAMPLE_LCD_V_RES 480

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB 2
#else
#define EXAMPLE_LCD_NUM_FB 1
#endif // CONFIG_EXAMPLE_DOUBLE_FB

// #define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

using namespace std;
static SemaphoreHandle_t lvgl_mux = NULL;
SemaphoreHandle_t lvgl_semaphore;
/*JMH ADD: next 2 lines*/
i2c_master_bus_handle_t i2c_master_bus_handle = nullptr;
i2c_master_dev_handle_t dev_handle = NULL;
/*JMH Added/moved to support reporting/logging flush callback values*/
int offsetx1 = 0;
int offsetx2 = 0;
int offsety1 = 0;
int offsety2 = 0;
bool FlushFired = false;
// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

/*JMH ADD*/
esp_lcd_touch_config_t lcd_touch_config = {
    .x_max = EXAMPLE_LCD_H_RES,
    .y_max = EXAMPLE_LCD_V_RES,
    .rst_gpio_num = (gpio_num_t)-1,
    .int_gpio_num = (gpio_num_t)-1,
    .levels = {
        .reset = 0,     // 0: low level, 1: high level
        .interrupt = 0, // 0: low level, 1: high level
    },
    .flags = {
        .swap_xy = false,
        .mirror_x = false,
        .mirror_y = false,
    },
    .process_coordinates = NULL,
    .interrupt_callback = NULL,
    .user_data = NULL,
};
static void lvgl_tick(void *arg);
extern void example_lvgl_demo_ui(lv_disp_t *disp);
extern "C"
{
    void app_main();
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    /* We were able to obtain the semaphore and can now access the
    shared resource. */
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE)
    {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}
void set_pixel(int x, int y, lv_color_t *color_p)
{
    ;
}
static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t display_handle = (esp_lcd_panel_handle_t)drv->user_data;
    // int offsetx1 = area->x1;
    // int offsetx2 = area->x2;
    // int offsety1 = area->y1;
    // int offsety2 = area->y2;
    offsetx1 = area->x1;
    offsetx2 = area->x2;
    offsety1 = area->y1;
    offsety2 = area->y2;
    // BaseType_t high_task_awoken = pdFALSE;
    /**
     * @brief Draw bitmap on LCD panel
     *
     * @param[in] panel LCD panel handle, which is created by other factory API like `esp_lcd_new_panel_st7789()`
     * @param[in] x_start Start index on x-axis (x_start included)
     * @param[in] y_start Start index on y-axis (y_start included)
     * @param[in] x_end End index on x-axis (x_end not included)
     * @param[in] y_end End index on y-axis (y_end not included)
     * @param[in] color_data RGB color data that will be dumped to the specific window range
     * @return
     *          - ESP_OK on success
     */
    // bool drawBitmap(uint16_t x_start, uint16_t y_start, uint16_t width, uint16_t height, const uint8_t *color_data);
    esp_lcd_panel_draw_bitmap(display_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

    // pass the draw buffer to the driver
    lv_disp_flush_ready(drv);
    FlushFired = true;

    // #if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    //     xSemaphoreGive(sem_gui_ready);
    //     xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
    // #endif
}

// extern lv_obj_t *scr;
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data((esp_lcd_touch_handle_t)drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates((esp_lcd_touch_handle_t)drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        // if (example_lvgl_lock(-1))
        // {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PR;
        example_lvgl_unlock();
        // }
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
        if (pdTRUE == xSemaphoreTake(lvgl_semaphore, portMAX_DELAY))
        {
            // Lock the mutex due to the LVGL APIs are not thread-safe
            if (example_lvgl_lock(-1))
            {
                // printf("example_lvgl_port_task\n");
                task_delay_ms = lv_timer_handler();
                // printf("example_lvgl_port_task delay = %d\n", (int)task_delay_ms);
                //  Release the mutex
                example_lvgl_unlock();
            }
            xSemaphoreGive(lvgl_semaphore);
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
    }
}

/**
 * @brief i2c master initialization
 * using new/current ESPRESSIF I2C ESPIDF 5.2 I2C protocol
 */
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 64,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_master_bus_handle));

    i2c_device_config_t lcd_touch_panel_io_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = (0x5D), // GT911 chip address
        .scl_speed_hz = 400000,
    };
    /*JMH ADD*/
    ESP_LOGD(TAG, "Create I2C BUS");
    /*OR maybe this way, based on ESPRESSIF ESPIDF Programming Guide*/
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus_handle, &lcd_touch_panel_io_config, &dev_handle));

    /*JMH ADD*/
    // host.addHostI2C(lcd_touch_config, ((i2c_port_t)(0)));

    // ESP_LOGD(TAG, "Create touch device");
    // ESP_LOGD(TAG, "Initialize host");
    // if(host.begin()){
    //     return ESP_OK;
    // }else
    // {
    //     printf("Initialize host failed\n");
    //     return ESP_FAIL;
    // }
    return ESP_OK;
    /*JMH Commented out*/
    // return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf;      // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;           // contains callback functions
    esp_log_level_set("*", ESP_LOG_VERBOSE); // ESP_LOG_NONE,       /*!< No log output */
                                             // ESP_LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
                                             // ESP_LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
                                             // ESP_LOG_INFO,       /*!< Information messages which describe normal flow of events */
                                             // ESP_LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
                                             // ESP_LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Install RGB LCD Display driver");
    esp_lcd_panel_handle_t display_handle = NULL;

    esp_lcd_rgb_panel_config_t display_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = (14 * 1000 * 1000),
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            .hsync_pulse_width = 4,
            .hsync_back_porch = 8,
            .hsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .vsync_back_porch = 8,
            .vsync_front_porch = 8,
            .flags = {
                .hsync_idle_low = 0,
                .vsync_idle_low = 0,
                .de_idle_high = 0,
                .pclk_active_neg = 0,
                .pclk_idle_high = 0,
            },
        },
        .data_width = 16,                                  // 8 | 16
        .bits_per_pixel = 16,                              // 24 | 16
        .num_fbs = 1,                                      // 1/2/3
        .bounce_buffer_size_px = (EXAMPLE_LCD_H_RES * 10), // Bounce buffer size in bytes. This function is used to avoid screen drift.
        .sram_trans_align = 4,
        .psram_trans_align = 64,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .disp_gpio_num = -1, // not used
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .flags = {
            .disp_active_low = 0,
            .refresh_on_demand = 0,
            .fb_in_psram = 1,
            .double_fb = 0,
            .no_fb = 0,
            .bb_invalidate_cache = 0,
        },
    };
    /*Register display & get a pointer/handle to it*/
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&display_config, &display_handle));
    ESP_LOGI(TAG, "Register Display event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(display_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD Display");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(display_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(display_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_ERROR_CHECK(i2c_master_init()); // setup I2C link to GT911 touch sensor
    ESP_LOGI(TAG, "I2C Master initialized successfully");
    /*JMH - For GT911 chip, the following added nothing to the touch process*/
    // uint8_t write_buf = 0x01;
    // //i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    // ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    // //Reset the touch screen. It is recommended that you reset the touch screen before using it.
    // write_buf = 0x2C;
    // //i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    // ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    // esp_rom_delay_us(400 * 1000);
    // write_buf = 0x2E;
    // //i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    // ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

    ESP_LOGI(TAG, "Initialize GT911 I2C touch sensor");
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG(); // defined in esp_lcd_touch_gt911.h

    /* instantuate touch object */
    if (esp_lcd_new_panel_io_i2c_v2(i2c_master_bus_handle, &tp_io_config, &tp_io_handle) != ESP_OK)
    {
        printf("failed to create GT911 touch interface\n");
    }

    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize/configure GT911 touch controller");
    // ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));//
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &lcd_touch_config, &tp));
    lvgl_semaphore = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    void *buf1 = NULL; // will be used by lvgl as a staging area (memory space) to create display images/maps before posting to display ST7262
    void *buf2 = NULL; // same as above, is the 2nd of two buffer spaces
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(display_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    int buffer_size = (LVGL_PORT_DISP_WIDTH) * (LVGL_PORT_DISP_HEIGHT);
    /*JMH commented out & replaced w/ the following: 
    Note: inspite of above config setting we are actually using two buffers*/
    // buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    buf1 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM); // MALLOC_CAP_DMA
    assert(buf1);
    buf2 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM); // MALLOC_CAP_DMA
    assert(buf2);

    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, buffer_size); // EXAMPLE_LCD_H_RES * 100
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = display_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick Interface for LVGL using esp_timer to generate 2ms periodic event
    const esp_timer_create_args_t lvgl_tick_timer_args =
        {
            .callback = &lvgl_tick,
            .name = "lvgl_tick",
            .skip_unhandled_events = true};
    esp_timer_handle_t lvgl_tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LV_TICK_PERIOD_MS * 1000)); // here time is in micro seconds
                                                                                          // Tick Interface for LVGL using esp_timer to generate 2ms periodic event

    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    /*Set project/application specific parameters*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    /* This not only creates the task, but it also starts it*/
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Scatter Chart");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1))
    {
        example_lvgl_demo_ui(disp);//moved the demos/examples to here
        // lv_example_btn_1();
        // lv_demo_widgets();
        // lv_demo_benchmark();
        // lv_demo_music();
        // lv_demo_stress();
        // Release the mutex
        example_lvgl_unlock();
    }

    static const char *TAG = "lvgl_flush_cb";
    /*Set to '1' if you want to keep doing some other activity*/
    while (0)
    {
        /*A way to show what buffer setting were in play during last 'flush' event*/
        if (FlushFired)
        {
            FlushFired = false;
            /* uncomment for test/debug */
            // printf("offsetx1:%d; offsetx2:%d; offsety1:%d; offsety2:%d\n", offsetx1, offsetx2, offsety1, offsety2);
            // ESP_LOGI(TAG, "offsetx1:%d; offsetx2:%d; offsety1:%d; offsety2:%d\n", offsetx1, offsetx2, offsety1, offsety2);
        }
        // printf("Another Process\n");
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void example_lvgl_demo_ui(lv_disp_t *disp)
{
    //lv_example_textarea_1();
    //lv_example_textarea_3();
    // lv_example_btn_1();
    lv_demo_widgets();
}
/*JMH not needed, when in lv_conf.h, #define LV_TICK_CUSTOM 1 .
 *Use a custom tick source that tells the elapsed time in milliseconds.
 *
 * @brief LVGL Tick Function Hook
 *        LVGL need to call function lv_tick_inc periodically @ LV_TICK_PERIOD_MS
 *        to keep timing information.
 * @param arg
 */
static void lvgl_tick(void *arg)
{
    (void)arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}
