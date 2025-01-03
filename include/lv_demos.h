/**
 * @file lv_demos.h
 *
 */

#ifndef LV_DEMOS_H
#define LV_DEMOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"

#if LV_USE_DEMO_WIDGETS
#include "lv_demo_widgets.h"
#endif

#if LV_USE_DEMO_BENCHMARK
//#include "../.pio/libdeps/esp32-s3-devkitc-1/lvgl/demos/benchmark/lv_demo_benchmark.h"
#include "demos/benchmark/lv_demo_benchmark.h"
#endif

#if LV_USE_DEMO_STRESS
#include "lv_demo_stress.h"
#endif

#if LV_USE_DEMO_KEYPAD_AND_ENCODER
#include "keypad_encoder/lv_demo_keypad_encoder.h"
#endif

#if LV_USE_DEMO_MUSIC
#include "music/lv_demo_music.h"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/


/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_DEMO_H*/
