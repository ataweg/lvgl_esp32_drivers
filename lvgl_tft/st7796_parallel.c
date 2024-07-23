/**
 * @file st7796_parallel.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "st7796_parallel.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "ST7796_PARALLEL"

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void st7796s_set_orientation(uint8_t orientation);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void st7796s_init(void)
{

}

void st7796s_flush(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map)
{

}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void st7796s_set_orientation(uint8_t orientation)
{

}
