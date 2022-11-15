/**
 * @file st7796_parallel.h
 */

#ifndef ST7796_PARALLEL_H
#define ST7796_PARALLEL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <stdint.h>

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include "../lvgl_helpers.h"

/*********************
 *      DEFINES
 *********************/

  /**********************
 *      TYPEDEFS
 **********************/

  /**********************
 * GLOBAL PROTOTYPES
 **********************/

  void st7796s_init(void);
  void st7796s_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);

  /**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ST7796_PARALLEL_H*/
