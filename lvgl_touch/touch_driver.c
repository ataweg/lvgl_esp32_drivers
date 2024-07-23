/**
 * @file touch_driver.c
 */

#include "touch_driver.h"
#include "esp_log.h"
#include "lvgl_i2c/i2c_manager.h"
#include "tp_spi.h"

// Is not being included in CMakeLists.txt (Research why)
#include "l58.h"

void touch_driver_init(void)
{
//    lvgl_i2c_init(CONFIG_LV_I2C_TOUCH_PORT);
#if defined (CONFIG_LV_TOUCH_CONTROLLER_XPT2046)
    xpt2046_init();
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_FT6X06)
    ft6x06_init(FT6236_I2C_SLAVE_ADDR);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_L58)
    l58_init();
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_STMPE610)
    stmpe610_init();
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_ADCRAW)
    adcraw_init();
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_FT81X)
    /* nothing to do */
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_RA8875)
    ra8875_touch_init();
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_GT911)
    gt911_init(GT911_I2C_SLAVE_ADDR);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_CST816T)
    cst816t_init(CST816T_I2C_SLAVE_ADDR);
#endif
}

#if LVGL_VERSION_MAJOR >= 8
void touch_driver_read(lv_indev_t *drv, lv_indev_data_t *data)
#else
bool touch_driver_read(lv_indev_t *drv, lv_indev_data_t *data)
#endif
{
    bool res = false;

#if defined (CONFIG_LV_TOUCH_CONTROLLER_XPT2046)
    res = xpt2046_read(drv, data);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_FT6X06)
    res = ft6x36_read(drv, data);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_L58)
    res = l58_read(drv, data);

#elif defined (CONFIG_LV_TOUCH_CONTROLLER_STMPE610)
    res = stmpe610_read(drv, data);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_ADCRAW)
    res = adcraw_read(drv, data);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_FT81X)
    res = FT81x_read(drv, data);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_RA8875)
    res = ra8875_touch_read(drv, data);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_GT911)
    res = gt911_read(drv, data);
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_CST816T)
    res = cst816t_read(drv, data);
#endif

#if LVGL_VERSION_MAJOR >= 8
    data->continue_reading = res;
#else
    return res;
#endif
}

void touch_driver_set_calibrate(touch_calibration_t *cb)
{
#if defined (CONFIG_LV_TOUCH_CONTROLLER_XPT2046)
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_FT6X06)
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_STMPE610)
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_ADCRAW)
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_FT81X)
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_RA8875)
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_GT911)
#elif defined (CONFIG_LV_TOUCH_CONTROLLER_CST816T)
    cst816_t_set_calibrate_data(cb->x_start, cb->x_end, cb->y_start, cb->y_end);
#endif
}

