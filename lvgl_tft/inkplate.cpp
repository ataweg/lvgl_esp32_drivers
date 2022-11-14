#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "inkplate.h"

// NOTE: This needs Epdiy component https://github.com/vroland/epdiy
// Run idf.py menuconfig-> Component Config -> E-Paper driver and select:
// Display type: LILIGO 4.7 ED047TC1
// Board: LILIGO T5-4.7 Epaper
// In the same section Component Config -> ESP32 Specifics -> Enable PSRAM
//#include "parallel/ED047TC1.h"
//Ed047TC1 display;

// SPI Generic epapers (Goodisplay/ Waveshare)
// Select the right class for your SPI epaper: https://github.com/martinberlin/cale-idf/wiki
//#include <gdew0583t7.h>
#include "inkplate.hpp"            //Include Inkplate library to the sketch

Inkplate display(DisplayMode::INKPLATE_1BIT); // Create an object on Inkplate library and also set library into 1-bit mode (BW)
//Gdew0583T7 display(io);

/** test Display dimensions
 * Do not forget to set: menuconfig -> Components -> LVGL configuration
 * Max. Horizontal resolution 264 -> WIDTH of your epaper
 * Max. Vertical resolution   176 -> HEIGHT
 */

/*********************
 *      DEFINES
 *********************/
#define TAG "Main"

uint16_t flushcalls = 0;

/* Display initialization routine */
void inkplate_init(void)
{
    printf("Inkplate_init\n");
    display.begin();
    display.setRotation(0);
    // Clear screen
    display.clearDisplay();
}

/* Required by LVGL */
void inkplate_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    ++flushcalls;
    printf("flush %d x:%d y:%d w:%d h:%d\n", flushcalls,area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area));

    // Full update
    if (lv_area_get_width(area)==display.width() && lv_area_get_height(area)==display.height()) {
      display.partialUpdate();
    } else {
      // Partial update:
      display.partialUpdate(); // Uncomment to disable partial update
      //display.updateWindow(area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area), true);
      //display.updateWindow(area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area));
    }

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing */
    lv_disp_flush_ready(drv);
}

/* Called for each pixel */
void inkplate_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t* buf,
    lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa)
{
    //If not drawing anything: Debug to see if this function is called:
    //printf("set_px %d %d\n",(int16_t)x,(int16_t)y);

    // Test using RGB232
    int16_t epd_color = WHITE;

    // Color setting use: RGB232
    // Only monochrome:All what is not white, turn black
    if ((int16_t)color.full<254) {
        epd_color = BLACK;
    }
    display.drawPixel((int16_t)x, (int16_t)y, epd_color);
}
