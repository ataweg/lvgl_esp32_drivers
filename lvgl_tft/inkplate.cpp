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

/* Display initialization routine */
void inkplate_init(void)
{
    printf("Inkplate_init\n");
    display.begin();
    display.setRotation(0);
    // Clear screen
    display.clearDisplay();
    // Do a partial update and print the text to the screen
    display.display();
}

/* Required by LVGL */
void inkplate_flush(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map)
{
    // Partial update
    display.display(); // Uncomment to disable partial update
    display.partialUpdate();
    printf("Updating screen \n");

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing */
    lv_disp_flush_ready(drv);
}

/* Called for each pixel */
void inkplate_set_px_cb(lv_display_t * disp, uint8_t* buf,
    lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    uint8_t color, lv_opa_t opa)
{
    if ((int16_t)color.full >= 1) {
      display.drawPixel((int16_t)x, (int16_t)y, WHITE);
    }
    else{
      display.drawPixel((int16_t)x, (int16_t)y, BLACK);
    }
}
