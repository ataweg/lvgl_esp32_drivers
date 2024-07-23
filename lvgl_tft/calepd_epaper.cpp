#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "calepd_epaper.h"

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
#include <gdew042t2.h>
EpdSpi io;
Gdew042t2 display(io);


/** test Display dimensions
 * Do not forget to set: menuconfig -> Components -> LVGL configuration
 * Max. Horizontal resolution 264 -> WIDTH of your epaper
 * Max. Vertical resolution   176 -> HEIGHT
 */

/*********************
 *      DEFINES
 *********************/
#define TAG "EPDIY"

uint16_t flushcalls = 0;

/* Display initialization routine */
void calepd_init(void)
{
    printf("calepd_init\n");
    display.init();
    display.setRotation(0);
    // Clear screen
    display.update();
}

/* Required by LVGL */
void calepd_flush(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map)
{
    ++flushcalls;
    printf("flush %d x:%d y:%d w:%d h:%d\n", flushcalls,area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area));
    printf("w:%d h:%d\n", display.width(),display.height());
    // Full update
    //if (area->x1 == 400 && area->y1 == 300) {
    display.update();
      //lv_disp_flush_ready(drv);
    //} 
    //else {
      // Partial update:
    //  display.update(); // Uncomment to disable partial update
      //display.updateWindow(area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area), true);
      //display.updateWindow(area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area));
    //}
    lv_disp_flush_ready(drv);
    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing */
    //lv_disp_flush_ready(drv);
}

/* Called for each pixel */
void calepd_set_px_cb(lv_display_t * disp, uint8_t* buf,
    lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    uint8_t color, lv_opa_t opa)
{
    //If not drawing anything: Debug to see if this function is called:
    //printf("set_px %d %d\n",(int16_t)x,(int16_t)y);
    
    // Test using RGB232
    //int16_t epd_color = EPD_WHITE;
    //printf("buf_w:%d x:%d y:%d color:%d\n", buf_w, x, y,(int16_t)color.full);
    // Color setting use: RGB232
    // Only monochrome:All what is not white, turn black
   // if ((int16_t)color.full<254) {
   //     epd_color = EPD_BLACK;        
   // }
    if ((int16_t)color.full >= 1) {
      display.drawPixel((int16_t)x, (int16_t)y, EPD_WHITE);
    }
    else{
      display.drawPixel((int16_t)x, (int16_t)y, EPD_BLACK);
    }
}