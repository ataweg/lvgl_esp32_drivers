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

EpdSpi IO;
Gdew042t2 display(IO);
//Gdew0583T7 display(io);

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


      display.setTextColor(EPD_BLACK, EPD_WHITE); // Set text color to be black and background color to be white
      display.setTextSize(2);             // Set text to be 4 times bigger than classic 5x7 px text

      // Prepare message text and position
      char message[] = "Init done!";
      int w = display.width();
      int h = display.height();
      int cursor_x = (w / 2) - (strlen(message) / 2);
      int cursor_y = h / 2;

      ESP_LOGI(TAG, "Display size: width: %d, height: %d", w, h);

      // Write text to the center of the screen
      display.setCursor(cursor_x, cursor_y);
      display.print(message);

      // Do a partial update and print the text to the screen
      display.update();

    // Clear screen
    //display.update();
}

/* Required by LVGL */
void calepd_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    ++flushcalls;
    printf("flush %d x:%d y:%d w:%d h:%d\n", flushcalls,area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area));

    // Full update
    if (lv_area_get_width(area)==display.width() && lv_area_get_height(area)==display.height()) {
      display.update();
    } else {
      // Partial update:
      display.update(); // Uncomment to disable partial update
      //display.updateWindow(area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area), true);
      //display.updateWindow(area->x1,area->y1,lv_area_get_width(area),lv_area_get_height(area));
    }

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing */
    lv_disp_flush_ready(drv);
}

/* Called for each pixel */
void calepd_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t* buf,
    lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa)
{
    //If not drawing anything: Debug to see if this function is called:
    //printf("set_px %d %d\n",(int16_t)x,(int16_t)y);

    // Test using RGB232
    int16_t epd_color = EPD_WHITE;

    // Color setting use: RGB232
    // Only monochrome:All what is not white, turn black
    if ((int16_t)color.full<254) {
        epd_color = EPD_BLACK;
    }
    display.drawPixel((int16_t)x, (int16_t)y, epd_color);
}
