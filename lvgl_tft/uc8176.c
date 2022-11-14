/**
    @file uc8176.c
    @brief   GoodDisplay GDEW0154M10 DES e-paper display w/ UltraChip uc8176
    @version 1.0
    @date    2020-08-28
    @author  Jackson Ming Hu <huming2207@gmail.com>


    @section LICENSE

    MIT License

    Copyright (c) 2020 Jackson Ming Hu

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
    sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "disp_spi.h"
#include "disp_driver.h"
#include "uc8176.h"

#define TAG "lv_uc8176"

#define PIN_DC              CONFIG_LV_DISP_PIN_DC
#define PIN_DC_BIT          ((1ULL << (uint8_t)(CONFIG_LV_DISP_PIN_DC)))
#define PIN_RST             CONFIG_LV_DISP_PIN_RST
#define PIN_RST_BIT         ((1ULL << (uint8_t)(CONFIG_LV_DISP_PIN_RST)))
#define PIN_BUSY            CONFIG_LV_DISP_PIN_BUSY
#define PIN_BUSY_BIT        ((1ULL << (uint8_t)(CONFIG_LV_DISP_PIN_BUSY)))
#define EVT_BUSY            (1UL << 0UL)
#define EPD_WIDTH           400
#define EPD_HEIGHT          300
#define EPD_ROW_LEN         (EPD_HEIGHT / 8u)

#define BIT_SET(a, b)       ((a) |= (1U << (b)))
#define BIT_CLEAR(a, b)     ((a) &= ~(1U << (b)))


void SetLut(void);

const unsigned char lut_vcom0[] =
{
    0x00, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x00, 0x0F, 0x0F, 0x00, 0x00, 0x01,
  0x00, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00,

};
const unsigned char lut_ww[] ={
  0x50, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x90, 0x0F, 0x0F, 0x00, 0x00, 0x01,
  0xA0, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

};
const unsigned char lut_bw[] ={
  0x50, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x90, 0x0F, 0x0F, 0x00, 0x00, 0x01,
  0xA0, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

};

const unsigned char lut_bb[] ={
  0xA0, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x90, 0x0F, 0x0F, 0x00, 0x00, 0x01,
  0x50, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

};

const unsigned char lut_wb[] ={
  0x20, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x90, 0x0F, 0x0F, 0x00, 0x00, 0x01,
  0x10, 0x08, 0x08, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

};

/******************************gray*********************************/
//0~3 gray
const unsigned char EPD_4IN2_4Gray_lut_vcom[] =
{
0x00	,0x0A	,0x00	,0x00	,0x00	,0x01,
0x60	,0x14	,0x14	,0x00	,0x00	,0x01,
0x00	,0x14	,0x00	,0x00	,0x00	,0x01,
0x00	,0x13	,0x0A	,0x01	,0x00	,0x01,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00

};
//R21
const unsigned char EPD_4IN2_4Gray_lut_ww[] ={
0x40	,0x0A	,0x00	,0x00	,0x00	,0x01,
0x90	,0x14	,0x14	,0x00	,0x00	,0x01,
0x10	,0x14	,0x0A	,0x00	,0x00	,0x01,
0xA0	,0x13	,0x01	,0x00	,0x00	,0x01,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
};
//R22H	r
const unsigned char EPD_4IN2_4Gray_lut_bw[] ={
0x40	,0x0A	,0x00	,0x00	,0x00	,0x01,
0x90	,0x14	,0x14	,0x00	,0x00	,0x01,
0x00	,0x14	,0x0A	,0x00	,0x00	,0x01,
0x99	,0x0C	,0x01	,0x03	,0x04	,0x01,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
};
//R23H	w
const unsigned char EPD_4IN2_4Gray_lut_wb[] ={
0x40	,0x0A	,0x00	,0x00	,0x00	,0x01,
0x90	,0x14	,0x14	,0x00	,0x00	,0x01,
0x00	,0x14	,0x0A	,0x00	,0x00	,0x01,
0x99	,0x0B	,0x04	,0x04	,0x01	,0x01,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
};
//R24H	b
const unsigned char EPD_4IN2_4Gray_lut_bb[] ={
0x80	,0x0A	,0x00	,0x00	,0x00	,0x01,
0x90	,0x14	,0x14	,0x00	,0x00	,0x01,
0x20	,0x14	,0x0A	,0x00	,0x00	,0x01,
0x50	,0x13	,0x01	,0x00	,0x00	,0x01,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
};

typedef struct
{
    uint8_t cmd;
    uint8_t data[3];
    size_t len;
} uc8176_seq_t;

#define EPD_SEQ_LEN(x) ((sizeof(x) / sizeof(uc8176_seq_t)))

static EventGroupHandle_t uc8176_evts = NULL;

static void IRAM_ATTR uc8176_busy_intr(void *arg)
{
    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xResult = xEventGroupSetBitsFromISR(uc8176_evts, EVT_BUSY, &xHigherPriorityTaskWoken);
    if (xResult == pdPASS) {
        portYIELD_FROM_ISR();
    }
}

static void uc8176_spi_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 0);     // DC = 0 for command
    disp_spi_send_data(&cmd, 1);
}

static void uc8176_spi_send_data(uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);  // DC = 1 for data
    disp_spi_send_data(data, len);
}

static void uc8176_spi_send_data_byte(uint8_t data)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);  // DC = 1 for data
    disp_spi_send_data(&data, 1);
}

static void uc8176_spi_send_fb(uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);   // DC = 1 for data
    disp_spi_send_colors(data, len);
}

static void uc8176_spi_send_seq(const uc8176_seq_t *seq, size_t len)
{
    ESP_LOGD(TAG, "Writing cmd/data sequence, count %u", len);

    if (!seq || len < 1) return;
    for (size_t cmd_idx = 0; cmd_idx < len; cmd_idx++) {
        uc8176_spi_send_cmd(seq[cmd_idx].cmd);
        if (seq[cmd_idx].len > 0) {
            uc8176_spi_send_data((uint8_t *) seq[cmd_idx].data, seq[cmd_idx].len);
        }
    }
}

static esp_err_t uc8176_wait_busy(uint32_t timeout_ms)
{
    uint32_t wait_ticks = (timeout_ms == 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms));
    EventBits_t bits = xEventGroupWaitBits(uc8176_evts,
                                           EVT_BUSY, // Wait for busy bit
                                           pdTRUE, pdTRUE,       // Clear on exit, wait for all
                                           wait_ticks);         // Timeout

    return ((bits & EVT_BUSY) != 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void uc8176_sleep()
{
  uc8176_spi_send_cmd(VCOM_AND_DATA_INTERVAL_SETTING);
  uc8176_spi_send_data_byte(0x17);                       //border floating
  uc8176_spi_send_cmd(VCM_DC_SETTING);          //VCOM to 0V
  uc8176_spi_send_cmd(PANEL_SETTING);
  vTaskDelay(pdMS_TO_TICKS(100)); // At least 10ms, leave 20ms for now just in case...

  uc8176_spi_send_cmd(POWER_SETTING);           //VG&VS to 0V fast
  uc8176_spi_send_data_byte(0x00);
  uc8176_spi_send_data_byte(0x00);
  uc8176_spi_send_data_byte(0x00);
  uc8176_spi_send_data_byte(0x00);
  uc8176_spi_send_data_byte(0x00);
  vTaskDelay(pdMS_TO_TICKS(100)); // At least 10ms, leave 20ms for now just in case...

  uc8176_spi_send_cmd(POWER_OFF);          //power off
          uc8176_wait_busy(0);
  uc8176_spi_send_cmd(DEEP_SLEEP);         //deep sleep
  uc8176_spi_send_data_byte(0xA5);
  vTaskDelay(pdMS_TO_TICKS(1000));
}

static void uc8176_panel_init()
{
    // Hardware reset for 3 times - not sure why but it's from official demo code
    for (uint8_t cnt = 0; cnt < 3; cnt++) {
        gpio_set_level(PIN_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(2)); // At least 10ms, leave 20ms for now just in case...
        gpio_set_level(PIN_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
//Pre power on sequence
      uc8176_spi_send_cmd(0x01);
       uc8176_spi_send_data_byte(0x03);                  // VDS_EN, VDG_EN
       uc8176_spi_send_data_byte(0x00);                  // VCOM_HV, VGHL_LV[1], VGHL_LV[0]
       uc8176_spi_send_data_byte(0x2b);                  // VDH
       uc8176_spi_send_data_byte(0x2b);                  // VDL
       uc8176_spi_send_cmd(0x06);
       uc8176_spi_send_data_byte(0x17);
       uc8176_spi_send_data_byte(0x17);
       uc8176_spi_send_data_byte(0x17);                  //07 0f 17 1f 27 2F 37 2f
       uc8176_spi_send_cmd(0x04);
          uc8176_wait_busy(0);
       uc8176_spi_send_cmd(0x00);
       uc8176_spi_send_data_byte(0xbf);    // KW-BF   KWR-AF  BWROTP 0f

       uc8176_spi_send_cmd(0x30);
       uc8176_spi_send_data_byte(0x3c);        // 3A 100HZ   29 150Hz 39 200HZ  31 171HZ

       uc8176_spi_send_cmd(0x61); // resolution setting
       uc8176_spi_send_data_byte(0x01);
       uc8176_spi_send_data_byte(0x90); //128
       uc8176_spi_send_data_byte(0x01); //
   uc8176_spi_send_data_byte(0x2c);

    uc8176_spi_send_cmd(0x82); // vcom_DC setting
    uc8176_spi_send_data_byte(0x12);

    uc8176_spi_send_cmd(0X50); // VCOM AND DATA INTERVAL SETTING
    uc8176_spi_send_data_byte(0x97); // 97white border 77black border    VBDF 17|D7 VBDW 97 VBDB 57    VBDF F7 VBDW 77 VBDB 37  VBDR B7
    // Panel settings

    uc8176_spi_send_cmd(0x00);
#if defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT_INVERTED)
    uc8176_spi_send_data_byte(0x13);
#elif defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT)
    uc8176_spi_send_data_byte(0x1f);
#endif

    // Power up

    /*
    uc8176_spi_send_cmd(0x04);
    uc8176_wait_busy(0);
    */
    SetLut();
}

void SetLut(void) {
    unsigned int count;
    uc8176_spi_send_cmd(LUT_FOR_VCOM);                            //vcom
    for(count = 0; count < 36; count++) {
        uc8176_spi_send_data_byte(lut_vcom0[count]);
    }

    uc8176_spi_send_cmd(LUT_WHITE_TO_WHITE);                      //ww --
    for(count = 0; count < 36; count++) {
        uc8176_spi_send_data_byte(lut_ww[count]);
    }

    uc8176_spi_send_cmd(LUT_BLACK_TO_WHITE);                      //bw r
    for(count = 0; count < 36; count++) {
        uc8176_spi_send_data_byte(lut_bw[count]);
    }

    uc8176_spi_send_cmd(LUT_WHITE_TO_BLACK);                      //wb w
    for(count = 0; count < 36; count++) {
        uc8176_spi_send_data_byte(lut_bb[count]);
    }

    uc8176_spi_send_cmd(LUT_BLACK_TO_BLACK);                      //bb b
    for(count = 0; count < 36; count++) {
        uc8176_spi_send_data_byte(lut_wb[count]);
    }
}

static void uc8176_full_update(uint8_t *buf)
{
    uc8176_panel_init();

    uint8_t *buf_ptr = buf;
    uint8_t old_data[400] = { 0 };

    // Fill old data
    uc8176_spi_send_cmd(0x10);
    for (size_t h_idx = 0; h_idx < 300; h_idx++) {
        uc8176_spi_send_data(old_data, 400);
    }

    // Fill new data
    uc8176_spi_send_cmd(0x13);
    for (size_t h_idx = 0; h_idx < 300; h_idx++) {
        uc8176_spi_send_data(buf_ptr, 400);
        buf_ptr += 400;
    }

    // Issue refresh
    uc8176_spi_send_cmd(0x12);
    vTaskDelay(pdMS_TO_TICKS(1000));
    uc8176_wait_busy(0);

    uc8176_sleep();
}

void uc8176_lv_fb_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
  uc8176_spi_send_cmd(RESOLUTION_SETTING);
  uc8176_spi_send_data_byte(400 >> 8);
  uc8176_spi_send_data_byte(400 & 0xff);
  uc8176_spi_send_data_byte(300 >> 8);
  uc8176_spi_send_data_byte(300 & 0xff);

  uc8176_spi_send_cmd(DATA_START_TRANSMISSION_1);
  vTaskDelay(pdMS_TO_TICKS(2));
  for(int i = 0; i < 400 / 8 * 300; i++) {
      uc8176_spi_send_data_byte(0xFF);
  }
  vTaskDelay(pdMS_TO_TICKS(2));
  uc8176_spi_send_cmd(DATA_START_TRANSMISSION_2);
  vTaskDelay(pdMS_TO_TICKS(2));

  for(int i = 0; i < 400 / 8 * 300; i++) {
      uc8176_spi_send_data_byte(0xFF);
  }
  vTaskDelay(pdMS_TO_TICKS(2));
  uc8176_spi_send_cmd(DISPLAY_REFRESH);
  vTaskDelay(pdMS_TO_TICKS(100));
  uc8176_wait_busy(0);

  uint8_t *buf = (uint8_t *) color_map;
  uc8176_full_update(buf);

/*
  size_t len = ((area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1)) / 8;

  ESP_LOGD(TAG, "x1: 0x%x, x2: 0x%x, y1: 0x%x, y2: 0x%x", area->x1, area->x2, area->y1, area->y2);
  ESP_LOGD(TAG, "Writing LVGL fb with len: %u", len);

  uint8_t *buf = (uint8_t *) color_map;

  uc8176_full_update(buf);

   //lv_disp_flush_ready(drv);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGD(TAG, "Ready");
    */
}

void uc8176_lv_set_fb_cb(lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                           lv_color_t color, lv_opa_t opa)
{
    uint16_t byte_index = (x >> 3u) + (y * 400);
    uint8_t bit_index = x & 0x07u;

    if (color.full) {
        BIT_SET(buf[byte_index], 7 - bit_index);
    } else {
        ESP_LOGD(TAG, "Clear at x: %u, y: %u", x, y);
        BIT_CLEAR(buf[byte_index], 7 - bit_index);
    }
}

void uc8176_lv_rounder_cb(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    // Always send full framebuffer if it's not in partial mode
    area->x1 = 0;
    area->y1 = 0;
    area->x2 = EPD_WIDTH - 1;
    area->y2 = EPD_HEIGHT - 1;
}

void uc8176_init()
{
    // Initialise event group
    uc8176_evts = xEventGroupCreate();
    if (!uc8176_evts) {
        ESP_LOGE(TAG, "Failed when initialising event group!");
        return;
    }

    // Setup output pins, output (PP)
    gpio_config_t out_io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = PIN_DC_BIT | PIN_RST_BIT,
            .pull_down_en = 0,
            .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&out_io_conf));

    // Setup input pin, pull-up, input
    gpio_config_t in_io_conf = {
            .intr_type = GPIO_INTR_POSEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = PIN_BUSY_BIT,
            .pull_down_en = 0,
            .pull_up_en = 1,
    };
    ESP_ERROR_CHECK(gpio_config(&in_io_conf));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BUSY, uc8176_busy_intr, (void *) PIN_BUSY);

    ESP_LOGI(TAG, "Panel init started");
    uc8176_panel_init();
    ESP_LOGI(TAG, "Panel initialised");
}
