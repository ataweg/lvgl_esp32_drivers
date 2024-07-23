
#include "st7789v.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include "disp_spi.h"
#include "driver/gpio.h"


/*********************
 *      DEFINES
 *********************/
#define TAG "st7789v"
/**********************
 *      TYPEDEFS
 **********************/

typedef struct {
    uint8_t cmd;        /*!< Command */
    uint8_t databytes;
    uint8_t data[16];   /*!< Data array */
} lcd_init_cmd_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void st7789v_set_orientation(uint8_t orientation);

static void st7789v_send_color(void *data, size_t length);



/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void st7789v_init(void)
{
    lcd_init_cmd_t st7789v_init_cmds[] = {
        // {0xCF,  3,{0x00, 0x83, 0X30}},
        {ST7789V_SWRESET,0 ,{0x80}},
        {ST7789V_SLPOUT, 0, {0x80}},
        {ST7789V_COLMOD, 1, {0x05}},
		{ST7789V_MADCTL, 1, {0x00}},
        {ST7789V_CASET, 4, {0x00, 0x00, 0x00, 0xEF}}, //宽度
        {ST7789V_RASET, 4, {0x00, 0x00, 0x01, 0x3F}},  //高度


        {ST7789V_PORCTRL, 5, {0x0c, 0x0c, 0x00, 0x33, 0x33}},
        {ST7789V_GCTRL,1, {0x75}},
        {ST7789V_VCOMS,1,{0x32}},
        {ST7789V_VDVVRHEN, 1, {0x01}},
        {ST7789V_VRHS, 1, {0x16}},
        {ST7789V_VDVSET, 1, {0x20}},
        {ST7789V_FRCTR2, 1, {0x0F}},
        {ST7789V_PWCTRL1, 2, {0xA4, 0xA1}},
        {0xD6, 1, {0xA1}},
        {ST7789V_VCOMS, 1, {0x3B}},
        {ST7789V_PVGAMCTRL, 4, {0xf0, 0x0b, 0x11, 0x0e, 0x0d, 0x19, 0x36, 0x33, 0x4b, 0x07, 0x14, 0x14, 0x2c, 0x2e}},
        {ST7789V_NVGAMCTRL, 4, {0xf0, 0x0d, 0x12, 0x0b, 0x09, 0x03, 0x32, 0x44, 0x48, 0x39, 0x16, 0x16, 0x2d, 0x30}},


        #if ST7789V_INVERT_COLORS == 1
		{ST7789V_INVON,  0,{0}}, // set inverted mode
        #else
 		{ST7789V_INVOFF, 0,{0}}, // set non-inverted mode
        #endif
        {ST7789V_RGBCTRL, 2,{0x00, 0x1B}},
        {0xF2,1, {0x08}},
        // {ST7789V_GAMSET, 1,{0x01}},

        {ST7789V_NORON, 0,{0x00}}, //s
        {ST7789V_DISPON,0, {0x00}},
        {ST7789V_RAMWR, 0, {0x00}},
        {0, 0xff,{0}},};




    //Initialize non-SPI GPIOs
    esp_rom_gpio_pad_select_gpio(ST7789V_DC);
    gpio_set_direction(ST7789V_DC, GPIO_MODE_OUTPUT);

#if !defined(ST7789V_SOFT_RST)
    esp_rom_gpio_pad_select_gpio(ST7789V_RST);
    gpio_set_direction(ST7789V_RST, GPIO_MODE_OUTPUT);
#endif

    //Reset the display
#if !defined(ST7789V_SOFT_RST)
    gpio_set_level(ST7789V_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(ST7789V_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
#else
    st7789_send_cmd(ST7789V_SWRESET);
#endif

    printf("ST7789V initialization.\n");

    //Send all the commands
    uint16_t cmd = 0;
    while (st7789v_init_cmds[cmd].databytes!=0xff) {
        st7789v_send_cmd(st7789v_init_cmds[cmd].cmd);
        st7789v_send_data(st7789v_init_cmds[cmd].data, st7789v_init_cmds[cmd].databytes&0x1F);
        if (st7789v_init_cmds[cmd].databytes & 0x80) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }

    st7789v_set_orientation(CONFIG_LV_DISPLAY_ORIENTATION);
}

void st7789v_display_on(char on)
{
    if (on) {
        st7789v_send_cmd(ST7789V_DISPON);
    } else {
        st7789v_send_cmd(ST7789V_DISPOFF);
    }
}

void st7789v_flush(lv_display_t * drv, const lv_area_t * area, uint8_t *color_map)
{
    uint8_t data[4] = {0};

    uint16_t offsetx1 = area->x1;
    uint16_t offsetx2 = area->x2;
    uint16_t offsety1 = area->y1;
    uint16_t offsety2 = area->y2;
    offsety1 += 20;
	offsety2 += 20;

    /*Column addresses*/
    st7789v_send_cmd(ST7789V_CASET);
    data[0] = (offsetx1 >> 8) & 0xFF;
    data[1] = offsetx1 & 0xFF;
    data[2] = (offsetx2 >> 8) & 0xFF;
    data[3] = offsetx2 & 0xFF;
    st7789v_send_data(data, 4);

    /*Page addresses*/
    st7789v_send_cmd(ST7789V_RASET);
    data[0] = (offsety1 >> 8) & 0xFF;
    data[1] = offsety1 & 0xFF;
    data[2] = (offsety2 >> 8) & 0xFF;
    data[3] = offsety2 & 0xFF;
    st7789v_send_data(data, 4);

    /*Memory write*/
    st7789v_send_cmd(ST7789V_RAMWR);

    size_t size = (size_t)lv_area_get_width(area) * (size_t)lv_area_get_height(area);

#if defined (ST7789V_COLOR_16_SWAP)
    uint16_t *color_p = (uint16_t *)color_map;
    for (int i = 0; i < size; i++) {
        color_p[i] = (color_p[i] >> 8) | (color_p[i] << 8);
    }
#endif
    st7789v_send_color((void*)color_map, size * 2 );


}


/**********************
 *   STATIC FUNCTIONS
 **********************/
void st7789v_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789V_DC, 0);
    disp_spi_send_data(&cmd, 1);
}

void st7789v_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789V_DC, 1);
    disp_spi_send_data(data, length);
}

static void st7789v_send_color(void * data, size_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789V_DC, 1);
    
    while (length > 0) {
        size_t send_len = length;
        if (send_len > DISP_BUF_SIZE) {
            send_len = DISP_BUF_SIZE;
        }
        disp_spi_send_colors(data, send_len);
        length -= send_len;
        data += send_len;
    }
}

static void st7789v_set_orientation(uint8_t orientation)
{
    // ESP_ASSERT(orientation < 4);

    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    ESP_LOGI(TAG, "Display orientation: %s", orientation_str[orientation]);

    uint8_t data[] =
    {
#if CONFIG_LV_PREDEFINED_DISPLAY_TTGO
	0x60, 0xA0, 0x00, 0xC0
#else
	0xC0, 0x00, 0x60, 0xA0
#endif
    };

    ESP_LOGI(TAG, "0x36 command value: 0x%02X", data[orientation]);

    st7789v_send_cmd(ST7789V_MADCTL);
    st7789v_send_data((void *) &data[orientation], 1);
}