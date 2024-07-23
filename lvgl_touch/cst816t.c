#include "cst816t.h"


#include <esp_log.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include <lvgl.h>
#else
#include <lvgl/lvgl.h>
#endif

#include "../lvgl_helpers.h"
#include "lvgl_i2c/i2c_manager.h"

#define TAG "CST816T"

typedef struct {
    uint32_t x_a;
    uint32_t x_b;
    uint32_t y_a;
    uint32_t y_b;
    uint8_t enabled;
} cst816t_calibrate_t;

static cst816t_calibrate_t cst816t_calibrate_data = {0};

static cst816t_status_t cst816t_status;
uint8_t cst816t_read_len(uint16_t reg_addr,uint8_t *data,uint8_t len)
{
    uint8_t res=0;
    res = i2c_master_write_read_device(CONFIG_LV_I2C_TOUCH_PORT, CST816T_ADDR, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);

    return res;
}


// uint8_t cst816t_register_write_byte(uint8_t reg_addr, uint8_t data)
// {
//     int ret;
//     uint8_t write_buf[2] = {reg_addr, data};

//     ret = i2c_master_write_to_device(I2C_PORT_NUM_TP, CST816T_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);

//     return ret;
// }


uint8_t cst816t_chipId(void)
{

    return 0;
}
static esp_err_t  cst816t_get_touch_points_num(uint8_t *touch_points_num)
{
    uint8_t res=0;
    res = cst816t_read_len(0x02, touch_points_num,1);
    return res;
}

esp_err_t cst816t_read_pos(uint8_t *touch_points_num, uint16_t *x, uint16_t *y)
{
    uint8_t data[4];

    cst816t_get_touch_points_num(touch_points_num);
    if (0 == *touch_points_num)
    {
        *x=0;
        *y=0;
        return 1;
    }else
    {
        cst816t_read_len(0x03, data,4);

        *x = ((data[0] & 0x0f) << 8) | data[1];
        *y = ((data[2] & 0x0f) << 8) | data[3];
        

    }

    return ESP_OK;
}




esp_err_t cst816t_i2c_read(uint8_t slave_addr, uint16_t register_addr, uint8_t *data_buf, uint8_t len) {
    return lvgl_i2c_read(CONFIG_LV_I2C_TOUCH_PORT, slave_addr, register_addr , data_buf, len);
}

esp_err_t cst816t_i2c_write8(uint8_t slave_addr, uint16_t register_addr, uint8_t data) {
    uint8_t buffer = data;
    return lvgl_i2c_write(CONFIG_LV_I2C_TOUCH_PORT, slave_addr, register_addr | I2C_REG_16, &buffer, 1);
}



void cst816t_init(uint16_t dev_addr)
{
    if (!cst816t_status.inited) {
        cst816t_status.i2c_dev_addr = dev_addr;
        uint8_t data_buf[10];
        esp_err_t ret;

        ESP_LOGI(TAG, "Checking for CST816T Touch Controller ");
        // uint8_t buf[10];
        // cst816t_read_len(0x15, buf,1);
        if ((ret = cst816t_i2c_read(dev_addr, 0x15, &data_buf, 1) != ESP_OK)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGE(TAG, "Error reading from device: %s, %d",
                        esp_err_to_name(ret), ret);    // Only show error the first time
            // return;
        }
        if ((ret = cst816t_i2c_read(dev_addr, 0xa7, &data_buf, 1) != ESP_OK)) {
            ESP_LOGE(TAG, "Error reading from device: %s, %d",
                        esp_err_to_name(ret), ret);    // Only show error the first time
            ESP_LOGE(TAG, "device ID: %02x", data_buf[0]);            
            // return;
        }

        // // Read 4 bytes for Product ID in ASCII
        // for (int i = 0; i < GT911_PRODUCT_ID_LEN; i++) {
        //     gt911_i2c_read(dev_addr, (GT911_PRODUCT_ID1 + i), (uint8_t *)&(gt911_status.product_id[i]), 1);
        // }
        // ESP_LOGI(TAG, "\tProduct ID: %s", gt911_status.product_id);

        // gt911_i2c_read(dev_addr, GT911_VENDOR_ID, &data_buf, 1);
        // ESP_LOGI(TAG, "\tVendor ID: 0x%02x", data_buf);

        // gt911_i2c_read(dev_addr, GT911_X_COORD_RES_L, &data_buf, 1);
        // gt911_status.max_x_coord = data_buf;
        // gt911_i2c_read(dev_addr, GT911_X_COORD_RES_H, &data_buf, 1);
        // gt911_status.max_x_coord |= ((uint16_t)data_buf << 8);
        // ESP_LOGI(TAG, "\tX Resolution: %d", gt911_status.max_x_coord);

        // gt911_i2c_read(dev_addr, GT911_Y_COORD_RES_L, &data_buf, 1);
        // gt911_status.max_y_coord = data_buf;
        // gt911_i2c_read(dev_addr, GT911_Y_COORD_RES_H, &data_buf, 1);
        // gt911_status.max_y_coord |= ((uint16_t)data_buf << 8);
        // ESP_LOGI(TAG, "\tY Resolution: %d", gt911_status.max_y_coord);
        // vTaskDelay(pdMS_TO_TICKS(10));
        // cst816t_read_len(0xa7, buf,1);
        // vTaskDelay(pdMS_TO_TICKS(10));
        cst816t_status.inited = true;
    }
}


bool cst816t_read(lv_indev_t *drv, lv_indev_data_t *data) {
    
    uint8_t touch_points_num = 0;
    uint16_t x = 0;
    uint16_t y = 0;

    cst816t_read_pos(&touch_points_num,&x,&y);  

#if CONFIG_LV_CST816T_INVERT_X
    x = LV_HOR_RES_MAX - x;
#endif
#if CONFIG_LV_CST816T_INVERT_Y
    y = LV_VER_RES_MAX - y;
#endif
#if CONFIG_LV_CST816T_SWAPXY
    int16_t swap_buf = x;
    x = y;
    y = swap_buf;
#endif

    if (touch_points_num > 0){
        if (cst816t_calibrate_data.enabled){
            // ESP_LOGI(TAG, "before X=%u Y=%u", (int)x, (int)y);
            data->point.x = (cst816t_calibrate_data.x_a * x + cst816t_calibrate_data.x_b) / 100;
            data->point.y = (cst816t_calibrate_data.y_a * y + cst816t_calibrate_data.y_b) / 100;
        } else {
    data->point.x = x;
    data->point.y = y;
        }
        data->state = LV_INDEV_STATE_PR;
        // ESP_LOGI(TAG, "X=%u Y=%u", (int)data->point.x, (int)data->point.y);
    }
    else{
        data->state = LV_INDEV_STATE_REL;
    }
    // ESP_LOGI(TAG, "X=%u Y=%u", data->point.x, data->point.y);
    // ESP_LOGV(TAG, "X=%u Y=%u", data->point.x, data->point.y);
    return false;
}

void cst816_t_set_calibrate_data(uint32_t x_start, uint32_t x_end, uint32_t y_start, uint32_t y_end)
{
    cst816t_calibrate_data.x_a = LV_HOR_RES_MAX * 100 / (x_end - x_start);
    cst816t_calibrate_data.x_b = LV_HOR_RES_MAX * 100 * x_start / (x_start - x_end);
    cst816t_calibrate_data.y_a = LV_VER_RES_MAX * 100 / (y_end - y_start);
    cst816t_calibrate_data.y_b = LV_VER_RES_MAX * 100 * y_start / (y_start - y_end);
    cst816t_calibrate_data.enabled = 1;
}
