/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a ADXL inertial measurement unit.
*/

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "adxl345.h"

static const char *TAG = "example";
static const char *ACC_DATA_TAG = "ADXL";

#define I2C_MASTER_SCL_IO           9       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define ADXL_SENSOR_ADDR         0x53        /*!< Address of the ADXL sensor */
#define ADXL_WHO_AM_I_REG_ADDR   0x00        /*!< Register addresses of the "who am I" register */
#define ADXL_PWR_MGMT_1_REG_ADDR 0x2D        /*!< Register addresses of the power management register */

union u_data{
    uint8_t data_8b[2];
    uint16_t data_16b;
} u_data_x, u_data_y, u_data_z;

struct imu_data{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

struct imu_data raw_data;
struct Vector norm_data;
/**
 * @brief Read a sequence of bytes from a ADXL sensor registers
 */
static esp_err_t adxl_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a ADXL sensor register
 */
static esp_err_t adxl_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static void adxl345_init(i2c_master_dev_handle_t dev_handle)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, ADXL345_REG_DEVID, data, 1));
    ESP_LOGI(ACC_DATA_TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by resetting the ADXL */
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, ADXL345_REG_POWER_CTL, data, 1)); 
    ESP_LOGI(ACC_DATA_TAG, "PWR REG READ: %X", data[0]);
    ESP_ERROR_CHECK(adxl_register_write_byte(dev_handle, ADXL345_REG_POWER_CTL, 0x8));
    ESP_LOGI(ACC_DATA_TAG, "Init ADXL345 Done");
}

static void adxl345_set_range(i2c_master_dev_handle_t dev_handle, adxl345_range_t range)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, ADXL345_REG_DATA_FORMAT, data, 1));
    data[0] &= 0xF0;
    data[0] |= range;
    data[0] |= 0x08; 
}

static void adxl345_set_datarate(i2c_master_dev_handle_t dev_handle, adxl345_dataRate_t datarate)
{
    adxl_register_write_byte(dev_handle, ADXL345_REG_BW_RATE, datarate);
}

static void adxl345_clear_settings(i2c_master_dev_handle_t dev_handle)
{
    adxl345_set_range(dev_handle, ADXL345_RANGE_2G);
    adxl345_set_datarate(dev_handle, ADXL345_DATARATE_100HZ);

    adxl_register_write_byte(dev_handle, ADXL345_REG_THRESH_TAP, 0x00);
    adxl_register_write_byte(dev_handle, ADXL345_REG_DUR, 0x00);
    adxl_register_write_byte(dev_handle, ADXL345_REG_LATENT, 0x00);
    adxl_register_write_byte(dev_handle, ADXL345_REG_WINDOW, 0x00);
    adxl_register_write_byte(dev_handle, ADXL345_REG_THRESH_ACT, 0x00);
    adxl_register_write_byte(dev_handle, ADXL345_REG_THRESH_INACT, 0x00);
    adxl_register_write_byte(dev_handle, ADXL345_REG_TIME_INACT, 0x00);
    adxl_register_write_byte(dev_handle, ADXL345_REG_THRESH_FF, 0x00);
    adxl_register_write_byte(dev_handle, ADXL345_REG_TIME_FF, 0x00);

    uint8_t data[2];

    data[0] = adxl_register_read(dev_handle, ADXL345_REG_ACT_INACT_CTL, data, 1);
    data[0] &= 0b10001000;
    adxl_register_write_byte(dev_handle, ADXL345_REG_ACT_INACT_CTL, data[0]);

    data[0] = adxl_register_read(dev_handle, ADXL345_REG_TAP_AXES, data, 1);
    data[0] &= 0b11111000;
    adxl_register_write_byte(dev_handle, ADXL345_REG_TAP_AXES, data[0]);
}

static struct imu_data adxl345_readraw(i2c_master_dev_handle_t dev_handle)
{
    struct imu_data raw_data;
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, 0x32, &u_data_x.data_8b[0], 1));
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, 0x33, &u_data_x.data_8b[1], 1));
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, 0x34, &u_data_y.data_8b[0], 1));
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, 0x35, &u_data_y.data_8b[1], 1));
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, 0x36, &u_data_z.data_8b[0], 1));
    ESP_ERROR_CHECK(adxl_register_read(dev_handle, 0x37, &u_data_z.data_8b[1], 1));
    raw_data.x =u_data_x.data_16b;
    raw_data.y =u_data_y.data_16b;
    raw_data.z =u_data_z.data_16b;
    return raw_data;
}

static struct Vector adxl345_readnorm(i2c_master_dev_handle_t dev_handle, float gravity_factor)
{
    
    raw_data = adxl345_readraw(dev_handle);

    norm_data.XAxis = raw_data.x * 0.004 * gravity_factor;
    norm_data.YAxis = raw_data.y * 0.004 * gravity_factor;
    norm_data.ZAxis = raw_data.z * 0.004 * gravity_factor;
    return norm_data;
}


void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    adxl345_init(dev_handle);
    adxl345_clear_settings(dev_handle);
    adxl345_set_range(dev_handle, ADXL345_RANGE_2G);
    ESP_LOGI(TAG, "ADXL345 Init Done");

    while(1)
    {
        struct imu_data imu_data = adxl345_readraw(dev_handle);
        ESP_LOGI(ACC_DATA_TAG, "RAW > X : %X\t\t | Y: %X\t\t | Z:%X\t\t ", imu_data.x, imu_data.y, imu_data.z);
        struct Vector imu_cal_data = adxl345_readnorm(dev_handle, ADXL345_GRAVITY_EARTH);
        ESP_LOGI(ACC_DATA_TAG, "Cal > X : %f\t | Y: %f\t | Z:%f\t ", imu_cal_data.XAxis, imu_cal_data.YAxis, imu_cal_data.ZAxis);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}