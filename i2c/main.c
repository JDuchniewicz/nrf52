/**
 * Copyright (c) 2009 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
#include "bmi160.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define I2C_INSTANCE    0 // I2C instance index

#define I2C_SCL_PIN     27
#define I2C_SDA_PIN     26

static const nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(I2C_INSTANCE);

static uint8_t I2C_RX_Buffer[20]; // buffer for I2C reads

struct bmi160_dev sensor; // instance of bmi160 sensor

/**
 * Function for configuring I2C(TWI) communication
 */
void i2c_config(void)
{
    uint32_t err_code;

    // use the default config
    const nrf_drv_twi_config_t i2c_config = {
        .scl                = I2C_SCL_PIN,
        .sda                = I2C_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false,
    };

    err_code = nrf_drv_twi_init(&i2c, &i2c_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&i2c); //turn on the i2c bus
}

/**
 * Helper function for calling I2C with proper parameters
 */
void bmi160_i2c_write(uint8_t hw_addr, uint8_t* reg_data, uint16_t len, bool no_stop)
{
    nrf_drv_twi_tx(&i2c, hw_addr, reg_data, len, no_stop);
}

void bmi160_i2c_read(uint8_t hw_addr, uint8_t* reg_data, uint16_t len)
{
    nrf_drv_twi_rx(&i2c, hw_addr, reg_data, len);
}

/**
 * Function for reading via I2C(TWI) into a data buffer provided by caller - called by the BMI160 sensor with a particular format
 */
int8_t bmi160_i2c_read_reg(uint8_t hw_addr, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
    bmi160_i2c_write(hw_addr, &reg_addr, 1, true); // we use the blocking mode for I2C (non-blocking seems to be not as simple as with SPI)
    bmi160_i2c_read(hw_addr, reg_data, len);
    return 0;
}

/**
 * Function for writing via I2C(TWI) from a data buffer provided by caller - called by the BMI160 sensor with a particular format
 */
int8_t bmi160_i2c_write_reg(uint8_t hw_addr, uint8_t reg_addr, uint8_t* reg_data, uint16_t len)
{
    memset(I2C_RX_Buffer, 0, 20);

    if ((len + 1) > sizeof(I2C_RX_Buffer))
    {
        return BMI160_E_OUT_OF_RANGE;
    }
    memcpy(I2C_RX_Buffer, &reg_addr, 1);
    memcpy(I2C_RX_Buffer + 1, reg_data, len);
    bmi160_i2c_write(hw_addr, I2C_RX_Buffer, len + 1, false);
    return 0;
}

/**
 * Function for configuring BMI160 sensor general settings
 */
int8_t sensor_config(void)
{
    int8_t res = BMI160_OK;

    sensor.id = 0x69;//BMI160_I2C_ADDR; because it SDO is left floating on the sensor
    sensor.interface = BMI160_I2C_INTF;
    sensor.read = bmi160_i2c_read_reg;
    sensor.write = bmi160_i2c_write_reg;
    sensor.delay_ms = nrf_delay_ms;

    res = bmi160_init(&sensor);

    return res;
}

/**
  * Function for configuring BMI160 sensor precise settings
  */
int8_t sensor_accel_gyro_config(void)
{
    int8_t res = BMI160_OK;

    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ; // Output Data Rate
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE; // normal power

    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    res = bmi160_set_sens_conf(&sensor);
    if (res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 failed sensor configuration");
    }
}

/**
 * Function for testing BMI160
 */
void bmi160_self_test(void)
{
    int8_t res = bmi160_perform_self_test(BMI160_GYRO_ONLY, &sensor);

    if (res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 failed gyroscope self test");
    }
    res = bmi160_perform_self_test(BMI160_ACCEL_ONLY, &sensor);

    if (res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 failed acceleration self test");
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    uint8_t bmi_res = BMI160_OK;
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;
    i2c_config();

    bmi_res = sensor_config();

    if (bmi_res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 not initialized");
    }

    bmi160_self_test(); // SELF-TEST resets configuration!!!

    sensor_accel_gyro_config();

    while (true)
    {
        bmi_res = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
        if (bmi_res != BMI160_OK)
        {
            NRF_LOG_ERROR("Could not get sensor data");
        }
        NRF_LOG_ERROR("BMI160 accel x: %d y: %d z: %d", accel.x, accel.y, accel.z);
        NRF_LOG_ERROR("BMI160 gyro x: %d y: %d z: %d", gyro.x, gyro.y, gyro.z);
        nrf_delay_ms(200); // poll the driver every 0.2s
    }
}
/** @} */
