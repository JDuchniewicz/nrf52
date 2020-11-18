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
#include "nrf_drv_gpiote.h"
#include "app_uart.h"
#include "nrf_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define I2C_INSTANCE    0 // I2C instance index

#define I2C_SCL_PIN     27
#define I2C_SDA_PIN     26
#define IRQ_PIN         19

#define FIFO_LEN        200
#define I2C_BUF_LEN     FIFO_LEN
#define ACCEL_BUF_LEN   (FIFO_LEN / 7) + 1

#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256
#define UART_HWFC        APP_UART_FLOW_CONTROL_DISABLED

static const nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(I2C_INSTANCE);

static uint8_t FIFO_BUFF[FIFO_LEN]; // buffer to store raw FIFO information (temporarily)

static uint8_t I2C_RX_Buffer[I2C_BUF_LEN]; // buffer for I2C communication (has to have size >= of FIFO, used solely for I2C)

struct bmi160_fifo_frame fifo_frame; // modify the FIFO buffer instance and link to device instance

struct bmi160_sensor_data acc_data[ACCEL_BUF_LEN]; // store parsed accelerometer data (this is the buffer which contains proper data)

struct bmi160_dev sensor; // instance of bmi160 sensor

 // forward declare
void get_bmi160_fifo_data(void);

/**
 *  UART error handler
 */
void uart_error_handler(app_uart_evt_t* p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**
 *  Event handler for GPIO IRQ's (can handle just one at a time)
 */
void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch (action)
    {
        case NRF_GPIOTE_POLARITY_LOTOHI:
          {
            uint8_t data = 0x00;
            // disable WM_INT
            //
            //bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, &sensor);
            get_bmi160_fifo_data();
            // enable WM_INT
            data = 0x40;
            //bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, &sensor);
            break;
          }

        default:
            break;
    }
}

/**
 *  Function for configuring GPIOs for IRQs
 */
void gpio_config(void)
{
    uint32_t err_code;
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    const nrf_drv_gpiote_in_config_t config = 
    {
        .sense        = NRF_GPIOTE_POLARITY_LOTOHI,
        .pull         = NRF_GPIO_PIN_NOPULL, // turned off to enable edge detection (can also be left as pullup)
        .is_watcher   = false,
        .hi_accuracy  = false
    };

    err_code = nrf_drv_gpiote_in_init(IRQ_PIN, &config, gpiote_evt_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(IRQ_PIN, true);
}


/**
 * Function for configuring I2C(TWI) communication
 */
void i2c_config(void)
{
    uint32_t err_code;

    // use the default config 
    const nrf_drv_twi_config_t i2c_config = 
    {
        .scl                = I2C_SCL_PIN,
        .sda                = I2C_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
        .clear_bus_init     = false,
    };

    err_code = nrf_drv_twi_init(&i2c, &i2c_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&i2c); //turn on the i2c bus
}

/**
 * Function for getting data out of BMI160 FIFO
 */
void get_bmi160_fifo_data(void)
{
    int8_t res = BMI160_OK;
    uint8_t acc_frames = ACCEL_BUF_LEN;

    res = bmi160_get_fifo_data(&sensor);
    if (res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 failed to get FIFO data");
    }
    res = bmi160_extract_accel(acc_data, &acc_frames, &sensor);
    if (res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 failed to extract FIFO accel data");
    }
    // print the acceloremeter z-axis via UART
    for (uint8_t i = 0; i < acc_frames; ++i)
    {
        printf("%d\r\n", acc_data[i].z);
    }
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
    memset(I2C_RX_Buffer, 0, I2C_BUF_LEN);

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
int8_t bmi160_config(void)
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
  * Function to set BMI160 FIFO settings
  */
int8_t bmi160_fifo_config(void)
{
    int8_t res = BMI160_OK;
  
    fifo_frame.data = FIFO_BUFF;
    fifo_frame.length = FIFO_LEN;
    sensor.fifo = &fifo_frame;

    res = bmi160_set_fifo_config(BMI160_FIFO_ACCEL, BMI160_ENABLE, &sensor);
    if (res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 failed FIFO configuration");
    }
    
    // instance for interrupt settings
    struct bmi160_int_settg int_config;
    int_config.int_channel = BMI160_INT_CHANNEL_1;
    int_config.int_type = BMI160_ACC_GYRO_FIFO_WATERMARK_INT; // choose FIFO watermark IRQ (fires when a certain level of fullness is reached)
    
    res = bmi160_set_fifo_wm((uint8_t) (FIFO_LEN - 20), &sensor);
    if (res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 failed FIFO wm setting");
    }

    int_config.int_pin_settg.output_en = BMI160_ENABLE; // enable output via INT_1 and INT_2 on BMI160
    int_config.int_pin_settg.output_mode = BMI160_DISABLE; // push-pull mode
    int_config.int_pin_settg.output_type = BMI160_ENABLE; // active high
    int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE; // edge-triggered
    int_config.int_pin_settg.input_en = BMI160_DISABLE; // don't act as input
    int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE; // non-latched output (no need to clear manually)
    int_config.fifo_wtm_int_en = BMI160_ENABLE; // enable FIFO WM IRQ

    res = bmi160_set_int_config(&int_config, &sensor);
    {
        NRF_LOG_ERROR("BMI160 failed IRQ config");
    }
}

/**
  * Function for configuring BMI160 sensor precise settings
  */
int8_t bmi160_accel_gyro_config(void)
{
    int8_t res = BMI160_OK;

    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_50HZ; // Output Data Rate
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
    //NRF_LOG_DEFAULT_BACKENDS_INIT();
    uint32_t err_code;

    const app_uart_comm_params_t comms_params =
    {
        .rx_pin_no =    RX_PIN_NUMBER,
        .tx_pin_no =    TX_PIN_NUMBER,
        .rts_pin_no =   RTS_PIN_NUMBER,
        .cts_pin_no =   CTS_PIN_NUMBER,
        .flow_control = UART_HWFC,
        .use_parity =   false,
        .baud_rate =    NRF_UART_BAUDRATE_115200
    };

    APP_UART_FIFO_INIT(&comms_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_error_handler, APP_IRQ_PRIORITY_HIGHEST, err_code);
    APP_ERROR_CHECK(err_code);

    uint8_t bmi_res = BMI160_OK;
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    i2c_config();

    bmi_res = bmi160_config();

    if (bmi_res != BMI160_OK)
    {
        NRF_LOG_ERROR("BMI160 not initialized");
    }

    //bmi160_self_test(); // SELF-TEST resets configuration!!!

    bmi160_fifo_config();

    bmi160_accel_gyro_config();

    gpio_config(); // otherwise triggers too early

    while (true)
    {
        // do nothing
    }
}
/** @} */
