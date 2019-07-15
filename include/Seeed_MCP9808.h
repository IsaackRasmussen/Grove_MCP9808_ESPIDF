/*
 * Seeed_MCP9808.h
 * Driver for DIGITAL I2C HUMIDITY AND TEMPERATURE SENSOR
 *  
 * Copyright (c) 2018 Seeed Technology Co., Ltd.
 * Website    : www.seeed.cc
 * Author     : downey
 * Create Time: May 2018
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef _SEEED_MCP9808_H
#define _SEEED_MCP9808_H

#ifndef SEEED_DN_DEFINES
#define SEEED_DN_DEFINES

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL_DB SerialUSB
#else
#define SERIAL_DB Serial
#endif

#include "driver/i2c.h"

typedef int s32;
typedef long unsigned int u32;
typedef short s16;
typedef unsigned short u16;
typedef char s8;
typedef unsigned char u8;

/* typedef enum
{
    NO_ERROR = 0,
    ERROR_PARAM = -1,
    ERROR_COMM = -2,
    ERROR_OTHERS = -128,
} err_t;*/

#define CHECK_RESULT(a, b)                    \
    do                                        \
    {                                         \
        if (a = b)                            \
        {                                     \
            SERIAL_DB.print(__FILE__);        \
            SERIAL_DB.print(__LINE__);        \
            SERIAL_DB.print(" error code ="); \
            SERIAL_DB.println(a);             \
            return a;                         \
        }                                     \
    } while (0)

#endif

#define SET_CONFIG_ADDR 0X01
#define SET_UPPER_LIMIT_ADDR 0X02
#define SET_LOWER_LIMIT_ADDR 0X03
#define SET_CRITICAL_LIMIT_ADDR 0X04

#define AMBIENT_TEMPERATURE_ADDR 0X05
#define SET_RESOLUTION_ADDR 0X08

#define DEFAULT_IIC_ADDR 0X18

#define RESOLUTION_0_5_DEGREE 0
#define RESOLUTION_0_25_DEGREE 0X01
#define RESOLUTION_0_125_DEGREE 0X02
#define RESOLUTION_0_0625_DEGREE 0X03
#define SIGN_BIT 0X10

#define ACK_VAL 0x0
#define NACK_VAL 0x1

class MCP_IIC_OPRTS
{
public:
    bool installI2CDriver()
    {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)22;
        conf.scl_io_num = (gpio_num_t)23;
        conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
        conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
        conf.master.clk_speed = 100000;
        i2c_param_config(I2C_NUM_0, &conf);
        esp_err_t ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

        if (ret)
        {
            printf("Unable to install I2C driver: (%s)\n", esp_err_to_name(ret));
            return (false);
        }
        return (true);
    }

    void IIC_begin()
    {
        if (installI2CDriver())
        {
            printf("Installed I2C driver\n");

            //_i2cCmd = i2c_cmd_link_create();

            //printf("I2C link created\n");
        }
    }
    s32 IIC_write_byte(u8 reg, u8 byte);
    void IIC_read_byte(u8 reg, u8 *byte);
    void set_iic_addr(u8 IIC_ADDR);
    void IIC_read_16bit(u8 start_reg, u16 *value);
    s32 IIC_write_16bit(u8 reg, u16 value);

private:
    i2c_cmd_handle_t _i2cCmd;
    u8 _IIC_ADDR;
};

class MCP9808 : public MCP_IIC_OPRTS
{
public:
    MCP9808(u8 IIC_ADDR = DEFAULT_IIC_ADDR);
    ~MCP9808(){};
    s32 init();
    s32 set_config(u8 addr, u16 cfg);
    s32 set_upper_limit(u8 addr, u16 cfg);
    s32 set_lower_limit(u8 addr, u16 cfg);
    s32 set_critical_limit(u8 addr, u16 cfg);
    s32 read_temp_reg(u8 addr, u16 *temp);
    void get_temp(float *temp);
    s32 set_resolution(u8 addr, u8 resolution);

private:
    float calculate_temp(u16 temp);
};

#endif
