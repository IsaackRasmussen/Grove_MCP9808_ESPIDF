/*
 * Seeed_MCP9808.cpp
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
 * THE SOFTWARE IS PROVIDED "AS IS\n", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Seeed_MCP9808.h"

/**@brief Set configuration of sensor.
 * @param Register addr of sensor configuration.
 * @param The value to set.
 * @return 0 if success;
 * 
 * */
s32 MCP9808::set_config(u8 addr, u16 cfg)
{
    return IIC_write_16bit(addr, cfg);
}

/**@brief Set upper limit of sensor.
 * @param Register addr of upper limit.
 * @param The value to set.
 * @return 0 if success;
 * */
s32 MCP9808::set_upper_limit(u8 addr, u16 cfg)
{
    printf("Setting UpperLimit: %.2f\n",calculate_temp(cfg));
    return IIC_write_16bit(addr, cfg);
}

/**@brief Set lower limit of sensor.
 * @param Register addr of lower limit.
 * @param The value to set.
 * @return 0 if success;
 * */
s32 MCP9808::set_lower_limit(u8 addr, u16 cfg)
{
    printf("Setting LowerLimit: %.2f\n",calculate_temp(cfg));
    return IIC_write_16bit(addr, cfg);
}

/**@brief Set critical limit of sensor.
 * @param Register addr of critical limit.
 * @param The value to set.
 * @return 0 if success;
 * */
s32 MCP9808::set_critical_limit(u8 addr, u16 cfg)
{
    printf("Setting CriticalLimit: %.2f\n",calculate_temp(cfg));
    return IIC_write_16bit(addr, cfg);
}

/**@brief Set resolution of sensor.range:0,0.25,0.125,0.0625
 * @param Register addr of resolution.
 * @param The value to set.
 * @return 0 if success;
 * */
s32 MCP9808::set_resolution(u8 addr, u8 resolution)
{
    return IIC_write_byte(addr, resolution);
}

/**@brief caculate u16 data to a float temp num;
 * @param register addr of ambient temp.
 * @param dst temp data.
 * */
s32 MCP9808::read_temp_reg(u8 addr, u16 *temp)
{
    IIC_read_16bit(addr, temp);
    return 0;
}

/**@brief caculate u16 data to a float temp num;
 * @param u16-form data.
 * @return float-form temp data.
 * */
float MCP9808::calculate_temp(u16 temp_value)
{
    float temp = 0;
    u8 temp_upper = 0, temp_lower = 0;
    temp_upper = (u8)(temp_value >> 8);
    temp_lower = (u8)temp_value;
    if (temp_upper & SIGN_BIT)
    {
        temp_upper &= 0x0f;
        temp = 256 - (temp_upper * 16 + temp_lower * 0.0625);
        temp *= -1;
    }
    temp_upper &= 0x0f;
    temp = temp_upper * 16 + temp_lower * 0.0625;
    return temp;
}

/**@brief Get the temperature data from sensor,caculate,output a float num;
 * @param dst data.
 * 
 * */
void MCP9808::get_temp(float *temp)
{
    u16 temp_value = 0;
    read_temp_reg(AMBIENT_TEMPERATURE_ADDR, &temp_value);
    *temp = calculate_temp(temp_value);
}

/**@brief Init IIC ,set resolution of sensor.default is 0.0625
 * @return 0 if success.
 * 
 * */
s32 MCP9808::init()
{
    IIC_begin();

    //High accuracy cost more time,if you want to reducing time of running,try to change this setting.
    if (set_resolution(SET_RESOLUTION_ADDR, RESOLUTION_0_0625_DEGREE))
    {
        return -1;
    }
    return 0;
}

MCP9808::MCP9808(u8 IIC_ADDR)
{
    set_iic_addr(IIC_ADDR);
}

/**********************************************************************************************************/
/************************************************IIC PART************************************************/
/**********************************************************************************************************/

s32 MCP_IIC_OPRTS::IIC_write_byte(u8 start_reg, u8 value)
{
    _i2cCmd = i2c_cmd_link_create();
    i2c_master_start(_i2cCmd);
    i2c_master_write_byte(_i2cCmd, (DEFAULT_IIC_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(_i2cCmd, (uint8_t)start_reg, true);
    i2c_master_write_byte(_i2cCmd, value, true);
    i2c_master_stop(_i2cCmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, _i2cCmd, 1000 / portTICK_RATE_MS);

    printf("Writing 8bit (%d), ret(%s)\n\n", value, esp_err_to_name(ret));

    i2c_cmd_link_delete(_i2cCmd);
    return (ret);

    /* Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    Wire.write(byte);
    return Wire.endTransmission();*/
}

s32 MCP_IIC_OPRTS::IIC_write_16bit(u8 start_reg, u16 value)
{
    _i2cCmd = i2c_cmd_link_create();
    i2c_master_start(_i2cCmd);
    i2c_master_write_byte(_i2cCmd, (DEFAULT_IIC_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(_i2cCmd, (uint8_t)start_reg, true);
    i2c_master_write_byte(_i2cCmd, value >> 8, true);
    i2c_master_write_byte(_i2cCmd, value & 0xFF, true);

    i2c_master_stop(_i2cCmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, _i2cCmd, 1000 / portTICK_RATE_MS);
    printf("Writing 16bit (%d), ret(%s)\n\n", value, esp_err_to_name(ret));

    i2c_cmd_link_delete(_i2cCmd);
    return (ret);
    /* Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);

    Wire.write((u8)(value >> 8));
    Wire.write((u8)value);
    return Wire.endTransmission();*/
}

void MCP_IIC_OPRTS::IIC_read_byte(u8 start_reg, u8 *byte)
{
    _i2cCmd = i2c_cmd_link_create();
    i2c_master_start(_i2cCmd);
    i2c_master_write_byte(_i2cCmd, (DEFAULT_IIC_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(_i2cCmd, (uint8_t)start_reg, true);
    i2c_master_stop(_i2cCmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, _i2cCmd, 1000 / portTICK_RATE_MS);
    printf("Read8.Ret1: %s\n", esp_err_to_name(ret));
    i2c_cmd_link_delete(_i2cCmd);

    if (ret == 0)
    {
        _i2cCmd = i2c_cmd_link_create();
        uint8_t byte1;
        i2c_master_start(_i2cCmd);
        i2c_master_write_byte(_i2cCmd, (DEFAULT_IIC_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(_i2cCmd, &byte1, (i2c_ack_type_t)ACK_VAL);
        i2c_master_stop(_i2cCmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, _i2cCmd, 1000 / portTICK_RATE_MS);
        printf("Read8.Ret2: %s\n", esp_err_to_name(ret));

        *byte = byte1;
        //_i2cCmd->requestFrom((uint8_t)_i2caddr, (uint8_t)2);
        /*val = _i2cCmd->read();
    val <<= 8;
    val |= _i2cCmd->read();*/
        i2c_cmd_link_delete(_i2cCmd);
    }

    printf("Read8.Val: %d\n", *byte);

    /*Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR, (u8)1);
    while (1 != Wire.available())
        ;
    *byte = Wire.read();*/
}

void MCP_IIC_OPRTS::IIC_read_16bit(u8 start_reg, u16 *value)
{
    _i2cCmd = i2c_cmd_link_create();
    i2c_master_start(_i2cCmd);
    i2c_master_write_byte(_i2cCmd, (DEFAULT_IIC_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(_i2cCmd, (uint8_t)start_reg, true);
    i2c_master_stop(_i2cCmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, _i2cCmd, 1000 / portTICK_RATE_MS);
    printf("Read16.Ret1: %s\n\n", esp_err_to_name(ret));
    i2c_cmd_link_delete(_i2cCmd);

    if (ret == 0)
    {
        _i2cCmd = i2c_cmd_link_create();
        uint8_t byte1, byte2;
        i2c_master_start(_i2cCmd);
        i2c_master_write_byte(_i2cCmd, (DEFAULT_IIC_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(_i2cCmd, &byte1, (i2c_ack_type_t)ACK_VAL);
        i2c_master_read_byte(_i2cCmd, &byte2, (i2c_ack_type_t)NACK_VAL);
        i2c_master_stop(_i2cCmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, _i2cCmd, 1000 / portTICK_RATE_MS);
        printf("Read16.Ret2: %s\n", esp_err_to_name(ret));

        *value = (byte1 << 8) + byte2;
        //_i2cCmd->requestFrom((uint8_t)_i2caddr, (uint8_t)2);
        /*val = _i2cCmd->read();
    val <<= 8;
    val |= _i2cCmd->read();*/
    i2c_cmd_link_delete(_i2cCmd);
    }

    printf("Read16.Val: %d\n", *value);
    /* u8 val = 0;
    *value = 0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(start_reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR, sizeof(u16));
    while (sizeof(u16) != Wire.available())
        ;
    val = Wire.read();
    *value |= (u16)val << 8;
    val = Wire.read();
    *value |= val;*/
}

void MCP_IIC_OPRTS::set_iic_addr(u8 IIC_ADDR)
{
    _IIC_ADDR = IIC_ADDR;
}
