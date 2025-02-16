/*
 * MCP9808_demo.ino
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


#include "Seeed_MCP9808.h"


MCP9808  sensor;

void setup()
{
    Serial.begin(115200);
    if(sensor.init())
    {
        Serial.println("sensor init failed!!");
    }
    //Set upper limit is 30°C
    sensor.set_upper_limit(SET_UPPER_LIMIT_ADDR,0x01e0);
    delay(10);
    //Set upper limit is 32°C
    sensor.set_critical_limit(SET_CRITICAL_LIMIT_ADDR,0x0200);
    delay(10);
    //Enable the alert bit.The alert bit outputs low when the temperature value beyond limit.Otherwise stays high.
    sensor.set_config(SET_CONFIG_ADDR,0x0008);

    Serial.println("sensor init!!");
}


void loop()
{
    float temp=0;
    //Get temperature ,a float-form value.
    sensor.get_temp(&temp);
    Serial.print("temperature value is: ");
    Serial.println(temp);
    delay(1000);
}

