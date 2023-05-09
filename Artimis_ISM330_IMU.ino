/*
   @file    ISM330DHCX_DataLogTerminal.ino
   @author  Frederic Pillon <frederic.pillon@st.com>
   @brief   Example to use the ISM330DHCX 3D accelerometer and 3D gyroscope
 *******************************************************************************
   Copyright (c) 2021, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/
/

// Includes
#include <ISM330DHCXSensor.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN PNUM_NOT_DEFINED
#warning "LED_BUILTIN is not defined."
#endif

#define SerialPort  Serial

#define USE_I2C_INTERFACE

// Uncomment to set I2C pins to use else default instance will be used
//17/18 (Qwiic connector)
 #define ISM330DHCX_I2C_SCL  27
 #define ISM330DHCX_I2C_SDA  25
#if defined(ISM330DHCX_I2C_SCL) && defined(ISM330DHCX_I2C_SDA)
TwoWire dev_interface(ISM330DHCX_I2C_SDA, ISM330DHCX_I2C_SCL);
ISM330DHCXSensor AccGyr(&dev_interface);
#endif

void setup() {
  // Led
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output
  SerialPort.begin(9600);
  // Initialize bus interface
  dev_interface.begin();
  // Initlialize component
  AccGyr.begin();
  AccGyr.ACC_Enable();  
  AccGyr.GYRO_Enable();
}

void loop() {
 
  // Read accelerometer and gyroscope
int32_t accelerometer[3];
int32_t gyroscope[3];
AccGyr.ACC_GetAxes(accelerometer);  
AccGyr.GYRO_GetAxes(gyroscope);


int16_t acc_raw[3];
float acc_ms2[3];
float acc_sensitivity;

AccGyr.ACC_GetAxesRaw(acc_raw);
AccGyr.ACC_GetSensitivity(&acc_sensitivity);

acc_ms2[0] = ((float)acc_raw[0] * acc_sensitivity) / 1000.0f * 9.81f;
acc_ms2[1] = ((float)acc_raw[1] * acc_sensitivity) / 1000.0f * 9.81f;
acc_ms2[2] = ((float)acc_raw[2] * acc_sensitivity) / 1000.0f * 9.81f; 
Serial.print("Acc[mg]:");
Serial.print(acc_ms2[0]);
Serial.print(", ");
Serial.print(acc_ms2[1]);
Serial.print(", ");
Serial.print(acc_ms2[2]);
Serial.println("");
}
