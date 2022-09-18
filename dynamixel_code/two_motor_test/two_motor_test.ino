/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif

uint8_t DXL_ID = 1;
uint8_t DXL_ID_2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
constexpr unsigned int pwmPin = 13;
bool wingState = false;

DynamixelShield dxl;

// //This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup()
{
    // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
    dxl.begin(57600);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    // Get DYNAMIXEL information

    dxl.ping(DXL_ID);
    dxl.ping(DXL_ID_2);
    //
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID);
    dxl.torqueOff(DXL_ID_2);
    dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
    dxl.setOperatingMode(DXL_ID_2, OP_CURRENT_BASED_POSITION);
    dxl.torqueOn(DXL_ID);
    dxl.torqueOn(DXL_ID_2);

    // Current limit is set to 200mA
    constexpr float currentLimit = 200;    

    dxl.setGoalCurrent(DXL_ID, currentLimit, UNIT_MILLI_AMPERE);
    dxl.setGoalCurrent(DXL_ID_2, currentLimit, UNIT_MILLI_AMPERE);
}

bool channelValue()
{
    constexpr unsigned long threshold = 1500;
    return pulseIn(pwmPin, HIGH) > threshold;
}

void setWingExtended(bool state)
{
    // Set target position values for the open and closed wing states
    constexpr int targetPositionClosed = 0;
    constexpr int targetPositionOpen = 600;

    int targetPosition = state ? targetPositionOpen : targetPositionClosed;

    // Set goal positions, one is negative 
    // which should ensure that they both actuate in the same direction
    dxl.setGoalPosition(DXL_ID, targetPosition, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID_2, -targetPosition, UNIT_DEGREE);

    // Set delay based on estimated time ranges
    delay(3000);
    wingState = !wingState;
}

void loop()
{
    if (channelValue() != wingState) setWingExtended(channelValue());
}
