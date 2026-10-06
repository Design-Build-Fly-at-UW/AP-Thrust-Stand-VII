//Pins, hardware constants, and fixed settings. Change wiring and sensor constants here.
#pragma once
#include <Arduino.h>

#define FIRMWARE_VERSION "Version 1.2"

//////////////////////////////////////////////////////////////////////////////////////////////////
//UI

#define USER_NOTIF_DELAY 1800 //ms, how long notification screens stay up

//keypad pins are in screens.cpp, next to the key layout

//////////////////////////////////////////////////////////////////////////////////////////////////
//EEPROM (stores the load cell calibrations)

#define THST_CAL_ADDRESS 0
#define TRQ_CAL_ADDRESS 100 //make sure this is sufficiently spaced from thst cal to avoid overwriting
#define TRQ2_CAL_ADDRESS 200 //second torque sensor calibration

//////////////////////////////////////////////////////////////////////////////////////////////////
//SD CARD

#define SD_CS_PIN 53 //change if your module uses a different CS
#define FLUSH_PERIOD_MS 5000 //how often the data file is flushed (saved to the SD card) during a test

//////////////////////////////////////////////////////////////////////////////////////////////////
//LOAD CELLS

#define TRQ_DOUT 48
#define TRQ_CLK 49
#define TRQ2_DOUT 44
#define TRQ2_CLK 45
#define TRQ_UNITS "(N.mm)"

#define THST_DOUT 46
#define THST_CLK 47
#define THST_UNITS "(mN)"

//////////////////////////////////////////////////////////////////////////////////////////////////
//CURRENT AND VOLTAGE SENSORS

#define CURRENT_PIN A2
#define VOLTAGE_PIN A3

const float Vcc = 5.0; //change to match VCC logic voltage of board
const float CURRENT_SENSITIVITY = 0.020; //V/A
const float VOLTAGE_CALIBRATION = 21; //voltage divider ratio
const int ANALOG_AVERAGE_COUNT = 40; //how many analog readings are averaged for each sensor reading

//////////////////////////////////////////////////////////////////////////////////////////////////
//RPM SENSOR

#define RPM_PIN 2 //must be an interrupt pin

//////////////////////////////////////////////////////////////////////////////////////////////////
//AIRSPEED SENSOR

#define AIRSPEED_PIN A7
const float AIRSPEED_ZERO_VOLTAGE = 2.7; //MODIFY THIS VALUE TO CORRESPOND TO VOLTAGE WITHOUT ANY AIRFLOW
const float AIRSPEED_SENSITIVITY = 1.0; //V/kPa
const float AIR_DENSITY = 1.2; //kg/m^3 at sea level

//////////////////////////////////////////////////////////////////////////////////////////////////
//ESC
//(For a HARGRAVE MICRODRIVE ESC, accepted PWM frequencies range from 50Hz to 499 Hz)

#define ESC_PIN 3
const int MIN_THROTTLE = 1050; //us
const int MAX_THROTTLE = 1950; //us
