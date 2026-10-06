//Load cells (thrust and two torque sensors), read through HX711 boards.
#pragma once
#include <HX711.h>

extern HX711 thrustSensor;
extern HX711 torqueSensor;
extern HX711 torqueSensor2;

void beginLoadCells(); //sets up the pins and gain on all three HX711 boards
void tareAllLoadCells(); //zeroes all three load cells, with no user prompts
void loadCalibrations(); //loads the calibration factors from EEPROM, and warns the user about any sensor that isn't calibrated

//menu actions. These walk the user through the process on screen
void tareThrust();
void tareTorque();
void calibrateThrust();
void calibrateTorque();
void calibrateTorque2();
