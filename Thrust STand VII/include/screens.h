//The OLED screen, the keypad, and the screens shown during tests.
#pragma once
#include <U8g2lib.h> //library for controlling the OLED screen
#include <Keypad.h> //library for reading the 4x4 matrix keyboard

extern U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2;
extern Keypad customKeypad;

//keypad helpers
void pressKeyToContinue(); //waits for any key
bool waitForConfirm(); //waits for # (returns true) or * (returns false), ignoring other keys

//general screens
void drawLoadingScreen(int loadPercent, const char* message); //pass load percent as an int from 0-100
void drawErrorScreen(const char* title, const char* line1, const char* line2);
void drawCalibrationWarning(bool thrustCalibrated, bool torque1Calibrated, bool torque2Calibrated);
void drawProfileSelection();

//test setup screens. Each returns true if the user pressed # to continue, false if they pressed * to go back
bool reviewSmoothParameters();
bool reviewIntervalParameters();
bool reviewBatteryParameters();
bool confirmOverwrite();
bool confirmStartTest();

//screens shown during tests
void displaySensorData(); //shows all the live readings
void drawPauseScreen();
void drawPropSwapPrompt();
void drawPlugInMotorPrompt();
void drawBatteryRecoveryScreen(long secondsLeft, float voltage);

void debugMenu(); //menu action: shows live sensor readings until * is pressed
