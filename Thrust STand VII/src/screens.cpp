#include <Arduino.h>
#include "screens.h"
#include "config.h"
#include "state.h"
#include "sensors.h"
#include "throttle.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
//SCREEN AND KEYPAD SETUP

//SSD1309, 128x64, I2C. Screen needs to be hooked up to SDA and SCL
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const byte ROWS = 4;
const byte COLS = 4;

static char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

static byte rowPins[ROWS] = {12, 11, 10, 9}; //adjust these if your wiring is different
static byte colPins[COLS] = {8, 7, 6, 5}; //adjust these if your wiring is different

Keypad customKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//////////////////////////////////////////////////////////////////////////////////////////////////
//KEYPAD HELPERS

void pressKeyToContinue(){
    while (customKeypad.getKey() == NO_KEY){
        //do nothing and wait for them to press a key
    }
}

bool waitForConfirm(){
    while (true){
        char key = customKeypad.getKey();
        if (key == '#') return true;
        if (key == '*') return false;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//GENERAL SCREENS

//bitmap for the UW logo. Stored in flash (U8X8_PROGMEM) to save RAM, so it's drawn with drawXBMP
static const unsigned char uwLogoBits[] U8X8_PROGMEM = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x02,0x00,0x00,0x00,0x00,0x00,0x1e,0x00,0x00,0x00,0x00,0x10,0xf8,0x00,0x00,0x00,0x00,0xe0,0xe3,0x03,0x00,0x00,0x00,0x00,0x8f,0x0f,0x05,0x00,0x00,0x3c,0x7c,0xff,0x0f,0x00,0x00,0xf0,0xf1,0xff,0x07,0x00,0x00,0x80,0xe7,0xff,0x07,0x00,0x00,0x00,0xbe,0xff,0x09,0x00,0x00,0x00,0xfc,0xff,0x00,0x00,0x00,0x00,0xf8,0xbf,0x01,0x01,0x00,0x00,0xf0,0x3f,0x00,0x02,0x00,0x00,0xe0,0x1f,0x10,0x08,0x00,0x00,0x80,0x20,0x40,0x10,0x00,0x00,0x00,0x00,0x00,0x21,0x00,0x00,0x00,0x00,0x08,0x42,0x00,0x00,0x00,0x00,0x10,0x84,0x00,0x00,0x00,0x00,0x40,0x88,0x00,0x00,0x00,0x00,0x80,0x90,0x00,0x00,0x00,0x00,0x00,0x31,0x00,0x00,0x00,0x00,0x00,0x1f,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x80,0x02,0x00,0x00,0x00,0xa8,0x2a,0x02,0x00,0x00,0x00,0x00,0xc0,0x00,0x00,0x00,0x00,0x40,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void drawLoadingScreen(int loadPercent, const char* message){
    Serial.println(message);

    u8g2.clearBuffer();

    //draw all the static stuff
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_t0_13b_tr);
    u8g2.drawStr(3, 14, "Design Build Fly");
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(3, 22, "At the University of Washington");
    u8g2.drawStr(98, 63, "2025-26");
    u8g2.drawXBMP(84, 15, 45, 40, uwLogoBits);
    u8g2.drawStr(2, 63, FIRMWARE_VERSION);

    //draw loading bar
    u8g2.drawRFrame(3, 28, 80, 21, 3);
    u8g2.drawRBox(3, 28, ((80-6)*loadPercent/100)+6, 21, 3); //loading bar fill in

    //draw message
    u8g2.drawStr(2, 56, message);

    u8g2.sendBuffer();
}

void drawErrorScreen(const char* title, const char* line1, const char* line2){
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_t0_14b_tr);
    u8g2.drawStr(2, 15, title);
    u8g2.drawLine(0, 18, 127, 18);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(2, 32, line1);
    u8g2.drawStr(2, 42, line2);
    u8g2.sendBuffer();
}

void drawCalibrationWarning(bool thrustCalibrated, bool torque1Calibrated, bool torque2Calibrated){
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_t0_14b_tr);
    u8g2.drawStr(2, 15, "Not Calibrated:");
    u8g2.drawLine(0, 18, 127, 18);
    u8g2.setFont(u8g2_font_5x7_tr);
    int y = 28;
    if (!thrustCalibrated) { u8g2.drawStr(2, y, "- Thrust Sensor"); y += 9; }
    if (!torque1Calibrated) { u8g2.drawStr(2, y, "- Torque 1"); y += 9; }
    if (!torque2Calibrated) { u8g2.drawStr(2, y, "- Torque 2"); y += 9; }
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(2, 63, "Press any key to continue...");
    u8g2.sendBuffer();
}

void drawProfileSelection(){
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(17, 12, "Select Test Profile");
    u8g2.drawLine(0, 14, 127, 14);

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(1, 26, "1 - Smooth Ramp Up");
    u8g2.drawStr(2, 37, "2 - Intervals Ramp Up");
    u8g2.drawStr(2, 48, "3 - Motor Profile Testing");
    u8g2.drawStr(2, 59, "4 - Battery Load Testing");
    u8g2.sendBuffer();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//TEST SETUP SCREENS

bool reviewSmoothParameters(){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12b_mf);
    u8g2.setCursor(1, 18); u8g2.print("Smooth Params:");
    u8g2.drawLine(0, 19, 128, 19);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(1, 28); u8g2.print("Ramp Time (s): "); u8g2.print(settings.rampTime);
    u8g2.setCursor(1, 35); u8g2.print("Top Hold Time (s): "); u8g2.print(settings.topTime);
    u8g2.setCursor(1, 42); u8g2.print("Max Throttle (%): "); u8g2.print(settings.throttleMax);
    u8g2.setFont(u8g2_font_4x6_mr);
    u8g2.drawStr(2, 56, "# to continue");
    u8g2.drawStr(2, 62, "* to go back");
    u8g2.sendBuffer();
    return waitForConfirm();
}

bool reviewIntervalParameters(){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12b_mf);
    u8g2.setCursor(1, 18); u8g2.print("Interval Params:");
    u8g2.drawLine(0, 19, 128, 19);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(1, 28); u8g2.print("Interval Amount: "); u8g2.print(settings.intervalCount);
    u8g2.setCursor(1, 35); u8g2.print("Interval Time (s): "); u8g2.print(settings.intervalTime);
    u8g2.setCursor(1, 42); u8g2.print("Max Throttle (%): "); u8g2.print(settings.throttleMax);
    u8g2.setCursor(1, 49); u8g2.print("Settle Time (ms): "); u8g2.print(settings.rampSettleTime);
    u8g2.setFont(u8g2_font_4x6_mr);
    u8g2.drawStr(2, 56, "# to continue");
    u8g2.drawStr(2, 62, "* to go back");
    u8g2.sendBuffer();
    return waitForConfirm();
}

bool reviewBatteryParameters(){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12b_mf);
    u8g2.setCursor(1, 13); u8g2.print("Battery Params:");
    u8g2.drawLine(0, 14, 128, 14);
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.setCursor(1, 21); u8g2.print("Discharge (mAh): "); u8g2.print(settings.dischargeAmount);
    u8g2.setCursor(1, 28); u8g2.print("Current Draw (A): "); u8g2.print(settings.targetAmpDraw);
    u8g2.setCursor(1, 35); u8g2.print("Max Throttle (%): "); u8g2.print(settings.throttleMax);
    u8g2.setCursor(1, 42); u8g2.print("Gain (ms): "); u8g2.print(settings.currentTestGain);
    u8g2.setCursor(1, 49); u8g2.print("Voltage Cutoff (V): "); u8g2.print(settings.voltageCutoff);
    u8g2.setCursor(1, 56); u8g2.print("Sag Recover Time (s): "); u8g2.print(settings.batteryRecoveryTime);
    u8g2.setFont(u8g2_font_4x6_mr);
    u8g2.drawStr(2, 62, "* to go back, # to continue");
    u8g2.sendBuffer();
    return waitForConfirm();
}

bool confirmOverwrite(){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_14b_tr);
    u8g2.drawStr(2, 15, "File Name Already");
    u8g2.drawStr(2, 26, "In Use");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(3, 47, "Overwrite: #");
    u8g2.drawStr(3, 55, "Cancel: *");
    u8g2.sendBuffer();
    return waitForConfirm();
}

bool confirmStartTest(){
    const char* testName = "";
    switch (settings.testType){
        case 1: testName = "  - Smooth Ramp Up - "; break;
        case 2: testName = " - Intervals Ramp Up -"; break;
        case 3: testName = " - Motor Profile Test -"; break;
        case 4: testName = " - Battery Load Test - "; break;
    }

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12b_tr);
    u8g2.drawStr(2, 15, "Start Test When Ready");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(3, 26, "Test Type:");
    u8g2.drawStr(3, 37, testName);
    u8g2.drawStr(3, 47, "Start: #");
    u8g2.drawStr(3, 55, "Cancel: *");
    u8g2.sendBuffer();
    return waitForConfirm();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//SCREENS SHOWN DURING TESTS

void displaySensorData(){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(2, 9, "Stop Test Any Key");
    u8g2.drawLine(0, 10, 128, 10);

    u8g2.setFont(u8g2_font_squeezed_r6_tr);

    //left column
    u8g2.setCursor(1, 19); u8g2.print("RPM: "); u8g2.print(readings.rpm);
    u8g2.setCursor(1, 26); u8g2.print("THST (N): "); u8g2.print(readings.thrust/1000); //N
    u8g2.setCursor(1, 33); u8g2.print("TRQ (N.m): "); u8g2.print(readings.torque/1000); //Nm
    u8g2.setCursor(1, 40); u8g2.print("VLTS: "); u8g2.print(readings.voltage);
    u8g2.setCursor(1, 47); u8g2.print("AMPS: "); u8g2.print(readings.current);
    u8g2.setCursor(1, 54); u8g2.print("ASPD (M/S): "); u8g2.print(readings.airspeed);
    u8g2.setCursor(1, 61); u8g2.print("MAH-DRAWN: "); u8g2.print(readings.mahDrawn);

    //right column
    u8g2.setCursor(64, 19); u8g2.print("THRTL: %"); u8g2.print(throttle);
    u8g2.setCursor(64, 26); u8g2.print("E-PWR: "); u8g2.print(readings.electricPower); //W
    u8g2.setCursor(64, 33); u8g2.print("MTR-PWR (W): "); u8g2.print(readings.mechanicalPower);
    u8g2.setCursor(64, 40); u8g2.print("PRP-PWR (W): "); u8g2.print(readings.propellerPower);
    u8g2.setCursor(64, 47); u8g2.print("MTR-EF: % "); u8g2.print(readings.motorEfficiency);
    u8g2.setCursor(64, 54); u8g2.print("PRP-EF: % "); u8g2.print(readings.propellerEfficiency);
    u8g2.setCursor(64, 61); u8g2.print("SYS-EF: % "); u8g2.print(readings.systemEfficiency);

    u8g2.sendBuffer();
}

void drawPauseScreen(){
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(2, 9, "Test Paused...");
    u8g2.drawLine(0, 12, 128, 12);

    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(8, 32, "Continue Test: Press #");
    u8g2.drawStr(33, 52, "End Test: Press *");
    u8g2.sendBuffer();
}

void drawPropSwapPrompt(){
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);

    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(32, 15, "UNPLUG MOTOR!");
    u8g2.drawStr(26, 31, "Swap Propellers");

    //warning triangles on either side of the title
    u8g2.drawLine(17, 3, 6, 19);
    u8g2.drawLine(6, 19, 27, 19);
    u8g2.drawLine(28, 19, 17, 3);
    u8g2.drawLine(121, 19, 110, 3);
    u8g2.drawLine(99, 19, 120, 19);
    u8g2.drawLine(110, 3, 99, 19);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(15, 17, "!");
    u8g2.drawStr(108, 17, "!");

    u8g2.drawLine(0, 34, 127, 34);
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(4, 46, "Press # to Continue Test");
    u8g2.drawStr(16, 58, "Press * to End Test");
    u8g2.sendBuffer();
}

void drawPlugInMotorPrompt(){
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(33, 14, "Plug in Motor");
    u8g2.drawLine(127, 17, 0, 17);
    u8g2.drawStr(4, 35, "Press # to Continue Test");
    u8g2.drawStr(18, 53, "Press * to End Test");
    u8g2.sendBuffer();
}

void drawBatteryRecoveryScreen(long secondsLeft, float voltage){
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_t0_14b_tr);
    u8g2.drawStr(5, 15, "Measuring Battery");
    u8g2.drawStr(19, 26, "Recovery Time");

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(3, 35, "Please Wait:");
    u8g2.drawStr(23, 43, "Voltage:");
    u8g2.drawStr(28, 52, "Cancel: *");

    u8g2.setCursor(69, 35); u8g2.print(secondsLeft);
    u8g2.setCursor(69, 43); u8g2.print(voltage);

    u8g2.sendBuffer();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//DEBUG MENU

void debugMenu() {
    while(true){
        readSensorData();

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(2, 9, "Debug");
        u8g2.drawLine(0, 10, 128, 10);
        u8g2.setFont(u8g2_font_squeezed_r6_tr);

        //left column
        u8g2.setCursor(1, 19); u8g2.print("RPM Sensor: "); u8g2.print(digitalRead(RPM_PIN));
        u8g2.setCursor(1, 26); u8g2.print("THST: "); u8g2.print(readings.thrust/1000); //N
        u8g2.setCursor(1, 33); u8g2.print("TRQ: "); u8g2.print(readings.torque/1000); //Nm
        u8g2.setCursor(1, 40); u8g2.print("VLTS: "); u8g2.print(readings.voltage);
        u8g2.setCursor(1, 47); u8g2.print("AMPS: "); u8g2.print(readings.current);
        u8g2.setCursor(1, 54); u8g2.print("ASPD: "); u8g2.print(readings.airspeed); //m/s

        //right column
        u8g2.setCursor(66, 26); u8g2.print("TRQ1: "); u8g2.print(readings.torque1/1000); //Nm
        u8g2.setCursor(66, 33); u8g2.print("TRQ2: "); u8g2.print(readings.torque2/1000); //Nm

        u8g2.drawStr(4, 63, "Back: *");
        u8g2.sendBuffer();

        if (customKeypad.getKey() == '*') {
            return;
        }
    }
}
