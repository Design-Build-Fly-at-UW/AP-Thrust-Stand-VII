#include <Arduino.h>
#include <EEPROM.h> //stores the thrust and torque calibrations
#include "load_cells.h"
#include "config.h"
#include "screens.h"
#include "menu.h"

HX711 thrustSensor;
HX711 torqueSensor;
HX711 torqueSensor2;

void beginLoadCells(){
    torqueSensor.begin(TRQ_DOUT, TRQ_CLK);
    torqueSensor.set_gain(128);

    torqueSensor2.begin(TRQ2_DOUT, TRQ2_CLK);
    torqueSensor2.set_gain(128);

    thrustSensor.begin(THST_DOUT, THST_CLK);
    thrustSensor.set_gain(128);
}

//the HX711 library blocks forever waiting for a sample, so check a sensor is sending data before reading it
static bool loadCellResponding(HX711* loadCell){
    return loadCell->wait_ready_timeout(LOAD_CELL_TIMEOUT);
}

//replaces HX711::tare(), which waits forever on every sample. Checks before each sample, since a floating DOUT pin
//can look ready once and then never again. Returns false (leaving the old offset) if the sensor stops responding
static bool tareWithTimeout(HX711* loadCell){
    const byte samples = 10;
    long sum = 0;
    for (byte i = 0; i < samples; i++) {
        if (!loadCellResponding(loadCell)) return false;
        sum += loadCell->read();
    }
    loadCell->set_offset(sum / samples);
    return true;
}

//whether each sensor answered at startup, so loadCalibrations() can warn about it
static bool thrustConnected = false;
static bool torque1Connected = false;
static bool torque2Connected = false;

void tareAllLoadCells(){ //skips any sensor that isn't sending data so startup doesn't hang
    torque1Connected = tareWithTimeout(&torqueSensor);
    Serial.print(F("Torque 1 tare: ")); Serial.println(torque1Connected ? F("ok") : F("no response"));
    torque2Connected = tareWithTimeout(&torqueSensor2);
    Serial.print(F("Torque 2 tare: ")); Serial.println(torque2Connected ? F("ok") : F("no response"));
    thrustConnected = tareWithTimeout(&thrustSensor);
    Serial.print(F("Thrust tare: ")); Serial.println(thrustConnected ? F("ok") : F("no response"));
}

static bool loadScale(int address, HX711* loadCell){ //loads a calibration factor from EEPROM into the load cell. Returns false if there isn't a valid one saved
    float scale;
    EEPROM.get(address, scale);
    //blank EEPROM reads as NaN. A scale of exactly 1 is the placeholder that old firmware could save by canceling calibration, a real calibration never lands on 1
    bool valid = !isnan(scale) && !isinf(scale) && scale != 0 && scale != 1;
    loadCell->set_scale(valid ? scale : 1); //use 1 as a placeholder so the reading isn't NaN, it will read raw counts until calibrated
    return valid;
}

void loadCalibrations(){
    bool thrustCalibrated = loadScale(THST_CAL_ADDRESS, &thrustSensor);
    bool torque1Calibrated = loadScale(TRQ_CAL_ADDRESS, &torqueSensor);
    bool torque2Calibrated = loadScale(TRQ2_CAL_ADDRESS, &torqueSensor2);

    //warn the user about any sensor that isn't connected, or without a saved calibration (it will read raw counts instead of real units)
    bool allOk = thrustConnected && torque1Connected && torque2Connected && thrustCalibrated && torque1Calibrated && torque2Calibrated;
    if (!allOk){
        drawSensorWarning(thrustConnected, thrustCalibrated, torque1Connected, torque1Calibrated, torque2Connected, torque2Calibrated);
        pressKeyToContinue();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//TARE AND CALIBRATION FLOWS

static void drawNoSensor(){ //tells the user the load cell isn't sending data
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_16b_tr);
    u8g2.drawStr(3, 15, "No sensor");
    u8g2.drawStr(3, 27, "detected.");
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(3, 44, "Check the load cell wiring.");
    u8g2.sendBuffer();
    delay(USER_NOTIF_DELAY);
}

static bool tareLoadCell(HX711* loadCell) { //pass a load cell object, will take the user through taring the load cell. Returns false if the sensor isn't responding

    //prompt the user to remove load from the load cell
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_t0_16b_tr);
    u8g2.drawStr(3, 15, "Remove all load");
    u8g2.drawStr(3, 27, "from sensor.");
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(3, 44, "Press any key to continue...");
    u8g2.sendBuffer();

    pressKeyToContinue();

    //tell user we are taring
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_22b_tr);
    u8g2.drawStr(14, 39, "Taring...");
    u8g2.sendBuffer();

    if (!tareWithTimeout(loadCell)){
        drawNoSensor();
        return false;
    }
    delay(USER_NOTIF_DELAY);
    return true;
}

static bool calibrateLoadCell(HX711* loadCell, String units) {//pass a load cell and the unit string, and will take the user through calibration. Returns true if a new calibration was set, false if canceled
    if (!tareLoadCell(loadCell)) return false; //start by taring, and give up if the sensor isn't there

    //tell user to place known load
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_16b_tr);
    u8g2.drawStr(3, 15, "Apply a known");
    u8g2.drawStr(3, 27, "load to sensor.");
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(3, 44, "Press any key to continue...");
    u8g2.sendBuffer();

    pressKeyToContinue();

    long knownLoad = 0;

    String messageString = ("Enter Load " + units); //this has to be two lines to avoid a dangling pointer to the string, because
    const char* message = messageString.c_str();    //of how .c_str() works

    bool accepted = valueEditMenu(&knownLoad, message); //ask user to input the calibration amount

    if (!accepted || knownLoad==0){ //if the user cancels (or enters 0, which can't be calibrated against), then don't calibrate
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_t0_22b_tr);
        u8g2.drawStr(10, 39, "Canceled");
        u8g2.sendBuffer();
        delay(USER_NOTIF_DELAY);
        return false;
    }

    //tell user calibration is in progress
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_22b_tr);
    u8g2.drawStr(4, 40, "Calibrating...");
    u8g2.sendBuffer();

    //read the load cell N times, keeping track of the sum, minimum, and maximum
    const int N = 50; //the number of samples to average out
    long sum = 0;
    long minVal = 0;
    long maxVal = 0;

    for (int i = 0; i < N; i++) {
        if (!loadCellResponding(loadCell)){ //sensor dropped out partway through, don't calibrate against bad data
            drawNoSensor();
            return false;
        }
        long v = loadCell->get_value(); //blocks until fresh sample. Important that it's get value, since that is with offset
        Serial.println(v);
        sum += v;
        if (i == 0 || v < minVal) minVal = v;
        if (i == 0 || v > maxVal) maxVal = v;
    }

    float avgReading = (float)sum / N;

    float maxDev = maxVal-minVal; //calculates the maximum deviation
    float percentDev = abs((maxDev / avgReading) * 100.0); //calculates the percent deviation

    //set the calibration factor, this is in counts/unit load
    loadCell->set_scale(avgReading/knownLoad);

    Serial.print("Known Force: "); Serial.println(knownLoad);
    Serial.print("Calibrated Force: "); Serial.println(loadCell->get_units());
    Serial.print("Read Force: "); Serial.println(avgReading);
    Serial.print("Max Deviation: "); Serial.println(maxDev);

    //tell user the calibration is over
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_16b_tr);
    u8g2.drawStr(22, 13, "Calibrated");

    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.setCursor(3, 24);
    u8g2.print("Raw Value: "); u8g2.print(avgReading);
    u8g2.setCursor(3, 31);
    u8g2.print("Max Sample Deviation: %"); u8g2.print(percentDev);
    u8g2.drawStr(3, 49, "Press any key to continue...");

    u8g2.sendBuffer();
    pressKeyToContinue();
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//MENU ACTIONS

void tareThrust(){
    tareLoadCell(&thrustSensor);
}

void tareTorque(){
    tareLoadCell(&torqueSensor);
}

void tareTorque2(){
    tareLoadCell(&torqueSensor2);
}

//each of these only saves if the user didn't cancel, so a placeholder scale never gets saved
void calibrateThrust(){
    if (calibrateLoadCell(&thrustSensor, THST_UNITS)){
        EEPROM.put(THST_CAL_ADDRESS, thrustSensor.get_scale());
    }
}

void calibrateTorque(){
    if (calibrateLoadCell(&torqueSensor, TRQ_UNITS)){
        EEPROM.put(TRQ_CAL_ADDRESS, torqueSensor.get_scale());
    }
}

void calibrateTorque2(){
    if (calibrateLoadCell(&torqueSensor2, TRQ_UNITS)){
        EEPROM.put(TRQ2_CAL_ADDRESS, torqueSensor2.get_scale());
    }
}
