#include <Arduino.h>
#include <SPI.h> //used for the SPI needed for the SD card
#include <SD.h>
#include "logging.h"
#include "config.h"
#include "state.h"
#include "screens.h"
#include "menu.h"
#include "sensors.h"
#include "throttle.h"

static File dataFile;
static bool sdReady = false; //false if the SD card was skipped or failed at startup. Tests will try to start it again
static unsigned long lastFlush = 0;
static long testNumber = 1;

void beginSD(){
    pinMode(SD_CS_PIN, OUTPUT); //pin 53 has to be an output for the Mega's SPI to work, even if CS is moved

    while (!(sdReady = SD.begin(SD_CS_PIN))) {
        Serial.println("SD card initialization failed!");
        drawErrorScreen("SD Card Error", "No SD card found. Insert", "one, or press any key to skip.");

        //watch the keypad for a second between attempts, since SD.begin is slow to fail and would miss key presses
        unsigned long waitStart = millis();
        while (millis() - waitStart < 1000) {
            if (customKeypad.getKey() != NO_KEY) {
                return; //continue without a card, for calibration or debugging
            }
        }
    }
}

bool setUpTest(){
    setThrottle(0);

    //if the card was skipped or missing at startup, try again now in case it has been inserted since
    if (!sdReady){
        sdReady = SD.begin(SD_CS_PIN);
        if (!sdReady){
            drawErrorScreen("SD Card Error", "No SD card found.", "Insert card and retry.");
            delay(USER_NOTIF_DELAY);
            return false;
        }
    }

    if (!valueEditMenu(&testNumber, "Enter Test Number")){
        return false; //user canceled, don't start the test
    }

    //the SD library only allows 8 character file names, and "Test_" uses 5 of them, so the number can be at most 3 digits
    if (testNumber > 999){
        drawErrorScreen("Invalid Number", "Test number must be", "between 0 and 999.");
        delay(USER_NOTIF_DELAY);
        return false;
    }

    char filename[20];
    snprintf(filename, sizeof(filename), "Test_%d.csv", (int)testNumber);

    //if the file already exists, ask the user whether to overwrite it
    if (SD.exists(filename)) {
        if (!confirmOverwrite()){
            return false;
        }
        SD.remove(filename);
    }

    dataFile = SD.open(filename, FILE_WRITE);
    if (!dataFile) {
        Serial.println("Failed to create file!");
        drawErrorScreen("SD Card Error", "Could not create file.", "Check the SD card.");
        delay(USER_NOTIF_DELAY);
        sdReady = false; //the card may have been removed, so re-initialize it next time
        return false;
    }

    Serial.print("Created file: ");
    Serial.println(filename);

    dataFile.println("Time (s),Current (A),Voltage (V),Torque(N.mm),Thrust(mN),RPM,Airspeed(m/s),Throttle (%),Electrical Power (W),Mechanical Power (W),Propulsive Power (W),Motor Efficiency (%), Propeller Efficiency (%), System Efficiency (%), mAh Drawn (mAh)");
    dataFile.flush(); //make sure the header is written to the card

    if (!confirmStartTest()){
        dataFile.close();
        SD.remove(filename); //delete the empty file if the user cancels
        return false;
    }

    resetSensorData(); //makes sure that if a sensor is missing, it shows as zero and not the value of the last test
    return true;
}

void writeSensorSD(){
    dataFile.print(readings.testTime/1000, 3);        dataFile.print(',');
    dataFile.print(readings.current, 3);              dataFile.print(',');
    dataFile.print(readings.voltage, 3);              dataFile.print(',');
    dataFile.print(readings.torque, 3);               dataFile.print(',');
    dataFile.print(readings.thrust, 3);               dataFile.print(',');
    dataFile.print(readings.rpm, 1);                  dataFile.print(',');
    dataFile.print(readings.airspeed, 3);             dataFile.print(',');
    dataFile.print(throttle, 1);                      dataFile.print(',');
    dataFile.print(readings.electricPower, 3);        dataFile.print(',');
    dataFile.print(readings.mechanicalPower, 3);      dataFile.print(',');
    dataFile.print(readings.propellerPower, 3);       dataFile.print(',');
    dataFile.print(readings.motorEfficiency, 3);      dataFile.print(',');
    dataFile.print(readings.propellerEfficiency, 3);  dataFile.print(',');
    dataFile.print(readings.systemEfficiency, 3);     dataFile.print(',');
    dataFile.print(readings.mahDrawn, 3);             dataFile.println();

    //don't flush all the time, it's slow
    if ((millis()-lastFlush) > FLUSH_PERIOD_MS){
        dataFile.flush();
        lastFlush = millis();
    }
}

void closeTestFile(){
    dataFile.close();
    testNumber++;
}
