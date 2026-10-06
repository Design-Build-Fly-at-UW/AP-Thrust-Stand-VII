//Thrust Stand VII firmware: Design Build Fly at the University of Washington
//
//Where things live:
//  config.h      pins and hardware constants
//  state.h       live sensor readings and the settings edited from the menus
//  menu.cpp      menu structure and navigation
//  screens.cpp   the screen, keypad, and screens shown during tests
//  tests.cpp     the test profiles (smooth ramp, intervals, motor profile, battery)
//  sensors.cpp   reads every sensor and calculates power and efficiency
//  load_cells.cpp, analog.cpp, rpm.cpp   individual sensors
//  logging.cpp   SD card data files
//  throttle.cpp  ESC control

#include <Arduino.h>
#include <avr/wdt.h>
#include "screens.h"
#include "menu.h"
#include "rpm.h"
#include "throttle.h"
#include "logging.h"
#include "load_cells.h"
#include "analog.h"

/*TODO:
Thrust Profiles
Drag Taring Menu
Pre-test info screen
RPM Verification
*/

//After a watchdog reset the watchdog stays on with a ~15ms timeout, which would reset the board over and over
//before setup() finishes. This runs before setup() (and before global constructors) to turn it off.
void disableWatchdogAtBoot() __attribute__((naked, used, section(".init3")));
void disableWatchdogAtBoot() {
    MCUSR = 0;
    wdt_disable();
}

void setup() {
    u8g2.begin();
    Serial.begin(115200); //keep this fast, the debug prints block the test loop when the serial buffer fills

    drawLoadingScreen(0, "Attaching pins");
    beginRPM();
    beginThrottle();

    drawLoadingScreen(10, "Initializing SD-Card");
    beginSD();

    drawLoadingScreen(20, "Force Sensor Initialization");
    beginLoadCells();

    drawLoadingScreen(30, "Thrust Sensors Zeroing");
    tareAllLoadCells();

    drawLoadingScreen(40, "Current Sensor Zeroing");
    zeroCurrentSensor(); //no current is flowing at startup, so this is the sensor's zero point

    drawLoadingScreen(50, "Loading Calibration Factors");
    loadCalibrations();
}

void loop() {
    updateMenu();
}
