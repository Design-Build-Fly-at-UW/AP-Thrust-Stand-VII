#include <Arduino.h>
#include <avr/wdt.h> //watchdog for resetting the board if it hangs during a test
#include "tests.h"
#include "state.h"
#include "screens.h"
#include "sensors.h"
#include "logging.h"
#include "throttle.h"

static bool cancelRequested(){ //true if the user pressed any key
    return customKeypad.getKey() != NO_KEY;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//TEST PROFILES
//Each of these runs with the watchdog on and the data file open. They should call wdt_reset() at
//least every 2 seconds, and stop the motor before returning if the user cancels.

//ramps throttle up smoothly, holds at the top, then ramps back down
static void smoothRamp(){
    const float rampMs = settings.rampTime*1000.0;
    const float topEndMs = (settings.rampTime + settings.topTime)*1000.0;
    const float testEndMs = (settings.rampTime*2 + settings.topTime)*1000.0;
    const float maxThrottle = settings.throttleMax;

    throttle = 0;
    unsigned long startTime = millis();

    while(true){
        wdt_reset(); //pet that dawg! (keeps the watchdog from going off)
        float time = millis() - startTime; //ms since the ramp started

        if (time < rampMs){ //ramping up
            throttle = maxThrottle * (time / rampMs);
        } else if (time < topEndMs){ //holding at the top
            throttle = maxThrottle;
        } else { //ramping down
            throttle = maxThrottle - maxThrottle * ((time - topEndMs) / rampMs);
        }

        readSensorData();
        displaySensorData();
        writeSensorSD();

        setThrottle(throttle);

        if (time > testEndMs || cancelRequested()) {
            return;
        }
    }
}

//steps throttle up in equal intervals, letting it settle and then recording data at each step
static void steppedRamp(){
    throttle = 0;
    float throttleStep = settings.throttleMax/(float)settings.intervalCount;

    for (int i = 1; i <= settings.intervalCount; i++) {
        float targetThrottle = i*throttleStep;

        //slew throttle to the next step 1% at a time, at most about 20% per second
        while (throttle <= targetThrottle-1){
            throttle = throttle+1;
            setThrottle(throttle);

            readSensorData();
            displaySensorData();
            if (cancelRequested()){
                stopMotor();
                return;
            }

            wdt_reset();
            delay(50);
        }

        throttle = targetThrottle;
        setThrottle(throttle); //snug up the throttle to the exact step

        //wait for the propulsion system to reach equilibrium
        unsigned long settleStart = millis();
        while (millis() - settleStart < (unsigned long)settings.rampSettleTime) {
            readSensorData();
            displaySensorData();
            if (cancelRequested()){
                stopMotor();
                return;
            }
            wdt_reset();
        }

        //record data at this step
        unsigned long stepStart = millis();
        while (millis() - stepStart <= (unsigned long)settings.intervalTime * 1000){
            wdt_reset();

            readSensorData();
            displaySensorData();
            writeSensorSD();

            if (cancelRequested()){
                stopMotor();
                return;
            }
        }
    }
}

//runs the stepped ramp over and over, pausing between runs so the user can swap propellers
static void motorProfileTest(){
    while (true){
        wdt_enable(WDTO_2S); //enabled every run, since it gets disabled while waiting on the user between props
        wdt_reset();
        restartMahTimer(); //don't count the time spent paused between props as current draw
        steppedRamp();

        stopMotor();
        wdt_disable();

        drawPauseScreen();
        if (!waitForConfirm()) return;
        drawPropSwapPrompt();
        if (!waitForConfirm()) return;
        drawPlugInMotorPrompt();
        if (!waitForConfirm()) return;
    }
}

//holds a target current draw until enough charge is drawn or the voltage drops too low, then records the battery recovering
static void batteryTest(){
    throttle = 0;

    while (true){
        readSensorData(); //update the current reading used for adjusting the throttle

        //step the throttle toward the target current. Small steps when close, big steps when far away
        float step = (abs(readings.current - settings.targetAmpDraw) < 1.5) ? 0.1 : 1;
        if (readings.current < settings.targetAmpDraw && throttle < settings.throttleMax){
            throttle += step;
        } else if (readings.current > settings.targetAmpDraw && throttle > 0){
            throttle -= step;
        }

        //keep the throttle within 0 to max, the step sizes can overshoot either end
        if (throttle > settings.throttleMax) throttle = settings.throttleMax;
        if (throttle < 0) throttle = 0;

        setThrottle(throttle);

        displaySensorData();
        writeSensorSD();

        //stop on user cancel, enough charge drawn, or voltage cutoff
        if (cancelRequested() || readings.mahDrawn >= settings.dischargeAmount || readings.voltage < settings.voltageCutoff){
            stopMotor();
            break;
        }

        //wait out the gain time while petting the watchdog, so gains of 2s or more don't reset the board
        unsigned long gainStart = millis();
        while (millis() - gainStart < (unsigned long)settings.currentTestGain) {
            wdt_reset();
        }
        wdt_reset();
    }

    //the motor is off now, so the watchdog isn't needed while recording the battery recovering from load
    wdt_disable();

    unsigned long recoveryStart = millis();
    unsigned long recoveryMs = settings.batteryRecoveryTime*1000;
    while (millis() - recoveryStart < recoveryMs) {
        readSensorData();
        writeSensorSD();

        long secondsLeft = settings.batteryRecoveryTime - (millis() - recoveryStart)/1000;
        drawBatteryRecoveryScreen(secondsLeft, readings.voltage);

        if (cancelRequested()) {
            break;
        }
        delay(50); //wait a bit to avoid overloading the SD card
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//RUNNING TESTS

//shows the review screen, sets up the data file, runs the test with the watchdog on, then always
//stops the motor and closes the file at the end
static void runWithSetup(bool (*reviewScreen)(), void (*test)()){
    setThrottle(0);
    if (!reviewScreen()){
        return;
    }
    if (!setUpTest()){
        return;
    }

    wdt_enable(WDTO_2S); //if it goes 2s without wdt_reset being called, the board will do a hardware reset
    wdt_reset();

    test();

    stopMotor();
    closeTestFile();
    wdt_disable();
}

void runTest(){
    if (settings.throttleMax > 100) { //cap it before any test uses it
        settings.throttleMax = 100;
    }

    switch (settings.testType){
        case 1: runWithSetup(reviewSmoothParameters, smoothRamp); break;
        case 2: runWithSetup(reviewIntervalParameters, steppedRamp); break;
        case 3: runWithSetup(reviewIntervalParameters, motorProfileTest); break;
        case 4: runWithSetup(reviewBatteryParameters, batteryTest); break;
    }
}

void selectProfile(){
    drawProfileSelection();

    while(true){
        char userInput = customKeypad.getKey();
        if (userInput >= '1' && userInput <= '4'){
            settings.testType = userInput - '0';
            return;
        }
    }
}
