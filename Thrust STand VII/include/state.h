//Shared state: live sensor readings and the settings the user can edit from the menus.
#pragma once

//Live sensor readings and values calculated from them. Reset at the start of every test
struct SensorData {
    unsigned long testStartTime = 0; //ms, millis() when the test started
    float testTime = 0; //ms since the test started

    float thrust = 0; //mN
    float torque = 0; //N.mm, average of both torque sensors
    float torque1 = 0; //N.mm, torque sensor 1
    float torque2 = 0; //N.mm, torque sensor 2
    float airspeed = 0; //m/s
    float current = 0; //amps
    float voltage = 0; //volts
    float rpm = 0;

    //calculated values
    float electricPower = 0; //watts
    float mechanicalPower = 0; //watts
    float propellerPower = 0; //watts
    float motorEfficiency = 0; //0-100%
    float propellerEfficiency = 0; //0-100%
    float systemEfficiency = 0; //0-100%
    float mahDrawn = 0; //mAh
};

//Test profile settings, edited from the Configure Test menu. These are longs so the menu can edit them
struct TestSettings {
    long testType = 1; //1 = smooth ramp, 2 = intervals, 3 = motor profile (intervals with prop swaps), 4 = battery load
    long throttleMax = 100; //%, used by every test

    //smooth ramp
    long rampTime = 15; //s
    long topTime = 4; //s

    //intervals
    long intervalCount = 8;
    long intervalTime = 4; //s
    long rampSettleTime = 1000; //ms

    //battery test
    long targetAmpDraw = 10; //A, target current draw during the depletion test
    long dischargeAmount = 1000; //mAh, total battery to deplete
    long currentTestGain = 50; //ms between throttle adjustments
    long voltageCutoff = 38; //V
    long batteryRecoveryTime = 30; //s
};

//Hardware settings, edited from the Configure Hardware menu
struct HardwareSettings {
    long pulsesPerRev = 4; //number of RPM markers
    long rpmUpdateRate = 250; //ms between RPM updates
    long airspeedOverride = 0; //m/s, 0 = use the airspeed sensor
    long averageGain = 25; //1-100, how strongly new current readings are weighted in the moving average
};

extern SensorData readings;
extern TestSettings settings;
extern HardwareSettings hardware;
