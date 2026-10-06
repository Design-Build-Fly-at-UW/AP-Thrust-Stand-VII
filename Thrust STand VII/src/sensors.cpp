#include <Arduino.h>
#include "sensors.h"
#include "state.h"
#include "load_cells.h"
#include "analog.h"
#include "rpm.h"

static unsigned long lastMahTime = 0; //ms, when mAh drawn was last updated

void resetSensorData(){
    readings = SensorData(); //sets everything back to 0
    readings.testStartTime = millis();
    lastMahTime = millis();
}

void restartMahTimer(){
    lastMahTime = millis();
}

void readSensorData(){

    //clamp the gain to 1-100. A gain of 0 would freeze the current reading
    if (hardware.averageGain > 100) {
        Serial.println("Avg gain out of bounds");
        hardware.averageGain = 100;
    } else if (hardware.averageGain < 1) {
        Serial.println("Avg gain out of bounds");
        hardware.averageGain = 1;
    }

    //RPM only updates once per update period, otherwise keeps the old value
    if (rpmReady()) {
        readings.rpm = readRPM();
    }

    //read torque and thrust if ready, otherwise keeps the old values
    if(thrustSensor.is_ready()){
        readings.thrust = thrustSensor.get_units();
    }
    if(torqueSensor.is_ready()){
        readings.torque1 = torqueSensor.get_units();
    }
    if(torqueSensor2.is_ready()){
        readings.torque2 = torqueSensor2.get_units();
    }
    readings.torque = (readings.torque1 + readings.torque2) / 2; //final torque is the average of both sensors

    //read analog sensors
    readings.voltage = getVoltage();
    float gain = hardware.averageGain/100.0;
    readings.current = (1-gain)*readings.current + gain*getCurrent(); //moving average
    readings.airspeed = getAirspeed();

    //calculated values
    readings.electricPower = abs(readings.voltage*readings.current); //watts
    readings.mechanicalPower = abs(readings.torque*readings.rpm*0.1047/1000); //RPM is converted to rad/s, torque is converted to N.m from N.mm
    readings.propellerPower = abs(readings.thrust*readings.airspeed/1000); //thrust is converted to N from mN
    readings.motorEfficiency = (readings.electricPower > 0) ? abs(readings.mechanicalPower/readings.electricPower)*100 : 0; //percent
    readings.propellerEfficiency = (readings.mechanicalPower > 0) ? abs(readings.propellerPower/readings.mechanicalPower)*100 : 0; //percent
    readings.systemEfficiency = (readings.electricPower > 0) ? abs(readings.propellerPower/readings.electricPower)*100 : 0; //percent

    unsigned long now = millis();
    readings.mahDrawn += readings.current*(now - lastMahTime)/3600.0; //amps * ms / 3600 = mAh
    lastMahTime = now;

    readings.testTime = now - readings.testStartTime;
}
