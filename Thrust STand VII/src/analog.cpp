#include <Arduino.h>
#include "analog.h"
#include "config.h"
#include "state.h"
#include "screens.h"

static float currentOffset = 0; //amps, set by zeroing the current sensor
static float voltageOffset = 0; //volts, the voltage divider reads 0 at 0V so this stays 0

static float averageAnalogRead(int pin){ //returns the average of ANALOG_AVERAGE_COUNT readings, in volts
    float sum = 0;
    for (int i = 0; i < ANALOG_AVERAGE_COUNT; i++) {
        sum = sum + analogRead(pin);
    }
    float average = sum/ANALOG_AVERAGE_COUNT; //kept as a float so the averaging adds resolution
    return average * (Vcc / 1023.0); //convert from analog 0-1023 back to volts
}

float getVoltage(){
    return VOLTAGE_CALIBRATION * averageAnalogRead(VOLTAGE_PIN) - voltageOffset;
}

float getCurrent(){
    return averageAnalogRead(CURRENT_PIN)/CURRENT_SENSITIVITY - currentOffset;
}

float getAirspeed(){
    if (hardware.airspeedOverride != 0){ //use the airspeed the user set, if they chose an override
        return hardware.airspeedOverride;
    }

    float pressure_kPa = (averageAnalogRead(AIRSPEED_PIN) - AIRSPEED_ZERO_VOLTAGE) / AIRSPEED_SENSITIVITY; //differential pressure
    float pressure_Pa = pressure_kPa * 1000.0;

    if (pressure_Pa <= 0) {
        return 0;
    }
    return sqrt((2.0 * pressure_Pa) / AIR_DENSITY); //Bernoulli equation
}

static float findAnalogOffset(float (*valueFunction)()){ //pass a function that returns a value. Averages a bunch of samples to find its offset from 0
    const int N = 30; //number of samples
    float sum = 0;
    for (int i = 0; i < N; i++) {
        sum = sum + valueFunction();
        delay(50);
    }
    return (sum/N);
}

void zeroCurrentSensor(){
    //getCurrent already subtracts the old offset, so add on whatever is left over
    currentOffset = currentOffset + findAnalogOffset(getCurrent);
}

void zeroAnalog(){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_22b_tr);
    u8g2.drawStr(14, 39, "Zeroing");
    u8g2.sendBuffer();

    //only the current sensor is zeroed. The voltage divider reads 0 at 0V already, and zeroing it with a battery plugged in would make voltage read 0
    zeroCurrentSensor();
}
