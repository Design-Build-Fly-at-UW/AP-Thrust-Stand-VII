//Analog sensors: battery voltage, current, and airspeed.
#pragma once

float getVoltage(); //volts
float getCurrent(); //amps, with the zero offset removed
float getAirspeed(); //m/s, or the override from the hardware menu if one is set

void zeroCurrentSensor(); //measures the current sensor's zero point. Only call with no current flowing
void zeroAnalog(); //menu action: shows a screen and zeroes the current sensor
