//RPM sensor, read by counting pulses on an interrupt pin.
#pragma once

void beginRPM(); //attaches the RPM interrupt
bool rpmReady(); //true once the RPM update period (set in the hardware menu) has passed
float readRPM(); //returns the RPM since the last read and restarts the count. Only call when rpmReady()
