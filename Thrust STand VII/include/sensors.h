//Reads every sensor into `readings` (see state.h) and calculates power and efficiency.
#pragma once

void readSensorData(); //updates all readings with the most recent sensor values
void resetSensorData(); //zeroes all readings and restarts the test clock. Call at the start of a test
void restartMahTimer(); //call after a pause so the paused time isn't counted toward mAh drawn
