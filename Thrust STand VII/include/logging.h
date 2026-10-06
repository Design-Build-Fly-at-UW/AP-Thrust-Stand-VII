//SD card logging. Each test writes one CSV file named Test_<number>.csv.
#pragma once

void beginSD(); //starts the SD card at boot. Keeps retrying until a card is found, or the user presses a key to skip it
bool setUpTest(); //asks for a test number, creates the data file, and asks the user to start. Returns true if the test should begin
void writeSensorSD(); //writes one row of the current readings to the data file
void closeTestFile(); //closes the data file and moves on to the next test number
