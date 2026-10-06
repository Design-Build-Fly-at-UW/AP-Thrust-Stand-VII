#include <Arduino.h>
#include "rpm.h"
#include "config.h"
#include "state.h"

static volatile unsigned long pulses = 0; //counted by the interrupt
static unsigned long lastRpmReadTime = 0; //ms

static void rpmISR() {
    pulses++;
}

void beginRPM(){
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, CHANGE);
}

bool rpmReady(){
    return (millis() - lastRpmReadTime) > (unsigned long)hardware.rpmUpdateRate;
}

float readRPM(){
    //grab the count and reset it in one go, so that the count corresponds to the period
    noInterrupts();
    unsigned long period = millis() - lastRpmReadTime; //ms since the last read
    unsigned long pulseCount = pulses;
    pulses = 0;
    lastRpmReadTime = millis();
    interrupts();

    long markers = (hardware.pulsesPerRev < 1) ? 1 : hardware.pulsesPerRev; //avoid dividing by zero if the marker count is set to 0
    return (pulseCount*60000.0)/(period*2.0*markers); //the 2 is because pulses are counted on both rising and falling edges
}
