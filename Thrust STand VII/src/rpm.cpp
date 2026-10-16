#include <Arduino.h>
#include "rpm.h"
#include "config.h"
#include "state.h"

static volatile unsigned long pulses = 0; //counted by the interrupt
static unsigned long lastRpmReadTime = 0; //ms
static volatile unsigned long lastEdgeTime = 0; //us, when the last counted edge happened
static volatile unsigned long lastPeriod = RPM_MAX_LOCKOUT_US; //us between the last two counted edges

static void rpmISR() {
    //the sensor oscillates for several ms when a marker passes slowly. Any edge sooner than half the last marker
    //period is chatter, since the speed can't double between two markers. This also skips the burst when the
    //marker leaves, as long as that's less than half the cycle
    unsigned long now = micros();
    unsigned long gap = now - lastEdgeTime;

    unsigned long lockout = lastPeriod / 2;
    if (lockout < RPM_MIN_LOCKOUT_US) lockout = RPM_MIN_LOCKOUT_US;
    if (lockout > RPM_MAX_LOCKOUT_US) lockout = RPM_MAX_LOCKOUT_US;
    if (gap < lockout) return;

    lastPeriod = gap;
    lastEdgeTime = now;
    pulses++;
}

void beginRPM(){
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RPM_EDGE);
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
    return (pulseCount*60000.0)/(period*markers); //one pulse is counted per marker
}
