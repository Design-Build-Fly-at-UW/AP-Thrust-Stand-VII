#include <Arduino.h>
#include "rpm.h"
#include "config.h"
#include "state.h"

static volatile unsigned long pulses = 0; //counted by the interrupt
static unsigned long lastRpmReadTime = 0; //ms
static volatile unsigned long lastEdgeTime = 0; //us, when the last counted edge happened
static volatile unsigned long lastPeriod = RPM_MAX_LOCKOUT_US; //us between the last two counted edges
static volatile int lastLevel = !RPM_MARKER_LEVEL; //pin level at the last interrupt
static volatile unsigned long lastLevelChange = 0; //us, when the pin last read a different level than before

static void rpmISR() {
    //every edge of the sensor spikes for ~30us, so the edge where the marker leaves also shows a short rising edge.
    //Read the level on every change and only count the marker arriving: the pin switching to RPM_MARKER_LEVEL after
    //it had been off that level for at least RPM_SETTLE_US. Spikes never stay off the marker level that long
    unsigned long now = micros();
    int level = digitalRead(RPM_PIN);
    if (level == lastLevel) return; //a late interrupt that missed the spike, nothing changed
    unsigned long offTime = now - lastLevelChange;
    lastLevel = level;
    lastLevelChange = now;
    if (level != RPM_MARKER_LEVEL || offTime < RPM_SETTLE_US) return;

    //the sensor oscillates for several ms when a marker passes slowly. Any marker sooner than 3/8 of the last marker
    //period is chatter, since the speed can't nearly triple between two markers.
    //Not 1/2: if a marker is ever skipped, lastPeriod doubles and a 1/2 lockout lands right on the next real
    //marker, so it can get stuck counting every 2nd or 4th marker (reading 2-4x low). 3/8 of any multiple of the
    //marker spacing (up to 7 skipped) sits between markers, so it always works back to counting every one
    unsigned long gap = now - lastEdgeTime;

    unsigned long lockout = (lastPeriod / 8) * 3; //divide first so a long gap can't overflow
    if (lockout < RPM_MIN_LOCKOUT_US) lockout = RPM_MIN_LOCKOUT_US;
    if (lockout > RPM_MAX_LOCKOUT_US) lockout = RPM_MAX_LOCKOUT_US;
    if (gap < lockout) return;

    lastPeriod = gap;
    lastEdgeTime = now;
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
    return (pulseCount*60000.0)/(period*markers); //one pulse is counted per marker
}
