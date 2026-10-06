#include <Arduino.h>
#include <Servo.h> //used for controlling the ESC, which is a servo
#include "throttle.h"
#include "config.h"

static Servo esc;
float throttle = 0;

void beginThrottle(){
    esc.attach(ESC_PIN);
    esc.writeMicroseconds(MIN_THROTTLE);
}

void setThrottle(float throttleSetting){
    int throttleMicroseconds = MIN_THROTTLE;

    if (throttleSetting > 100){ //clamp to full throttle rather than cutting the motor, float rounding can land slightly above 100
        throttleSetting = 100;
    }

    if (!isnan(throttleSetting) && throttleSetting >= 0){ //if the throttle is negative or not a number, leave it at 0
        throttleMicroseconds = ((throttleSetting/100.0)*(MAX_THROTTLE-MIN_THROTTLE)+MIN_THROTTLE);
    }

    esc.writeMicroseconds(throttleMicroseconds);
}

void stopMotor(){
    throttle = 0;
    setThrottle(0);
}
