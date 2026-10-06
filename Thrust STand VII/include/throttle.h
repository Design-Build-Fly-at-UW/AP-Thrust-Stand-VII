//ESC control. The ESC is driven like a servo.
#pragma once

extern float throttle; //current throttle setting, 0-100%. Logged with each row of data

void beginThrottle(); //attaches the ESC and sets it to zero throttle
void setThrottle(float throttleSetting); //pass a throttle from 0-100 and it will safely write it to the ESC
void stopMotor(); //sets the throttle to zero
