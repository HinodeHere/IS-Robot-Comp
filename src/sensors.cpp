#include "sensors.h"
#include <Arduino.h>

void sensorsInit(){
    pinMode(IRsensor,INPUT);
}


int readIRsensor(){
    int value = digitalRead(IRsensor);
    return value;
}