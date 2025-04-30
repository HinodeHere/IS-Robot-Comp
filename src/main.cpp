#include <Arduino.h>
#include "sensors.h"
#include "motor.h"

void setup(){
    //Initiate everything
    sensorsInit();
    motorInit();
}


void loop(){
    // for when if reach white line it will reverse back for 500ms
    // moveRobot(100,0,0);
    // while(readIRsensor() != 0){
    //     Serial.println(readIRsensor());
    // }

    // moveRobot(-50,0,0);
    // delay(1000);
    // stopAllMotor();
    // while(1){}
    

    // move robot in a square
    // for (int i = 0; i < 4; i++){
    //     moveRobot(100,0,0);
    //     delay(500);
    //     stopAllMotor();
    //     delay(500);
    
    //     moveRobot(0,100,0);
    //     delay(500);
    //     stopAllMotor();
    //     delay(500);

    //     moveRobot(0,0,70);
    //     delay(1000);
    //     stopAllMotor();
    //     delay(500);
    // }
    // while(1){}

    moveRobot(0,50,0);


    // int rotation = 6;
    // PIDController(rotation);
    // Serial.println(pos);
    // delay(15);

}
