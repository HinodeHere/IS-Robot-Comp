#include <Arduino.h>
#include <Wire.h> //For I2C protocol
#include "sensors.h"
#include "motor.h"

#define SLAVE_ADDR 0x08
#define SCL 39
#define SDA 38
#define freq 100000

//data packet: (type, speed/rotation);
//type = {0 = stop motors,1 = forward/back, 2 = left/right,3 = left for a certain rotation}
//speed should be 120rpm (need to tune PID const!)

int8_t data[2];
volatile uint8_t type = 0;
uint8_t speed = 0;
uint8_t rotation = 0;
bool motorsRunning = false;

void receivedEvent(int bytes){
    int index = 0;
    while (Wire.available() && index < sizeof(data)){
        data[index++] = (int8_t) Wire.read();
    }

    //Print the real data packet for trying
    Serial.print("Received from Master: ");
    for (int i = 0; i < sizeof(data); i++){
        Serial.print(data[i]);
        Serial.print(" ");
    }
    Serial.println();
    
    type = data[0];
    Serial.print("Type: "); Serial.println(type);

    // TYPE 0: stop all motors
    if (type == 0){
        Serial.println("Stopping all motors");
        stopAllMotor();
        motorsRunning = false;
    }

    // TYPE 3: left movement based on rotation
    else if (type == 3){

        // write the functions later!!

        rotation = data[1];
        Serial.print("Rotation: "); Serial.println(type);
    }

    // TYPE 1: forward/back
    else if(type == 1){
        speed = data[1];
        Serial.print("Speed: "); Serial.println(type);
        motorsRunning = true;
        PIDControl(speed); //no need to stop it, will send a stop signal from the Master ESP
    }

    //TYPE 2: left/right (might not be used)
    else if(type == 2){

        //Write PID for left right!

        speed = data[1];
        Serial.print("Speed: "); Serial.println(type);
    }
}


//only used when i need to send data back to Master!
// void requestedEvent(){
//   String reply = "Pong";
//   Wire.write(reply.c_str());
//   Serial.println("Replied: Pong!");
// }


void setup(){
    motorInit();
    Wire.begin(SLAVE_ADDR,SDA,SCL,freq);
    Wire.onReceive(receivedEvent);

    // only used when i need to send data back to Master
    //   Wire.onRequest(requestedEvent);

  Serial.begin(115200);
}

void loop(){
  if (motorsRunning && type == 1){
    PIDControl(speed);
  }
  delay(50);
}


/*
ONLY FOR WHEN THE ESP32 IS NOT A SLAVE!

void loop(){
    PIDControl(60);
    delay(200);
    // Serial.print("Printing Pos C: ");
    // Serial.println(posC);
    // delay(100);


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

    // moveRobot(0,50,0);


    // int rotation = 6;
    // PIDController(rotation);
    // Serial.println(pos);
    // delay(15);



    // Serial.println(posA);

}

*/