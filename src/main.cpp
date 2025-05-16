#include <Arduino.h>
#include <Wire.h> //For I2C protocol
#include "motor.h"

#define SLAVE_ADDR 0x08
#define SCL 9
#define SDA 8
#define freq 100000

#define maxSpeed 90 //in pwm

volatile uint8_t pendingType = 0;
volatile uint8_t distance = 0;
volatile bool newCommand = false;
bool motorsRunning = false;
bool needReset = false;

//@brief data (type, distance);
//@param type {0 = stop motors,1 = forward, 2 = back, 3 = left, 4 = right, 5 = control moveRobot left side, 6 for right}
//@param distance how many cm OR speed by RPM
void receivedEvent(int bytes){
    int index = 0;
    if (Wire.available() >= 2){
        pendingType = Wire.read();
        distance = Wire.read();
        newCommand = true;
        needReset = true;
    }

    //dont print it to make it more faster
    // //Print the real data packet for trying
    // Serial.print("Received from Master: ");
    // for (int i = 0; i < sizeof(data); i++){
    //     Serial.print(data[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();
    
    // type = data[0];
    // Serial.print("Type: "); Serial.println(type);
}


//only used when i need to send data back to Master!
void requestedEvent(){
    //sends 0 if done, 1 if running
  uint8_t status = motorsRunning ? 1 : 0;
  Wire.write(&status,1);
}

// For when the ESP is a SLAVE!!

void setup(){
    Serial.begin(115200);
    motorInit();
    Wire.begin(SLAVE_ADDR,SDA,SCL,freq);
    Wire.onReceive(receivedEvent);

    // only used when i need to send data back to Master
    Wire.onRequest(requestedEvent);
}

void loop(){
    if (needReset == true){
      needReset = false;
      resetPIDError();
    }
    if (newCommand == true || pendingType == 5 || pendingType == 6){
        newCommand = false;
        int s;
        switch (pendingType) {
            case 0:
              stopAllMotor();
              motorsRunning = false;
              break;
      
            case 1:
              motorsRunning = true;
              moveByDistanceDecel(DIR_FORWARD,  distance, maxSpeed);
              break;
      
            case 2:
              motorsRunning = true;
              moveByDistanceDecel(DIR_BACKWARD, distance, maxSpeed);
              break;
      
            case 3:
              motorsRunning = true;
              moveByDistanceDecel(DIR_LEFT,     distance, maxSpeed);
              break;
      
            case 4:
              motorsRunning = true;
              moveByDistanceDecel(DIR_RIGHT,    distance, maxSpeed);
              break;
            case 5: //- for going left PID RPM
              s = distance;
              motorsRunning = true;
              PIDControl(DIR_LEFT,s);
              delay(50);
              break;
            case 6:
              s = distance;
              motorsRunning = true;
              PIDControl(DIR_RIGHT,s);
              delay(50);
              break;
        }

        motorsRunning = false;
    }
}



//ONLY FOR WHEN THE ESP32 IS NOT A SLAVE!

// void setup(){
//     motorInit();

//   Serial.begin(115200);
// }

// void loop(){
//   // moveByDistanceDecel(DIR_FORWARD,140,maxSpeed);
//   // delay(500);
//   // moveByDistanceDecel(DIR_LEFT,60,maxSpeed);
//   // delay(500);
//   // moveByDistanceDecel(DIR_BACKWARD,280,maxSpeed);
//   // delay(500);
//   // moveByDistanceDecel(DIR_LEFT,70,maxSpeed);
//   // delay(500);


//   // moveByDistanceDecel(DIR_FORWARD,280,maxSpeed);
//   // delay(500);
//   // moveByDistanceDecel(DIR_LEFT,60,maxSpeed);
//   // delay(500);
//   // moveByDistanceDecel(DIR_BACKWARD,280,maxSpeed);
//   // delay(500);
  
//   // moveByDistanceDecel(DIR_RIGHT,210,maxSpeed);
//   // delay(500);
//   // moveByDistanceDecel(DIR_FORWARD,140,maxSpeed);
//   // delay(500);


//   // moveByDistanceDecel(DIR_FORWARD,70,maxSpeed);
//   // delay(500);
//   // moveByDistanceDecel(DIR_BACKWARD,70,maxSpeed);
//   // delay(500);
  
//   // for (int i = 0; i < 15; i++){
//   //   PIDControl(60);
//   //   delay(250);
//   // }

//   // stopAllMotor();
//   // while(1){
//   //   delay(1000);
//   // }
//     // moveRobot(100,0,0);

//     // Serial.print("Printing Pos A: ");
//     // Serial.print(posA);
//     // delay(10);

//     // Serial.print(" Printing Pos B: ");
//     // Serial.print(posB);
//     // delay(10);

//     // Serial.print(" Printing Pos C: ");
//     // Serial.print(posC);
//     // delay(10);

//     // Serial.print(" Printing Pos D: ");
//     // Serial.println(posD);
//     // delay(10);

//     // // for when if reach white line it will reverse back for 500ms
//     // moveRobot(100,0,0);
//     // while(readIRsensor() != 0){
//     //     Serial.println(readIRsensor());
//     // }

//     // moveRobot(-50,0,0);
//     // delay(1000);
//     // stopAllMotor();
//     // while(1){}
    

//     // // move robot in a square
//     // for (int i = 0; i < 4; i++){
//     //     moveRobot(100,0,0);
//     //     delay(500);
//     //     stopAllMotor();
//     //     delay(500);
    
//     //     moveRobot(0,100,0);
//     //     delay(500);
//     //     stopAllMotor();
//     //     delay(500);

//     //     moveRobot(0,0,70);
//     //     delay(1000);
//     //     stopAllMotor();
//     //     delay(500);
//     // }
//     // moveRobot(0,50,0);


//     // int rotation = 6;
//     // PIDController(rotation);
//     // Serial.println(pos);
//     // delay(15);



//     // Serial.println(posA);

//     // while(1){}
// }