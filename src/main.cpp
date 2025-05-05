#include <Arduino.h>
#include <Wire.h> //For I2C protocol
#include "sensors.h"
#include "motor.h"

#define SLAVE_ADDR 0x08
#define SCL 39
#define SDA 38
#define freq 100000

#define maxSpeed 200

volatile uint8_t pendingType = 0;
volatile uint8_t distance = 0;
volatile bool newCommand = false;

//@brief data (type, distance);
//@param type {0 = stop motors,1 = forward, 2 = back, 3 = left, 4 = right}
//@param speed （not using) should be 120rpm(?) (need to tune PID const!) 
//@param distance how many cm
void receivedEvent(int bytes){
    int index = 0;
    if (Wire.available() >= 2){
        pendingType = Wire.read();
        distance = Wire.read();
        newCommand = true;
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
  String reply = "Pong";
  Wire.write(reply.c_str());
  Serial.println("Replied: Pong!");
}

// For when the ESP is a SLAVE!!

void setup(){
    Serial.begin(115200);
    motorInit();
    Wire.begin(SLAVE_ADDR,SDA,SCL,freq);
    Wire.onReceive(receivedEvent);

    // only used when i need to send data back to Master
    //   Wire.onRequest(requestedEvent);
}

void loop(){
    if (newCommand == true){
        newCommand = false;

        switch (pendingType) {
            case 0:
              stopAllMotor();
              break;
      
            case 1:
              moveByDistanceDecel(DIR_FORWARD,  distance, maxSpeed);
              break;
      
            case 2:
              moveByDistanceDecel(DIR_BACKWARD, distance, maxSpeed);
              break;
      
            case 3:
              moveByDistanceDecel(DIR_LEFT,     distance, maxSpeed);
              break;
      
            case 4:
              moveByDistanceDecel(DIR_RIGHT,    distance, maxSpeed);
              break;
        }
    }
}



//ONLY FOR WHEN THE ESP32 IS NOT A SLAVE!

// void setup(){
//     motorInit();

//   Serial.begin(115200);
// }

// void loop(){
    // moveRobot(100,0,0);

    // Serial.print("Printing Pos A: ");
    // Serial.print(posA);
    // delay(100);

    // Serial.print(" Printing Pos B: ");
    // Serial.print(posB);
    // delay(100);

    // Serial.print(" Printing Pos C: ");
    // Serial.print(posC);
    // delay(100);

    // Serial.print(" Printing Pos D: ");
    // Serial.println(posD);
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

// }