#include <Arduino.h>

// Motor A:
#define PWMA 42
#define AIN1 40
#define AIN2 41
// #define EN1A 35
// #define EN1B 36

//Motor B:
#define PWMB 7
#define BIN1 5
#define BIN2 6
// ENCODER PINS
#define EN4A 47
#define EN4B 48

//Motor C:
#define PWMC 2
#define CIN1 37 
#define CIN2 36

//Motor D:
#define PWMD 13
#define DIN1 11
#define DIN2 12


//for ledcWrite
#define CH_A 0
#define CH_B 1
#define CH_C 2
#define CH_D 3

#define IRsensor 36


// #define pulsePerRotation 15000 //for only using one encoder
#define pulsePerRotation 30000
#define STANDBY 1


//PID Controller
volatile int pos = 0;
float ePrev = 0;
float eIntegral = 0;
long prevT = 0;

void readPulseA(){
    int a = digitalRead(EN4A);
    int b = digitalRead(EN4B);
    a == b ? pos-- : pos++;
}

void readPulseB(){
    int a = digitalRead(EN4A);
    int b = digitalRead(EN4B);
    a != b ? pos-- : pos++;
}

void setup(){
    Serial.begin(115200);
    pinMode(STANDBY,OUTPUT);
    pinMode(IRsensor,INPUT);

    ledcSetup(CH_A,5000,8);
    ledcSetup(CH_B,5000,8);
    ledcSetup(CH_C,5000,8);
    ledcSetup(CH_D,5000,8);

    ledcAttachPin(PWMA,CH_A);
    ledcAttachPin(PWMB,CH_B);
    ledcAttachPin(PWMC,CH_C);
    ledcAttachPin(PWMD,CH_D);

    pinMode(AIN1,OUTPUT);
    pinMode(AIN2,OUTPUT);
    pinMode(BIN1,OUTPUT);
    pinMode(BIN2,OUTPUT);
    pinMode(CIN1,OUTPUT);
    pinMode(CIN2,OUTPUT);
    pinMode(DIN1,OUTPUT);
    pinMode(DIN2,OUTPUT);

    //Encoders
    pinMode(EN4A,INPUT_PULLUP);
    pinMode(EN4B,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EN4A),readPulseA,RISING);
    attachInterrupt(digitalPinToInterrupt(EN4B),readPulseB,RISING);


    digitalWrite(STANDBY,1);

    // attachInterrupt(digitalPinToInterrupt(EN1A),readEncoder,CHANGE);
    delay(500);
}

void setmotor(int dir, int in1, int in2, int pwmChannel, int speed){ //1 for cw 0 for ccw
    digitalWrite(in1,dir? 1 : 0);
    digitalWrite(in2,dir? 0 : 1);
    ledcWrite(pwmChannel,speed);
}

void moveRobot(int Vx,int Vy, int Rot){ //speed from -255 to 255
    int fl = constrain(Vx + Vy + Rot, -255, 255);
    int fr = constrain(Vx - Vy - Rot, -255, 255);
    int bl = constrain(Vx - Vy + Rot, -255, 255);
    int br = constrain(Vx + Vy - Rot, -255, 255);

    setmotor(-fl > 0 ? 1 : 0,BIN1,BIN2,CH_B,abs(fl));  //theres a negative because the motor was upside-down
    setmotor(-fr > 0 ? 1 : 0,AIN1,AIN2,CH_A,abs(fr));
    setmotor(bl > 0 ? 1 : 0,DIN1,DIN2,CH_D,abs(bl));
    setmotor(br > 0 ? 1 : 0,CIN1,CIN2,CH_C,abs(br));
}

void stopAllMotor(){
    setmotor(1,AIN1,AIN2,CH_A,0);
    setmotor(1,BIN1,BIN2,CH_B,0);
    setmotor(1,CIN1,CIN2,CH_C,0);
    setmotor(1,DIN1,DIN2,CH_D,0);
}

int readIRsensor(){
    int value = digitalRead(IRsensor);
    return value;
}


void PIDController(int rotation){
    //target as in rotations
    int target = rotation * pulsePerRotation;
    float Kp = 0.015;
    float Ki = 0;
    float Kd = 0;

    long curr = micros();

    float deltaT = ((float)curr-prevT)/1.0e6;
    prevT = curr;

    //error calculation
    int e = target - pos;
    
    //deriative calculation dedt = de/dt
    int dedt = (e-ePrev)/(deltaT);

    //integral
    eIntegral = eIntegral + e*deltaT;

    //Control signal finally: (speed)
    float u = Kp*e + Kd*dedt + Ki*eIntegral;

    //store previous error for use in de/dt
    ePrev = e;

    u = constrain(u,-100,100); //constrain u between -255 to 255 (valid pwm range) but -100 to 100 is enuf

    moveRobot(u,0,0);
    Serial.print(target);
    Serial.print(" ");
    Serial.print(e);
    Serial.print(" ");
    Serial.print(u);
    Serial.print(" ");
    Serial.println(pos);
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
