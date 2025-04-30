#include "motor.h"
#include <Arduino.h>


volatile int64_t posA = 0;
volatile int64_t posB = 0;
volatile int64_t posC = 0;
volatile int64_t posD = 0;

void motorInit(){
    pinMode(STANDBY,OUTPUT);

    //ledcSetup(Channel number, frequency, resolution)
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


    //Motor A encoder
    pinMode(ENA1,INPUT_PULLUP);
    pinMode(ENA2,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENA1),readPulseA1,RISING);
    attachInterrupt(digitalPinToInterrupt(ENA2),readPulseA2,RISING);

    //Motor B encoder
    pinMode(ENB1,INPUT_PULLUP);
    pinMode(ENB2,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENB1),readPulseB1,CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENB2),readPulseB2,CHANGE);

    //Motor C encoder
    pinMode(ENC1,INPUT_PULLUP);
    pinMode(ENC2,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC1),readPulseC1,RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2),readPulseC2,RISING);

    //Motor D encoder
    pinMode(END1,INPUT_PULLUP);
    pinMode(END2,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(END1),readPulseD1,RISING);
    attachInterrupt(digitalPinToInterrupt(END2),readPulseD2,RISING);



    digitalWrite(STANDBY,1); //make all motors to be in Stanby Mode (i.e. it can move now)

    //Make all the prevPos to be the current pos when the motor is started
    pidA.prevPos = *(pidA.pos);
    pidB.prevPos = *(pidB.pos);
    pidC.prevPos = *(pidC.pos);
    pidD.prevPos = *(pidD.pos);
}

void readPulseA1(){
    int a = digitalRead(ENA1);
    int b = digitalRead(ENA2);
    a == b ? posA-- : posA++;
}

void readPulseA2(){
    int a = digitalRead(ENA1);
    int b = digitalRead(ENA2);
    a != b ? posA-- : posA++;
}

void readPulseB1(){
    int a = digitalRead(ENB1);
    int b = digitalRead(ENB2);
    a == b ? posB-- : posB++;
}

void readPulseB2(){
    int a = digitalRead(ENB1);
    int b = digitalRead(ENB2);
    a != b ? posB-- : posB++;
}

void readPulseC1(){
    int a = digitalRead(ENC1);
    int b = digitalRead(ENC2);
    a == b ? posC-- : posC++;
}

void readPulseC2(){
    int a = digitalRead(ENC1);
    int b = digitalRead(ENC2);
    a != b ? posC-- : posC++;
}

void readPulseD1(){
    int a = digitalRead(END1);
    int b = digitalRead(END2);
    a == b ? posD-- : posD++;
}

void readPulseD2(){
    int a = digitalRead(END1);
    int b = digitalRead(END2);
    a != b ? posD-- : posD++;
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


PIDMotor pidA = {&posA,0.0f,0.0f,0L,0.0f,0.0f,0};
PIDMotor pidB = {&posB,0.0f,0.0f,0L,0.0f,0.0f,0};
PIDMotor pidC = {&posC,0.0f,0.0f,0L,0.0f,0.0f,0};
PIDMotor pidD = {&posD,0.0f,0.0f,0L,0.0f,0.0f,0};

//update the output FOR ONE MOTOR needed to reach the target RPM!
void updateOnePID(PIDMotor &pid,int targetRPM){
    //Calculate deltaT and save currT to prevT
    long currT = micros();
    if (pid.prevT == 0){
        pid.prevT   = currT;
        pid.prevPos = *(pid.pos);
        return;
    }
    float deltaT = (currT-pid.prevT)/1.0e6;
    pid.prevT = currT;

    //calculate pos moved in deltaT time
    int64_t currPos = *(pid.pos);
    int64_t deltaPos = currPos - pid.prevPos;
    pid.prevPos = currPos;

    //calculate the RPM at the current timestamp
    pid.rpm = (deltaPos/pulsePerRotation) / deltaT * 60.0;

    //calculate the error now!
    float e = targetRPM - pid.rpm;

    //Update the values in eIntegral,de/dt and ePrev
    pid.eIntegral += e*deltaT;
    float dedt = (e-pid.ePrev)/deltaT;
    pid.ePrev = e;

    //Tune these constants
    float Kp = 0.005;
    float Ki = 0;
    float Kd = 0;

    pid.output = Kp * e + Ki * pid.eIntegral + Kd * dedt;
}

//Now if some exceed 255 or lower than -255, we need to scale all the motors by ratio!
void scaleAllPID(){
    float outA = abs(pidA.output);
    float outB = abs(pidB.output);
    float outC = abs(pidC.output);
    float outD = abs(pidD.output);

    float maxOut = max(max(outA,outB),max(outC,outD));
    if (maxOut >= 255.0f){
        float ratio = 255.0f / maxOut;
        pidA.output *= ratio;
        pidB.output *= ratio;
        pidC.output *= ratio;
        pidD.output *= ratio;
    }
}

//Apply all the PID Output to all the motors
void applyPIDoutputs(){
    setmotor(pidA.output>=0 , AIN1, AIN2, CH_A, (int)abs(pidA.output));
    setmotor(pidB.output>=0 , BIN1, BIN2, CH_B, (int)abs(pidB.output));
    setmotor(pidC.output>=0 , CIN1, CIN2, CH_C, (int)abs(pidC.output));
    setmotor(pidD.output>=0 , DIN1, DIN2, CH_D, (int)abs(pidD.output));    
}


//start the PID process with a target rpm
void PIDControl(int targetRPM){
    updateOnePID(pidA,targetRPM);
    updateOnePID(pidB,targetRPM);
    updateOnePID(pidC,targetRPM);
    updateOnePID(pidD,targetRPM);

    scaleAllPID();
    applyPIDoutputs();

    //Print target rpm
    Serial.print("RPM: "); Serial.print(targetRPM);

    //Print each different motor's RPM now
    Serial.print(" | A: "); Serial.print(pidA.rpm, 1);
    Serial.print(" B: ");   Serial.print(pidB.rpm, 1);
    Serial.print(" C: ");   Serial.print(pidC.rpm, 1);
    Serial.print(" D: ");   Serial.println(pidD.rpm, 1);
}








// NOT USED
//Only for one Motor
void rotationPIDController(int rotation){
    //Variables needed
    volatile int pos = 0;
    float ePrev = 0;
    float eIntegral = 0;
    long prevT = 0;


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