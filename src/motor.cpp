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
    attachInterrupt(digitalPinToInterrupt(ENB1),readPulseB1,RISING);
    attachInterrupt(digitalPinToInterrupt(ENB2),readPulseB2,RISING);

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

    //IR Sensors:
    pinMode(IR_LEFT,INPUT);
    pinMode(IR_RIGHT,INPUT);

    

    //Make all the prevPos to be the current pos when the motor is started
    pidA.prevPos = *(pidA.pos);
    pidB.prevPos = *(pidB.pos);
    pidC.prevPos = *(pidC.pos);
    pidD.prevPos = *(pidD.pos);
}

void readPulseA1(){
    int a = digitalRead(ENA1);
    int b = digitalRead(ENA2);
    a == b ? posA++ : posA--;
}

void readPulseA2(){
    int a = digitalRead(ENA1);
    int b = digitalRead(ENA2);
    a != b ? posA++ : posA--;
}

void readPulseB1(){
    int a = digitalRead(ENB1);
    int b = digitalRead(ENB2);
    a == b ? posB++ : posB--;
}

void readPulseB2(){
    int a = digitalRead(ENB1);
    int b = digitalRead(ENB2);
    a != b ? posB++ : posB--;
}

void readPulseC1(){
    int a = digitalRead(ENC1);
    int b = digitalRead(ENC2);
    a == b ? posC++ : posC--;
}

void readPulseC2(){
    int a = digitalRead(ENC1);
    int b = digitalRead(ENC2);
    a != b ? posC++ : posC--;
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

    setmotor(fl > 0 ? 1 : 0,CIN1,CIN2,CH_C,abs(fl));  //theres a negative because the motor was upside-down
    setmotor(fr > 0 ? 1 : 0,BIN1,BIN2,CH_B,abs(fr));
    setmotor(-bl > 0 ? 1 : 0,DIN1,DIN2,CH_D,abs(bl));
    setmotor(-br > 0 ? 1 : 0,AIN1,AIN2,CH_A,abs(br));
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
    pid.rpm = ((float)deltaPos/pulsePerRotation) / deltaT * 60.0f;

    //calculate the error now!
    float e = targetRPM - pid.rpm;

    //Update the values in e    Integral,de/dt and ePrev
    pid.eIntegral += e*deltaT;
    float dedt = (e-pid.ePrev)/deltaT;
    pid.ePrev = e;

    //Tune these constants
    float Kp = 0.8;
    float Ki = 0.5;
    float Kd = 0.005;

    pid.output = Kp * e + Ki * pid.eIntegral + Kd * dedt;

    Serial.print("error: "); Serial.println(e);
    Serial.print("deltaPos: "); Serial.print(deltaPos);
    Serial.print(" | deltaT: "); Serial.print(deltaT,6);
    Serial.print(" | rpm: ");    Serial.println(pid.rpm,2);
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
    float factor = 1.13f;
    setmotor(pidA.output<=0 , AIN1, AIN2, CH_A, (int)abs(pidA.output)*factor);
    setmotor(pidB.output>=0 , BIN1, BIN2, CH_B, (int)abs(pidB.output)*factor);
    setmotor(pidC.output>=0 , CIN1, CIN2, CH_C, (int)abs(pidC.output));
    setmotor(pidD.output<=0 , DIN1, DIN2, CH_D, (int)abs(pidD.output));    
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


const float wheelRadiusCm = 5.0f;   // wheel radius in cm
const int   creep         = 30;    // minimum PWM to overcome static friction

// Position-PID gains (tune these)
const float Kp_pos = 0.8f;
const float Ki_pos = 0.01f;
const float Kd_pos = 0.08f;

// Heading-PID gains (tune these)(rotation) //left and right
const float Kp_h = 0.75f;
const float Ki_h = 0.01;
const float Kd_h = 0.01f;

int ALIGN_SPEED = 40; //for correcting yaw of robot when it crosses a while line

void moveByDistanceDecel(Direction dir, float distanceCm, int maxSpeed){
    if (dir == DIR_LEFT || dir == DIR_RIGHT){
        maxSpeed = 50;
    }
    double ratioForwards = 140.0f/147.5f;
    double ratioBackwards = 140.0f/145.0f;
    double ratioLeft = 140.0f/147.5f;
    double ratioRight = 140.0f/147.6f;
    switch(dir){
        case DIR_FORWARD: distanceCm*=ratioForwards; break;
        case DIR_BACKWARD: distanceCm*=ratioBackwards; break;
        case DIR_LEFT: distanceCm*=ratioLeft; break;
        case DIR_RIGHT: distanceCm*=ratioRight; break;
    }

    float rotation = distanceCm / (2.0f * PI * wheelRadiusCm);
    int64_t target = (int64_t)(rotation * pulsePerRotation);
    if(target < 0) return;

    int64_t sA = posA, sB = posB, sC = posC, sD = posD;

    float prevErrPos = float(target);
    float intErrPos    = 0.0f;
    float prevErrHead  = 0.0f;
    float intErrHead   = 0.0f;
    uint32_t prevTime  = micros();
    bool firstLoop     = true;

    int64_t rampUpDist = target * 0.15;
    int64_t rampDownDist = target * 0.15; //ramp up/down

    if (dir == DIR_LEFT || dir == DIR_RIGHT){
        rampUpDist = 0;
    }

    while(true){
        //measure progress
        int64_t dA = llabs(posA - sA);
        int64_t dB = llabs(posB - sB);
        int64_t dC = llabs(posC - sC);
        int64_t dD = llabs(posD - sD);
        int64_t travelled = min(min(dA,dB), min(dC,dD));
        if (travelled >= target) break;

        //calculate dt
        uint32_t now = micros();
        float dt = (now - prevTime) * 1e-6f;
        prevTime = now;
        if (firstLoop || dt < 1e-4f) {
            dt = 0.0f;
            firstLoop = false;
        }

        //position PID
        float errPos = float(target - travelled);
        intErrPos   += errPos * dt;
        float dErrPos = dt>0 ? (errPos - prevErrPos)/dt : 0.0f;
        prevErrPos = errPos;
        float uPos   = Kp_pos * errPos + Ki_pos * intErrPos + Kd_pos * dErrPos;
        int s        = constrain((int)uPos, creep, maxSpeed);


        //RampUp / RampDown
        float rampUp   = constrain((float)travelled    / rampUpDist, 0, 1);
        float rampDown = constrain((float)(target - travelled) / rampDownDist, 0, 1);
        float rampMul  = min(rampUp, rampDown);
        rampMul = max(rampMul, 0.20f); //ensure atleast got power to move

        s *= rampMul;
        if (s > 0 && s < creep){
            s = creep;
        }
        
        //heading PID (yaw)
        float leftAvg;
        float rightAvg;
        if (dir == DIR_BACKWARD || dir ==DIR_FORWARD){
            leftAvg  = ( llabs(posC - sC) + llabs(posD - sD) ) * 0.5f;
            rightAvg = ( llabs(posA - sA) + llabs(posB - sB) ) * 0.5f;
        } else if (dir == DIR_LEFT){
            leftAvg  = ( llabs(posA - sA) + llabs(posD - sD) ) * 0.5f;
            rightAvg = ( llabs(posC - sC) + llabs(posB - sB) ) * 0.5f; 
        } else{
            rightAvg  = ( llabs(posA - sA) + llabs(posD - sD) ) * 0.5f;
            leftAvg = ( llabs(posC - sC) + llabs(posB - sB) ) * 0.5f; 

        }
        float errHead   = rightAvg - leftAvg;
        intErrHead     += errHead * dt;
        float dErrHead  = dt>0 ? (errHead - prevErrHead)/dt : 0.0f;
        prevErrHead    = errHead;
        float uHead     = Kp_h * errHead + Ki_h * intErrHead + Kd_h * dErrHead;
        int rotVel     = constrain((int)uHead, -maxSpeed, maxSpeed);
        Serial.print("errYaw="); Serial.println(intErrHead);


        //change vx,vy and rotvel by PID!
        int vx=0, vy=0;
        switch (dir) {
            case DIR_FORWARD:  vx =  s; vy = 0; break;
            case DIR_BACKWARD: vx = -s; vy = 0; rotVel = -rotVel; break;
            case DIR_LEFT:     vx =  0; vy = -s; break;
            case DIR_RIGHT:    vx =  0; vy =  s; break;
        }

        // //line sensor overwrite yaw PID on white line
        // bool leftLine = (digitalRead(IR_LEFT) == LOW);
        // bool rightLine = (digitalRead(IR_RIGHT) == LOW);
        // if ((leftLine || rightLine) && (dir == DIR_BACKWARD || dir == DIR_FORWARD)){
        //     //halt x and y direction movement
        //     vx*=0.3;
        //     vy*=0.3;
        //     if (leftLine && !rightLine) rotVel = -ALIGN_SPEED; //turn right
        //     else if (rightLine && !leftLine) rotVel = ALIGN_SPEED;
        //     else rotVel = 0;

        //     prevErrHead = 0;
        //     intErrHead = 0;
        //     if (dir == DIR_BACKWARD) rotVel = -rotVel;
        // }

        moveRobot(vx, vy, rotVel);
        // Serial.print("vx="); Serial.println(vx);
        // Serial.print(" vy="); Serial.println(vy);
        // Serial.print(" rot="); Serial.println(rotVel);

        delay(2);
    }
    stopAllMotor();

}