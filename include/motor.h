#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

// === Motor A ===
#define PWMA 42
#define AIN1 41
#define AIN2 40
// Encoder pins:
#define EN1A 21
#define EN1B 48
//translate it to more readable format
#define ENA1 EN1A
#define ENA2 EN1B

// === Motor B ===
#define PWMB 2
#define BIN1 37
#define BIN2 36
// Encoder pins:
#define EN2A 35
#define EN2B 45
#define ENB1 EN2A
#define ENB2 EN2B

// === Motor C ===
#define PWMC 13
#define CIN1 11
#define CIN2 12
// Encoder pins:
#define EN3A 8
#define EN3B 9
#define ENC1 EN3A
#define ENC2 EN3B

// === Motor D ===
#define PWMD 7
#define DIN1 6 
#define DIN2 5
// Encoder pins:
#define EN4A 10
#define EN4B 46
#define END1 EN4A
#define END2 EN4B

//for motor ledcWrite (CHANNELS!)
#define CH_A 0
#define CH_B 1
#define CH_C 2
#define CH_D 3

//for Standby Pin
#define STANDBY 47

//For PID Controller
// #define pulsePerRotation 15000 //for only using one encoder
#define pulsePerRotation 30000

//Position of each motor (in pulses)
extern volatile int64_t posA;
extern volatile int64_t posB;
extern volatile int64_t posC;
extern volatile int64_t posD;

extern volatile bool motorsRunning;

struct PIDMotor{
    volatile int64_t* pos;
    
    float ePrev;
    float eIntegral;
    long prevT;
    float output;
    float rpm;
    int64_t prevPos;
};

extern PIDMotor pidA;
extern PIDMotor pidB;
extern PIDMotor pidC;
extern PIDMotor pidD;

enum Direction {
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT
  };

//for motor A's encoder
void readPulseA1();
void readPulseA2();

//for motor B's encoder
void readPulseB1();
void readPulseB2();

//for motor C's encoder
void readPulseC1();
void readPulseC2();

//for motor D's encoder
void readPulseD1();
void readPulseD2();

//Initiate all motors (like pin those)
void motorInit();

void setmotor(int dir, int in1, int in2, int pwmChannel, int speed); //dir = 1 for cw ,dir = 0 for ccw

void moveRobot(int Vx,int Vy, int Rot); //speed from -255 to 255 (PWM-valid signals)

//Stop all the motor motions
void stopAllMotor();

//update the output FOR ONE MOTOR needed to reach the target RPM!
void updateOnePID(PIDMotor &pid,int targetRPM);

//if the output exceed 255 or lower than -255, we need to scale all the motors by the same ratio!
void scaleAllPID();

//Apply all the PID Output to all the motors
void applyPIDoutputs();

//start the PID process with a target rpm
void PIDControl(int targetRPM);

void rotationPIDController(int rotation);

void moveByDistanceDecel(Direction dir, float distanceCm, int maxSpeed);


#endif