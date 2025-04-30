#ifndef MOTOR_H
#define MOTOR_H


// === Motor A ===
#define PWMA 42
#define AIN1 40
#define AIN2 41
// Encoder pins:
#define EN1A -1
#define EN1B -1
//translate it to more readable format
#define ENA1 EN1A
#define ENA2 EN1B

// === Motor B ===
#define PWMB 7
#define BIN1 5
#define BIN2 6
// Encoder pins:
#define EN2A -1
#define EN2B -1
#define ENB1 EN2A
#define ENB2 EN2B

// === Motor C ===
#define PWMC 2
#define CIN1 37 
#define CIN2 36
// Encoder pins:
#define EN3A -1
#define EN3B -1
#define ENC1 EN3A
#define ENC2 EN3B

// === Motor D ===
#define PWMD 13
#define DIN1 11
#define DIN2 12
// Encoder pins:
#define EN4A -1
#define EN4B -1
#define END1 EN4A
#define END2 EN4B

//for motor ledcWrite (CHANNELS!)
#define CH_A 0
#define CH_B 1
#define CH_C 2
#define CH_D 3

//for Standby Pin
#define STANDBY 1


//For PID Controller
// #define pulsePerRotation 15000 //for only using one encoder
#define pulsePerRotation 30000
#define STANDBY 1


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


void stopAllMotor();

void PIDController(int rotation);


#endif