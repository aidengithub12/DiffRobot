#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>

// put function declarations here:
double PIDControl(double setpoint) {};

//pin definitions

  //servo pins
int headServoPin = 0; //TODO: change pin numbers based off hardware
int leftArmServoPin = 0;
int RightArmServoPin = 0;
  //motorController pins
int ENA = 0;
int ENB = 0;
int IN1 = 0;
int IN2 = 0;
int IN3 = 0; 
int IN4 = 0;

  //GYRO pins
int SCL = 0;
int SDA = 0;

//encoder pins
int motorRightHallA;
int motorRightHallB;
int motorLeftHallA;
int motorLeftHallB; //TODO: set pin values

//encoder variables
volatile int leftMotorPulses = 0;
volatile int rightMotorPulses = 0;


//logging information


//Gyro information


//Servo definitions
Servo headServo;
Servo leftArmServo;
Servo RightArmServo;


//Servo zeroes
int headServoZero = 0;
int leftArmServoZero = 0;
int rightArmServoZero = 0;


//PID values
const double kP = 1;
const double kI = 0;
const double KD = 0;
double setpoint;
double inputleft;
double outputleft;
double inputright;
double outputright;


PID pidLeft(&inputleft, &outputleft, &setpoint, kP, kI, KD, DIRECT);
PID pidRight(&inputright, &outputright, &setpoint, kP, kI, KD, DIRECT);


void setup() {
  //Start serial monitor
  Serial.begin(9600);

  //pinmodes
  pinMode(headServoPin, OUTPUT);
  pinMode(leftArmServoPin, OUTPUT);
  pinMode(RightArmServoPin, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SCL, OUTPUT);
  pinMode(SDA, INPUT); //TODO: Check this
  pinMode(motorLeftHallA, INPUT_PULLUP);
  pinMode(motorLeftHallB, INPUT_PULLUP);
  pinMode(motorRightHallA, INPUT_PULLUP);
  pinMode(motorRightHallB, INPUT_PULLUP);


  //servo pin declarations
  headServo.attach(headServoPin);
  leftArmServo.attach(leftArmServoPin);
  RightArmServo.attach(RightArmServoPin);

  //set servo zeroes
  headServo.write(headServoZero);
  leftArmServo.write(leftArmServoZero);
  RightArmServo.write(rightArmServoZero);

  //reset encoder values for drivetrain motors
  rightMotorPulses = 0.0;
  leftMotorPulses = 0.0;


  //set interruptable pin functions
  attachInterrupt(motorLeftHallA, (readEncoderLeft),RISING);
  attachInterrupt(motorRightHallA, (readEncoderRight), RISING);

  //read encoder value
  // inputleft = analogRead(encoderPinLeft);
  // inputright = =analogRead(encoderPinRight);

}

void loop() {
  // put your main code here, to run repeatedly:
  PIDControlLeft(100);
  PIDControlRight(100);
  analogWrite(ENA, outputleft);
  analogWrite(ENB, outputright);
}

// put function definitions here:
double PIDControlLeft(double Newsetpoint) {
  setpoint = Newsetpoint;
  
  return pidLeft.Compute(); 
}
double PIDControlRight(double Newsetpoint) {
  setpoint = Newsetpoint;
  
  return pidRight.Compute(); 
}

void readEncoderLeft(){
  int b = digitalRead(motorLeftHallB);
  if(b > 0){
    leftMotorPulses++;
  }
  else{
    leftMotorPulses--;
  }
}
void readEncoderRight(){
  int b = digitalRead(motorRightHallB);
  if(b > 0){
    rightMotorPulses++;
  }
  else{
    rightMotorPulses--;
  }
}