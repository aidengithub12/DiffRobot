#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>

// put function declarations here:
double PIDControl(double setpoint);
void readEncoderRight();
void readEncoderLeft();
double PIDControlLeft(double Newsetpoint);
double PIDControlRight(double Newsetpoint);
//pin definitions

  //servo pins
int headServoPin = 0; //TODO: change pin numbers based off hardware
int leftArmServoPin = 0;
int RightArmServoPin = 0;
  //motorController pins
int ENA = 53;
int ENB = 52;
int IN1 = 51;
int IN2 = 49;
int IN3 = 50; 
int IN4 = 48;

  //GYRO pins
// int SCL = 0;
// int SDA = 0;

//encoder pins
int motorRightHallA = 2;
int motorRightHallB = 3;
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
const double kP = 0;
const double kI = 0;
const double KD = 0;
double setpoint=100;
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
  attachInterrupt(digitalPinToInterrupt(motorLeftHallA), (readEncoderLeft),RISING);
  attachInterrupt(digitalPinToInterrupt(motorRightHallA), (readEncoderRight), RISING);

  //read encoder value
  inputleft = analogRead(motorLeftHallA);
  inputright  =analogRead(motorRightHallA);
  pidLeft.SetMode(AUTOMATIC);
  pidRight.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  pidLeft.Compute();
  pidRight.Compute(); 
  int comp = PIDControlLeft(100);
  int c2 = PIDControlRight(100);
  if (outputright < 0 || outputleft < 0) {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  }
  else {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  }
  inputleft = 0;
  inputright = 10;
  analogWrite(ENA, outputleft);
  analogWrite(ENB, outputright);

  Serial.print(leftMotorPulses);
  Serial.print(",");
  Serial.print(rightMotorPulses);
  Serial.println("");

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