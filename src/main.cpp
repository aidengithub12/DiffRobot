#include <Arduino.h>
#include <Servo.h>
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

//encoder variables



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


  //servo pin declarations
  headServo.attach(headServoPin);
  leftArmServo.attach(leftArmServoPin);
  RightArmServo.attach(RightArmServoPin);

  //set servo zeroes
  headServo.write(headServoZero);
  leftArmServo.write(leftArmServoZero);
  RightArmServo.write(rightArmServoZero);

  //reset encoder values for drivetrain motors



  //set interruptable pin functions


}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
double PIDControl(double setpoint) {


  
}