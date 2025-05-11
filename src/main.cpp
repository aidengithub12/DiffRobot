//necessary imports
#include <Arduino.h>
//function definitions
void joystickControl();
void moveMotor(int speed, int direction, int currentSpeed);
void accelerate();
void countPulsesA();
void countPulsesB();
void runPID(int target, int pulses, int motorEnable, int motorIN1, int motorIN2);         
void setMotor(int dir, int speed, int motorEnable, int in1, int in2);
//Motor Controller A
int enA = 12;
int IN1 = 25;
int IN2 = 33;

//motor controller B
int enB = 52;
int IN1B = 50;
int IN2B = 48;  

//Joystick pins for manual control -- may be changed to actual controller in future
int vrx = 18;
int vry = 20;



//encoder pins -- motor controller A2
// int encoderPhaseA2 = 300;
// int encoderPhaseB2 = 180;
//encoder pins -- Motor Controller A1
int encoderPhaseA = 35; //digital interrupt pin
//Encoder power = 5V pin
//Encoder ground = gnd pin
int encoderPhaseB = 32;

//voltage values
float maxVoltage = 12.0f;

//Encoder Values
volatile long encoderPulsesA = 0;
volatile long encoderPulsesB = 0;

//PID Constants
double kP = 15;
double kI = 5;
double KD = 0;

//time variables
float startTime = 0;
float currentTime = 0;

//wheel diameter
float wheelDiameter = 0.035; //meters
//motor speed
float motorRPM = 0; //RPM
//car speed
float linearDisplacement = 0;
//number of motors
int numMotors = 2;
//current values
int currentPosition = 0; //in rotations
int desiredPosition = 0; //in meters
int error = 0;
int currentSpeed = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
const int tolerance = 0.5;
int target = 10; //meters
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Serial1.begin(115200,Serial_8N1,17,16);
  // Serial.println("Startup process began...");
  //setup pins
  pinMode(enA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  //set up encoder pins
  pinMode(encoderPhaseA, INPUT_PULLUP);
  pinMode(encoderPhaseB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPhaseA), countPulsesA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderPhaseA2), countPulsesB, CHANGE);
  //turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // digitalWrite(IN1B, LOW);
  // digitalWrite(IN2B, LOW);
  

  //timer start
  startTime = micros() / 1e-6;
  //Setup Finished
  // Serial.println("Setup has finished.");
}

void loop() {
  joystickControl();
  delay(200);
}


void runPID(int target, int pulses, int motorEnable, int motorIN1, int motorIN2) {

  currentTime = micros() / 1e-6;


  if ((currentTime - startTime) > 1) {
    startTime = micros() / 1e-6;
    noInterrupts();
    motorRPM = ((encoderPulsesA + encoderPulsesB) / ( micros() - currentTime) * .5);
    interrupts();
  }
  linearDisplacement = ((encoderPulsesA)* 3.14 * wheelDiameter) / 12; 

  // //time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT))/(1.0e6);
  prevT = currT;
  int pos = 0;
  noInterrupts();
  pos = linearDisplacement;
  interrupts();
  //error
  int e = pos-target;

  // //derivative
  float dedt = (e-eprev) / (deltaT);

  //integral
  eintegral = eintegral + e * deltaT;

  //control signal
  float u = kP*e + KD*dedt + kI*eintegral;
  //motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  //motor direction
  int dir = 1;
  if (u<0) {
    dir = -1;
  }
  if (abs(e) < tolerance || pwr < 75) {
    pwr = 0;
  }
  Serial.println(e);
  Serial.println(pwr);

  //signal the motor
  setMotor(dir, pwr, enA, IN1, IN2);

  //store previous error
  eprev = e;

  //logs
  Serial.print(">setpoint:");
  Serial.println(target);
  Serial.print(">currentpos:");
  Serial.println(pos);
  // Serial.println("*********************************************************");
  // Serial.print(target);
  // Serial.print(" ");
  // Serial.print(encoderPulsesA);
  // Serial.println();
  // Serial.println(pwr);
  // Serial.println(analogRead(enA));
  // Serial.println(currentSpeed);
  // Serial.println(desiredPosition);
  // Serial.println(motorRPM);
  Serial.println("*********************************************************");
  while (Serial.available() > 0) {
    target = Serial.readString().toInt();
    Serial.flush();
  }
  delay(200);
}

void countPulsesA() {
  
  if (digitalRead(encoderPhaseB) != digitalRead(encoderPhaseA)) {
    // Serial.println("Encoder --");
    encoderPulsesA--;
    // Serial.println((encoderPulses));
    
    // Serial.println("encoder A: " + (String) digitalRead(encoderPhaseA));
    // Serial.println("Encoder B: " + (String) digitalRead(encoderPhaseB));
  }
  else {
    encoderPulsesA++;
  }
}
// void countPulsesB() {
  
//   if (digitalRead(encoderPhaseB2) != digitalRead(encoderPhaseA2)) {
//     // Serial.println("Encoder --");
//     encoderPulsesB--;
//     // Serial.println((encoderPulses));
    
//     // Serial.println("encoder A: " + (String) digitalRead(encoderPhaseA));
//     // Serial.println("Encoder B: " + (String) digitalRead(encoderPhaseB));
//   }
//   else {
//     encoderPulsesB++;
//   }
// }
void setMotor(int dir, int speed, int motorEnable, int in1, int in2) {
  analogWrite(motorEnable, speed);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


void joystickControl() {
  int xVal = analogRead(vrx);
  int yval = analogRead(vry);

  float xScalar = map(xVal, 0, 1023, 0, 1);
  float yScalar = map(yval, 0, 1023, 0, 1);

  float pwr = maxVoltage * yScalar;


  if (xScalar < 0.35) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (xScalar > 0.65) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(enA, pwr);
}
