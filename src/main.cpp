//necessary imports
#include <Arduino.h>
//function definitions
void moveMotor(int speed, int direction, int currentSpeed);
void accelerate();
void countPulsesA();
void countPulsesB();
void runPID(int targetChange);         
void setMotor(int dir, int speed, int motorEnable, int in1, int in2);
//Motor Controller A
int enA = 12;
int IN1 = 25;
int IN2 = 33;

//motor controller B
int enB = 52;
int IN1B = 50;
int IN2B = 48;  

//encoder pins -- motor controller A2
// int encoderPhaseA2 = 300;
// int encoderPhaseB2 = 180;
//encoder pins -- Motor Controller A1
int encoderPhaseA = 35; //digital interrupt pin
//Encoder power = 5V pin
//Encoder ground = gnd pin
int encoderPhaseB = 32;



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
float target = 10; //meters
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
  //auto code
  while (Serial.available() > 0) {
    float distance = Serial.readString().toFloat();
    runPID(distance);
  }
  
}


void runPID(int targetChange) {
  target = target + targetChange;
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
  delay(200);
}


void moveMotor(int speed, int direction /*forward = 1 backward = -1*/, int currentSpeed) {
  
  if (direction == 1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    for (int i = 0; i < speed; i++) {
      analogWrite(enA, i);
      delay(20);
      // Serial.println(i);
    }
    // Serial.println("forward ran");
    // Serial.println("EP: " + encoderPulses);
    
    // Serial.println("DA: " + digitalRead(encoderPhaseA));
  }
  else if (direction == -1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    for (int i = 0; i < speed; i++) {
      analogWrite(enA, i);
      delay(20);
    }
    // Serial.println("backward Ran");
  }
  // analogWrite(enA, speed);
  // Serial.println("speed changed");
  currentSpeed = speed;
}
void accelerate() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
  for (int i = 0; i < 256; ++i) {
    analogWrite(enA, i);
    delay(20);
  }
  delay(2000);
  for (int i = 255; i <=0; --i) {
    analogWrite(enA, i);
    delay(20);
  }
}
void decelerateMotor(int currentSpeed) {
  for (int i = currentSpeed; i >=0; i--){
    analogWrite(enA, i);

  }
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
