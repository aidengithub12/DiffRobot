//necessary imports
#include <Arduino.h>
#include <Servo.h>

//function definitions
void moveMotor(int speed, int direction, int currentSpeed);
void accelerate();
void countPulsesA();
void countPulsesB();
void runPID(int targetChange, int en, int in, int inn,int enca);       
void setMotor(int dir, int speed, int motorEnable, int in1, int in2);
void distanceSensor();

//led pins
int ledLeft;
int ledRight;

//Servos

int lapin;
int rapin;
int headpin;

Servo leftArm;
Servo rightArm;
Servo Head;

//distance sensor
int TrigPin;
int EchoPin;
float distance, duration = 0.0f;

//Motor Controller A
int enA = 12;
int IN1 = 25;
int IN2 = 33;

//motor controller B
int enB = 5;
int IN1B = 23;
int IN2B = 22;  

//encoder pins -- motor controller A2
// int encoderPhaseA2 = 300;
// int encoderPhaseB2 = 180;
//encoder pins -- Motor Controller A1
int encoderPhaseA = 35; //digital interrupt pin
//Encoder power = 5V pin
//Encoder ground = gnd pin
int encoderPhaseB = 32;

int encoderA2 = 14;
int encoderB2 = 4;

//Encoder Values
volatile long encoderPulsesA = 0;
volatile long encoderPulsesB = 0;
volatile long encoderPulsesC = 0;
volatile long encoderPulsesD = 0;
//PID Constants
double kP = 15;
double kI = 5;
double KD = 0;

//time variables
float startTime = 0;
float currentTime = 0;

//wheel diameter
float wheelDiameter = 0.046; //meters
//motor speed
float motorRPM = 0; //RPM
//car speed
float linearDisplacement = 0;
//current values
int currentPosition = 0; //in rotations
int desiredPosition = 0; //in meters
long prevT = 0;
float eprev = 0;
float eintegral = 0;
const int tolerance = 0.005;
float target = 0; //meters
float distance = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Serial1.begin(115200,Serial_8N1,17,16);
  // Serial.println("Startup process began...");
  //setup pins
  pinMode(enA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);
  
  //set up encoder pins
  pinMode(encoderPhaseA, INPUT_PULLUP);
  pinMode(encoderPhaseB, INPUT_PULLUP);
  pinMode(encoderA2, INPUT_PULLUP);
  pinMode(encoderB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPhaseA), countPulsesA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA2), countPulsesB, CHANGE);
  //turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, LOW);
  
  //set up servos and set them to their zero position
  leftArm.attach(lapin);
  rightArm.attach(rapin);
  Head.attach(headpin);

  leftArm.write(0);
  rightArm.write(0);
  Head.attach(0);

  //set up distance sensor
  pinMode(TrigPin, OUTPUT);  
	pinMode(EchoPin, INPUT);  

  //timer start
  startTime = micros() / 1e-6;
  //Setup Finished
  // Serial.println("Setup has finished.");
}

void loop() {
  //auto code
  
  runPID(distance, enA, IN1, IN2,encoderPulsesA);
  runPID(distance, enB,IN1B, IN2B,encoderPulsesC);


  //servo functionality
  if (linearDisplacement >= 2) {
    leftArm.write(leftArm.read() + 15);
    rightArm.write(rightArm.read() - 15); //TODO: not sure this is possible but will be fixed in tests
    Head.write(Head.read() + 15);
    if (Head.read() >= 180) {
      Head.write(0);
    }
    if (leftArm.read() >= 180) {
      leftArm.write(0);
    }
    if (rightArm.read() >= 180) {
      rightArm.write(0);
    }
  }

  //distance sensor logic
  distanceSensor();
  if (distance <= 10) {
    //stop robot due to possible crash
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2B, LOW);
    eprev = 0; //TODO: not sure if this is right
  }
  //logs
  Serial.print(">setpoint:");
  Serial.println(target);
  Serial.print(">currentposright:");
  Serial.println(encoderPulsesA);
  Serial.print(">currentposleft:");
  Serial.println(encoderPulsesC);
  Serial.print(">enb:");
  Serial.println(analogRead(enB));
  Serial.print(">lda:");
  Serial.println(((encoderPulsesA)* 3.14 * wheelDiameter) / 12);
  Serial.print(">ldc:");
  Serial.println(((encoderPulsesC)* 3.14 * wheelDiameter) / 12);
  
  
  
}


void runPID(int targetChange, int en, int in, int inn, int enca) {
  target = target + targetChange;
  currentTime = micros() / 1e-6;


  if ((currentTime - startTime) > 1) {
    startTime = micros() / 1e-6;
    noInterrupts();
    motorRPM = ((encoderPulsesA + encoderPulsesB) / ( micros() - currentTime) * .5);
    interrupts();
  }
  linearDisplacement = ((enca)* 3.14 * wheelDiameter) / 12;
  

  // //time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT))/(1.0e6);
  prevT = currT;
  int pos = 0;
  noInterrupts();
  pos = ((enca)* 3.14 * wheelDiameter) / 12;
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
  if (abs(e) <= tolerance || pwr < 75) {
    pwr = 0;
    digitalWrite(ledLeft,LOW);
    digitalWrite(ledRight, LOW);
  }
  else {
    digitalWrite(ledLeft,HIGH);
    digitalWrite(ledRight, HIGH);
  }
  Serial.println(e);
  Serial.println(pwr);

  //signal the motor
  setMotor(dir, pwr, en, in, inn);
  // setMotor(dir, pwr, enB, IN1B, IN2B);
  Serial.print(">dir:");
  Serial.println(dir);
  //store previous error
  eprev = e;

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
void countPulsesB() {
  
  if (digitalRead(encoderB2) != digitalRead(encoderA2)) {
    // Serial.println("Encoder --");
    encoderPulsesC--;
    // Serial.println((encoderPulses));
    
    // Serial.println("encoder A: " + (String) digitalRead(encoderPhaseA));
    // Serial.println("Encoder B: " + (String) digitalRead(encoderPhaseB));
  }
  else {
    encoderPulsesC++;
  }
}
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


void distanceSensor() {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  duration = pulseIn(EchoPin, HIGH);
  distance = (duration*.0343)/2;
  delay(100);  
}
