#include "Movement.h"
//TODO: for pid the roll, pitch, and yaw are in radians

//time variables
uint32_t dist_sens_timer = 0;
uint32_t gyro_read_timer = 0;
uint32_t pid_loop_timer = 0;
uint32_t servo_loop_timer = 0;
uint32_t joystick_loop_timer = 0;
uint32_t dataTransfer_loop_timer = 0;
uint32_t encoder_loop_timer = 0;

//Function definitions
String parseSerial(float data, bool isDiag, String dtype);

//distance sensor variables
float distance, duration = 0.0f;

//PID objects
PID BalancePID(150, 0, 0, 0.0);
servoMovement sm;

//constants
#define DIST_TOLERANCE 50.0f

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //time setup
  currTime = millis();
  BalancePID.setStartTime(micros() / 1e-6);
  BalancePID.setTolerance(0.1);
  BalancePID.setTarget(0.0);

  //setup servo
  sm.createServo(servoPin);

  //set up encoder pins
  pinMode(encoderPhaseA, INPUT_PULLUP);
  pinMode(encoderPhaseB, INPUT_PULLUP);
  pinMode(encoderA2, INPUT_PULLUP);
  pinMode(encoderB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPhaseA), countPulsesA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA2), countPulsesB, CHANGE);

  //Gyro Init
  setupGyro();

}

void loop() {
  //update time
  currTime = millis();
  //run PID loop
  if (currTime - pid_loop_timer > 5) {
    BalancePID.runPID(pitch, ENA, encoderPulsesA);
    pid_loop_timer = currTime;
  }
  //joystick loop
  if (currTime - joystick_loop_timer > 50) {
    //read joystick values
    readJoystick();
    //change PID target based on joystick values

    joystick_loop_timer = currTime;
  }

  //read from distance sensor
  if (currTime - dist_sens_timer > 200) {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    duration = pulseIn(ECHO, HIGH);
    distance = (duration*.0343)/2;
    Serial.println("Distance: ");
    Serial.println(distance);


    //dynamic speed adustment based on distance from obstacle
    int newSpeed = analogRead(ENA) - 50;
    newSpeed /= 205;
    newSpeed *= 255;
    if (newSpeed > 255) {
      newSpeed = 255;
    }
    if (newSpeed < 0) {
      newSpeed = 0;
    }
    setSpeed(newSpeed, ENA);
    setSpeed(newSpeed, ENB);

    dist_sens_timer = millis();
  }
  //read from gyro/accelerometer
  if (currTime - gyro_read_timer > 50) {
    //collect data from gyroscope/Acceleromter
    readData();
    //get usable data
    Serial.println(yaw);
    Serial.println(roll);
    Serial.println(pitch);
    if (isnan(yaw) || isnan(pitch) || isnan(roll)) {
      Serial.println("Failed to read from MPU6050 sensor!");
      yaw = 0;
      pitch = 0;
      roll = 0;
      return;
    }
    gyro_read_timer = millis();
  }
  
  if (currTime - servo_loop_timer > 300) {
    float currentPos = sm.getAngle();
    if (distance < DIST_TOLERANCE) {
      sm.resetServo(); 
      servo_loop_timer = millis(); 
      return;
    } else{
      if (currentPos < 90) {
        sm.Rotate(30);
      }
      else {
        sm.Rotate(-30);
      }
    }
    servo_loop_timer = millis();
  }
  //encoder reading loop
  if (currTime - encoder_loop_timer > 1000) {
    int numpulses = encoderPulsesA;
    int deltapulses = 0;
    //encoder readings are handled in interrupts
    noInterrupts();
    deltapulses = encoderPulsesA - numpulses;
    interrupts();
    float deltaTime = ((float)(currTime - encoder_loop_timer))/(1.0e6); //time in seconds
    pulsesPerSecond = deltapulses / (deltaTime / 1.0f); //pulses per second
    speedMetersPerSecond = getSpeedMetersPerSecond(WHEEL_CIRCUMFERENCE, pulsesPerRevolution);
    encoder_loop_timer = currTime;
  }
  //data transfer loop
  if (currTime - dataTransfer_loop_timer > 3000) {
    //GPS,DS,SPD,BV,BC,IR,
    Serial.println(
      parseSerial(yaw,true,"g") + parseSerial(distance, true, "p") 
      + parseSerial(speedMetersPerSecond, true, "s")
      + parseSerial(0,true, "b") //battery voltage: change when voltage reading code/hardware is added
      + parseSerial(0,true, "c") //battery current: change when current reading code/hardware is added
      + parseSerial(1,true, "i") //Is running flag
    );
    dataTransfer_loop_timer = currTime;
  }
} //loop bracket -comment is here for future formatting issues

/**
 * Parses data for serial transmission to python script ran on computer
 * @note Function expects correct dtype variable to be passed. Therfore, if invalid dtype is passed the python script may not interpret the data correctly.
 * @param data Numerical data to be sent
 * @param isDiag Whether the data is diagnostic - if true, adds diag tag
 * @param dtype String representing the data type (e.g., "GYRO", "Distance", "Speed", etc.)
 */
String parseSerial(float data, bool isDiag, String dtype) {
  String sendString = "";
  sendString += dtype;
  sendString += data;
  if (isDiag) {
    sendString += "d";
  }
  return sendString;
}