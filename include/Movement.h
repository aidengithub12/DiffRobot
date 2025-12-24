#include "PID.h"
#include <Servo.h>
//Store movement related functions and variables

//constants
#define MAX_ANGLE 180
#define START_POSITION 90
#define DEADZONE 0.1f
#define WHEEL_CIRCUMFERENCE 2



//joystick variables
float joystickX = 0.0f;
float joystickY = 0.0f;

//speed calculation variables
float pulsesPerRevolution = 20.0f; //encoder pulses per wheel revolution
float speedMetersPerSecond = 0.0f;
float pulsesPerSecond = 0.0f;
float pulse_frequency = 0.0f;

//Servo Movement Class
class servoMovement {
    public:
        void createServo(int pin) {
            s.attach(pin);
        }
        void Rotate(int increment) {
            if (s.read() + increment > MAX_ANGLE || s.read() + increment < 0) {
                s.write(0);
                Serial.println("Servo angle out of bounds, resetting to 0");
            }
            s.write(s.read() + increment);
        }
        void resetServo() {
            s.write(START_POSITION);
        }
        int getAngle() {
            return s.read();
        }
    private:
        Servo s;

};




//movement PID variables
float ForwardAngle = RAD_TO_DEG * 2.0f; //desired forward angle in degrees
float BackwardAngle = RAD_TO_DEG * -2.0f; //desired backward angle in degrees
float TurnSpeedDifference = 20; //constant difference in order to turn while moving forward/backward

//Movement functions
void moveForward(PID &pid) {
    pid.setTarget(ForwardAngle);
}
void moveBackward(PID &pid) {
    pid.setTarget(BackwardAngle);
}
void turn(bool isRight, int enA, int enB) {
    switch (isRight)
    {
    case true:
        setSpeed(analogRead(enA)-TurnSpeedDifference, enA);
        break;
    case false:
        setSpeed(analogRead(enB)-TurnSpeedDifference, enB);
        break;
    }
}
void stopMoving(PID &pid) {
    pid.setTarget(0.0);
}

//Joystick reading
void readJoystick() {
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      int commaIndex = data.indexOf(',');
      if (commaIndex > 0) {
        joystickX = data.substring(0, commaIndex).toFloat();
        joystickY = data.substring(commaIndex + 1).toFloat();
      }
    }
}

float getSpeedMetersPerSecond(float wheelCircumference, int pulsesPerRevolution) {
    float speed = wheelCircumference / pulsesPerRevolution;
    return speed;
}

float getPulseFrequency(float pulsesPerSecond) {
    pulse_frequency = pulsesPerSecond / pulsesPerRevolution; //not required but could be useful in the future
    return pulse_frequency;
}

//Joystick Control
void JoystickControl(float x, float y, PID &pid) {
    if (fabs(joystickY) < DEADZONE) {
        y = 0.0f;
    }
    if (fabs(joystickX) < DEADZONE) {
        x = 0.0f;
    }
    if (y > 0.0f) {
        moveForward(pid);
    }
    else if (y < 0.0f) {
        moveBackward(pid);
    }
    if (x > y) {
        turn(true, ENA, ENB);
    }
    else if (x < y) {
        turn(false, ENA, ENB);
    }
    else {
        stopMoving(pid);
    }
    
}