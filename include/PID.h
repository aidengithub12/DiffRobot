#include <Arduino.h>
#include "Pins.h"
#include "Gyro.h"

//func definitions
void setMotor(int dir, float power, int en);
void stop(int p1, int p2);


//Encoder Values
volatile long encoderPulsesA = 0;
volatile long encoderPulsesC = 0;
//PID Constants
double kP = 150;
double kI = 0;
double KD = 0;
//PID Variables
// long prevT = 0;
// float eprev = 0;
// float eintegral = 0;
// const int tolerance = 0.5;
// float target = 0;
// float startTime = 0;
// float currentTimeSec = 0;
//time variable
uint32_t currTime = 0;



class PID {
  public:
    PID(double p, double i, double d) {
      kP = p;
      kI = i;
      KD = d;
    }
    /**
     * @param p Proportional Gain
     * @param i Integral Gain
     * @param d Derivative Gain
     * @param targ Target setpoint
     */
    PID(double p, double i, double d, double targ) {
      kP = p;
      kI = i;
      KD = d;
      target = targ;
    }
    void runPID(int input, int en, int enca) {
      target = target; 
      currentTimeSec = micros() / 1e-6;


      if ((currentTimeSec - startTime) > 1) {
        startTime = micros() / 1e-6;
        noInterrupts();
        // motorRPM = ((encoderPulsesA + encoderPulsesB) / ( micros() - currentTime1) * .5);
        interrupts();
      }
      //   linearDisplacement = ((enca)* 3.14 * wheelDiameter) / 12;
    

      // //time difference
      float deltaT = ((float)(currTime - prevT))/(1.0e6);
      prevT = currTime;
      float Input = input;
      noInterrupts();
      Input = input ;
      interrupts();
      //error
      float e = Input-target;

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
      }
      setMotor(dir, (float) pwr, en);
    
      // Serial.print("RAN MOTOR ***********************************************************************");
      // setMotor(dir, pwr, enB, IN1B, IN2B);
      // Serial.print(">dir:");
      // Serial.println(dir);
      //store previous error
      Serial.println(pwr);
      eprev = e;

    }
    void setTarget(double targ) {
      target = targ;
    }
    void setStartTime(float t) {
      startTime = t;
    }
    void setTolerance(float tol) {
      tolerance = tol;
    }
  private:
    double kP;
    double kI;
    double KD;
    double target;
    double eprev;
    double eintegral;
    uint32_t prevT;
    float currentTimeSec;
    float startTime;
    float tolerance = 0.5;
};


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

void stop(int p1, int p2) {
  digitalWrite(p1, LOW);
  digitalWrite(p2, LOW);
}

/**
 * speed = pwm value
 */
void setSpeed(int speed, int e) {
  analogWrite(e, speed);
}
void setMotor(int dir, float power, int en) {
  if (dir > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}


