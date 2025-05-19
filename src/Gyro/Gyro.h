// #include <Arduino.h>
// #include <Wire.h>

// const int MPU = 0x68; // MPU6050 I2C address
// float AccX, AccY, AccZ;
// float GyroX, GyroY, GyroZ;
// float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
// float roll, pitch, yaw;
// float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
// int c = 0;

// void readData() {
//   // === Read acceleromter data === //
//   Wire.beginTransmission(MPU);
//   Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//   //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
//   AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
//   AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
//   AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
//   // Calculating Roll and Pitch from the accelerometer data
//   accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
//   accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 0; // AccErrorY ~(-1.58)
//   // === Read gyroscope data === //
//   previousTime = currentTime;        // Previous time is stored before the actual time read
//   currentTime = millis();            // Current time actual time read
//   elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
//   Wire.beginTransmission(MPU);
//   Wire.write(0x43); // Gyro data first register address 0x43
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
//   GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
//   GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
//   GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
//   // Correct the outputs with the calculated error values
//   GyroX = GyroX - 0; // GyroErrorX ~(-0.56)
//   GyroY = GyroY - 0; // GyroErrorY ~(2)
//   GyroZ = GyroZ + 0; // GyroErrorZ ~ (-0.8)
//   // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
//   gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
//   gyroAngleY = gyroAngleY + GyroY * elapsedTime;
//   yaw =  yaw + GyroZ * elapsedTime;
//   // Complementary filter - combine acceleromter and gyro angle values
//   roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
//   pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
//   // Print the values on the serial monitor
//   Serial.print(roll);
//   Serial.print("/");
//   Serial.print(pitch);
//   Serial.print("/");
//   Serial.print(yaw);
//   Serial.println('q');
  
// }
// void calculate_IMU_error() {
//   // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
//   // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
//   // Read accelerometer values 200 times
//   while (c < 200) {
//     Wire.beginTransmission(MPU);
//     Wire.write(0x3B);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU, 6, true);
//     AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     // Sum all readings
//     AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
//     AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
//     c++;
//   }
//   //Divide the sum by 200 to get the error value
//   AccErrorX = AccErrorX / 200;
//   AccErrorY = AccErrorY / 200;
//   c = 0;
//   // Read gyro values 200 times
//   while (c < 200) {
//     Wire.beginTransmission(MPU);
//     Wire.write(0x43);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU, 6, true);
//     GyroX = Wire.read() << 8 | Wire.read();
//     GyroY = Wire.read() << 8 | Wire.read();
//     GyroZ = Wire.read() << 8 | Wire.read();
//     // Sum all readings
//     GyroErrorX = GyroErrorX + (GyroX / 131.0);
//     GyroErrorY = GyroErrorY + (GyroY / 131.0);
//     GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
//     c++;
//   }
//   //Divide the sum by 200 to get the error value
//   GyroErrorX = GyroErrorX / 200;
//   GyroErrorY = GyroErrorY / 200;
//   GyroErrorZ = GyroErrorZ / 200;
//   // Print the error values on the Serial Monitor
//   Serial.print("AccErrorX: ");
//   Serial.println(AccErrorX);
//   Serial.print("AccErrorY: ");
//   Serial.println(AccErrorY);
//   Serial.print("GyroErrorX: ");
//   Serial.println(GyroErrorX);
//   Serial.print("GyroErrorY: ");
//   Serial.println(GyroErrorY);
//   Serial.print("GyroErrorZ: ");
//   Serial.println(GyroErrorZ);
// }


// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
float roll; 
float yaw;
float pitch;
float accAngleX;
float accAngleY ;
float gyroAngleX;
float gyroAngleY;
void setupGyro(void) {
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
  delay(2);
  // Try to initialize!
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void readData() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  
 
  accAngleX = (atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / PI); // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI) + 0; // AccErrorY ~(-1.58)
  gyroAngleX = gyroAngleX + g.gyro.x * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + g.gyro.y * elapsedTime;
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  yaw =  yaw + g.gyro.z * elapsedTime;
  /* Print out the values */
  Serial.println(roll);
  Serial.println(yaw);
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(20);
}
