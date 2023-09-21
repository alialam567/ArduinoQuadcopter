#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

int Throttle1;
int Throttle2;
int Throttle3;
int Throttle4;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// Define PID constants for pitch, roll, and yaw
double kpPitch = 1.0;  // Proportional constant for pitch control
double kiPitch = 0.1;  // Integral constant for pitch control
double kdPitch = 0.01; // Derivative constant for pitch control

double kpRoll = 1.0;   // Proportional constant for roll control
double kiRoll = 0.1;   // Integral constant for roll control
double kdRoll = 0.01;  // Derivative constant for roll control

double kpYaw = 1.0;    // Proportional constant for yaw control
double kiYaw = 0.1;    // Integral constant for yaw control
double kdYaw = 0.01;   // Derivative constant for yaw control

// Define target angles and initial values
double targetPitch = 0.0; // Target pitch angle in degrees
double targetRoll = 0.0;  // Target roll angle in degrees
double targetYaw = 0.0;   // Target yaw angle in degrees

// Initialize PID controllers for pitch, roll, and yaw
double inputPitch, outputPitch, setpointPitch;
PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, kpPitch, kiPitch, kdPitch, DIRECT);

double inputRoll, outputRoll, setpointRoll;
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, kpRoll, kiRoll, kdRoll, DIRECT);

double inputYaw, outputYaw, setpointYaw;
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, kpYaw, kiYaw, kdYaw, DIRECT);

void setup() {
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

    // Set the PID sample time
  pidPitch.SetSampleTime(10); // Update PID every 10 milliseconds
  pidRoll.SetSampleTime(10);  // Update PID every 10 milliseconds
  pidYaw.SetSampleTime(10);   // Update PID every 10 milliseconds

    //turn the PID on
  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);
  pidYaw.SetMode(AUTOMATIC);

  ESC1.attach(4,1000,2000);
  ESC2.attach(5,1000,2000);
  ESC3.attach(6,1000,2000);
  ESC4.attach(7,1000,2000);

  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
}
void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.84; // AccErrorX ~(0.84) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 89.09; // AccErrorY ~(89.09)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - 2.06; // GyroErrorX ~(2.06)
  GyroY = GyroY + 0.14; // GyroErrorY ~(-0.14)
  GyroZ = GyroZ + 1.59; // GyroErrorZ ~ (-1.59)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Set the input values for the PID controllers
  inputPitch = GyroY;
  inputRoll = GyroX;
  inputYaw = GyroZ;

  // Set the setpoints for the PID controllers
  setpointPitch = targetPitch;
  setpointRoll = targetRoll;
  setpointYaw = targetYaw;

  // Compute PID outputs for pitch, roll, and yaw
  pidPitch.Compute();
  pidRoll.Compute();
  pidYaw.Compute();

  Serial.print("Pitch Output: ");
  Serial.print(outputPitch);
  Serial.print(" | Roll Output: ");
  Serial.print(outputRoll);
  Serial.print(" | Yaw Output: ");
  Serial.println(outputYaw);

  Throttle1 = outputPitch - outputRoll - outputYaw;
  Throttle2 = outputPitch + outputRoll + outputYaw;
  Throttle3 = outputPitch + outputRoll - outputYaw;
  Throttle4 = outputPitch - outputRoll + outputYaw;

  //Set Limits so motors dont turn off completely and don't exceed max 
  if (Throttle1 < 1100) ESC1.wite(1100);                                
  else if (Throttle1 > 2000) ESC1.wite(2000);
  else ESC1.write(Throttle1);

  if (Throttle2 < 1100) ESC2.wite(1100);
  else if (Throttle2 > 2000) ESC2.wite(2000);
  else ESC2.write(Throttle2);

  if (Throttle3 < 1100) ESC3.wite(1100);
  else if (Throttle3 > 2000) ESC3.wite(2000);
  else ESC3.write(Throttle3);

  if (Throttle4 < 1100) ESC4.wite(1100);
  else if (Throttle4 > 2000) ESC4.wite(2000);
  else ESC4.write(Throttle4);

  // Add a delay to control the loop rate (adjust as needed)
  delay(10);
}
void calculate_IMU_error() {
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}