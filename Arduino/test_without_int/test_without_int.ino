/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.0100;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
float pitch_vel;
float roll_vel;
float yaw_vel;

unsigned long t, t_prev;
int dt;

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  // mpu.setThreshold(3);
  t_prev = millis();
}

void loop(){
  if ((millis()-t_prev)>=10){

    t = millis();
    dt = t - t_prev; 

    // Read normalized values
    Vector norm = mpu.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    pitch_vel = norm.YAxis;
    roll_vel = norm.XAxis;
    yaw_vel = norm.ZAxis;
    pitch = pitch + pitch_vel * dt/1000;
    roll = roll + roll_vel * dt/1000;
    yaw = yaw + yaw_vel * dt/1000;

    // Output raw
    Serial.print("Pitch_vel:");
    Serial.print(pitch_vel);
    Serial.print(",");
    Serial.print("Roll_vel:");
    Serial.print(roll_vel);  
    Serial.print(",");
    Serial.print("Yaw_vel:");
    Serial.println(yaw_vel);
    Serial.print(",");
    Serial.print("Pitch:");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print("Roll:");
    Serial.print(roll);  
    Serial.print(",");
    Serial.print("Yaw:");
    Serial.println(yaw);
    Serial.print(",");

    Serial.println("");

    t_prev = t;
    // Wait to full timeStep period
    // delay((timeStep*1000) - (millis() - timer));
  }
}