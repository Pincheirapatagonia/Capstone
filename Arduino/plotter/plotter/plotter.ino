// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#define MPU6050_DEVICE_ID 0x2
Adafruit_MPU6050 mpu;

float pos_x, pos_y, pos_z, Theta;

float err_y;
float err_x;
float err_z;
float med_y;
float med_x;
float med_z;

int pico =0;
float theta;
float vel_theta = 0.0;
float vel_theta_old = 0.0;
float vel_theta_old_old = 0.0;
float vel_theta_old_old_old = 0.0;
float vel_theta_prom = 0.0;

int dt  = 0;
unsigned long t, t_prev;

void setup(void) {
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if ((millis() - t_prev) >= 100){
    t = millis();
    dt = t - t_prev;

    /* Print out the values */
    //Serial.print("AccelX:");
    //Serial.print(a.acceleration.x);
    //Serial.print(",");
    //Serial.print("AccelY:");
    //Serial.print(a.acceleration.y);
    //Serial.print(",");
    //Serial.print("AccelZ:");
    //Serial.print(a.acceleration.z);
    //Serial.print(", ");
    // Serial.print("GyroZ:");
    // Serial.print(g.gyro.z);
  
  
    if (pico==0){
      err_y = a.acceleration.y;
      err_x = a.acceleration.x;
      err_z = a.acceleration.z;
      pico = 1;
    }
  
  

    // vel_theta_old_old_old = vel_theta_old_old;
    // vel_theta_old_old = vel_theta_old;
    // vel_theta_old = vel_theta;
    // vel_theta = g.gyro.z; 
    // vel_theta_prom = (vel_theta_old_old_old  + vel_theta_old_old  + vel_theta_old + vel_theta)/4;
    // theta = theta + vel_theta_prom*dt/1000;

    of 

    // Serial.print("velX:");
    // Serial.print(vel_x);
    // Serial.print(",");
    // Serial.print("velY:");
    // Serial.print(vel_y);
    // Serial.print(",");
    // Serial.print("velZ:");
    // Serial.print(vel_z);
    // Serial.print(",");
    Serial.print("velTheta:");
    Serial.print(vel_theta);
    Serial.print(",");

    // Serial.print("posx:");
    // Serial.print(pos_x);
    // Serial.print(",");
    // Serial.print("posy:");
    // Serial.print(pos_y);
    // Serial.print(",");
    Serial.print("Theta:");
    Serial.print(theta*180/3.1415);
    Serial.print(",");
    Serial.println("");

    t_prev = t;
  }
}