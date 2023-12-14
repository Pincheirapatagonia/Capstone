#include <Servo.h> //Imports the library Servo
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


#define ENA 6
#define ENB 11
#define AIN1 4
#define AIN2 5
#define BIN1 10
#define BIN2 9
#define AC1 21
#define AC2 20
#define BC1 19
#define BC2 18

#define baud 9600

#define pi 3.1415



// ************ DEFINITIONS A (L)************
volatile long EncoderCountA = 0;
float Enc_A, Enc_A_prev;
float Dist_A, vel_A, RPM_A;
int PWM_A_val;

// ************ DEFINITIONS B (R)************
volatile long EncoderCountB = 0;
float Enc_B, Enc_B_prev;
float Dist_B, vel_B, RPM_B;
int PWM_B_val;

// ************ VARIABES PID RPM************
int PWM_MAX = 255;
int PWM_MIN = 50;

// ************ VARIABES PID POSE************
float Pose_X, Pose_Y, Pose_Theta;
float Vel_X, Vel_Y, Vel_Theta, Vel_Lin;

float Dist_ref = 1;
float Rot_ref = 0;
float kp_Dist = 1;
float ki_Dist = 0.1;
float kd_Dist = 0.1;
float kp_Rot = 0.05;
float ki_Rot = 0.001;
float kd_Rot = 0.001;

float e_Dist, e_Dist_prev, inte_Dist, ctrl_Dist;
float e_Rot, e_Rot_prev, inte_Rot, ctrl_Rot;


// ************ TIEMPO************
int dt;
unsigned long t, t_prev;

// PARAMETROS FISICOS ROBOT
float Diam_ruedas = 0.09;
float R_ruedas = Diam_ruedas/2;
float L_robot = 0.370-0.055;
float NFactor = 1500;
int PWM_min = 150;
int PWM_max = 255;


void setup() {
  Serial.begin(baud);
  while (!Serial) {
    delay(10);
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

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if ((millis() - t_prev)>= 100) {
    t = millis();
    dt = t - t_prev; // [ms]

    //---------------CALCULAR VELOCIDADES------------------
    Enc_A = EncoderCountA; 
    Enc_B = EncoderCountB;
    // Dist_A = Dist_A + ((Enc_A - Enc_A_prev)) / NFactor * pi * Diam_ruedas; // [m]
    // Dist_B = Dist_B + ((Enc_B - Enc_B_prev)) / NFactor * pi * Diam_ruedas; // [m]
    RPM_A = 1000 * (Enc_A - Enc_A_prev) / dt * 60.0 / NFactor;
    RPM_B = 1000 * (Enc_B - Enc_B_prev) / dt * 60.0 / NFactor;
    vel_A = RPM_A * pi * 2 / 60.0;                  // [rad/s]
    vel_B = RPM_B * pi * 2 / 60.0;                  // [rad/s]
    Enc_A_prev = Enc_A;
    Enc_B_prev = Enc_B;

    //---------------ODOMETRIA ENCODER------------------

    // Giro con IMU
    Vel_Theta = g.gyro.z;                                  // [rad/s]
    Pose_Theta = Pose_Theta + (Vel_Theta * dt) / 1000; // [rad]

    //Giro con ENCODER
    // Vel_Theta = R_ruedas * (vel_B - vel_A) / L_robot; // [rad/s]
    // Pose_Theta = Vel_Theta * dt / 1000 + Pose_Theta;  // [rad] 

    Vel_Lin = R_ruedas * (vel_A + vel_B) / 2; // [m/s]
    Vel_X = Vel_Lin * cos(Pose_Theta);             // [m/s]
    Vel_Y = Vel_Lin * sin(Pose_Theta);             // [m/s]
    Pose_X = (Vel_X*dt)/1000 + Pose_X;      // [m]
    Pose_Y = (Vel_Y*dt)/1000 + Pose_Y;      // [m]


    e_Rot_prev = e_Rot;
    e_Dist_prev = e_Dist;
    e_Rot = Rot_ref - Pose_Theta;
    e_Dist = Dist_ref - Pose_X;
    inte_Rot = (dt * (e_Rot_prev + e_Rot) / 2) / 1000;
    inte_Dist = (dt * (e_Dist_prev + e_Dist) / 2) / 1000;
    ctrl_Rot = kp_Rot*e_Rot + ki_Rot*inte_Rot + kd_Rot*(e_Rot - e_Rot_prev)/dt*1000;
    ctrl_Dist = kp_Dist*e_Dist + ki_Dist*inte_Dist + kd_Dist*(e_Dist - e_Dist_prev)/dt*1000;
    PWM_A_val = ctrl_Dist - ctrl_Rot;
    PWM_B_val = ctrl_Dist + ctrl_Rot;
    WriteDriverVoltageA(PWM_A_val);
    WriteDriverVoltageA(PWM_B_val);

    Serial.print("PosX:");
    Serial.print(Pose_X);
    Serial.print(",");
    Serial.print("PosY:");
    Serial.print(Pose_Y);
    Serial.print(",");
    Serial.print("Pos_Theta:");
    Serial.print(round(Pose_Theta*180/pi));
    Serial.println("");

    t_prev = t;
  }
}


void ISR_EncoderA2() {
  bool PinB = digitalRead(AC2);
  bool PinA = digitalRead(AC1);
  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCountA++;
    } else {
      EncoderCountA--;
    }
  } else {
    if (PinA == HIGH) {
      EncoderCountA--;
    } else {
      EncoderCountA++;
    }
  }
}
void ISR_EncoderA1() {
  bool PinB = digitalRead(AC2);
  bool PinA = digitalRead(AC1);
  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCountA--;
    } else {
      EncoderCountA++;
    }
  } else {
    if (PinB == HIGH) {
      EncoderCountA++;
    } else {
      EncoderCountA--;
    }
  }
}
void ISR_EncoderB2() {
  bool PinB = digitalRead(BC2);
  bool PinA = digitalRead(BC1);
  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCountB++;
    } else {
      EncoderCountB--;
    }
  } else {
    if (PinB == HIGH) {
      EncoderCountB--;
    } else {
      EncoderCountB++;
    }
  }
}
void ISR_EncoderB1() {
  bool PinB = digitalRead(BC2);
  bool PinA = digitalRead(BC1);
  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCountB--;
    } else {
      EncoderCountB++;
    }
  } else {
    if (PinB == HIGH) {
      EncoderCountB++;
    } else {
      EncoderCountB--;
    }
  }
}
void WriteDriverVoltageA(int PWM_val){
    if (PWM_val > 0){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else if (PWM_val < 0){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
    // if (abs(PWM_val) > PWM_MAX) PWM_val = PWM_MAX;
    // if (abs(PWM_val) < PWM_MIN) PWM_val= PWM_MIN;
    analogWrite(ENA, abs(PWM_val));
}
void WriteDriverVoltageB(int PWM_val){
    if (PWM_val < 0){
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    else if (PWM_val > 0){
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }
    else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }
    // if (abs(PWM_val) > PWM_MAX) PWM_val = PWM_MAX;
    // if (abs(PWM_val) < PWM_MIN) PWM_val= PWM_MIN;
    analogWrite(ENB, abs(PWM_val));
}
