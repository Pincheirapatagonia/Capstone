#include <Servo.h> //Imports the library Servo
#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;

#define ENA 7 
#define ENB 11

#define AIN1 4
#define AIN2 5
#define BIN1 10
#define BIN2 9

#define AC1 19 // D2
#define AC2 18 // D3
#define BC1 2 // D7
#define BC2 3 // D8

#define baud 9600

#define pi 3.14159265358979


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

float Vel_Theta_enc, Pose_Theta_enc;


// MPU: Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
float pitch_vel;
float roll_vel;
float yaw_vel;

// ************ VARIABES PID RPM************
int PWM_MAX = 255;
int PWM_MIN = 10;
int max = 130;
// ************ VARIABES PID POSE************
float Pose_X, Pose_Y, Pose_Theta;
float Vel_X, Vel_Y, Vel_Theta, Vel_Lin;

float Dist_ref = 1.0;
float Rot_ref = 0;

// 0m
float kp_Dist = 175; //130// 20 // 15 // 15 // 175
float ki_Dist = 10;//11 (para2m) //0.005 //0.006 //0.0058; //10
float kd_Dist = 0.00043; // 0.0;  // 0.0003 //0.0003 // 0.0004
float kp_Rot = 1.053; // 1 // 1.05
float ki_Rot = 0.001; // 0.000001 // 0.001
float kd_Rot = 0.0000045; // 0.003 // 0.00000450


// 1m
float kp_Dist = 175; //130// 20 // 15 // 15 // 175
float ki_Dist = 10;//11 (para2m) //0.005 //0.006 //0.0058; //10
float kd_Dist = 0.00043; // 0.0;  // 0.0003 //0.0003 // 0.0004
float kp_Rot = 1.053; // 1 // 1.05
float ki_Rot = 0.001; // 0.000001 // 0.001
float kd_Rot = 0.0000045; // 0.003 // 0.00000450


float e_Dist, e_Dist_prev, inte_Dist, ctrl_Dist;
float e_Rot, e_Rot_prev, inte_Rot, ctrl_Rot;


// ************ TIEMPO************
int Ts = 10; // [ms]
int dt;
unsigned long t, t_prev;

// PARAMETROS FISICOS ROBOT
float Diam_ruedas = 0.09;
float R_ruedas = Diam_ruedas/2;
float L_robot = 0.370-0.055;
float NFactor = 1400;
int PWM_max = 255;

int clipPWM(int PWM_val) {
    if (PWM_val > 255) {
        PWM_val = 255;
    } else if (PWM_val < -255) {
        PWM_val = -255;
      
    }
    return PWM_val;
}

int clipPWM_err(int PWM_val, int max) {
    if (PWM_val > max) {
        PWM_val = max;
    } else if (PWM_val < -max) {
        PWM_val = -max;
      
    }
    return PWM_val;
}
void setup() {
  Serial.begin(baud);
  while (!Serial) {
    delay(10);
  }
  
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  pinMode(AC1, INPUT_PULLUP);
  pinMode(AC2, INPUT_PULLUP);
  pinMode(BC1, INPUT_PULLUP);
  pinMode(BC2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(AC1), ISR_EncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AC2), ISR_EncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BC1), ISR_EncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BC2), ISR_EncoderB2, CHANGE);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  delay(1000);
  mpu.calibrateGyro(100);
  t_prev = millis();
}

void loop() {
  if ((millis() - t_prev)>= Ts) {
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

    //---------------ODOMETRIA------------------

    // Giro con IMU
    Vector norm = mpu.readNormalizeGyro();
    Vel_Theta = norm.ZAxis;   // [°/s]
    Pose_Theta = Vel_Theta*dt/1000 + Pose_Theta;  // [°]

    //Giro con ENCODER
    Vel_Theta_enc = R_ruedas * (vel_B - vel_A) / L_robot; // [rad/s]
    Pose_Theta_enc = Vel_Theta_enc * dt / 1000 + Pose_Theta_enc;  // [rad] 

    Vel_Lin = R_ruedas * (vel_A + vel_B) / 2; // [m/s]
    Vel_X = Vel_Lin * cos(Pose_Theta*pi/180);             // [m/s]
    Vel_Y = Vel_Lin * sin(Pose_Theta*pi/180);             // [m/s]
    Pose_X = (Vel_X*dt)/1000 + Pose_X;      // [m]
    Pose_Y = (Vel_Y*dt)/1000 + Pose_Y;      // [m]

    //---------------PID------------------
    e_Rot_prev = e_Rot;
    e_Dist_prev = e_Dist;
    e_Rot = Rot_ref - Pose_Theta;
    e_Dist = Dist_ref - Pose_X;
    inte_Rot = (dt * (e_Rot_prev + e_Rot) / 2) / 1000;
    inte_Dist = (dt * (e_Dist_prev + e_Dist) / 2) / 1000;
    ctrl_Rot = kp_Rot*e_Rot + ki_Rot*inte_Rot + kd_Rot*(e_Rot - e_Rot_prev)/dt*1000;
    ctrl_Dist = kp_Dist*e_Dist + ki_Dist*inte_Dist + kd_Dist*(e_Dist - e_Dist_prev)/dt*1000;

  
    ctrl_Dist = clipPWM_err(ctrl_Dist, max);
    if(ctrl_Dist + ctrl_Rot > 255){
      PWM_B_val = 255;
      PWM_A_val = ctrl_Dist - 2 * ctrl_Rot;
    } else {
      if(ctrl_Dist - ctrl_Rot > 255){
        PWM_A_val = 255;
        PWM_B_val = ctrl_Dist + 2 * ctrl_Rot;
      } else
      {
        PWM_A_val = ctrl_Dist - ctrl_Rot;
        PWM_B_val = ctrl_Dist + ctrl_Rot;
      }
    }
    
    PWM_A_val = clipPWM(PWM_A_val);
    PWM_B_val = clipPWM(PWM_B_val);

    WriteDriverVoltageA(PWM_A_val);
    WriteDriverVoltageB(PWM_B_val);

    /*Serial.print("RPMA:");
    Serial.print(RPM_A);
    Serial.print(",");
    Serial.print("RPMB:");
    Serial.print(RPM_B);
    Serial.print(",");*/
    Serial.print("Enc_A");
    Serial.print(Enc_A);
    Serial.print(",");
    Serial.print("Enc_B");
    Serial.print(Enc_B);
    Serial.print(",");
    Serial.print("PosX:");
    Serial.print(Pose_X);
    /*Serial.print(",");
    Serial.print("PosY:");
    Serial.print(Pose_Y);
    Serial.print(",");
    Serial.print("Pos_Theta:");
    Serial.print(round(Pose_Theta));
    Serial.print(",");
    Serial.print("Pos_Theta_enc:");
    Serial.print(round(Pose_Theta_enc*180/pi));
    Serial.print(",");*/
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
    if (PWM_val > PWM_MIN){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else if (PWM_val < PWM_MIN){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        PWM_val = 0;
    }
    // if (abs(PWM_val) > PWM_MAX) PWM_val = PWM_MAX;
    // if (abs(PWM_val) < PWM_MIN) PWM_val= PWM_MIN;
    analogWrite(ENA, abs(PWM_val));
}
void WriteDriverVoltageB(int PWM_val){
    if (PWM_val < PWM_MIN){
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    else if (PWM_val > PWM_MIN){
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }
    else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        PWM_val = 0;
    }
    // if (abs(PWM_val) > PWM_MAX) PWM_val = PWM_MAX;
    // if (abs(PWM_val) < PWM_MIN) PWM_val= PWM_MIN;
    analogWrite(ENB, abs(PWM_val));
}