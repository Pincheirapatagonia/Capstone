#include <Servo.h> //Imports the library Servo
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//GeeKee CeeBee
#define trigPin 3 // TriggerSensor
#define echoPin 2 // EchoSensor

#define ServoPin 8 // Servo pin
#define MAXANG 180 // Servo máx angle
#define MINANG 0 // Servo min angle
#define SCOOPDELAY 5
#define ENA 6
#define ENB 11

#define AIN1 4
#define AIN2 5
#define BIN1 10
#define BIN2 9



#define baud 9600

#define pi 3.1415

Servo servo; //Defines the object Servo of type(class) Servo
int angle = 0; // Defines an integer

// ************ DEFINITIONS A (L)************
volatile long EncoderCountA = 0;
float ThetaA, ThetaA_prev;
float Dist_A, vel_A, RPM_A;
int PWM_A_val;
float kp_A = 1.24;
float ki_A = 0.02;
float kd_A = 10.8;

// ************ DEFINITIONS B (R)************
volatile long EncoderCountB = 0;
float ThetaB, ThetaB_prev;
float Dist_B, vel_B, RPM_B;
int PWM_B_val;
float kp_B = 1;
float ki_B = 0.01 ;
float kd_B = 10;

// ************ VARIABES PID RPM************
float e_A, e_prev_A;
float inte_A, inte_prev_A;
float e_B, e_prev_B;
float inte_B, inte_prev_B;
float RPM_A_ref, RPM_B_ref;
int PWM_MAX = 255;
int PWM_MIN = 50;


// ************ VARIABES PID POSE************
float Pose_X, Pose_Y, Pose_Theta;
float Vel_X, Vel_Y, Vel_Theta;
float Pose_X_final = 0.1;
float Pose_Y_final = 0;
float Pose_Theta_final = 0;

int state_pid_pose = 0;
int select_pose_encoder = 0; // 0: mpu, 1: encoder

float kp_giro = 1;
float ki_giro = 0.1;
float kp_desp = 0.05;
float ki_desp = 0.001;
float e_giro_margin = 0.09; // approx 5° 
float e_giro_margin_factor = 1.1;
float e_desp_margin = 0.025;

float e_giro, e_prev_giro, inte_giro, inte_prev_giro;
float Vel_Rot_ref, RPM_Rot_ref;
float e_desp, e_prev_desp, inte_desp, inte_prev_desp;
float Vel_Desp_ref, RPM_Desp_ref;

// ************ ODOMETRIA ENCODER ************
float enc_Pos_x, enc_Pos_y, enc_Theta;
float enc_Vel_ang, enc_Vel_lin, enc_Vel_x, enc_Vel_y;
float enc_vel_A, enc_vel_B, enc_RPM_A, enc_RPM_B;

// ************ ODOMETRIA MPU ************
float mpu_x_vel, mpu_y_vel;
float mpu_Pos_x, mpu_Pos_y, mpu_Theta;
float mpu_Vel_ang, mpu_Vel_lin, mpu_Vel_x, mpu_Vel_y;
float mpu_vel_A, mpu_vel_B, mpu_RPM_A, mpu_RPM_B;


// ************ TIEMPO************
int dt;
unsigned long t, t_prev;


char msg[60];

// PARAMETROS FISICOS ROBOT
float Diam_ruedas = 0.09;
float R_ruedas = Diam_ruedas/2;
float L_robot = 0.370-0.055;
float NFactor = 1400;
int PWM_min = 150;
int PWM_max = 255;

int instruction = -1;

int duration;
int distance;

unsigned long startScoop = 0;
unsigned long currentMillis = 0;

int agarro_castana = 0;
int scooping = 0;


void WriteDriverVoltageA(int PWM_val){
    if (PWM_val > 0){
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else if (PWM_val < 0){
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    else{
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
    if (abs(PWM_val) > PWM_MAX) PWM_val = PWM_MAX;
    if (abs(PWM_val) < PWM_MIN) PWM_val= PWM_MIN;
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
    else{
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }
    if (abs(PWM_val) > PWM_MAX) PWM_val = PWM_MAX;
    if (abs(PWM_val) < PWM_MIN) PWM_val= PWM_MIN;
    analogWrite(ENB, abs(PWM_val));
}


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
  state_pid_pose = 0;
}
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if ((millis() - t_prev)>= 100) {
    t = millis();

    //---------------CALCULAR VELOCIDADES------------------
    ThetaA = EncoderCountA; 
    ThetaB = EncoderCountB;
    Dist_A = Dist_A + ((ThetaA - ThetaA_prev)) / NFactor * pi * Diam_ruedas; // [m]
    Dist_B = Dist_B + ((ThetaB - ThetaB_prev)) / NFactor * pi * Diam_ruedas; // [m]
    dt = t - t_prev; // [ms]
    enc_RPM_A = 1000 * (ThetaA - ThetaA_prev) / dt * 60.0 / NFactor;
    enc_RPM_B = 1000 * (ThetaB - ThetaB_prev) / dt * 60.0 / NFactor;
    enc_vel_A = RPM_A * pi * 2 / 60.0;                  // [rad/s]
    enc_vel_B = RPM_B * pi * 2 / 60.0;                  // [rad/s]
    ThetaA_prev = ThetaA;
    ThetaB_prev = ThetaB;

    //---------------ODOMETRIA ENCODER------------------
    enc_Vel_ang = R_ruedas * (enc_vel_B - enc_vel_A) / L_robot;   // [rad/s]
    enc_Theta = enc_Theta + enc_Vel_ang * dt / 1000;      // [rad] 
    enc_Vel_lin = R_ruedas * (enc_vel_A + enc_vel_B) / 2;         // [m/s]
    enc_Vel_x = enc_Vel_lin * cos(enc_Theta);             // [m/s]
    enc_Vel_y = enc_Vel_lin * sin(enc_Theta);             // [m/s]
    enc_Pos_x = enc_Pos_x + (enc_Vel_x * dt) / 1000;      // [m]
    enc_Pos_y = enc_Pos_y + (enc_Vel_y * dt) / 1000;      // [m]
    
    //---------------ODOMETRIA ACELEROMETRO------------------
    mpu_Vel_ang = g.gyro.z;
    mpu_Theta = mpu_Theta + (mpu_Vel_ang * dt) / 1000;                     // [rad/s]
    mpu_x_vel = (g.acceleration.x * dt)/1000 + mpu_x_vel;                  // [m/s]
    mpu_y_vel = (g.acceleration.y * dt)/1000 + mpu_y_vel;                  // [m/s]
    mpu_Vel_lin = sqrt(mpu_x_vel*mpu_x_vel + mpu_y_vel*mpu_y_vel);         // [m/s]
    mpu_Vel_x = mpu_x_vel*(-sin(mpu_Theta)) + mpu_y_vel*(-cos(mpu_Theta)); // [m/s]
    mpu_Vel_y = mpu_x_vel*cos(mpu_Theta) + mpu_y_vel*(-sin(mpu_Theta));    // [m/s]
    mpu_Pos_x = mpu_Pos_x + (mpu_Vel_x * dt) / 1000;                       // [m]
    mpu_Pos_y = mpu_Pos_y + (mpu_Vel_y * dt) / 1000;                       // [m]
    mpu_vel_A = mpu_Vel_lin - mpu_Vel_ang*L_robot/2;
    mpu_vel_B = mpu_Vel_lin + mpu_Vel_ang*L_robot/2;
    mpu_RPM_A = mpu_vel_A * 60 / (2*pi);
    mpu_RPM_B = mpu_vel_B * 60 / (2*pi);

  //---------------MUX DE ODOMETRIA------------------
    if (select_pose_encoder == 1){
        Pose_X = enc_Pos_x;
        Pose_Y = enc_Pos_y;
        Pose_Theta = enc_Theta;
        Vel_X = enc_Vel_x;
        Vel_Y = enc_Vel_y;
        Vel_Theta = enc_Vel_ang;
        RPM_A = enc_RPM_A;
        RPM_B = enc_RPM_B;
    } else {
        Pose_X = mpu_Pos_x;
        Pose_Y = mpu_Pos_y;
        Pose_Theta = mpu_Theta;
        Vel_X = mpu_Vel_x;
        Vel_Y = mpu_Vel_y;
        Vel_Theta= mpu_Vel_ang;
        RPM_A = mpu_RPM_A;
        RPM_B = mpu_RPM_B;
    } 
        
    //---------------POSE PI CONTROL------------------
    e_desp = sqrt((Pose_X_final - Pose_X) * (Pose_X_final - Pose_X) + (Pose_Y_final - Pose_Y) * (Pose_Y_final - Pose_Y));
    if (state_pid_pose == 0){ // 0: girando para alinear
        e_giro = atan2(Pose_Y_final, Pose_X_final) - Pose_Theta;
        if (abs(e_giro) < e_giro_margin){
            state_pid_pose = 1;
            e_prev_giro = 0;
            inte_prev_giro = 0;
          
        }
        else{
            inte_giro = inte_prev_giro + (dt * (e_giro + e_prev_giro) / 2);
            Vel_Rot_ref = float(kp_giro * e_giro + ki_giro * inte_giro); // rad/s
            RPM_Rot_ref = Vel_Rot_ref * L_robot / 2 * 60 / (2 * pi);     //
            RPM_A_ref = -RPM_Rot_ref;
            RPM_B_ref = RPM_Rot_ref;
            e_prev_giro = e_giro;
            inte_prev_giro = inte_giro;
        }
    }
    else if (state_pid_pose == 1){ // 1: aavanzado apra llegara
        e_giro = atan2(Pose_Y_final, Pose_X_final) - Pose_Theta;
        if (abs(e_giro) > e_giro_margin*e_giro_margin_factor){
            state_pid_pose = 0;
        }
        else {
            if (e_desp < e_desp_margin){
                state_pid_pose = 2;
                e_prev_desp = 0;
                inte_prev_desp = 0;
            }
            else {
                inte_desp = inte_prev_desp + (dt * (e_desp + e_prev_desp) / 2);
                Vel_Desp_ref = float(kp_desp * e_desp + ki_desp * inte_desp); // m/s
                RPM_Desp_ref = Vel_Desp_ref * 60 / (2*pi);
                RPM_A_ref = RPM_Desp_ref; 
                RPM_B_ref = RPM_Desp_ref; 
                e_prev_desp = e_desp;
                inte_prev_desp = inte_desp;
            }
        }
    }
    else if (state_pid_pose == 2){// 2: girando para llegar
        e_giro = Pose_Theta_final - Pose_Theta;
        if (e_giro < e_giro_margin){
            state_pid_pose = 3;
            
        }
        else{
            inte_giro = inte_prev_giro + (dt * (e_giro + e_prev_giro) / 2);
            Vel_Rot_ref = float(kp_giro * e_giro + ki_giro * inte_giro); // rad/s
            RPM_Rot_ref = Vel_Rot_ref * L_robot / 2 * 60 / (2*pi); //
            RPM_A_ref = -RPM_Rot_ref; 
            RPM_B_ref = RPM_Rot_ref; 
            e_prev_giro = e_giro;
            inte_prev_giro = inte_giro;
        }
      }
    else if (state_pid_pose == 3){ // 3: idlee
        RPM_A_ref = 0; 
        RPM_B_ref = 0;    
    }
      
     //---------------POSE PI CONTROL------------------
    e_A = RPM_A_ref - RPM_A;
    e_B = RPM_B_ref - RPM_B;
    inte_A = inte_prev_A + (dt * (e_A + e_prev_A) / 2);
    inte_B = inte_prev_B + (dt * (e_B + e_prev_B) / 2);
    PWM_A_val = int(kp_A * e_A + ki_A * inte_A + (kd_A * (e_A - e_prev_A) / dt));
    PWM_B_val = int(kp_B * e_B + ki_B * inte_B + (kd_B * (e_B - e_prev_B) / dt));
    e_prev_A = e_A;
    e_prev_B = e_B;
    inte_prev_A = inte_A;
    inte_prev_B = inte_B;
    

    Serial.print("t:");
    Serial.print(",");
    Serial.print(t/1000);
    Serial.print("VelX:");
    Serial.print(",");
    Serial.print(Vel_X);
    Serial.print(",");
    Serial.print("VelY:");
    Serial.print(Vel_Y);
    Serial.print(",");
    Serial.print(",");
    Serial.print("VelTheta:");
    Serial.print(Vel_Theta);
    Serial.print(",");
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
