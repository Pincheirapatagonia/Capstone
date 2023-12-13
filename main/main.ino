#include <Servo.h>  //Imports the library Servo

//GeeKee CeeBee
#define trigPin 3  // TriggerSensor
#define echoPin 2  // EchoSensor

#define ServoPin 8  // Servo pin
#define MAXANG 180  // Servo máx angle
#define MINANG 0    // Servo min angle
#define SCOOPDELAY 5
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

Servo servo;    //Defines the object Servo of type(class) Servo
int angle = 0;  // Defines an integer


// ************ DEFINITIONS A (L)************
volatile long EncoderCountA = 0;
float ThetaA, ThetaA_prev;
float Dist_A, vel_A, RPM_A;
int PWM_A_val;
float kp_A = 0.902;
float ki_A = 708;
float kd_A = 0;

// ************ DEFINITIONS B (R)************
volatile long EncoderCountB = 0;
float ThetaB, ThetaB_prev;
float Dist_B, vel_B, RPM_B;
int PWM_B_val;
float kp_B = 0.109;
float ki_B = 0.0;
float kd_B = 0;

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
float Pose_Theta_ref;
float Vel_X, Vel_Y, Vel_Theta;
float Pose_X_final = 0.1;
float Pose_Y_final = 0;
float Pose_Theta_final = 0;

float kp_giro = 1;
float ki_giro = 0.0;
float kp_desp = 0.05;
float ki_desp = 0.0;
float e_giro_margin = 0.09;  // approx 5°
float e_giro_margin_factor = 1.1;
float e_desp_margin = 0.025;

int state_pid_pose = 0;
float e_giro, e_prev_giro, inte_giro, inte_prev_giro;
float Vel_Rot_ref, RPM_Rot_ref;
float e_desp, e_prev_desp, inte_desp, inte_prev_desp;
float Vel_Desp_ref, RPM_Desp_ref;

// ************ ODOMETRIA ENCODER ************
float enc_Pos_x, enc_Pos_y, enc_Theta;
float enc_Vel_ang, enc_Vel_lin, enc_Vel_x, enc_Vel_y;
float enc_vel_A, enc_vel_B, enc_RPM_A, enc_RPM_B;

// ************ TIEMPO************
int dt;
unsigned long t_init, t, t_prev;



char msg[60];

// PARAMETROS FISICOS ROBOT
float Diam_ruedas = 0.09;
float R_ruedas = Diam_ruedas / 2;
float L_robot = 0.370 - 0.055;
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

void setup() {
  Serial.begin(baud);
  while (!Serial) {
    delay(10);
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
  t_init = millis();
  t_prev = t_init;
}

void loop() {
  if ((millis() - t_prev) >= 100) {
    t = millis();
    kp_A = kp_A + 0.05;
    kp_B = kp_B + 0.05;

    //---------------CALCULAR VELOCIDADES------------------
    ThetaA = EncoderCountA;
    ThetaB = EncoderCountB;
    Dist_A = Dist_A + ((ThetaA - ThetaA_prev)) / NFactor * pi * Diam_ruedas;  // [m]
    Dist_B = Dist_B + ((ThetaB - ThetaB_prev)) / NFactor * pi * Diam_ruedas;  // [m]
    dt = t - t_prev;                                                          // [ms]
    enc_RPM_A = 1000 * (ThetaA - ThetaA_prev) / dt * 60.0 / NFactor;
    enc_RPM_B = 1000 * (ThetaB - ThetaB_prev) / dt * 60.0 / NFactor;
    enc_vel_A = enc_RPM_A * pi * 2 / 60.0;  // [rad/s]
    enc_vel_B = enc_RPM_B * pi * 2 / 60.0;  // [rad/s]
    ThetaA_prev = ThetaA;
    ThetaB_prev = ThetaB;

    //---------------ODOMETRIA ENCODER------------------
    enc_Vel_ang = R_ruedas * (enc_vel_B - enc_vel_A) / L_robot;  // [rad/s]
    enc_Theta = enc_Theta + enc_Vel_ang * dt / 1000;             // [rad]
    enc_Vel_lin = R_ruedas * (enc_vel_A + enc_vel_B) / 2;        // [m/s]
    enc_Vel_x = enc_Vel_lin * cos(enc_Theta);                    // [m/s]
    enc_Vel_y = enc_Vel_lin * sin(enc_Theta);                    // [m/s]
    enc_Pos_x = enc_Pos_x + (enc_Vel_x * dt) / 1000;             // [m]
    enc_Pos_y = enc_Pos_y + (enc_Vel_y * dt) / 1000;             // [m]
    Pose_X = enc_Pos_x;
    Pose_Y = enc_Pos_y;
    Pose_Theta = enc_Theta;
    Vel_X = enc_Vel_x;
    Vel_Y = enc_Vel_y;
    Vel_Theta = enc_Vel_ang;
    RPM_A = enc_RPM_A;
    RPM_B = enc_RPM_B;

    //---------------POSE PI CONTROL------------------
    e_desp = sqrt((Pose_X_final - Pose_X) * (Pose_X_final - Pose_X) + (Pose_Y_final - Pose_Y) * (Pose_Y_final - Pose_Y));
    if (state_pid_pose == 0) {  // 0: girando para alinear
      Pose_Theta_ref = atan2(Pose_Y_final, Pose_X_final);
      e_giro = Pose_Theta_ref - Pose_Theta;
      if (abs(e_giro) < e_giro_margin) {
        state_pid_pose = 1;
        e_prev_giro = 0;
        inte_prev_giro = 0;
      } else {
        inte_giro = inte_prev_giro + (dt * (e_giro + e_prev_giro) / 2);
        Vel_Rot_ref = float(kp_giro * e_giro + ki_giro * inte_giro);  // rad/s
        RPM_Rot_ref = Vel_Rot_ref * L_robot / 2 * 60 / (2 * pi);      //
        RPM_A_ref = -RPM_Rot_ref;
        RPM_B_ref = RPM_Rot_ref;
        e_prev_giro = e_giro;
        inte_prev_giro = inte_giro;
      }
    } else if (state_pid_pose == 1) {  // 1: aavanzado apra llegara
      Pose_Theta_ref = atan2(Pose_Y_final, Pose_X_final);
      e_giro = Pose_Theta_ref - Pose_Theta;
      if (abs(e_giro) > e_giro_margin * e_giro_margin_factor) {
        state_pid_pose = 0;
      } else {
        if (e_desp < e_desp_margin) {
          state_pid_pose = 2;
          e_prev_desp = 0;
          inte_prev_desp = 0;
        } else {
          inte_desp = inte_prev_desp + (dt * (e_desp + e_prev_desp) / 2);
          Vel_Desp_ref = float(kp_desp * e_desp + ki_desp * inte_desp);  // m/s
          RPM_Desp_ref = Vel_Desp_ref * 60 / (2 * pi);
          RPM_A_ref = RPM_Desp_ref;
          RPM_B_ref = RPM_Desp_ref;
          e_prev_desp = e_desp;
          inte_prev_desp = inte_desp;
        }
      }
    } else if (state_pid_pose == 2) {  // 2: girando para llegar
      Pose_Theta_ref = Pose_Theta_final;
      e_giro = Pose_Theta_ref - Pose_Theta;
      if (e_giro < e_giro_margin) {
        state_pid_pose = 3;
      } else {
        inte_giro = inte_prev_giro + (dt * (e_giro + e_prev_giro) / 2);
        Vel_Rot_ref = float(kp_giro * e_giro + ki_giro * inte_giro);  // rad/s
        RPM_Rot_ref = Vel_Rot_ref * L_robot / 2 * 60 / (2 * pi);      //
        RPM_A_ref = -RPM_Rot_ref;
        RPM_B_ref = RPM_Rot_ref;
        e_prev_giro = e_giro;
        inte_prev_giro = inte_giro;
      }
    } else if (state_pid_pose == 3) {  // 3: idlee
      RPM_A_ref = 0;
      RPM_B_ref = 0;
    }

    RPM_A_ref = 160.0;
    RPM_B_ref = 160.0;
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

    // if(kp_A >=  3){
    //   WriteDriverVoltageA(0);
    //   WriteDriverVoltageB(0);
    // } else{
    //   WriteDriverVoltageA(PWM_A_val);
    //   WriteDriverVoltageB(PWM_B_val);
    // }
    if (t<6000){
      WriteDriverVoltageA(150);
      WriteDriverVoltageB(150);
    }
    else{
      WriteDriverVoltageA(0);
      WriteDriverVoltageB(0);
    }
    

    //---------------PRINTS------------------
    Serial.print("t:");
    Serial.print((t - t_init)/10);
    Serial.print(",");
    // Serial.print("PosX:");
    // Serial.print(Pose_X);
    // Serial.print(",");
    // Serial.print("PosY:");
    // Serial.print(Pose_Y);
    // Serial.print(",");
    // Serial.print("Pos_Theta:");
    // Serial.print(round(Pose_Theta*180/pi));
    // Serial.print(",");
    // Serial.print("VelX:");
    // Serial.print(Vel_X);
    // Serial.print(",");
    // Serial.print("VelY:");
    // Serial.print(Vel_Y);
    // Serial.print(",");
    // Serial.print("VelTheta:");
    // Serial.print(Vel_Theta);
    // Serial.print(",");

    // Serial.print("Kp:");
    // Serial.print(kp_A);
    // Serial.print(",");
    Serial.print("RPMA:");
    Serial.print(RPM_A);
    Serial.print(",");
    Serial.print("RPMB:");
    Serial.print(RPM_B);
    Serial.print(",");
    // Serial.print("RPMA_REF:");
    // Serial.print(RPM_A_ref);
    // Serial.print(",");
    // Serial.print("RPMB_REF:");
    // Serial.print(RPM_B_ref);
    // Serial.print(",");

    // Serial.print("DESP_REF:");
    // Serial.print(",");
    // Serial.print(0);
    // Serial.print("DESP");
    // Serial.print(",");
    // Serial.print(e_desp);


    // Serial.print("Theta_REF:");
    // Serial.print(Pose_Theta_ref);
    // Serial.print(",");
    // Serial.print("Pose_Theta:");
    // Serial.print(Pose_Theta);
    // Serial.print(",")



    Serial.println("");

    t_prev = t;
  }
}
