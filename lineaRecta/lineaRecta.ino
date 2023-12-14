#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define pi 3.1415
#define ENA 6
#define ENB 11

#define AIN1 4
#define AIN2 5
#define BIN1 10
#define BIN2 9

#define AC1 21 // D2
#define AC2 20 // D3
#define BC1 19 // D7
#define BC2 18 // D8

#define baud 9600


Adafruit_MPU6050 mpu;

// Definitions
float dist, err_dist,  e_dist = 0, e_prev_dist = 0;
float err_vel, e_vel = 0, e_prev_vel = 0;
//--------------------Ref-------------------------


float ref_dist = 1;


volatile long EncoderCountA = 0;
float ThetaA, ThetaA_prev;
float Dist_A, vel_A, RPM_A, enc_RPM_A, enc_vel_A;

volatile long EncoderCountB = 0;
float ThetaB, ThetaB_prev;
float Dist_B, vel_B, RPM_B, enc_RPM_B, enc_vel_B;

//----------------KONTROL----------
float kp_dist = 15; // 20 // 15 // 15
float ki_dist = 0.0058; //0.005 //0.006 //0.0058; 
float kd_dist = 0.0003; // 0.0;  // 0.0003 //0.0003

float kp_vel = 1; // 2 //1
float ki_vel = 0.00000153; // 0.000001 // 0.0000015 // 0.00000153
float kd_vel = 0.000011; //0.00001 // 0.000011 // 0.000011



float Diam_ruedas = 0.09;
float R_ruedas = Diam_ruedas/2;
float L_robot = 0.370-0.055;
float NFactor = 1400;
int PWM_min = 0;
int PWM_max = 255;

int dt, t, t_prev;
int PWM_dist, PWM_vel, PWMA, PWMB;

void ISR_EncoderA2() {
  bool PinB = digitalRead(AC2);
  bool PinA = digitalRead(AC1);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCountA++;
    }
    else {
      EncoderCountA--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCountA--;
    }
    else {
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
    }
    else {
      EncoderCountA++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCountA++;
    }
    else {
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
int clipPWM(int PWM_val) {
    if (PWM_val > 255) {
        PWM_val = 255;
    } else if (PWM_val < -255) {
        PWM_val = -255;
      
    }
    return PWM_val;
}
void setup() {
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(AC1), ISR_EncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AC2), ISR_EncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BC1), ISR_EncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BC2), ISR_EncoderB2, CHANGE);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  t = millis();
  ThetaA = EncoderCountA; 
  ThetaB = EncoderCountB;
  Dist_A = Dist_A + ((ThetaA - ThetaA_prev)) / NFactor * pi * Diam_ruedas; // [m]
  Dist_B = Dist_B + ((ThetaB - ThetaB_prev)) / NFactor * pi * Diam_ruedas; // [m]
  dist = (Dist_A + Dist_B)/2;
  dt = t - t_prev; // [ms]
  enc_RPM_A = 1000 * (ThetaA - ThetaA_prev) / dt * 60.0 / NFactor;
  enc_RPM_B = 1000 * (ThetaB - ThetaB_prev) / dt * 60.0 / NFactor;
  enc_vel_A = enc_RPM_A * pi * 2 / 60.0;                  // [rad/s]
  enc_vel_B = enc_RPM_B * pi * 2 / 60.0;                  // [rad/s]
  ThetaA_prev = ThetaA;
  ThetaB_prev = ThetaB;

  theta = (g.gyro.z*dt)/1000+theta_old;
  e_dist = ref_dist - dist;
  e_vel = (enc_vel_A - enc_vel_B) / 2;

  PWM_dist = int(kp_dist * e_dist + ki_dist * err_dist + (kd_dist * (e_dist - e_prev_dist) / dt));
  PWM_vel = int(kp_vel * e_vel + ki_vel * err_vel + (kd_vel * (e_vel - e_prev_vel) / dt));
  PWMA = (PWM_dist - PWM_vel)*200;
  PWMB = (PWM_dist + PWM_vel)*200;
  PWMA = clipPWM(PWMA);
  PWMB = clipPWM(PWMB);

  Serial.print("ref_dist:"); Serial.println(ref_dist);
  Serial.print("PWMA:"); Serial.println(PWMA);
  Serial.print("PWMB:"); Serial.println(PWMB);
  Serial.print("dist:"); Serial.println(dist);
  Serial.print("e_vel:"); Serial.println(e_vel);

  WriteDriverVoltageA(PWMA);
  WriteDriverVoltageB(PWMB);
  e_prev_dist = e_dist;
  e_prev_vel = e_vel;
  t_prev = t;
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
    else{
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
    if (abs(PWM_val) > PWM_max) PWM_val = PWM_max;
    if (abs(PWM_val) < PWM_min) PWM_val= PWM_min;
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
    if (abs(PWM_val) > PWM_max) PWM_val = PWM_max;
    if (abs(PWM_val) < PWM_min) PWM_val= PWM_min;
    analogWrite(ENB, abs(PWM_val));
}
