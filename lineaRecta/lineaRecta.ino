#include <Wire.h>
#include <MPU6050.h>

#define pi 3.1415
#define ENA 7 // D6
#define ENB 11

#define AIN1 4 // D1
#define AIN2 5  // D0

#define BIN1 10
#define BIN2 9

#define AC1 19 // D2
#define AC2 18 // D3

#define BC1 2 // D7
#define BC2 3 // D8

#define baud 9600


MPU6050 mpu;


// Definitions
float dist, err_dist,  e_dist = 0, e_prev_dist = 0;
float e_int, e_om = 0, e_prev_om = 0;
//--------------------Ref-------------------------


float ref_dist = 1;

float theta, theta_old, start_time;

volatile long EncoderCountA = 0;
float ThetaA, ThetaA_prev;
float Dist_A, vel_A, RPM_A, enc_RPM_A, enc_vel_A;

volatile long EncoderCountB = 0;
float ThetaB, ThetaB_prev;
float Dist_B, vel_B, RPM_B, enc_RPM_B, enc_vel_B;

//----------------KONTROL----------
float kp_dist = 180; // 20 // 15 // 15
float ki_dist = 0.01; //0.005 //0.006 //0.0058; 
float kd_dist = 0.1; // 0.0;  // 0.0003 //0.0003

float kp_om = 1.1; // 1
float ki_om = 0.000001; // 0.000001
float kd_om = 0.0035; // 0.003


float Diam_ruedas = 0.09;
float R_ruedas = Diam_ruedas/2;
float L_robot = 0.370-0.055;
float NFactor = 1400;
int PWM_min = 0;
int PWM_max = 255;

int vel_line;

int dt, t, t_prev;
int PWM_dist, PWM_vel, PWMA, PWMB;

float pitch = 0;
float roll = 0;
float yaw = 0;
float pitch_vel;
float roll_vel;
float yaw_vel;

void ISR_EncoderA1()
{
    if (digitalRead(AC1) == digitalRead(AC2))
    {
        EncoderCountA++;
    }
    else
    {
        EncoderCountA--;
    }
}

void ISR_EncoderA2()
{
    if (digitalRead(AC1) == digitalRead(AC2))
    {
        EncoderCountA--;
    }
    else
    {
        EncoderCountA++;
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
  

  pinMode(AC1, INPUT_PULLUP);
  digitalWrite(AC1, HIGH);
  pinMode(AC2, INPUT_PULLUP);
  digitalWrite(AC2, HIGH);

  pinMode(BC1, INPUT_PULLUP);
  digitalWrite(BC1, HIGH);
  pinMode(BC2, INPUT_PULLUP);
  digitalWrite(BC2, HIGH);

  attachInterrupt(digitalPinToInterrupt(AC1), ISR_EncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AC2), ISR_EncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BC1), ISR_EncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BC2), ISR_EncoderB2, CHANGE);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
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
  start_time = millis();
  Serial.println("iniciando");
  vel_line = 110;
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((millis()-t_prev)>=10){

    t = millis();
    dt = t - t_prev; 

    // Read normalized values
    Vector norm = mpu.readNormalizeGyro();

    // Calculate Pitch, Roll and Yaw
    yaw_vel = norm.ZAxis; // cambio de signo ya que esta dado vuelta
    pitch = pitch + pitch_vel * dt/1000;
    roll = roll + roll_vel * dt/1000;
    yaw = yaw + yaw_vel * dt/1000;

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
    e_dist = ref_dist - dist;
    e_om = yaw;

    e_int = e_int + e_om*dt;

    PWM_dist = int(kp_dist * e_dist + ki_dist * err_dist + (kd_dist * (e_dist - e_prev_dist) / dt));
    PWM_dist = clipPWM(PWM_dist);
    PWM_vel = int(kp_om * e_om + ki_om * e_int + (kd_om * (e_om - e_prev_om) / dt));

    if(PWM_dist + PWM_vel > 255){
      PWMA = 255;
      PWMB = PWM_dist - 2 * PWM_vel;
    } else
    {
      if(PWM_dist - PWM_vel > 255){
        PWMB = 255;
        PWMA = PWM_dist + 2 * PWM_vel;
      } else
      {
        PWMA = (PWM_dist + PWM_vel);
        PWMB = (PWM_dist - PWM_vel);
      }
    }
    PWMA = clipPWM(PWMA);
    PWMB = clipPWM(PWMB);

    //Serial.print("ref_dist:"); Serial.println(ref_dist);
    
    Serial.print("PosX:"); Serial.println(PWMA);
    Serial.print("PosY:"); Serial.println(PWMB);
    Serial.print("PosZ:"); Serial.println(PWMB);

    Serial.print("PWMA:"); Serial.println(PWMA);
    Serial.print("PWMB:"); Serial.println(PWMB);
    Serial.print("e_om:"); Serial.println(e_om);
    /*Serial.print("dist:"); Serial.println(dist);
    Serial.print("e_vel:"); Serial.println(e_vel);*/
    Serial.print("vel_angular:"); Serial.println(yaw_vel);
    Serial.print("encoderA:"); Serial.println(EncoderCountA);
    Serial.print("encoderB:"); Serial.println(EncoderCountB);

    WriteDriverVoltageA(PWMA);
    WriteDriverVoltageB(PWMB);
    e_prev_dist = e_dist;
    e_prev_om = e_om;
    t_prev = t;
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
    else{
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
    if (abs(PWM_val) > PWM_max) PWM_val = PWM_max;
    if (abs(PWM_val) < PWM_min) PWM_val= PWM_min;
    analogWrite(ENA, PWM_val);
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
    analogWrite(ENB,PWM_val);
}
