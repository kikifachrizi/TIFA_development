#include <TeensyThreads.h>
#include <Arduino.h>
#include <IntervalTimer.h>
#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>


// SDA 18
// SCL 19
#define PI 3.1415926535897932384626433832795
#define DEG2RAD 0.01745329251


float angleX,angleY,angleZ,accelX,accelY,accelZ,gyroX,gyroY,gyroZ;
float thetta,th,thetta_rad;
float dy_global,dx_global;
float ab = 1.0472; // 60 deg
float aa = 5.23599; //300 deg
float dX, dY;
float X,Y;

MPU6050 mpu6050(Wire);

SoftwareSerial pc(21, 20);
SoftwareSerial pc2(34, 35);

void parseString(String data);

IntervalTimer pidTimer;
IntervalTimer comTimer;

volatile bool pidFlag = false;
volatile bool comFlag = false;

const float PPR = 1050.0; 
const float RPM_CONVERSION = 60.0; 
float mes_rpm[4];

#define ENCODER_A_1 38
#define ENCODER_B_1 39

#define ENCODER_A_2 33
#define ENCODER_B_2 30

#define ENCODER_A_3 36
#define ENCODER_B_3 37

#define ENCODER_A_4 40
#define ENCODER_B_4 41

#define ENCODER_A_5 23
#define ENCODER_B_5 22

#define ENCODER_A_6 15
#define ENCODER_B_6 14

volatile int encoder_value_1 = 0; 
volatile int encoder_value_2 = 0; 
volatile int encoder_value_3 = 0; 
volatile int encoder_value_4 = 0; 
volatile int encoder_value_5 = 0; 
volatile int encoder_value_6 = 0; 

int pwm1 = 8;
int pwm2 = 24;
int pwm3 = 2;
int pwm4 = 5;

int dir1A = 9;
int dir2A = 25;
int dir3A = 4;
int dir4A = 6;

int dir1B = 10;
int dir2B = 29;
int dir3B = 3;
int dir4B = 7;

float rpm[4];
float radpers[4];
float e[4],eI[4],eD[4],laste[4];
float rP[4],rI[4],rD[4],satI[4];
float out[4] , output[4];
float dt = 0.1; //100ms
float sp = 50;
float w[4];
float Vx = 0.0,Vy = 0.0,Vth = 0.0;

float kp[4], ki[4], kd[4];

float R = 0.0635; //D = 127mm --> R = 0.0635m
float a1 = 0.785; //rad
float a2 = 2.356; //rad
float a3 = 3.927; //rad
float a4 = 5.498; //rad

void pidTask(){
  while(true){
    if(pidFlag){
      pidFlag = false;
      // Vx = 0;Vy = 0;Vth = -3;
      // rise time .5 , settling time 1
      kp[0] = 1.0726;
      ki[0] = 15.9984;
      kd[0] = 0;

      kp[1] = 1.3723;
      ki[1] = 17.8826;
      kd[1] = 0;

      kp[2] = 0;
      ki[2] = 20.755;
      kd[2] = 0;

      kp[3] = 0;
      ki[3] = 9.7198;
      kd[3] = 0;

      // rise time 0.54 , settling time 0.96
      // kp[0] = 1.121;
      // ki[0] = 14.4345;
      // kd[0] = 0;

      // kp[1] = 1.0083;
      // ki[1] = 14.885;
      // kd[1] = 0;

      // kp[2] = 0.87788;
      // ki[2] = 14.4707;
      // kd[2] = 0;

      // kp[3] = 1.0735;
      // ki[3] = 12.82;
      // kd[3] = 0;

      rpm[0] = encoder_value_1;
      rpm[1] = encoder_value_2;
      rpm[2] = encoder_value_3;
      rpm[3] = encoder_value_4;

      w[0] = ((sin(a1)*Vx + cos(a1)*Vy + Vth*R) / R)*(60/(2*PI));
      w[1] = ((-sin(a2)*Vx - cos(a2)*Vy + Vth*R) / R)*(60/(2*PI));
      w[2] = ((sin(a3)*Vx + cos(a3)*Vy + Vth*R) / R)*(60/(2*PI));
      w[3] = ((-sin(a4)*Vx - cos(a4)*Vy + Vth*R) / R)*(60/(2*PI));

      for(int j = 0;j< 4 ;j++){
        mes_rpm[j] = (rpm[j]/PPR)/(0.1)*RPM_CONVERSION;

        e[j] = w[j] - mes_rpm[j];
        eI[j] += e[j]*dt;
        eD[j] = (e[j] - laste[j])/dt;

        rP[j] = e[j]*kp[j];
        rI[j] = eI[j]*ki[j];
        rD[j] = eD[j]*kd[j];

        if(rI[j] > 250){
          satI[j] = 250;
        }else if(rI[j] < -250){
          satI[j] = -250;
        }else{
          satI[j] = rI[j];
        }

        out[j] = rP[j] + satI[j] + rD[j];

        if(out[j] > 255){
          output[j] = 255;
        }else if(out[j] < -255){
          output[j] = -255;
        }else{
          output[j] = out[j];
        }

        laste[j] = e[j];
        encoder_value_1 = 0;
        encoder_value_2 = 0;
        encoder_value_3 = 0;
        encoder_value_4 = 0;
      }

      pc2.print(out[0]);
      pc2.print(";");
      pc2.print(out[1]);
      pc2.print(";");
      pc2.print(out[2]);
      pc2.print(";");
      pc2.print(out[3]);
      pc2.print(";");
      pc2.print(mes_rpm[0]);
      pc2.print(";");
      pc2.print(mes_rpm[1]);
      pc2.print(";");
      pc2.print(mes_rpm[2]);
      pc2.print(";");
      pc2.print(mes_rpm[3]);
      pc2.println(";");
    }
  }
}


void encoder_isr_1() {
  int A_1 = digitalRead(ENCODER_A_1);
  int B_2 = digitalRead(ENCODER_B_1);

  if ((A_1 == HIGH) != (B_2 == LOW)) {
    encoder_value_1--;
  } else {
    encoder_value_1++;
  }
}
void encoder_isr_2() {
  int A_3 = digitalRead(ENCODER_A_2);
  int B_4 = digitalRead(ENCODER_B_2);

  if ((A_3 == HIGH) != (B_4 == LOW)) {
    encoder_value_2--;
  } else {
    encoder_value_2++;
  }
}
void encoder_isr_3() {
  int A_5 = digitalRead(ENCODER_A_3);
  int B_6 = digitalRead(ENCODER_B_3);

  if ((A_5 == HIGH) != (B_6 == LOW)) {
    encoder_value_3--;
  } else {
    encoder_value_3++;
  }
}
void encoder_isr_4() {
  int A_7 = digitalRead(ENCODER_A_4);
  int B_8 = digitalRead(ENCODER_B_4);

  if ((A_7 == HIGH) != (B_8 == LOW)) {
    encoder_value_4--;
  } else {
    encoder_value_4++;
  }
}
void encoder_isr_5() {
  int A_9 = digitalRead(ENCODER_A_5);
  int B_10 = digitalRead(ENCODER_B_5);

  if ((A_9 == HIGH) != (B_10 == LOW)) {
    encoder_value_5--;
  } else {
    encoder_value_5++;
  }
}
void encoder_isr_6() {
  int A_11 = digitalRead(ENCODER_A_6);
  int B_12 = digitalRead(ENCODER_B_6);

  if ((A_11 == HIGH) != (B_12 == LOW)) {
    encoder_value_6--;
  } else {
    encoder_value_6++;
  }
}


void motorTask(){
  while (true){
    analogWrite(pwm1, fabs(output[0]));
    analogWrite(pwm2, fabs(output[1]));
    analogWrite(pwm3, fabs(output[2]));
    analogWrite(pwm4, fabs(output[3]));
    
    if(out[0] > 0){
      digitalWrite(dir1A, HIGH);
      digitalWrite(dir1B, LOW);
    }else {
      digitalWrite(dir1A, LOW);
      digitalWrite(dir1B, HIGH);
    }

    if(out[1] > 0){
      digitalWrite(dir2A, HIGH);
      digitalWrite(dir2B, LOW);
    }else {
      digitalWrite(dir2A, LOW);
      digitalWrite(dir2B, HIGH);
    }

    if(out[2] > 0){
      digitalWrite(dir3A, HIGH);
      digitalWrite(dir3B, LOW);
    }else {
      digitalWrite(dir3A, LOW);
      digitalWrite(dir3B, HIGH);
    }

    if(out[3] > 0){
      digitalWrite(dir4A, HIGH);
      digitalWrite(dir4B, LOW);
    }else {
      digitalWrite(dir4A, LOW);
      digitalWrite(dir4B, HIGH);
    }
    
  }
}


void pcComm(){
  while (true){
    if (pc.available() > 0) {
      String input = pc.readStringUntil('\n');
      // Serial.println(input);
      input.trim();  
      parseString(input);
      comFlag = true;
    }
  }
}
float scale = 0.01; // nilai skala belum di kalibrasi per brp meternya
void wheel_odom(){
  while(true){
    thetta_rad = thetta * DEG2RAD; 
    
    dY = (encoder_value_6 * cos(ab) + encoder_value_5 * cos(aa));
    dX = (encoder_value_6 * sin(ab) + encoder_value_5 * sin(aa));

    dx_global = dX * cos(thetta_rad) - dY * sin(thetta_rad);
    dy_global = dX * sin(thetta_rad) + dY * cos(thetta_rad);
  
    encoder_value_5 = 0;
    encoder_value_6 = 0;
   
    X += (dx_global*-scale);
    Y += (dy_global*-scale); //berapa dx pulse per 1m

    Serial.print(accelX);
    Serial.print(";");
    Serial.print(accelY);
    Serial.print(";");
    Serial.print(accelZ);
    Serial.print(";");
    Serial.print(gyroX);
    Serial.print(";");
    Serial.print(gyroY);
    Serial.print(";");
    Serial.print(gyroZ);
    Serial.print(";");
    Serial.print(angleX);
    Serial.print(";");
    Serial.print(angleY);
    Serial.print(";");
    Serial.print(thetta);
    Serial.print(";");
    Serial.print(X/100);
    Serial.print(";");
    Serial.print(Y/100);
    Serial.println(";");
  }
}

void odom(){
  while(true){
    mpu6050.update();
    accelX = mpu6050.getAccX();
    accelY = mpu6050.getAccY();
    accelZ = mpu6050.getAccZ();

    gyroX = mpu6050.getGyroX();
    gyroY = mpu6050.getGyroY();
    gyroZ = mpu6050.getGyroZ();

    angleX = mpu6050.getAngleX();
    angleY = mpu6050.getAngleY();
    thetta = mpu6050.getAngleZ();      
  }
}

void pidISR() {
  pidFlag = true; 
}

void comISR(){
  if(!comFlag){
    Vx = 0.0; Vy = 0.0; Vth = 0.0;
    for(int k = 0;k<4;k++){
      encoder_value_1 = 0;
      encoder_value_2 = 0;
      encoder_value_3 = 0;
      encoder_value_4 = 0;
      w[k] = 0;
      mes_rpm[k] = 0;
      e[k] = 0;
      laste[k] = 0;
      eI[k] = 0;
      eD[k] = 0;
      rP[k] = 0;
      rI[k] = 0;
      rD[k] = 0;
      satI[k] = 0;
      out[k] = 0;
      output[k] = 0;
      analogWrite(pwm1, 0);
      digitalWrite(dir1A, LOW);
      digitalWrite(dir1B, LOW);

      analogWrite(pwm2, 0);
      digitalWrite(dir2A, LOW);
      digitalWrite(dir2B, LOW);

      analogWrite(pwm3, 0);
      digitalWrite(dir3A, LOW);
      digitalWrite(dir3B, LOW);

      analogWrite(pwm4, 0);
      digitalWrite(dir4A, LOW);
      digitalWrite(dir4B, LOW);
    }
  }
  comFlag = false;
}


void setup() {
  pc.begin(57600);
  pc2.begin(19200);
  Serial.begin(115200); 

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(pwm1 , OUTPUT);
  pinMode(dir1A , OUTPUT);
  pinMode(dir1B , OUTPUT);
  pinMode(pwm2 , OUTPUT);
  pinMode(dir2A , OUTPUT);
  pinMode(dir2B , OUTPUT);
  pinMode(pwm3 , OUTPUT);
  pinMode(dir3A , OUTPUT);
  pinMode(dir3B , OUTPUT);
  pinMode(pwm4 , OUTPUT);
  pinMode(dir4A , OUTPUT);
  pinMode(dir4B , OUTPUT);

  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  pinMode(ENCODER_A_2, INPUT_PULLUP);
  pinMode(ENCODER_B_2, INPUT_PULLUP);
  pinMode(ENCODER_A_3, INPUT_PULLUP);
  pinMode(ENCODER_B_3, INPUT_PULLUP);
  pinMode(ENCODER_A_4, INPUT_PULLUP);
  pinMode(ENCODER_B_4, INPUT_PULLUP);
  pinMode(ENCODER_A_5, INPUT_PULLUP);
  pinMode(ENCODER_B_5, INPUT_PULLUP);
  pinMode(ENCODER_A_6, INPUT_PULLUP);
  pinMode(ENCODER_B_6, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), encoder_isr_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_2), encoder_isr_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_3), encoder_isr_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_4), encoder_isr_4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_5), encoder_isr_5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_6), encoder_isr_6, CHANGE);

  threads.addThread(motorTask,3);
  threads.addThread(pidTask,3);
  threads.addThread(pcComm,3);
  threads.addThread(odom,1);
  threads.addThread(wheel_odom,2);

  pidTimer.begin(pidISR, 100000);
  comTimer.begin(comISR, 350000);
}

void loop() {
  // Serial.print(encoder_value_1);
  // Serial.print(" ");
  // Serial.print(encoder_value_2);
  // Serial.print(" ");
  // Serial.print(encoder_value_3);
  // Serial.print(" ");
  // Serial.print(encoder_value_4);
  // Serial.println(" ");
}


void parseString(String data) {
  int startIndex = 0;
  int endIndex = data.indexOf(' ');  
  int tokenCount = 0;

  while (endIndex != -1) {
    String token = data.substring(startIndex, endIndex);
    token.trim();  
    
    if (tokenCount == 0) {
      Vx = token.toFloat();  
    } else if (tokenCount == 1) {
      Vy = token.toFloat();  
    }
    startIndex = endIndex + 1;
    endIndex = data.indexOf(' ', startIndex);
    tokenCount++;
  }

  String lastToken = data.substring(startIndex);
  lastToken.trim();
  if (lastToken.length() > 0) {
    if (tokenCount == 2) {
      Vth = lastToken.toFloat(); 
    }
  }
}
