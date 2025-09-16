#include <PS4Controller.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

//odom var start
float dPA,dPB;
float prev_enc1,prev_enc2;
float curr_enc_1, curr_enc_2;
float thetta;
float ab = 30.8;
float aa = 29.2;
float dX,dY;
//odom var end

#define ENCODER_A_1 32 
#define ENCODER_B_1 33 

#define ENCODER_A_2 25 
#define ENCODER_B_2 26 

MPU6050 mpu6050(Wire);

volatile int encoder_value_1 = 0; 
volatile int encoder_value_2 = 0; 

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

const int trigPin = 5;
const int echoPin = 18;

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

bool lStickX1 = false;
bool lStickX2 = false;
bool lStickY1 = false;
bool lStickY2 = false;
bool rStickX1 = false;
bool rStickX2 = false;
bool rStickY1 = false;
bool rStickY2 = false;
bool upStick = false;
bool downStick = false;
bool rightStick = false;
bool leftStick = false;
bool triangleStick = false;
bool crossStick = false;
bool circleStick = false;
bool squareStick = false;
bool shareStick = false;
bool optionStick = false;
bool l1State = false;
bool r1State = false;

String Vx = "xxx";
String Vy = "xxx";
String tetta = "xxx";
float VxTemp;
float VyTemp;
float tettaTemp;
char kar = 'x';

float mapPecahan(long x, long fromLow, long fromHigh, float toLow, float toHigh)
{
  return (x - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


TaskHandle_t Task1;
TaskHandle_t Task2;

void read_ultra(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * SOUND_SPEED/2;
  distanceInch = distanceCm * CM_TO_INCH;
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm );
}

void setup() {
  Serial.begin(115200); 
  Serial2.begin(38400);
  PS4.begin("00:1A:7D:DA:71:12");
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  pinMode(ENCODER_A_2, INPUT_PULLUP);
  pinMode(ENCODER_B_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), encoder_isr_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_2), encoder_isr_2, CHANGE);
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
      mpu6050.update();
      thetta = mpu6050.getAngleZ();
      dX = (encoder_value_1 * cos(ab + (thetta/57.3)) + encoder_value_2 * cos(aa + (thetta/57.3))) / sin(ab-aa);
      dY = (encoder_value_1 * sin(ab + (thetta/57.3)) + encoder_value_2 * sin(aa + (thetta/57.3))) / sin(ab-aa);
    
//      Serial.print(dX);
//      Serial.print(' ');
//      Serial.println(dY);
  } 
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
     if (PS4.isConnected()) {
    //tombol
    if (PS4.Square()) {kar = '1';}
    if (PS4.Cross()) {kar = '2';}
    if (PS4.Circle()) {kar = '3';}
    if (PS4.Triangle()) {kar = '4';}

    float vx,vy,vth;
    
    if(abs(PS4.LStickX()) < 10){
      vx = 0;
    }else{
      vx = PS4.LStickX();
    }
    if(abs(PS4.LStickY()) < 10){
      vy = 0;
    }else{
      vy = PS4.LStickY();
    }
    if(abs(PS4.RStickX()) < 10){
      vth = 0;
    }else{
      vth = PS4.RStickX();
    }
    //analog kiri
    if (PS4.LStickX()) {
      VxTemp = mapPecahan(vx,-127.0,127.0,-3.7,3.7);
      if(VxTemp <= 5 && VxTemp >= -5){
        Vx = String(VxTemp);
      }
    }else{
      Vx = "0";
    }
    if (PS4.LStickY()) {
      VyTemp = mapPecahan(vy,-127.0,127.0,-3.7,3.7);
      if(VyTemp <= 5 && VyTemp >= -5){
        Vy = String(VyTemp);
      }else{Vy = "0";}
    }else{
      Vy = "0";
    }
    //analog kanan
    if (PS4.RStickX()) {
        tettaTemp = mapPecahan(vth,-127,127,-3.7,3.7);
        if(tettaTemp <= 5 && tettaTemp >= -5){
          tetta = String(tettaTemp);
        }
    }else{
        tetta = "0";
    }
  }

      Serial2.print(Vx);
      Serial2.print(";");
      // analog kiri
      Serial2.print(Vy);
      Serial2.print(";");
      //analog kanan
      Serial2.print(tetta);
      Serial2.print(";");
      //tombol
      Serial2.print(kar);
      Serial2.print(";");
      Serial2.print("\n");
      Serial.print(Vx);
      Serial.print(";");
      // analog kiri
      Serial.print(Vy);
      Serial.print(";");
      //analog kanan
      Serial.print(tetta);
      Serial.print(";");
      //tombol
//      Serial.print(kar);
//      Serial.print(";");
      Serial.print("\n");
      delay(100);
}
}

void loop() {
  
}
