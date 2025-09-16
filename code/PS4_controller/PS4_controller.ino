#include <PS4Controller.h>

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

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  PS4.begin("94:E6:86:38:07:50");
//  Serial2.println("Ready.");
}

void loop() {
  if (PS4.isConnected()) {
    //tombol
    if (PS4.Right()){
      rightStick = true;
      Serial2.write('D');
      delay(10);
    }else{
      rightStick = false;
    }
    
    if (PS4.Down()){
      downStick = true;
      Serial2.write('B');
      delay(10);
    }else{
      downStick = false;
    }
    
    if (PS4.Up()){
      upStick = true;
      Serial2.write('A');
      delay(10);
    }else{
      upStick = false;
    }
    
    if (PS4.Left()){
      leftStick = true;
      Serial2.write('C');
      delay(10);
    }else{
      leftStick = false;
    }

    if (PS4.Square()){
      squareStick = true;
      Serial2.write('L');
      delay(10);
    }else{
      squareStick = false;
    }
    if (PS4.Cross()){
      crossStick = true;
      Serial2.write('K');
      delay(10);
    }else{
      crossStick = false;
    }
    if (PS4.Circle()){
      circleStick = true;
      Serial2.write('M');
      delay(10);
    }else{
      circleStick = false;
    }
    if (PS4.Triangle()){
      triangleStick = true;
      Serial2.write('J');
      delay(10);
    }else{
      triangleStick = false;
    }

    if (PS4.Share()) {
      shareStick = true;
      Serial2.write('I');
      delay(10);
    }else{
      shareStick = false;
    }
    if (PS4.Options()) {
      optionStick = true;
      Serial2.write('Q');
      delay(10);
    }else{
      optionStick = false;
    }

    
    //analog kiri
    if (PS4.LStickX()) {
//      Serial2.printf("Left Stick x at %d\n", PS4.LStickX());
        if(PS4.LStickX() < -110){
//          Serial2.println('d');
            lStickX2 = true;
            Serial2.write('U');
            Serial.println('U');
            delay(10);
        }else if(PS4.LStickX() > 110){
//          Serial2.println('c');
            lStickX1 = true;
            Serial2.write('T');
            Serial.println('T');
            delay(10);
        }  
    }else{
      lStickX1 = false;
      lStickX2 = false;
      lStickY1 = false;
      lStickY2 = false;  
    }
    if (PS4.LStickY()) {
//      Serial2.printf("Left Stick y at %d\n", PS4.LStickY());
        if(PS4.LStickY() < -110){
//          Serial2.println('b');
            lStickY1 = true;
            Serial2.write('S');
            Serial.println('S');
            delay(10);
        }else if(PS4.LStickY() > 110){
//          Serial2.println('a');
            lStickY2 = true;
            Serial2.write('R');
            Serial.println('R');
            delay(10);
        }
    }else{
      lStickY1 = false;
      lStickY2 = false;
      lStickX1 = false;
      lStickX2 = false;    
    }

    //analog kanan
        if (PS4.RStickX()) {
        if(PS4.RStickX() < -110){
//          Serial2.println('k');
            rStickX2 = true;
            Serial2.write('3');
            delay(10);
            //rotasi kiri
        }else if(PS4.RStickX() > 110){
//          Serial2.println('l');
            rStickX1 = true;
            Serial2.write('4');
            delay(10);
            //rotasi kanan
        }
    }else{
      rStickX1 = false;
      rStickX2 = false;
      rStickY1 = false;
      rStickY2 = false;  
    }
    if (PS4.RStickY()) {
        if(PS4.RStickY() < -110){
//          Serial2.println('m');
            rStickY1 = true;
            Serial2.write('2');
            delay(10);
        }else if(PS4.RStickY() > 110){
//          Serial2.println('n');
            rStickY2 = true;
            Serial2.write('1');
            delay(10);
        }
    }else{
      rStickY1 = false;
      rStickY2 = false;
      rStickX1 = false;
      rStickX2 = false;    
    }

//  if (PS4.L1()){l1State = true;Serial2.print('q');}
//  else {l1State = false;}
//  if (PS4.R1()){r1State = true;Serial2.print('p');}
//  else {r1State = false;}
  if(l1State == false && r1State == false && rStickX1 == false && rStickX2 == false && rStickY1 == false && rStickY2 == false && lStickX1 == false && lStickX2 == false  && lStickY1 == false && lStickY2 == false && upStick == false && downStick == false && rightStick == false && leftStick == false && triangleStick == false && crossStick == false && squareStick == false && circleStick == false && optionStick == false && shareStick == false){
    Serial2.write('X');  
    Serial.println('X');
    delay(10);
  }
  }//is connected
}
