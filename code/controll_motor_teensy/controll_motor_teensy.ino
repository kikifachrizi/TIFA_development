#define ENCODER_A_1 8 
#define ENCODER_B_1 9 

volatile int encoder_value_1 = 0;

void encoder_isr_1() {
  int A_1 = digitalRead(ENCODER_A_1);
  int B_2 = digitalRead(ENCODER_B_1);

  if ((A_1 == HIGH) != (B_2 == LOW)) {
    encoder_value_1--;
  } else {
    encoder_value_1++;
  }
}

int pwm_ = 2;
int dir_1 = 3;
int dir_2 = 4;

float rps;
float rpm;
float ppr = 950.0;
float pulse = 0;
float last_pulse = 0;

unsigned long start_t = 0.00;
unsigned long curr_t = 0.00;
unsigned long per = 100;
unsigned long dt;



void setup() {
  Serial.begin(115200);
  pinMode(pwm_, OUTPUT);
  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), encoder_isr_1, CHANGE);
  analogWriteResolution(15);
  start_t = millis();
}

void loop() {
  curr_t = millis();
  if(curr_t - start_t >= per){
    analogWrite(pwm_, 32757);
    digitalWrite(dir_1, HIGH);
    digitalWrite(dir_2, LOW);
    
    pulse = encoder_value_1;
    rps = (pulse - last_pulse) / ppr / 0.1;
    rpm = rps *60;
//    encoder_value_1 = 0;
    
    Serial.print("rpm : ");
    Serial.println(rpm);
//    Serial.print("  ppr : ");
//    Serial.println(pulse);
    start_t = curr_t;
    last_pulse = pulse;
  }
  
  


  
//  delay(5000);
//  analogWrite(pwm_, 0);
//  digitalWrite(dir_1, LOW);
//  digitalWrite(dir_2, LOW);
//  delay(5000);
//  analogWrite(pwm_, 255);
//  digitalWrite(dir_1, LOW);
//  digitalWrite(dir_2, HIGH);  
//  Serial.println(encoder_value_1);
//  delay(5000);
//  analogWrite(pwm_, 0);
//  digitalWrite(dir_1, LOW);
//  digitalWrite(dir_2, LOW);
//  delay(5000);
  
}
