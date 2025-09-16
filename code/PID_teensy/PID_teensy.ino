#define ENCODER_A_1 8
#define ENCODER_B_1 9
#define PWM_PIN 2
#define DIR_PIN_1 3
#define DIR_PIN_2 4

volatile int encoder_pulses = 0; // Total pulsa dari encoder

// Konstanta encoder
const float PPR = 950.0; // Pulses per revolution
const float RPM_CONVERSION = 60.0; // RPS ke RPM

// PID Variables
float measured_rpm = 0.0;
float error = 0.0, prev_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float output_pwm = 0.0;
float out = 0.0;

// Timing
unsigned long last_time = 0;
unsigned long sample_time = 100; // 100 ms
float dt = 0.1; // Detik

void encoder_isr() {
  int A = digitalRead(ENCODER_A_1);
  int B = digitalRead(ENCODER_B_1);
  if ((A == HIGH) != (B == LOW)) {
    encoder_pulses--;
  } else {
    encoder_pulses++;
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), encoder_isr, CHANGE);
  
  last_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_time >= sample_time) {
//    dt = (current_time - last_time) / 1000;
    measured_rpm = (encoder_pulses / PPR) / (sample_time / 1000.0) * RPM_CONVERSION;
    encoder_pulses = 0; 
    
    output_pwm = pid_val(50,measured_rpm,2.0,0.1,0.0);
    motor_kontrol();
    
    last_time = current_time;
    Serial.print("Measured: ");
    Serial.print(measured_rpm);
    Serial.print(" RPM, PWM: ");
    Serial.println(output_pwm);
  }
}


float pid_val(float sp , float input , float p_, float i_ , float d_){
    error = sp - input;
    integral += error * dt; 
    derivative = (error - prev_error) / dt; 
    
    out = (p_ * error) + (i_ * integral) + (d_ * derivative);
    return out;

    prev_error = error;    
}

void motor_kontrol(){
    if (output_pwm > 255) output_pwm = 255;
    if (output_pwm < -255) output_pwm = -255;

    if (output_pwm < 0) {
      digitalWrite(DIR_PIN_1, LOW);
      digitalWrite(DIR_PIN_2, HIGH);
      output_pwm = -output_pwm; 
    } else {
      digitalWrite(DIR_PIN_1, HIGH);
      digitalWrite(DIR_PIN_2, LOW);
    }

    analogWrite(PWM_PIN, output_pwm);
}
