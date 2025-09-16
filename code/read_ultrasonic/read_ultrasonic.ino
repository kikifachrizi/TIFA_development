const int trigPin1 = 13;
const int echoPin2 = 12;

const int trigPin3 = 18;
const int echoPin4 = 5;

const int trigPin5 = 27;
const int echoPin6 = 14;

const int trigPin7 = 19;
const int echoPin8 = 34;

const int trigPin9 = 23;
const int echoPin10 = 35;

const int trigPin11 = 15;
const int echoPin12 = 2;

float duration, distance;
float mes_1,mes_2,mes_3,mes_4,mes_5,mes_6;

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin6, INPUT);
  pinMode(trigPin7, OUTPUT);
  pinMode(echoPin8, INPUT);
  pinMode(trigPin9, OUTPUT);
  pinMode(echoPin10, INPUT);
  pinMode(trigPin11, OUTPUT);
  pinMode(echoPin12, INPUT);
  Serial.begin(115200);
}

void loop() {
  mes_1 = read_ultra(trigPin1, echoPin2);
  mes_3 = read_ultra(trigPin3, echoPin4);
  mes_2 = read_ultra(trigPin5, echoPin6);
  mes_4 = read_ultra(trigPin7, echoPin8);
  mes_5 = read_ultra(trigPin9, echoPin10);
  mes_6 = read_ultra(trigPin11, echoPin12);

  Serial.print(mes_1);
  Serial.print("   ");
  Serial.print(mes_2);
  Serial.print("   ");
  Serial.print(mes_3);
  Serial.print("   ");
  Serial.print(mes_4);
  Serial.print("   ");
  Serial.print(mes_5);
  Serial.print("   ");
  Serial.println(mes_6);
  delay(50);
}

float read_ultra(int pin_trig, int pin_echo){
  digitalWrite(pin_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trig, LOW);

  duration = pulseIn(pin_echo, HIGH);
  distance = (duration*.0343)/2;
  return distance;
}
