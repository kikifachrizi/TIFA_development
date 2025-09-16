#include <SoftwareSerial.h>

SoftwareSerial Serial2 (3, 2); // 2 rx, 3 tx

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);

}

void loop() {
  Serial2.write('V');
  delay(50);
}
