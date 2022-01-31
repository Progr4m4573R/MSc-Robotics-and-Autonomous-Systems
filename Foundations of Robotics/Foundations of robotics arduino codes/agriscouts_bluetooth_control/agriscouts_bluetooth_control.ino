#include <SoftwareSerial.h>

SoftwareSerial btSerial(10, 11); // RX, TX PIN

String bt_rx;
int ledpin = 13;

void setup() {
  Serial.begin(9600);
  pinMode(ledpin, OUTPUT);
  btSerial.begin(9600);
}

void loop() {
  if (btSerial.available()) {
    bt_rx = btSerial.readString();
    Serial.print("Received:");
    Serial.println(bt_rx);
    if (bt_rx == "on") {
      digitalWrite(ledpin, HIGH);
      btSerial.println("LED turned on from command 'on'");
    }
    if (bt_rx == "off") {
      digitalWrite(ledpin, LOW);
      btSerial.println("LED turned off from command 'off'");
    }
    if (bt_rx == 111110) {
      digitalWrite(ledpin, HIGH);
      btSerial.println("LED turned on from ascii command 111 110");
    }
    if (bt_rx == 111102102) {
      digitalWrite(ledpin, LOW);
      btSerial.println("LED turned off from ascii command 111 102 102");
    }
    if (bt_rx == 3411110134) {
      digitalWrite(ledpin, HIGH);
      btSerial.println("LED turned on from command 34 111 101 34");
    }
    if (bt_rx == 3411110210234) {
      digitalWrite(ledpin, LOW);
      btSerial.println("LED turned off from ascii command 34 111 102 102 34");
    }
  }
}
