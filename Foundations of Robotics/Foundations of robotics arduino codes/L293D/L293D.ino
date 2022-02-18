//www.elegoo.com
#include <SoftwareSerial.h>
SoftwareSerial btSerial(12, 13); // RX, TX PIN
#define ENA 21
#define ENB 51
#define IN1 22
#define IN2 23
#define LED 5

const int RB = 8;
const int LB = 9;

unsigned char carSpeed = 250;//250
bool state = LOW;

void forward() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(RB, HIGH);
  digitalWrite(LB, HIGH);
  Serial.println("Forward");
}

void back() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  Serial.println("Back");
}

void left() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(LB, LOW);
  digitalWrite(RB, HIGH);
  Serial.println("Left");
}

void right() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(RB, LOW);
  digitalWrite(LB, HIGH);
  Serial.println("Right");
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
}

void stateChange() {
  state = !state;
  digitalWrite(LED, state);
  Serial.println("Light");
}

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
}

void loop() {
  if (btSerial.available())
  {
    char getstr = btSerial.read();
    switch (getstr) {
      case 'f': forward(); break;
      case 'b': back();   break;
      case 'l': left();   break;
      case 'r': right();  break;
      case 's': stop();   break;
      case 'a': stateChange(); break;
      default:  break;
    }
  }
}
