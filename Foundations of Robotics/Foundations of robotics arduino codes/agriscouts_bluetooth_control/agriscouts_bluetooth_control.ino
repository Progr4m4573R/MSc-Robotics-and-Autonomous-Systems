`#include <SoftwareSerial.h>
//Baised off the softwareSerial library located at:
//https://www.arduino.cc/en/Reference/SoftwareSerial



SoftwareSerial btSerial(12, 13); // RX, TX PIN
//used a char to store the command sent from the 
//BLE Terminal app on Android ehich sends ASCII or Hexidecimal commands
char bt_rx;

//use ledpins to show which way is front and which is back?
int ledpin =5;

void setup() {
  Serial.begin(9600);
  pinMode(ledpin, OUTPUT);//set ledpin as output so we can set it on and off
  btSerial.begin(9600);
}

void loop() {
  if (btSerial.available()) {//If the bluetooth buffer has recieved a command from a mobile device
    Serial.println(callback());
    while(btSerial.available()){
      btSerial.read();
  }
}
}
