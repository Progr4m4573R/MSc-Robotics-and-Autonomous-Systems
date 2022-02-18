#define NMOTORS 4
// Declare Pins
// RB, LB, RF, LF
const int ENC_A[] = {21,20,19,18};
const int ENC_B[] = {51,53,50,52};
const int PWM[] = {8,9,10,11};
const int INP_1[] = {22,25,27,29};//BACKWARD DIRECTIONAL PINS
const int INP_2[] = {23,24,26,28};//FOWARD DIRECTIONAL PINS
const int LED = 5; 


unsigned char carSpeed = 250;//250
bool state = LOW;
String callback(void){


  bt_rx = btSerial.read();
    Serial.print("Received:");//Instruction recieved from 
    String state = "still";
    
    if (bt_rx == 'f') {
      //put code for forward motion here
      forward();

      state ="Forward";
      btSerial.println(state);
      return state;
    }
    else if (bt_rx == 'b') {
      //put code for backward motion here
      back();

      state = "Back";
      btSerial.println(state);
      return state;
    }
    else if (bt_rx == 'r'){
      //put code for right motion here, can be a set speed value such as 0.5 for example
      right();
      state = "Right";
      btSerial.println(state);
      return state;
     }
     else if (bt_rx == 'l'){
      //put code for right motion here, can be a set speed value such as 0.5 for example
      left();

      state = "Left";
      btSerial.println(state);
      return state;
     }
     else if (bt_rx == 's'){
        stop();
        state = "Stop";
        btSerial.println(state);
        return state;
      }
      else if (bt_rx =='d'){
        
        state = "diagnosing...";
        diagnose();
        btSerial.print("Current car speed is: ");
        btSerial.println(carSpeed);
        return state;
        }
     else {
      return state;
    }
    
}
  void forward() {
    for(int k = 0; k <NMOTORS;k++){
      digitalWrite(ENC_A[k], HIGH);
      digitalWrite(ENC_B[k], HIGH);
      digitalWrite(INP_1[k], LOW);
      digitalWrite(INP_2[k], HIGH);
      digitalWrite(PWM[k], HIGH);
    }
  }
  
  void back() {
    for(int k = 0; k <NMOTORS;k++){
      digitalWrite(ENC_A[k], HIGH);
      digitalWrite(ENC_B[k], HIGH);
      digitalWrite(INP_1[k], HIGH);//FORWARDS DIRECTION PINS
      digitalWrite(INP_2[k], LOW); 
      digitalWrite(PWM[k], HIGH);
    }
  }
  
  void left() {
    for(int k = 0; k <NMOTORS;k++){
      analogWrite(ENC_A[k], carSpeed);
      analogWrite(ENC_B[k], carSpeed);
      digitalWrite(PWM[k], HIGH);
    }
    // RB, LB, RF, LF
    digitalWrite(INP_2[2], HIGH);//RF SPIN FOWARDS
    digitalWrite(INP_2[0], HIGH);//RB SPIN FOWARDS
    digitalWrite(INP_1[3], HIGH);//LF SPIN BACKWARDS
    digitalWrite(INP_1[1], HIGH);//LB SPIN BACKWARDS  
  }
  
  void right() {
    for(int k = 0; k <NMOTORS;k++){
      analogWrite(ENC_A[k], carSpeed);
      analogWrite(ENC_B[k], carSpeed);
      digitalWrite(PWM[k], HIGH);
    }
    // RB, LB, RF, LF
    digitalWrite(INP_1[2], HIGH);//RF SPIN BACKWARDS 
    digitalWrite(INP_1[0], HIGH);//RB SPIN BACKWARDS  
    digitalWrite(INP_2[3], HIGH);//LF SPIN FOWARDS
    digitalWrite(INP_2[1], HIGH);//LB SPIN FOWARDS  
  }
 void diagnose(){
  Serial.print("Current car speed is: ");
  Serial.println(carSpeed);   
  }
  void stop() {
    for(int k = 0; k <NMOTORS;k++){
      digitalWrite(ENC_A[k], LOW);
      digitalWrite(ENC_B[k], LOW);
      digitalWrite(PWM[k], LOW);
    }
  }

  void stateChange() {
    state = !state;
    digitalWrite(LED, state);
  }
