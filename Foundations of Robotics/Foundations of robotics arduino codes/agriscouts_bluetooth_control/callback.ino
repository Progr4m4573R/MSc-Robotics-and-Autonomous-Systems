String callback(void){
  bt_rx = btSerial.read();
    Serial.print("Received:");//Instruction recieved from 
    String state = "still";
    
    if (bt_rx == 'f') {
      //put code for forward motion here
      digitalWrite(ledpin, HIGH);
      state ="Forward";
      btSerial.println(state);
      return state;
      
    }
    else if (bt_rx == 'b') {
      //put code for backward motion here
      digitalWrite(ledpin, LOW);
      state = "Back";
      btSerial.println(state);
      return state;
    }
     else if (bt_rx == 'r'){
      //put code for right motion here, can be a set speed value such as 0.5 for example
      digitalWrite(ledpin,LOW);
      delay(1000);
      }
    else {
      return state;
    }
    
}
