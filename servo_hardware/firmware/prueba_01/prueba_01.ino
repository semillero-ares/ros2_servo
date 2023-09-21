#include <Servo.h>
Servo myservo;  
int pos = 90;   
int ref = 0;

void setup() {
  myservo.attach(3);  
  Serial.begin(115200);
}

void loop() {
    pos = pos + min(1,max(-1,ref-pos));
    //Serial.print(ref);
    //Serial.print(",");
    //Serial.println(pos);
    myservo.write(180+18-pos);             
    delay(15);                      
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
String inputString = "";      // a String to hold incoming data
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      ref = inputString.toInt();
      Serial.println(pos);
      inputString = "";
    }
  }
}
