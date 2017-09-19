//test buttons

//pins
#define encoderPinA 2
#define encoderPinB 3
#define zigzagPin 7 
#define revPin 5
#define stopPin 4
#define fwdPin 6
#define enabledriver 8
#define accelMovePin 11
#define motorEnableDisablePin A4



void setup() {
  //debug
    Serial.begin(9600);  // output
    Serial.println("start");
  
  //button
  pinMode(revPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(fwdPin, INPUT_PULLUP);
  pinMode(zigzagPin, INPUT_PULLUP);
  pinMode(accelMovePin, INPUT_PULLUP);
  pinMode(motorEnableDisablePin, INPUT_PULLUP);


  Serial.println("pins state");
  Serial.println(digitalRead(stopPin));
  Serial.println(digitalRead(revPin));
  Serial.println(digitalRead(fwdPin));
  Serial.println(digitalRead(zigzagPin));
  Serial.println(digitalRead(accelMovePin));
  Serial.println(digitalRead(motorEnableDisablePin));

  Serial.println("");

}

void loop() {


  // If a switch is pushed down (low), set the sign value appropriately
  if (digitalRead(revPin) == 0) {
    delay(250);
    // debug
    Serial.println("revPin");

  }

  else if (digitalRead(fwdPin) == 0) {
    delay(250);
    // debug
    Serial.println("fwdPin");
  }

  else if (digitalRead(stopPin) == 0) {
    delay(250);
    // debug
    Serial.println("stopPin");
  }

  else if (digitalRead(zigzagPin) == 0) {
    delay(250);
    // debug
    Serial.println("zigzagPin");
  }

  else if (digitalRead(accelMovePin) == 0 ) {
    delay(250);
      //debug
      Serial.println("accelMovePin");
  }

  else if (digitalRead(motorEnableDisablePin) == 0 ) {
    delay(250);
      //debug
      Serial.println("motorEnableDisablePin");
  }
  
}

