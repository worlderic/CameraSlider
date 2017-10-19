//test buttons

//vals
static char sign = 0;  // Holds -1, 1 or 0 to turn the motor on/off and control direction

//pins
#define encoderPinA 2
#define encoderPinB 3
#define ZIGZAG_PIN A3 
#define LEFT_PIN A1
#define STOP_PIN A0
#define RIGHT_PIN A2
#define enabledriver 8


void setup() {
  //debug
    Serial.begin(9600);  // output
    Serial.println("start");
  
  //button
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(ZIGZAG_PIN, INPUT_PULLUP);

  Serial.println(digitalRead(LEFT_PIN));
  Serial.println(digitalRead(STOP_PIN));
  Serial.println(digitalRead(RIGHT_PIN));
  Serial.println(digitalRead(ZIGZAG_PIN));


}

void loop() {


  // If a switch is pushed down (low), set the sign value appropriately
  if (digitalRead(LEFT_PIN) == 0) {
    delay(250);
    sign = 1;
    // debug
    Serial.println("-1");

  }

  else if (digitalRead(RIGHT_PIN) == 0) {
    delay(250);
    sign = -1;
    // debug
    Serial.println("1");
  }

  else if (digitalRead(STOP_PIN) == 0) {
    delay(250);
    sign = 0;
    // debug
    Serial.println("0");
  }

  else if (digitalRead(ZIGZAG_PIN) == 0) {
    delay(250);
    sign = 1;
    // debug
    Serial.println("ZIGZAG");
  }


}

