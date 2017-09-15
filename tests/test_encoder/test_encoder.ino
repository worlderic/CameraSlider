int encoderPos = 0;  
int encoderPosTemp = 0; 

unsigned int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management
boolean A_set = false;
boolean B_set = false;

//pins
const int encoderPinA = 2;
const int encoderPinB = 3;

void setup() {
  //debug
    Serial.begin(9600);  // output
    Serial.println("start");
  
  //encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  // turn on pullup resistors
  //digitalWrite(encoderPinA, HIGH);
  //digitalWrite(encoderPinB, HIGH);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
  
}

void loop() {
  rotating = true;  // reset the debouncer
  encoderPosTemp = -encoderPos;
  
  if ( encoderPosTemp > 24 ){
        encoderPosTemp = 24;
      }
  if ( encoderPosTemp < 0 ){
        encoderPosTemp = 0;
      }
      
  if (lastReportedPos != encoderPos) {
    Serial.print("Index:");
    Serial.println(encoderPosTemp);
    Serial.println("");


    lastReportedPos = encoderPos;
  }

  
}

// Interrupt on A changing state
void doEncoderA() {
  // debounce
  if ( rotating ) delayMicroseconds(250);  // wait a little until the bouncing is done
  // Test transition, did things really change?
  if (  bitRead(PIND, 2) != A_set ) { // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
      encoderPos += 1;

    rotating = false;  // no more debouncing until loop() hits again
    Serial.println(bitRead(PIND, 2));

  }
}

// Interrupt on B changing state, same as A above
void doEncoderB() {
  if ( rotating ) delayMicroseconds(250);
  if ( bitRead(PIND, 3) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
      encoderPos -= 1;

    rotating = false;
    Serial.println(bitRead(PIND, 3));

  }
}

