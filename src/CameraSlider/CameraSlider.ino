//CameraSlider based on V9 file

#include <AccelStepper.h>
#include <LiquidCrystal.h>

/* PIN numbers 
*  2 encoderPinA
*  3 encoderPinB
*  4 stopPin
*  5 revPin
*  6 fwdPin
*  7 zigzagPin
*  8 enableDriver
*  9 STEP
*  10 DIRECTION
*  11 accelMovePin
*  12 pinSdReset
*  A4 motorEnbDisPin
*  A0 pin_MS_M1 microstepping
*  A1 pin_MS_M2 microstepping
*  A2 pin_MS_M2 microstepping
*  A3 pin_MS_STATE
*/ 

/*  Step resolution DRV8825
*  MODE0  MODE1 MODE2 Microstep Resolution 
*  Low Low Low Full step
*  High  Low Low Half step
*  Low High  Low 1/4 step 
*  High  High  Low 1/8 step
*  Low Low High  1/16 step
*  High  Low High  1/32 step
*  Low High  High  1/32 step
*  High  High  High  1/32 step
*/

#define DEBUG
#define DEBUG_SETUP
//#define DEBUG_BUTTONS
//#define DEBUG_ENCODER
//#define DEBUG_RUNMOTOR

//define features
#define ACCEL_MOVE // accelerate until reach the end of slider
#define ZIGZAG_MOVE // zigzap to the end of slider and back to home pos
#define MICRO_STEP    // define MICRO_STEP or MICRO_STEP_BASIC
//#define MICRO_STEP_BASIC // define MICRO_STEP or MICRO_STEP_BASIC
#define MOTOR_ENABLE_DISABLE_OPTION // button to enable or disable motors when not running OR #define MOTOR_DISABLE_OPTION
//#define MOTOR_DISABLE_OPTION // no button to disable or enable motores when not running OR #define MOTOR_ENABLE_DISABLE_OPTION

//define basic pins
  const int encoderPinA = 2, encoderPinB = 3; //Encoder pin interrupts
  const int revPin = 5, stopPin = 4, fwdPin = 6; //Reverse Foward Stop pins
  const int enableDriver = 8; // enable driver pin
  AccelStepper stepper1(AccelStepper::DRIVER, 9, 10); // pins 9 STEP  10 DIRECTION

#ifdef MOTOR_ENABLE_DISABLE_OPTION
  const int motorEnableDisablePin = A4;
#endif

#ifdef ZIGZAG_MOVE
  const int zigzagPin = 7;
  float zigzagMaxSpeed, zigzagAccel; 
#endif

#ifdef ACCEL_MOVE
  const int accelMovePin = 11;
  float accelMoveMaxSpeed, accelMovePinAccel, accelMoveMaxSpeedBrake, accelMoveMaxAccelBrake;
  long accelPosOvershoot;
  long accelPosIntermediate;
#endif

#ifdef MICRO_STEP_BASIC
  const int pinSdReset = 12; //stepper driver reset pin
  const int pin_MS_M0 = A0, pin_MS_M1 = A1, pin_MS_M2 = A2;
  const float stepper1MaxAccel = 2000;
  const long sliderTotalSteps = 27000;
#endif

#ifdef MICRO_STEP
  const int pinSdReset = 12; //stepper driver reset pin for microstepping change
  const int pin_MS_M0 = A0, pin_MS_M1 = A1, pin_MS_M2 = A2, pin_MS_STATE = A3;
  float stepper1MaxSpeed, stepper1MaxAccel;
  long sliderTotalSteps;
#endif

//Variables
  const int encoderMax = 23; // encoder turns 24
  boolean stateRun = false;
  boolean stateOutputs = false;
  const long sliderHomePosition = 0;
  const float homeSpeed = 4000;
  float currentSpeed = 0;
  int encoderPos = 0;
  int encoderPosTemp = 0;
  unsigned long lastReportedPos = 1;   
  static boolean rotating = false;   
  boolean A_set = false;
  boolean B_set = false;
  int speedArray[24]={0,1,2,4,8,16,32,64,128,256,512,768,1024,1280,1536,1792,2048,2304,2560,2816,3072,3328,3584,3840};


///////////////// SETUP
void setup() {

#ifdef DEBUG
  Serial.begin(9600);
  delay(500);
#endif

//encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
  attachInterrupt(1, doEncoderB, CHANGE);


//buttons
  pinMode(revPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(fwdPin, INPUT_PULLUP);
  
  #ifdef ZIGZAG_MOVE
    pinMode(zigzagPin, INPUT_PULLUP);
  #endif
  
  #ifdef ACCEL_MOVE
    pinMode(accelMovePin, INPUT_PULLUP);
  #endif

//stepper driver setup
  delay(1000);
  stepper1.setPinsInverted(false, false, true);
  stepper1.setEnablePin(enableDriver);

//micro step setup
#ifdef MICRO_STEP
  pinMode( pin_MS_M0, OUTPUT);
  pinMode( pin_MS_M1, OUTPUT);
  pinMode( pin_MS_M2, OUTPUT);
  pinMode( pin_MS_STATE, OUTPUT); //change to INPUP_PULLUP when button upgrade
  pinMode( pinSdReset, OUTPUT);
  digitalWrite( pinSdReset, HIGH);
  
  digitalWrite( pin_MS_STATE, HIGH); // LOW 1/2 ; HIGH 1/4 //delete when button upgrade

  // micro stepping 1/2 HLL
  if ( digitalRead(pin_MS_STATE) == 0 ){
    digitalWrite( pin_MS_M0, HIGH);
    digitalWrite( pin_MS_M1, LOW);   
    digitalWrite( pin_MS_M2, LOW);   
    digitalWrite( pinSdReset, LOW);
    delay(500);
    digitalWrite( pinSdReset, HIGH);
    delay(500);      
    stepper1MaxAccel = 2000;
    sliderTotalSteps = 13500;

    #ifdef ZIGZAG_MOVE
      zigzagMaxSpeed = 4000;
      zigzagAccel = 150; 
    #endif

    #ifdef ACCEL_MOVE // 1/2
      accelMoveMaxSpeed = 4000;
      accelMovePinAccel = 850;
      accelPosOvershoot = 20000;
      accelPosIntermediate = 12000;
      accelMoveMaxSpeedBrake = 3000;
      accelMoveMaxAccelBrake = 4000;
    #endif
  }

  // micro stepping 1/4 LHL
  else if ( digitalRead(pin_MS_STATE) == 1 ){
    digitalWrite( pin_MS_M0, LOW);
    digitalWrite( pin_MS_M1, HIGH); 
    digitalWrite( pin_MS_M2, LOW);   
    digitalWrite( pinSdReset, LOW);
    delay(500);
    digitalWrite( pinSdReset, HIGH);
    delay(500);
    stepper1MaxAccel = 2000;
    sliderTotalSteps = 27000;
    
    #ifdef ZIGZAG_MOVE
      zigzagMaxSpeed = 4000;
      zigzagAccel = 600; 
    #endif
    
    #ifdef ACCEL_MOVE
      accelMoveMaxSpeed = 4000;
      accelMovePinAccel = 350;
      accelPosOvershoot = 55000;
      accelPosIntermediate = 24000;
      accelMoveMaxSpeedBrake = 4000;
      accelMoveMaxAccelBrake = 4000;
    #endif
  }

  stepper1.setCurrentPosition(sliderHomePosition);
#endif

//MICRO_STEP_BASIC 1/4 LHL
#ifdef MICRO_STEP_BASIC 
  pinMode( pin_MS_M0, OUTPUT);
  pinMode( pin_MS_M1, OUTPUT);
  pinMode( pin_MS_M2, OUTPUT);
  pinMode( pinSdReset, OUTPUT);
  
  digitalWrite( pinSdReset, LOW);
  digitalWrite( pinSdReset, HIGH);
  
  digitalWrite( pin_MS_M0, LOW);
  digitalWrite( pin_MS_M1, HIGH);    
  digitalWrite( pin_MS_M2, LOW);   

  
  #ifdef ZIGZAG_MOVE
    zigzagMaxSpeed = 4000;
    zigzagAccel = 600;  
  #endif

  #ifdef ACCEL_MOVE
    accelMoveMaxSpeed = 4000;
    accelMovePinAccel = 350;
    accelPosOvershoot = 55000;
    accelPosIntermediate = 24000;
    accelMoveMaxSpeedBrake = 4000;
    accelMoveMaxAccelBrake = 4000;
  #endif
  
  stepper1.setCurrentPosition(sliderHomePosition);
#endif

#ifdef MOTOR_ENABLE_DISABLE_OPTION
  pinMode( motorEnableDisablePin, INPUT_PULLUP);
  motorEnableDisable();
#endif

#ifdef MOTOR_DISABLE_OPTION
  stepper1.disableOutputs();
#endif 

delay(1000);

//////////////////   DEBUG
#ifdef DEBUG
    Serial.println ("START DEBUG");
    Serial.println ("");
    Serial.println ("DEBUG: ON");
  
  #ifdef DEBUG_SETUP
    Serial.println ("DEBUG_SETUP: ON");
  #endif
  
  #ifdef DEBUG_BUTTONS
    Serial.println ("DEBUG_BUTTONS: ON");
  #endif
  
  #ifdef DEBUG_ENCODER
    Serial.println ("DEBUG_ENCODER: ON");
  #endif
  
  #ifdef DEBUG_RUNMOTOR
    Serial.println ("DEBUG_RUNMOTOR: ON");
  #endif
  
  #ifdef MICRO_STEP
    Serial.print("MICRO_STEP: ON");
    Serial.print("\t");
    if ( digitalRead(pin_MS_STATE) == LOW){
      Serial.print("SET 1/2");
    }
    else {
      Serial.print("SET 1/4");
    }
    Serial.println ("");
  #endif

  #ifdef MICRO_STEP_BASIC
    Serial.println ("MICRO_STEP_BASIC: ON");
  #endif

  #ifdef MOTOR_ENABLE_DISABLE_OPTION
    Serial.println ("MOTOR_ENABLE_DISABLE_OPTION: ON");
  #endif

  #ifdef MOTOR_DISABLE_OPTION
    Serial.println ("MOTOR_DISABLE_OPTION: ON");
  #endif

  #ifdef ZIGZAG_MOVE
    Serial.println ("ZIGZAG_MOVE: ON");     
  #endif

  #ifdef ACCEL_MOVE
    Serial.println ("ACCEL_MOVE: ON");     
  #endif 
  delay(1000);
#endif 
}

//////////////////// LOOP

void loop() {
  rotating = true;  // reset the debouncer

  readbuttons();
  runstepper1();
  debug_encoder_runmotor_speed();

}

///////////////////// FUNCTIONS

//Run motor function
void runstepper1(){
  encoderPosTemp = -encoderPos;
  if ( encoderPosTemp > encoderMax ){
    encoderPosTemp = encoderMax;
  }
  if ( encoderPosTemp < 0 ){
    encoderPosTemp = 0;
  }

  currentSpeed = speedArray[encoderPosTemp];
  stepper1.setMaxSpeed(currentSpeed);
  stepper1.run();
  #ifdef MOTOR_ENABLE_DISABLE_OPTION
    if (stepper1.isRunning() == false) {
      delay(50);
      motorEnableDisable();
    }
  #endif
    
  #ifdef MOTOR_DISABLE_OPTION
    if (stepper1.isRunning() == false) {
      delay(50);
      stepper1.disableOutputs();
    }
  #endif
}

//readbuttons function REV FWD STOP and upgrades
void readbuttons() {
  if (digitalRead(fwdPin) == 0) {
    delay(250);
  
    #ifdef DEBUG_BUTTONS
      Serial.println("FWD");
      delay(500);      
    #endif

    stepper1.stop();
    stepper1.enableOutputs();
    delay(250);
    stepper1.setAcceleration(stepper1MaxAccel);
    stepper1.moveTo(sliderTotalSteps);
    encoderPos = 0;
    encoderPosTemp = 0;
  }

  else if (digitalRead(revPin) == 0) {
    delay(250);
  
    #ifdef DEBUG_BUTTONS
      Serial.println("REV");
      delay(500);      
    #endif
  
    stepper1.stop();
    stepper1.enableOutputs();
    delay(250);
    stepper1.setAcceleration(stepper1MaxAccel);
    stepper1.moveTo(sliderHomePosition);
    encoderPos = 0;
    encoderPosTemp = 0;
  }

  else if (digitalRead(stopPin) == 0) {
    delay(250);
  
    #ifdef DEBUG_BUTTONS
      Serial.println("STOP 0");
      delay(500);      
    #endif
  
    stepper1.stop();
  
    #ifdef MOTOR_DISABLE_OPTION
      stepper1.disableOutputs();
    #endif    
  
    #ifdef MOTOR_ENABLE_DISABLE_OPTION
      motorEnableDisable();
    #endif
    
    encoderPos = 0;
    encoderPosTemp = 0;
  }

#ifdef ZIGZAG_MOVE
  else if (digitalRead(zigzagPin) == 0) {
    delay(250);
  
    #ifdef DEBUG_BUTTONS
      Serial.println("ZIGZAG");
      delay(500);
    #endif
    
    stepper1.stop();
    stepper1.enableOutputs();
    delay(250);
    stepper1.setMaxSpeed(homeSpeed);
    stepper1.setAcceleration(homeSpeed);
    stepper1.runToNewPosition(sliderHomePosition);
    delay(1000);
    stepper1.setMaxSpeed(zigzagMaxSpeed);
    stepper1.setAcceleration(zigzagAccel);
    stepper1.runToNewPosition(sliderTotalSteps);
    stepper1.runToNewPosition(sliderHomePosition);
    
    #ifdef MOTOR_DISABLE_OPTION
      stepper1.disableOutputs();
    #endif
    
    #ifdef MOTOR_ENABLE_DISABLE_OPTION
      motorEnableDisable();
    #endif
    
    encoderPos = 0;
    encoderPosTemp = 0;     
  }
#endif

#ifdef ACCEL_MOVE
  else if (digitalRead(accelMovePin) == 0 ) {
    delay(250);
    
    #ifdef DEBUG_BUTTONS
      Serial.println("ACCEL_MOVE");
      delay(500);
    #endif
    
    stepper1.stop();
    stepper1.enableOutputs();
    delay(250);
    stepper1.setMaxSpeed(homeSpeed);
    stepper1.setAcceleration(homeSpeed);
    stepper1.runToNewPosition(sliderHomePosition);
    delay(1000);
    
    stepper1.setMaxSpeed(accelMoveMaxSpeed);
    stepper1.setAcceleration(accelMovePinAccel);
    stepper1.moveTo(accelPosOvershoot);
    while ( stepper1.currentPosition() < accelPosIntermediate){     
      stepper1.run();
    }
    stepper1.stop();
    encoderPos = 0;
    encoderPosTemp = 0;
    
    stepper1.setMaxSpeed(accelMoveMaxSpeedBrake);
    stepper1.setAcceleration(accelMoveMaxAccelBrake);
    stepper1.moveTo(sliderTotalSteps);
    while ( stepper1.currentPosition() < sliderTotalSteps){     
      stepper1.run();
    }
    stepper1.stop();
    
    encoderPos = 0;
    encoderPosTemp = 0;         
    
    #ifdef MOTOR_DISABLE_OPTION
      stepper1.disableOutputs();
    #endif  
  
    #ifdef MOTOR_ENABLE_DISABLE_OPTION
    motorEnableDisable();
    #endif
    
    encoderPos = 0;
    encoderPosTemp = 0;     
  }
#endif
}

// Interrupt on A changing state
void doEncoderA() {

// debounce
  if ( rotating ) delayMicroseconds(250);  // wait a little until the bouncing is done
  
  // Test transition, did things really change?
    if ( bitRead(PIND, 2) != A_set ) { // debounce once more
      A_set = !A_set;
  
  // adjust counter + if A leads B
    if ( A_set && !B_set )
      encoderPos += 1;
      rotating = false;  // no more debouncing until loop() hits again
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
    }
}

// MOTOR_ENABLE_DISABLE_OPTION function
#ifdef MOTOR_ENABLE_DISABLE_OPTION
void motorEnableDisable(){
    if ( digitalRead(motorEnableDisablePin) == 0 ){
      stepper1.enableOutputs();
     }  
    else if ( digitalRead(motorEnableDisablePin) == 1 ) {
      stepper1.disableOutputs();
    }      
}
#endif

//debug fucntion DEBUG_ENCODER DEBUG_RUNMOTOR
void debug_encoder_runmotor_speed(){
  
#ifdef DEBUG_ENCODER
  if (lastReportedPos != encoderPos) {
    Serial.print("ENCODER:");
    Serial.print(encoderPosTemp);
    Serial.println("");
    lastReportedPos = encoderPos;
  }
#endif

#ifdef DEBUG_RUNMOTOR
  if (lastReportedPos != encoderPos) {
    Serial.print("ENCODER:"); Serial.print(encoderPosTemp);
    Serial.print("\t");
    Serial.print("SPEED:"); Serial.print(stepper1.speed());
    Serial.print("\t");
    Serial.print("POSITION:"); Serial.print(stepper1.currentPosition());
    Serial.println("");
    lastReportedPos = encoderPos;
  }
#endif 
} 
