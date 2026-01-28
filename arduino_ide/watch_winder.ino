#define BUTTON_PIN 3 
#define LONGPRESS_MS 1000 
#define MAX_ROTATIONS 500
#include <AccelStepper.h>
const int TOTAL_RUN_TIMES_STEPPER = 3;
const byte Fullstep = 4;
const byte Halfstep = 8;
const short fullResolution = 2038;
const float StepDegreeHalf = 11.32;
const float StepDegreeFull = 5.82;
bool firstMove = true;
int runTimes = 0;

long long touchStart = 0;

enum OperationState{
  STANDBY = 0,
  CALIBRATION_ENTRY = 1,
  CALIBRATION = 2,
  CALIBRATION_STOP = 3,
  OPERATION_START = 4,
  OPERATION = 5,
  OPERATION_STOP = 6
};

enum TouchState{
  TOUCH_STANDBY = 0,
  TOUCH_START = 1,
  TOUCH = 2,
  TOUCH_STOP = 3
};

enum TouchType{
  NONE = 0,
  SIMPLE = 1,
  LONG = 2
};

OperationState opState = STANDBY;
TouchState touchState = TOUCH_STANDBY;
TouchType touchType = NONE;

struct TouchButton { 
	 byte wasPressed = LOW; 
	 byte isPressed = LOW; 
};

TouchButton touch;


// IN1-IN3-IN2-IN4
AccelStepper motor1(Halfstep, 4, 6, 5, 7);

float degreeNormal = 180;
float degreeCalibration = 360;


void setup(void) {
  motor1.setMaxSpeed(1000.0);   // set the maximum speed
  motor1.setAcceleration(400); // set acceleration
  motor1.setSpeed(700);         // set initial speed
  motor1.setCurrentPosition(0); // set position
  pinMode(BUTTON_PIN, INPUT);
  // Serial.begin(115200);
}

void loop(void) {

  

  touch.isPressed = isTouchPressed(BUTTON_PIN); 
	touch.wasPressed = touch.isPressed;
  
  processTouch();
  processOperating();

}

void processTouch(){
  if(touchState == TOUCH_STANDBY && touch.isPressed){
    touchStart = millis();
    touchState = TOUCH_START;
  }
  else if(touchState == TOUCH_START)
  {
    touchState = TOUCH;
  }
  else if(touchState == TOUCH)
  {
    if(!touch.isPressed)
    {
      touchState = TOUCH_STOP;
      if( millis() - touchStart < LONGPRESS_MS){
        touchType = SIMPLE;
      }
    }
    else{
      if( millis() - touchStart >= LONGPRESS_MS && touchType != LONG)
      {
        touchType = LONG;
      }
    }
  }
  else if(touchState == TOUCH_STOP){
    touchState = TOUCH_STANDBY;
  }
}

void processOperating(){
  if(opState == STANDBY && touchType == LONG){
    opState = CALIBRATION_ENTRY;
  }
  else if(opState == STANDBY && touchType == SIMPLE){
    opState = OPERATION_START;
  }
  else if(opState == CALIBRATION_ENTRY){
    motor1.setMaxSpeed(500.0);   // set the maximum speed
    motor1.setAcceleration(200); // set acceleration
    motor1.setSpeed(300);         // set initial speed
    motor1.setCurrentPosition(0); // set position

    opState = CALIBRATION;
  }
  else if(opState == CALIBRATION){
    if(touchState == TOUCH){
      float moveRev = degreeCalibration * StepDegreeHalf;
      motor1.move(moveRev);
    }
    else{
      motor1.stop();
      opState = CALIBRATION_STOP;
    }
  }
  else if(opState == CALIBRATION_STOP){
    motor1.setCurrentPosition(0);
    motor1.disableOutputs();
    opState = STANDBY;
    touchType = NONE;
  }
  else if(opState == OPERATION_START){
    motor1.setMaxSpeed(1000.0);   // set the maximum speed
    motor1.setAcceleration(400); // set acceleration
    motor1.setSpeed(700);         // set initial speed
    float moveRev = degreeNormal * StepDegreeHalf;
    motor1.moveTo(moveRev);
    touchType = NONE;
    opState = OPERATION;
  }
  else if(opState == OPERATION){
    if(touchType == SIMPLE){
      opState = OPERATION_STOP;
      touchType = NONE;
      motor1.moveTo(0);
    }
    else{
      float moveRev = degreeNormal * StepDegreeHalf;
      if(motor1.distanceToGo() == 0){
          motor1.moveTo(-motor1.currentPosition());
          runTimes++;
      }
      if(runTimes >= MAX_ROTATIONS)
      {
        opState = OPERATION_STOP;
        motor1.moveTo(0);
      }
    }
  }
  else if(opState == OPERATION_STOP && motor1.distanceToGo() == 0){
    opState = STANDBY;
    runTimes = 0;
    motor1.disableOutputs();
  }


  if (opState != STANDBY)
  {
    motor1.run();
  }
}


bool isTouchPressed(int pin) 
{ 
	 return digitalRead(pin) == HIGH; 
}

