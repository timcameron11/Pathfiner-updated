#include <QTRSensors.h>
#include <Adafruit_MotorShield.h>

//Definitions
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

#define LEFT_MOTOR_NUM  1
#define RIGHT_MOTOR_NUM 3

#define START_BUTTON 1
#define DEST_SENSOR  12

typedef enum {
  WAIT_TO_START,
  CALIBRATE,
  TRACKING,
  DONE
} RobotStates;

//Global Variables
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor;
Adafruit_DCMotor *rightMotor;
int errorB;
int defaultSpeed = 60;
int frontMicroSwitch = 12;

int frontSensorReading;

QTRSensorsRC lineSensor((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10},  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS]; 
RobotStates currentState;

long gameStart = 0;

//Code INIT
void setup() {
  Serial.begin(9600);
  AFMS.begin();
  leftMotor = AFMS.getMotor(LEFT_MOTOR_NUM);
  rightMotor = AFMS.getMotor(RIGHT_MOTOR_NUM);
  

  pinMode(START_BUTTON,INPUT);
  pinMode(DEST_SENSOR, INPUT);
  pinMode(frontMicroSwitch, INPUT_PULLUP);

  currentState = WAIT_TO_START;
}

void calibrate() {
   lineSensor.calibrate();
   
   leftMotor->setSpeed(50);
   leftMotor->run(FORWARD);
   rightMotor->setSpeed(50);
   rightMotor->run(BACKWARD);
}
long tLeft = 0;
long tRight = 0;

void loop() {
  
  switch(currentState) {
    case WAIT_TO_START:
      if(digitalRead(START_BUTTON) == 0) {
        currentState = CALIBRATE;
        gameStart = millis();
      }
    break; 

    case CALIBRATE:
      if((millis()- gameStart) > 3400){
        leftMotor->setSpeed(0);
        rightMotor->setSpeed(0);
        if(digitalRead(START_BUTTON) == 0) {
          currentState = TRACKING;  
        }
      }
      else {
        calibrate();
      }
    break;
    
    case TRACKING:
    { //leftMotor->setSpeed(0);
     //leftMotor->run(RELEASE);
     //rightMotor->setSpeed(0);
     //rightMotor->run(RELEASE);     
      //double kp = 0.0087642857;
      int val = lineSensor.readLine(sensorValues);
      int errorN = 3500 - val;
      int deltaError = errorN - errorB;

      //Sanity check to prevent crazyness
      if(deltaError > 500) {
        deltaError = 0;
      }
      
      errorB = errorN;
      
      //(working prior to addition of counter weight) double P = 0.0127642857 * errorN;
      double P = 0.0193642857 * errorN;
      //double D = 0.1148785713 * deltaError;
      double D/*stops over shoot*/ = /*0.1198785713*/.217848214125 * deltaError;
      //(working prior to addition of counter weight) double D = 0.17231785695 * deltaError;
      int correction = P+D;


//      if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] >= 2500){ //left turn
//        leftMotor->setSpeed(45);
//        leftMotor->run(FORWARD);
//        rightMotor->setSpeed(45);
//        rightMotor->run(BACKWARD);
//      }
//
//      if(sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] <= 2500){ //right turn
//        leftMotor->setSpeed(45);
//        leftMotor->run(BACKWARD);
//        rightMotor->setSpeed(45);
//        rightMotor->run(FORWARD);
//     }
      //Sanity check to prevent crazyness
      if(correction > 40) {
        correction = 40;
      }
      if(correction < -40) {
        correction = -40;
      }
      
      //Serial.println(val);
      //Serial.print(P);
      //Serial.print(" ");
      //Serial.print(D);
      //Serial.println();
      leftMotor->setSpeed (defaultSpeed - correction);
      rightMotor->setSpeed(defaultSpeed + correction);
      leftMotor->run(BACKWARD);
      rightMotor->run(BACKWARD);


      if((tLeft == 0) && (sensorValues[0] > 750)) {
        tLeft = millis();    
      }

      if((tRight == 0) && (sensorValues[7] > 750)) {
        tRight = millis();    
      }

      if((tLeft != 0) && (tRight != 0)) {
        Serial.println(tLeft - tRight);
        tLeft = 0;
        tRight = 0;
      }

      frontSensorReading = digitalRead(frontMicroSwitch);
      if(frontSensorReading == 1){
       
       currentState = WAIT_TO_START;
     }
    }
     break;

     case DONE:
       leftMotor->setSpeed(0);
       leftMotor->run(RELEASE);
       rightMotor->setSpeed(0);
       rightMotor->run(RELEASE);
       
    
      Serial.println(frontSensorReading);
     break;
     
   }
 }
