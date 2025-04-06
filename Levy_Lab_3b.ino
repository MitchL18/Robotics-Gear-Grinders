// Mitchell Levy
// Professor Gormanly
// Robotics Lab 3a
// 03/13/2025

#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;

Servo headServo; // create servo object to control a servo

// switches
const boolean HEAD_DEBUG = false;
const boolean TIMING_DEBUG = false;
const boolean US_DEBUG = true;

const boolean SERVO_ON = true;
const boolean US_ON = true;

// Head Servo Timing 
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 120;

// head servo constants 
const int HEAD_SERVO_PIN = 12;
const int NUM_HEAD_POSITIONS = 8;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {172, 172, 172, 172, 85, 85, 85, 85}; 

// head servo data
boolean headDirectionClockwise = false;
int currentHeadPosition = 0;

// Initialize Ultrasonic
const int ECHO_PIN = 5;
const int TRIG_PIN = 4;

// Ultrasonic max distance
const float MAX_DISTANCE = 300.0;

// determine the normalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 5; // don't care about data less than 5cm

// Ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long WAIT_AFTER_HEAD_STARTS_MOVING = 80; // Time to wait between pings
boolean usReadFlag = false;                             // ensures 1 reading

// current US distance reading
int currentReadPosition = 0;

float distanceReadings[NUM_HEAD_POSITIONS];

// New stuff below:

float distance;
double previousError;

// desired position
const double desiredState = (double) 30;

//1.5, 0, 100
const double kp = 1;
const double ki = 0.0;
const double kd = 1.0;

double kiTotal = 0.0;
double priorError = 0.0;
long prevTime = millis();

// Threshold for detecting an object in front of the robot
const float OBJECT_THRESHOLD = 10.0;  // Distance in cm, adjust as needed

void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);

  // initialize the head position to start
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(172); // start at positin 172

  // initialize the US pins
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // initialize distance readings
  for(int i=0; i<NUM_HEAD_POSITIONS; i++) {
    distanceReadings[i] = MAX_DISTANCE;
  }

  // start delay
  delay(3000);
  buzzer.play("c32");

  motors.setSpeeds(-80, -80);
}

void loop() {
  // put your main code here, to run repeatedly:

  // perfrom head movement
  moveHead();

  // update the current distance
  usReadCm();

  // Check if there is an object in front
  bool objectDetected = (distanceReadings[5] < OBJECT_THRESHOLD || distanceReadings[6] < OBJECT_THRESHOLD || distanceReadings[7] < OBJECT_THRESHOLD);

  // If there is an object detected, apply object avoidance PID
  objectDetected = false;
  if (objectDetected) {
    Serial.print("OBJECT DETECTED!");
    // Object avoidance PID (if there's an object in front)
    double error = STOP_DISTANCE - distance;  // Distance we want to maintain
    double proportional = kp * error;
    kiTotal += error;
    double integral = ki * kiTotal;
    if(integral > 15) integral = 15;
    if(integral < -15) integral = -15;
    float derivative = kd * (error - previousError);
    previousError = error;
    float pidResult = proportional + integral + derivative;

    // Apply object avoidance PID result to motors
    motors.setSpeeds(-80 + pidResult, -80 - pidResult);  // Modify as needed for your avoidance behavior
  }
  else if ((currentReadPosition < 4) && (currentReadPosition > 0)) { // do wall following pid
    
    // sideways pid
    // calculate error

    double newDistance;
    if(distanceReadings[currentReadPosition] > 200) {
      newDistance = 30;
    }
    else {
      newDistance = distanceReadings[currentReadPosition];
    }
    double error = desiredState - newDistance;

    // Proportional correction 
    // no time multiplier because the rate of the servo turning is constant
    double proportional = kp * error;

    // ki get integral correction
    // add error to kiTotal
    kiTotal += error;


    if(kiTotal > 20) {
      kiTotal = 20;
    }

    if(kiTotal < -20) {
      kiTotal = -20;
    }

    // calculate integral correction
    double integral = ki * kiTotal;

    

    // may have to apply limits on Ki to prevent integral windup

    // derivative is difference between error and the priorError
    float derivative = kd * (error - previousError);
    // set previous error for next round
    previousError = error;

    // Are previousError and priorError the same thing?

    // sum P. I. and D. together
    float pidResult = proportional + integral + derivative;

    // apply the pid result to the motors
    // one wheel will be + pidResult, the other will be - pidResult
    // positive or negative pidResult value will determine which
    // direction the robot will turn
    
  
    motors.setSpeeds(-80 + pidResult, -80 - pidResult);
  }
  
}

void moveHead() {
  headCm = millis();
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD) {


    // position head to the current position in the array
    if(SERVO_ON) {
      headServo.write( HEAD_POSITIONS[currentHeadPosition] );
    }
    // the position the US sensor should read at
    currentReadPosition = currentHeadPosition;

    // check timing debug
    if(TIMING_DEBUG) {
      Serial.print("Move head initiated: ");
      Serial.println(headCm);
    }

    // head debug output
    if(HEAD_DEBUG) {
      Serial.print(currentHeadPosition);
      Serial.print(" - ");
      Serial.print(HEAD_POSITIONS[currentHeadPosition]);
    }

    /**
     * Set next head position
     * Moves servo to the next head position and goes back to beginning of array when done.
     */
    
    currentHeadPosition++;
    if (currentHeadPosition >= NUM_HEAD_POSITIONS) {
      currentHeadPosition = 0;
    }
    
    // reset previous millis
    headPm = headCm;
    // reset read flag
    usReadFlag = false;
  }
}

void usReadCm() {
  usCm = millis();
  if(usCm > headPm + WAIT_AFTER_HEAD_STARTS_MOVING && !usReadFlag) {
    // timing debug
    if(TIMING_DEBUG) {
      Serial.print("US read initiated: ");
      Serial.println(usCm);
    }

    if(US_ON) {
      // Clears the TRIG_PIN (set low)
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);

      // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds (this will shoot out the sound wave)
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      // Note: The echo pin turns high automatically after 8 sound waves are shot out
      // It stays high until it times out, or it gets the soundwave back.
      // The duration of time that it takes for us to get the bounce back is how we will calc distance.

      // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
      // note the duration (30000 microseconds) that will allow for reading up to max distance supported by the sensor
      // HIGH is an arg so that when the pin goes from LOW to HIGH, the timer begins
      // When it goes from HIGH to LOW the timer will end.
      long duration = pulseIn(ECHO_PIN, HIGH, 30000);
      // Calculating the distance
      distance = duration * 0.034 / 2; // Time of flight equation: Speed of sound wave divided by 2
      // We divide by 2 because the sound wave goes out AND comes back

      // apply limits
      if(distance > MAX_DISTANCE) distance = MAX_DISTANCE; // If distance is over max, just set it to 0.5 meters
      if(distance == 0) distance = MAX_DISTANCE;           // if sensor reads 0, it did not come back, so make it max_dist

      // assign the value to the current position in the array.
      distanceReadings[currentReadPosition] = distance;
    }

    if(TIMING_DEBUG) {
      Serial.print("US read finished: ");
      Serial.println(millis());
    }

    // Displays the distance on the Serial Monitor
    if(US_DEBUG) {
      Serial.print("Distance Readings: [ ");
      for(int i = 0; i < NUM_HEAD_POSITIONS; i++) {
        Serial.print(distanceReadings[i]);
        if(i < (NUM_HEAD_POSITIONS - 1)) Serial.print(" - ");

      }
      Serial.println(" ]");
    }
    usReadFlag = true;
  }
}
