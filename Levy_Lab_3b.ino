// Mitchell Levy
// Professor Gormanly
// Robotics Lab 3a
// 03/13/2025

#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;

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
const int NUM_HEAD_POSITIONS = 5;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {150, 120, 90, 60, 30};

// head servo data
boolean headDirectionClockwise = false;
int currentHeadPosition = 1;

// Initialize Ultrasonic
const int ECHO_PIN = 5;
const int TRIG_PIN = 4;

// Ultrasonic max distance
const float MAX_DISTANCE = 300.0;

// determine the normalization factorbased on MAX_DISTANCE
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

void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);

  // initialize the head position to start
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90); // start at position 40

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
}

void loop() {
  // put your main code here, to run repeatedly:

  // perfrom head movement
  moveHead();

  // update the current distance
  usReadCm();

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
     * Moves servo to the next head position and changes direction when needed.
     */
    if(headDirectionClockwise) {
      if(currentHeadPosition >= (NUM_HEAD_POSITIONS -1)) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      }
      else {
        currentHeadPosition++;
      }
    } 
    else {
      if(currentHeadPosition <= 0) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      }
      else {
        currentHeadPosition--;
      }
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
      float distance = duration * 0.034 / 2; // Time of flight equation: Speed of sound wave divided by 2
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

