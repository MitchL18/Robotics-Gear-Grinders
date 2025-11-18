// Mitchell Levy, Rian Dickman, Mike Pavia, Jack Schneider
// Professor Gormanly
// Robotics
// 02/28/2025

#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;

// Prevent jitter when robot is supposed to stop
bool stopped = false;

// Initialize Ultrasonic
const int ECHO_PIN = 5;
const int TRIG_PIN = 4;

// Ultrasonic Max distance
const float MAX_DISTANCE = 20.0; // (20 cm or 0.2 meters)

// Determine the normalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 5.0; 

// Motor Constants
const float MOTOR_BASE_SPEED = 175.0;
const int MOTOR_MIN_SPEED = 40;
// determine the normalization factor based on MOTOR__BASE_SPEED
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

// Motor Compensation
const float L_MOTOR_FACTOR = 1.03; // left wheel is too slow at slow speeds
const float R_MOTOR_FACTOR = 1.0;
const float L_MOTOR_FACTOR_THRESHOLD = 80.0; // start compensating at 80 wheel speed
const float R_MOTOR_FACTOR_THRESHOLD = 80.0; 

// Ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50;  // Time to wait for 1st Ultrasonic (US) to activate

// Motor Timing
unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 20; // Time to wait between adjusting the motor speed.

// Current Ultrasonic distance reading
float distance = 0;

void setup() {
  // put your setup code here, to run once:

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  delay(1000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Update the current distance
  usReadCm();

  // Update the motor speeds
  setMotors();
}

void usReadCm() {
  usCm = millis();
  if(usCm > usPm + US_PERIOD) {

    // Clears the TRIG_PIN (set low) (we need trigger pin to be low before we can set it high)
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
    // note the duration (38000 microseconds) that will allow for reading up to max distance supported by the sensor
    // HIGH is an arg so that when the pin goes from LOW to HIGH, the timer begins
    // When it goes from HIGH to LOW the timer will end.
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Time of flight equation: Speed of sound wave divided by 2
    // We divide by 2 because the sound wave goes out AND comes back

    // apply limits
    if(distance > MAX_DISTANCE) distance = MAX_DISTANCE; // If distance is over max, just set it to 0.5 meters
    if(distance == 0) distance = MAX_DISTANCE;           // if sensor reads 0, it did not come back, so make it max_dist

    // Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // update the prevmillis
    usPm = usCm;
  }
}

void setMotors() {
  motorCm = millis();
  if(motorCm > motorPm + MOTOR_PERIOD) {

    // start out with the motor base speed
    float leftSpeed = MOTOR_BASE_SPEED;
    float rightSpeed = MOTOR_BASE_SPEED;

    // check if most current distance measurement is less than or equal to MAX_DISTANCE
    if(distance <= MAX_DISTANCE) {

      // determine the magnitude of the distance by taking the difference (short distance = high magnitude)
      // divide by the DISTANCE_FACTOR to ensure uniform response as MAX_DISTANCE changes
      // This maps the distance range (1 - MAX_RANGE) to 0-100 for the magnitude
      float magnitude = (float)(MAX_DISTANCE - distance) / DISTANCE_FACTOR;
      // example 1: MAX_DISTANCE = 80, distance = 40: 80-40= 40/0.8 = 50 (mid range!)
      // example 2: MAX_DISTANCE = 160, distance = 40: 160-40= 120/1.6 = 75 (top 1/4)

      // Multiply the magnitude by the MOTOR_FACTOR to map the magnitude range (0-100) of the motors
      // (0 - MOTOR_BASE_SPEED)
      leftSpeed = (MOTOR_BASE_SPEED * 0.9887) - (magnitude * MOTOR_FACTOR);  // This looks ridiculous, but it makes my robot not drift when going fast!
      rightSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);

    }

    // lower limit check
    if(leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
    if(rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

    // add in motor compensation
    if(leftSpeed <= L_MOTOR_FACTOR_THRESHOLD) {
      leftSpeed *= L_MOTOR_FACTOR;
    }
    if(rightSpeed <= R_MOTOR_FACTOR_THRESHOLD) {
      rightSpeed *= R_MOTOR_FACTOR;
    }

    // check stop distance
    if(distance <= STOP_DISTANCE) {
      leftSpeed = 0;
      rightSpeed = 0;
      stopped = true; // we are now stopped
    }

    // if we are far enough away from an object, we need to follow it
    if(distance >= 6.5) { 
      stopped = false;
    }
    
    // if we are still too close to the object, keep the speeds at 0
    // This code and the few lines above effectively stop the robot from jittering when it should smoothly stop if it's going to hit a wall.
    if(stopped == true) {
      leftSpeed = 0;
      rightSpeed = 0;
    }

    Serial.print("Left: ");
    Serial.print(leftSpeed);
    Serial.print(" Right: ");
    Serial.println(rightSpeed);

    // Use negative (-) because our robot is going backwards in reality.
    motors.setSpeeds(-leftSpeed, -rightSpeed);

    motorPm = motorCm;
  }
}







