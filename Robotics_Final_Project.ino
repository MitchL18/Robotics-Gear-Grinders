// Jack Schneider, Mike Pavia, Rian Dickman, Mitchell Levy
// Professor Gormanly
// Robotics Final Project!
// 05/12/2025

#include <Pololu3piPlus32U4.h>
#include <math.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

// Localization Constants

unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20; // minimum amount of time that needs to pass before we check encoders.

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

const int CLICKS_PER_ROTATION = 12; // rotation of dc motor and encoder shaft
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.0531;
const double pi = 3.14159;

float Sl = 0.0F; // distance traveled by left wheel
float Sr = 0.0F; // distance traveled by right wheel

double currentAngle = 0;
double currentX = 0;
double currentY = 0;

double goalAngle;
double deltaX;
double deltaY;

const int NUMBER_OF_GOALS = 1;
float xGoals[NUMBER_OF_GOALS] = {494};
float yGoals[NUMBER_OF_GOALS] = {-130};

double localkp = 30;

int currentGoalIndex = 0;

const float goalThreshold = 2.0;

// Servo Constants

Servo headServo; // create servo object to control a servo

// switches
const boolean HEAD_DEBUG = false;
const boolean TIMING_DEBUG = false;
const boolean US_DEBUG = false;

const boolean SERVO_ON = true;
const boolean US_ON = true;

// Head Servo Timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 120;

// head servo constants
const int HEAD_SERVO_PIN = 12;
const int NUM_HEAD_POSITIONS = 5;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {163, 133, 93, 63, 33};

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
int currentReadPosition = 2;

float distanceReadings[NUM_HEAD_POSITIONS];

// PID Constants and Variables:

float frontDistance;
float leftDistance;
float rightDistance;
float farLeftDistance;
float farRightDistance;

float kpAvoidSide = 2.2;
float kpAvoidFront = 3.0;

float errorAvoidFarLeft = 0;
float errorAvoidLeft = 0;
float errorAvoidFarRight = 0;
float errorAvoidRight = 0;
float errorAvoidFront = 0;

float desiredAvoidState = 35;

float pidResultAvoid;

double proportionalFrontAvoid;
double proportionalLeftAvoid;
double proportionalRightAvoid;

void setup()
{
    // put your setup code here, to run once:

    // set baud rate
    Serial.begin(57600);

    // initialize the head position to start
    headServo.attach(HEAD_SERVO_PIN);
    headServo.write(93); // start at position 90

    // initialize the US pins
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    // initialize distance readings
    for (int i = 0; i < NUM_HEAD_POSITIONS; i++)
    {
        distanceReadings[i] = MAX_DISTANCE;
    }

    // make robot not move or do anything for one second after you power it up
    delay(3000);

    // play note right before robot goes
    buzzer.play("c32");
}

void loop()
{
    // put your main code here, to run repeatedly:

    // Basic encoder reading
    checkEncoders();

    // perfrom head movement
    moveHead();

    // update the current distance
    usReadCm();
}

void checkEncoders()
{
    currentMillis = millis(); // tells you how long the program has been running. Starts as soon as you turn on the device.

    if (currentGoalIndex == NUMBER_OF_GOALS)
    {
        // DO NOTHING WE HAVE REACHED THE END OF THE ARRAY
    }
    else
    { // get to the goal

        // if we haven't checked the encoders in 20 milliseconds, run this code (the period is 20 milliseconds)
        if (currentMillis > prevMillis + PERIOD)
        {
            countsLeft = encoders.getCountsAndResetLeft();
            countsRight = encoders.getCountsAndResetRight();

            // Convert directly to distance increments
            float deltaSl = countsLeft / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;
            float deltaSr = countsRight / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;

            Sl += deltaSl; // calculate total wheel distance
            Sr += deltaSr;

            float distanceTravelled = (deltaSl + deltaSr) / 2;
            float changeInAngle = (deltaSl - deltaSr) / 8.5; // switched order because our robot is going backwards

            currentAngle = currentAngle + changeInAngle;
            double currentAnglePos = currentAngle * -1; // we are going backwards

            deltaX = distanceTravelled * cos(currentAngle + (changeInAngle / 2));
            deltaY = distanceTravelled * sin(currentAngle + (changeInAngle / 2));
            currentX = currentX + deltaX;
            currentY = currentY + deltaY;

            float posCurrentX = currentX * -1;

            goalAngle = atan2(yGoals[currentGoalIndex] - currentY, xGoals[currentGoalIndex] - posCurrentX);

            int wheelSpeed = -200;

            float distanceToGoal = sqrt(pow(posCurrentX - xGoals[currentGoalIndex], 2) + pow(currentY - yGoals[currentGoalIndex], 2));

            // calculate the base speed dynamically
            int baseSpeed = calculateBaseSpeed(distanceToGoal, -200, -55, 12.0);
            wheelSpeed = baseSpeed;

            if (distanceToGoal < goalThreshold)
            {
                // stop robot
                wheelSpeed = 0;
                motors.setSpeeds(wheelSpeed, wheelSpeed);
                buzzer.play("f5");
                delay(1000);
                currentGoalIndex++;
                if (currentGoalIndex == NUMBER_OF_GOALS)
                {
                    buzzer.play("!T120 L8 e e r e r c e r g2 r g2 r g g a g c b"); // play music when done
                }
            }
            else
            {

                // check if we have a new reading
                if (usReadFlag)
                {

                    // calculate PID
                    // calculate the error
                    double angleError = atan2(sin(goalAngle - currentAnglePos), cos(goalAngle - currentAnglePos));
                    // Proportional correction
                    // no time multiplier because it is constant
                    double proportionalPull = localkp * angleError;
                    // Serial.print("current anglePos: ");
                    // Serial.print(currentAnglePos);
                    // Serial.print(",   goal angle: ");
                    // Serial.print(goalAngle);
                    // Serial.print(",   angle error: ");
                    // Serial.println(angleError);

                    int leftSpeed = wheelSpeed - proportionalPull;
                    int rightSpeed = wheelSpeed + proportionalPull;

                    usReadFlag = false; // we need to reset flag

                    // Get the current distances from the readings
                    farLeftDistance = distanceReadings[0];  // Side distance (index 0)
                    leftDistance = distanceReadings[1];     // Side distance (index 1)
                    frontDistance = distanceReadings[2];    // Front distance (index 2)
                    rightDistance = distanceReadings[3];    // Side distance (index 3)
                    farRightDistance = distanceReadings[4]; // Side distance (index 4)

                    // if walls are very far away
                    if (frontDistance > desiredAvoidState && leftDistance > desiredAvoidState && farLeftDistance > desiredAvoidState && rightDistance > desiredAvoidState && farRightDistance > desiredAvoidState)
                    {
                        // don't need wall avoidance PID
                        proportionalFrontAvoid = 0;
                        proportionalLeftAvoid = 0;
                        proportionalRightAvoid = 0;
                    }

                    // there is an obstacle
                    else
                    { // do PID for object avoidance

                        // Calculate errors for all sensors
                        errorAvoidFarLeft = calculateError(farLeftDistance, desiredAvoidState);
                        errorAvoidLeft = calculateError(leftDistance, desiredAvoidState);
                        errorAvoidFarRight = calculateError(farRightDistance, desiredAvoidState);
                        errorAvoidRight = calculateError(rightDistance, desiredAvoidState);
                        errorAvoidFront = calculateError(frontDistance, desiredAvoidState);

                        // Proportional correction
                        // no time multiplier because the rate of the servo turning is constant
                        double proportionalLeftAvoid = kpAvoidSide * (errorAvoidFarLeft + errorAvoidLeft);
                        double proportionalRightAvoid = kpAvoidSide * (errorAvoidFarRight + errorAvoidRight);

                        // Scale front force smoothly based on angle error
                        double angleScale = cos(angleError); // Creates a smooth transition between -1 and 1
                        proportionalFrontAvoid = kpAvoidFront * errorAvoidFront * angleScale;

                        // DEFAULT the robot's front force will be positive and turn to the right
                        // When the angleError is (-) the robot wants to turn to the right to get to the goal
                        // When the angleError is (+) the robot wants to turn to the left to get to the goal

                        // sum together to get repulsive force
                        pidResultAvoid = proportionalLeftAvoid - proportionalRightAvoid + proportionalFrontAvoid;

                        // add pulling force
                        leftSpeed = wheelSpeed - proportionalPull - pidResultAvoid;
                        rightSpeed = wheelSpeed + proportionalPull + pidResultAvoid;
                    }
                    motors.setSpeeds(leftSpeed, rightSpeed);
                }
            }
            prevLeft = countsLeft;
            prevRight = countsRight;
            prevMillis = currentMillis;
        }

        // Serial.print("Left: ");
        // Serial.print(Sl);
        // Serial.print(" Right: ");
        // Serial.println(Sr);
    }

    // once all goals have been reached, stop the robot and infinite void loop()
    if (currentGoalIndex >= NUMBER_OF_GOALS)
    {
        motors.setSpeeds(0, 0);
        while (true)
        {
            // infinite loop
        }
    }
}

// Servo Functions

void moveHead()
{
    headCm = millis();
    if (headCm > headPm + HEAD_MOVEMENT_PERIOD)
    {

        // position head to the current position in the array
        if (SERVO_ON)
        {
            headServo.write(HEAD_POSITIONS[currentHeadPosition]);
        }
        // the position the US sensor should read at
        currentReadPosition = currentHeadPosition;

        // check timing debug
        if (TIMING_DEBUG)
        {
            Serial.print("Move head initiated: ");
            Serial.println(headCm);
        }

        // head debug output
        if (HEAD_DEBUG)
        {
            Serial.print(currentHeadPosition);
            Serial.print(" - ");
            Serial.print(HEAD_POSITIONS[currentHeadPosition]);
        }

        /**
         * Set next head position
         * Moves servo to the next head position and changes direction when needed.
         */
        if (headDirectionClockwise)
        {
            if (currentHeadPosition >= (NUM_HEAD_POSITIONS - 1))
            {
                headDirectionClockwise = !headDirectionClockwise;
                currentHeadPosition--;
            }
            else
            {
                currentHeadPosition++;
            }
        }
        else
        {
            if (currentHeadPosition <= 0)
            {
                headDirectionClockwise = !headDirectionClockwise;
                currentHeadPosition++;
            }
            else
            {
                currentHeadPosition--;
            }
        }

        // reset previous millis
        headPm = headCm;
        // reset read flag
        usReadFlag = false;
    }
}

void usReadCm()
{
    usCm = millis();
    if (usCm > headPm + WAIT_AFTER_HEAD_STARTS_MOVING && !usReadFlag)
    {
        // timing debug
        if (TIMING_DEBUG)
        {
            Serial.print("US read initiated: ");
            Serial.println(usCm);
        }

        if (US_ON)
        {
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
            if (distance > MAX_DISTANCE)
                distance = MAX_DISTANCE; // If distance is over max, just set it to 0.5 meters
            if (distance == 0)
                distance = MAX_DISTANCE; // if sensor reads 0, it did not come back, so make it max_dist

            // assign the value to the current position in the array.
            distanceReadings[currentReadPosition] = distance;
        }

        if (TIMING_DEBUG)
        {
            Serial.print("US read finished: ");
            Serial.println(millis());
        }

        // Displays the distance on the Serial Monitor
        if (US_DEBUG)
        {
            Serial.print("Distance Readings: [ ");
            for (int i = 0; i < NUM_HEAD_POSITIONS; i++)
            {
                Serial.print(distanceReadings[i]);
                if (i < (NUM_HEAD_POSITIONS - 1))
                    Serial.print(" - ");
            }
            Serial.println(" ]");
        }
        usReadFlag = true;
    }
}

// Helper Functions

// Function to calculate the distance to the goal
float calculateError(float distance, float desiredState)
{
    if (distance < desiredState)
    {
        return distance - desiredState;
    }
    return 0; // no repulsive force needed if no object detected
}

// Function to calculate the base speed dynamically
int calculateBaseSpeed(float distanceToGoal, int maxSpeed, int minSpeed, float decelerationThreshold)
{
    if (distanceToGoal < decelerationThreshold)
    {
        // Smoothly decrease speed as the robot gets closer to the goal
        int adjustedSpeed = maxSpeed * (distanceToGoal / decelerationThreshold);
        if (adjustedSpeed > minSpeed)
        {
            return minSpeed; // Ensure speed doesn't go below minSpeed
        }
        else
        {
            return adjustedSpeed;
        }
    }
    return maxSpeed; // Maintain max speed if distance is greater than the threshold
}