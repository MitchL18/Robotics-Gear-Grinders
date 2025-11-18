// Jack Schneider, Mike Pavia, Rian Dickman, Mitchell Levy
// Professor Gormanly
// Robotics Lab 1B
// 04/21/2025

#include <Pololu3piPlus32U4.h>
#include <math.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

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

const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};

double kp = 30;

int currentGoalIndex = 0;

const float goalThreshold = 2.0;

void setup()
{
    // put your setup code here, to run once:

    // set baud rate
    Serial.begin(57600);

    // make robot not move or do anything for one second after you power it up
    delay(1000);

    // play note right before robot goes
    buzzer.play("c32");
}

void loop()
{
    // put your main code here, to run repeatedly:
    // Basic encoder reading

    checkEncoders();
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
            
            int wheelSpeed = 75;

            float distanceToGoal = sqrt(pow(posCurrentX - xGoals[currentGoalIndex], 2) +
                                        pow(currentY - yGoals[currentGoalIndex], 2));

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
            { // calculate PID
                // calculate the error
                double angleError = atan2(sin(goalAngle - currentAnglePos), cos(goalAngle - currentAnglePos));
                // Proportional correction
                // no time multiplier because it is constant
                double proportional = kp * angleError;
                Serial.print("current anglePos: ");
                Serial.print(currentAnglePos);
                Serial.print(",   goal angle: ");
                Serial.print(goalAngle);
                Serial.print(",   angle error: ");
                Serial.println(angleError);
                motors.setSpeeds(-wheelSpeed - proportional, -wheelSpeed + proportional);
                
            }
            prevLeft = countsLeft;
            prevRight = countsRight;
            prevMillis = currentMillis;
        }

        Serial.print("Left: ");
        Serial.print(Sl);
        Serial.print(" Right: ");
        Serial.println(Sr);
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
