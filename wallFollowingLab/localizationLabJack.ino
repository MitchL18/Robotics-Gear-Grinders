// Mitchell Levy
// Professor Gormanly
// Robotics Lab 1B
// 04/15/2025

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

float Sl = 0.0F; // distance traveled by left wheel
float Sr = 0.0F; // distance traveled by right wheel

// boolean variables to control each step of the program
bool forwardOneFoot = true;
bool backwardOneFoot = false;
bool forward18 = false;

double currentAngle = 0;
double currentX = 0;
double currentY = 0;

double goalAngle;
double deltaX;
double deltaY;

const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};

double kp = 20;

int currentGoalIndex = 0;

const float goalThreshold = 1.0;

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
// Mitchell Levy
// Professor Gormanly
// Robotics Lab 1B
// 04/15/2025

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

float Sl = 0.0F; // distance traveled by left wheel
float Sr = 0.0F; // distance traveled by right wheel

// boolean variables to control each step of the program
bool forwardOneFoot = true;
bool backwardOneFoot = false;
bool forward18 = false;

double currentAngle = 0;
double currentX = 0;
double currentY = 0;

double goalAngle;
double deltaX;
double deltaY;

const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};

double kp = 20;

int currentGoalIndex = 0;

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

    checkEncoders();
}

void checkEncoders()
{
    currentMillis = millis(); // tells you how long the program has been running. Starts as soon as you turn on the device.

    //
    if (currentGoalIndex == NUMBER_OF_GOALS)
    {
        // DO NOTHING WE HAVE REACHED THE END OF THE ARRAY
    }
    else
    { // get to the goal

        // if we haven't checked the encoders in 20 milliseconds, run this code (the period is 20 milliseconds)
        if (currentMillis > prevMillis + PERIOD)
        {
            countsLeft += encoders.getCountsAndResetLeft();
            countsRight += encoders.getCountsAndResetRight();

            // distance travelled in a certain number of clicks in centimeters for each wheel
            Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
            Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

            float distanceTravelled = (Sr - Sl) / 2;
            float changeInAngle = (Sr - Sl) / 8.75;
            currentAngle = currentAngle + changeInAngle;

            deltaX = distanceTravelled * cos(currentAngle + (changeInAngle / 2));
            deltaY = distanceTravelled * sin(currentAngle + (changeInAngle / 2));
            currentX = currentX + deltaX;
            currentY = currentY + deltaY;

            goalAngle = atan2(yGoals[currentGoalIndex] - currentY, xGoals[currentGoalIndex] - currentX);

            int wheelSpeed = 100;

            // Check if we hit goal
            if ((currentX == xGoals[currentGoalIndex]) && (currentY == yGoals[currentGoalIndex]))
            {
                // stop robot
                wheelSpeed = 0;
                buzzer.play("f5");
                delay(1000);
                currentGoalIndex++;
                if (currentGoalIndex == NUMBER_OF_GOALS)
                {
                  buzzer.play("e5 e5 0 e5 0 c5 e5 0 g5 0 0 g4"); // play music when done
                }
            }
            else { // calculate PID
              // calculate the error
              double error = currentAngle - goalAngle;
    
              // Proportional correction 
              // no time multiplier because it is constant
              double proportional = kp * error;
              motors.setSpeeds(-wheelSpeed + proportional , -wheelSpeed - proportional);
            }
            
        }

        Serial.print("Left: ");
        Serial.print(Sl);
        Serial.print(" Right: ");
        Serial.println(Sr);

        prevLeft = countsLeft;
        prevRight = countsRight;
        prevMillis = currentMillis;
    }
}

void loop()
{
    // put your main code here, to run repeatedly:

    checkEncoders();
}

void checkEncoders()
{
    currentMillis = millis(); // tells you how long the program has been running. Starts as soon as you turn on the device.

    //
    if (currentGoalIndex == NUMBER_OF_GOALS)
    {
        // DO NOTHING WE HAVE REACHED THE END OF THE ARRAY
    }
    else
    { // get to the goal

        // if we haven't checked the encoders in 20 milliseconds, run this code (the period is 20 milliseconds)
        if (currentMillis > prevMillis + PERIOD)
        {
            countsLeft += encoders.getCountsAndResetLeft();
            countsRight += encoders.getCountsAndResetRight();

            // distance travelled in a certain number of clicks in centimeters for each wheel
            Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
            Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

            float distanceTravelled = (Sr + Sl) / 2;
            float changeInAngle = (Sr - Sl) / 8.75;
            currentAngle = currentAngle + changeInAngle;

            deltaX = distanceTravelled * cos(currentAngle + (changeInAngle / 2));
            deltaY = distanceTravelled * sin(currentAngle + (changeInAngle / 2));
            currentX = currentX + deltaX;
            currentY = currentY + deltaY;

            goalAngle = atan2(yGoals[currentGoalIndex] - currentY, xGoals[currentGoalIndex] - currentX);

            int wheelSpeed = 100;

            // Check if we hit goal with added threshold
            if (abs(currentX - xGoals[currentGoalIndex]) < goalThreshold && 
                abs(currentY - yGoals[currentGoalIndex]) < goalThreshold)
            {
                // stop robot
                wheelSpeed = 0;
                buzzer.play("f5");
                delay(1000);
                currentGoalIndex++;
                if (currentGoalIndex == NUMBER_OF_GOALS)
                {
                  buzzer.play("e5 e5 0 e5 0 c5 e5 0 g5 0 0 g4"); // play music when done
                }
            }
            else { // calculate PID
              // calculate the error
              double error = currentAngle - goalAngle;
    
              // Proportional correction 
              // no time multiplier because it is constant
              double proportional = kp * error;
              motors.setSpeeds(-wheelSpeed + proportional , -wheelSpeed - proportional);
            }
            
        }

        Serial.print("Left: ");
        Serial.print(Sl);
        Serial.print(" Right: ");
        Serial.println(Sr);

        prevLeft = countsLeft;
        prevRight = countsRight;
        prevMillis = currentMillis;
    }

    // once all goals have been reached, stop the robot and infinite void loop()
    if (currentGoalIndex >= NUMBER_OF_GOALS) {
        motors.setSpeeds(0,0);
        while(true){
            // infinite loop
        }
    }
}
