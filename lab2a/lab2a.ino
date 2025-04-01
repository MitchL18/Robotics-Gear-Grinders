#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>

using namespace Pololu3piPlus32U4;


Buzzer buzzer;
Motors motors;

const int ECHO_PIN = 5;
const int TRIG_PIN = 4;

// max distance 
const int MAX_DISTANCE = 150;


//timing 
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long period = 100;
unsigned long minDistance = 30;
unsigned long moveBackDistance = 25;

//current period reading 
int distance = 0;





void setup() {
  // put your setup code here, to run once:
pinMode(ECHO_PIN, INPUT);
pinMode(TRIG_PIN, OUTPUT);
delay(1000);
buzzer.play("c32");

}

void loop() {
  // put your main code here, to run repeatedly:
  readCM();
}






void readCM() {
  currentMillis = millis();
  if(currentMillis > prevMillis + period) {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    //Read echo pin return wave travel time in microseconds 
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    // calculate distance
    distance = duration * 0.034 / 2; // speed pf sound divided by 2 

    // limits 
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = MAX_DISTANCE;

    // displays distance on serial

    Serial.print("distance:  ");
    Serial.print(distance);
    Serial.println(" cm");

    //update prevMillis
    prevMillis = currentMillis;

    int foward = -50;
    int backward = 50;
    if (distance >= minDistance) {
      foward = -50 * ((minDistance + distance) / 10);
      motors.setSpeeds(foward, foward);
      
    }else if(distance < minDistance - 5)  {
      backward = 50 * ((minDistance - distance) / 10);
      motors.setSpeeds(backward, backward);
    }else {
       motors.setSpeeds(0,0);
    }

  }

  



}
