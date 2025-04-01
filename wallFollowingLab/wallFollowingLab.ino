// desired position
const double desiredState = (double) 30;

const double kp = 1;
const double ki = 0;
const double kd = 0;

double kiTotal = 0.0;
double priorError = 0.0;
long prevTime = new Date().getTime();

void setup() {
  // put your setup code here, to run once:

  // Fetch ultrasonic readings (use code from last lab)

  // calculate error
  double error = desiredState - currentReading;

  // Proportional correction 
  // no time multiplier because the rate of the servo turning is constant
  double proportional = kp * error;

  // ki get integral correction
  // add error to kiTotal
  kiTotal += error;

  // calculate integral correction
  double integral = ki * kiTotal

  // may have to apply limits on Ki to prevent integral windup

  
}

void loop() {
  // put your main code here, to run repeatedly:

}
