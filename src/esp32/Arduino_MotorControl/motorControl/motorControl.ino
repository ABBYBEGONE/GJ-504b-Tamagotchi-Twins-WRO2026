#include <AccelStepper.h>

float serialDir, serialSpd;
int targetSpeed, steeringTarget;

int maxDrivingSpeed = 2000;

int cornersTurned = 0; //check if this is kept track of in py

float frontDistance, leftDistance, rightDistance;

#define STEERING_STEP_PIN 32
#define STEERING_DIRECTION_PIN 33

#define DRIVING_STEP_PIN 19
#define DRIVING_DIRECTION_PIN 18

#define F_TRIG_PIN 2
#define F_ECHO_PIN 0

#define L_TRIG_PIN 13
#define L_ECHO_PIN 14

#define R_TRIG_PIN 16
#define R_ECHO_PIN 4

//Driving motor
AccelStepper drivingStepper(AccelStepper::DRIVER, 19, 18);

AccelStepper steeringStepper(AccelStepper::DRIVER, 32, 33);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(F_TRIG_PIN, OUTPUT);
  pinMode(F_ECHO_PIN, INPUT);

  pinMode(L_TRIG_PIN, OUTPUT);
  pinMode(L_ECHO_PIN, INPUT);

  pinMode(R_TRIG_PIN, OUTPUT);
  pinMode(R_ECHO_PIN, INPUT);

  drivingStepper.setMaxSpeed(maxDrivingSpeed);
  // Set the speed in steps per second:
  drivingStepper.setSpeed(maxDrivingSpeed);
  // Step the motor with a constant speed as set by setSpeed():
  drivingStepper.setAcceleration(250);
  drivingStepper.move(100);

  steeringStepper.setMaxSpeed(1000);
  // Set the speed in steps per second:
  steeringStepper.setSpeed(1000);
  // Step the motor with a constant speed as set by setSpeed():
  steeringStepper.setAcceleration(100);


}

void loop() {

  frontDistance = getDistance(F_TRIG_PIN, F_ECHO_PIN);
  leftDistance = getDistance(L_TRIG_PIN, L_ECHO_PIN);
  rightDistance = getDistance(R_TRIG_PIN, R_ECHO_PIN);

  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      serialDir = cmd.substring(0, comma).toFloat();
      serialSpd = cmd.substring(comma + 1).toFloat();
    }


    Serial.print(frontDistance);
    Serial.print(",");
    Serial.print(leftDistance);
    Serial.print(",");
    Serial.println(rightDistance);
    //format of serial write (f_dist,l_dist,r_dist)
  }

  steeringTarget = (int)(round(25 * serialDir));
  targetSpeed = (int)(round(maxDrivingSpeed * serialSpd));

  //continually adjust speed and wheel angle using values supplied from CV
  drivingStepper.setSpeed(targetSpeed);
  steeringStepper.moveTo(steeringTarget);

  drivingStepper.move(100); //keeps the wheels turning by continually pushing the target rotation 100 steps away

  drivingStepper.run();
  steeringStepper.run();


}

float getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int duration = pulseIn(echoPin, HIGH);
  return duration / 58;
}

void LeaveParking()
{
  Serial.println("trying to leave parking");
  if (leftDistance <= 5)
  {
    steeringStepper.moveTo(25);
    steeringStepper.runToPosition();

    drivingStepper.moveTo(400);
    drivingStepper.runToPosition();

    steeringStepper.moveTo(0);
    steeringStepper.runToPosition();

    drivingStepper.moveTo(800);
    drivingStepper.runToPosition();
  }
  else if (rightDistance <= 5)
  {
    steeringStepper.moveTo(-25);
    steeringStepper.runToPosition();

    drivingStepper.moveTo(400);
    drivingStepper.runToPosition();

    steeringStepper.moveTo(0);
    steeringStepper.runToPosition();

    drivingStepper.moveTo(800);
    drivingStepper.runToPosition();
  }
}
