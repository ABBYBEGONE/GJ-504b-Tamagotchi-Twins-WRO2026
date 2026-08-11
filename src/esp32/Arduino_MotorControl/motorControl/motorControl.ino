#include <AccelStepper.h>
#include <HC_SR04.h>

float serialDir, serialSpd;
int targetSpeed, steeringTarget;

int maxDrivingSpeed = 2000;

int cornersTurned = 0; //check if this is kept track of in py

float frontDistance, leftDistance, rightDistance;

int asyncTimeForUSS = 100000;

#define STEERING_STEP_PIN 32
#define STEERING_DIRECTION_PIN 33

#define DRIVING_STEP_PIN 19
#define DRIVING_DIRECTION_PIN 18

#define F_TRIG_PIN 2
#define F_ECHO_PIN 15

#define L_TRIG_PIN 13
#define L_ECHO_PIN 14

#define R_TRIG_PIN 16
#define R_ECHO_PIN 4

#define ECHO_INT 0

HC_SR04<F_ECHO_PIN> frontSensor(F_TRIG_PIN);
HC_SR04<L_ECHO_PIN> leftSensor(L_TRIG_PIN);
HC_SR04<R_ECHO_PIN> rightSensor(R_TRIG_PIN);

//Driving motor
AccelStepper drivingStepper(AccelStepper::DRIVER, 19, 18);

AccelStepper steeringStepper(AccelStepper::DRIVER, 32, 33);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  frontSensor.beginAsync();
  leftSensor.beginAsync();
  rightSensor.beginAsync();

  pinMode(F_TRIG_PIN, OUTPUT);
  pinMode(F_ECHO_PIN, INPUT);

  pinMode(L_TRIG_PIN, OUTPUT);
  pinMode(L_ECHO_PIN, INPUT);

  pinMode(R_TRIG_PIN, OUTPUT);
  pinMode(R_ECHO_PIN, INPUT);

  frontSensor.startAsync(asyncTimeForUSS);
  leftSensor.startAsync(asyncTimeForUSS);
  rightSensor.startAsync(asyncTimeForUSS);


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

  Serial.setTimeout(10);
}

void loop() {

  //Read sensor data asynchronously; allows other code to run while readings are being taken
  if (frontSensor.isFinished())
  {
    frontDistance = frontSensor.getDist_cm();
    frontSensor.startAsync(asyncTimeForUSS);
  }
  if (leftSensor.isFinished())
  {
    leftDistance = leftSensor.getDist_cm();
    leftSensor.startAsync(asyncTimeForUSS);
  }
  if (rightSensor.isFinished())
  {
    rightDistance = rightSensor.getDist_cm();
    rightSensor.startAsync(asyncTimeForUSS);
  }

  if (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('\n');
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      serialDir = cmd.substring(0, comma).toFloat();
      serialSpd = cmd.substring(comma + 1).toFloat();
    }
  }


  //send USS data via serial means
  Serial.print(frontDistance);
  Serial.print(",");
  Serial.print(leftDistance);
  Serial.print(",");
  Serial.println(rightDistance);



  steeringTarget = (int)(round(25 * serialDir));
  targetSpeed = (int)(round(maxDrivingSpeed * serialSpd));

  //continually adjust speed and wheel angle using values supplied from CV
  drivingStepper.setSpeed(targetSpeed);
  steeringStepper.moveTo(steeringTarget);

  drivingStepper.move(100); //keeps the wheels turning by continually pushing the target rotation 100 steps away

  drivingStepper.runSpeed();
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
