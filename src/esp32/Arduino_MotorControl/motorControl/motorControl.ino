#include <AccelStepper.h>//This library allows me to have more control over the motors
#include <HC_SR04.h> //This library allows for sensors to read data asynchronously

float serialDir, serialSpd;
int targetSpeed, steeringTarget;

int maxDrivingSpeed = 2000;

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

//Steering motor
AccelStepper steeringStepper(AccelStepper::DRIVER, 32, 33);

void setup() {
  Serial.begin(115200); //Turn on Serial communication with a baud rate of 115200

  //Set the sensors up for asynchronous (non-blocking) reading
  frontSensor.beginAsync();
  leftSensor.beginAsync();
  rightSensor.beginAsync();
  //

  pinMode(F_TRIG_PIN, OUTPUT);
  pinMode(F_ECHO_PIN, INPUT);

  pinMode(L_TRIG_PIN, OUTPUT);
  pinMode(L_ECHO_PIN, INPUT);

  pinMode(R_TRIG_PIN, OUTPUT);
  pinMode(R_ECHO_PIN, INPUT);

  //Sends out pulse
  frontSensor.startAsync(asyncTimeForUSS);
  leftSensor.startAsync(asyncTimeForUSS);
  rightSensor.startAsync(asyncTimeForUSS);
  //

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

  Serial.setTimeout(10); //sets how long the board should wait for each serial read to come through
  //A low time ensures that code does not get stalled for a long time because the board is waiting for serial input
}

void loop() {

  //Read sensor data asynchronously; allows other code to run while readings are being taken
  if (frontSensor.isFinished())
  {
    frontDistance = frontSensor.getDist_cm();
    frontSensor.startAsync(asyncTimeForUSS); //Sends out another pulse
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
  //

  if (Serial.available() > 0) //Checks if there is data to be read from serial communication
  {
    String cmd = Serial.readStringUntil('\n');
    int comma = cmd.indexOf(',');
    //Assigns serialDir and serialSpd their values by using substrings to gather the neccesary data 
    if (comma > 0) {
      serialDir = cmd.substring(0, comma).toFloat();
      serialSpd = cmd.substring(comma + 1).toFloat();
    }
  }


  //send USS data via serial means to Raspi board
  Serial.print(frontDistance);
  Serial.print(",");
  Serial.print(leftDistance);
  Serial.print(",");
  Serial.println(rightDistance);
  //

   //continually adjust speed and wheel angle using values supplied from computer vision's serial data
  steeringTarget = (int)(round(25 * serialDir));
  targetSpeed = (int)(round(maxDrivingSpeed * serialSpd));

  drivingStepper.setSpeed(targetSpeed);
  steeringStepper.moveTo(steeringTarget);
  //

  drivingStepper.move(100); //keeps the wheels running by continually pushing the target rotation 100 steps away

  //Make the motors move
  drivingStepper.runSpeed();
  steeringStepper.run();
  //

}

float getDistance(int trigPin, int echoPin) //Unused; blocking type method used to get distance 
{
  //Sends out a pulse 
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //

  int duration = pulseIn(echoPin, HIGH); //calculates how long it took for the pulse to return
  return duration / 58; //Convert to cm
}

void LeaveParking() //Unused; kept for reference to its logic
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
