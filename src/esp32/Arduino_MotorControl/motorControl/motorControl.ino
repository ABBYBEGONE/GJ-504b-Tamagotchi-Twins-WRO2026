#include <AccelStepper.h>

int numOfSteps = 0;
String Placeholder = "Stuff: ";
String data;
float serialDir, serialSpd;

bool isTurningCorner = false; 
bool firstInstanceTurning = true;

int maxDrivingSpeed = 2000;

int cornersTurned = 0;

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
  Serial.println("Hello, ESP32!");


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
  if (Serial.available() > 0)
  {
    serialDir = Serial.parseFloat();
    serialSpd =  Serial.parseFloat();
  }

  data = Placeholder + serialDir + " " + serialSpd;
  Serial.println(data);

  drivingStepper.setSpeed(maxDrivingSpeed * serialSpd);

  drivingStepper.run();
  steeringStepper.run();


  //Driving Straight
  if(getDistance(F_TRIG_PIN, F_ECHO_PIN) >= 10 && serialDir < 0.05f && serialDir > -0.05f)
  { //might remove get distance from this conditional
    DriveStraight();
  }

  //TurningCorner
  else if (isTurningCorner || (serialDir >= 0.9f && getDistance(R_TRIG_PIN, R_ECHO_PIN) > 30) || (serialDir <= -0.9f && getDistance(L_TRIG_PIN, L_ECHO_PIN) > 30) )
  {
    TurnCorner();
  }

  //Passing t.lights
  else if (serialDir != 0)
  {
    SteeringAroundTrafficLight();
  }

    //Leaving Parking Zone
  else if (cornersTurned = 0 && getDistance(F_TRIG_PIN, F_ECHO_PIN) <= 5 && (getDistance(L_TRIG_PIN, L_ECHO_PIN) <= 5 || getDistance(R_TRIG_PIN, R_ECHO_PIN) <= 5)) //&&magenta
  {
    LeaveParking();
  }
}

float getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int duration = pulseIn(echoPin, HIGH);
  return duration / 58;
}

void DriveStraight()
{
  Serial.println("Driving straight");
  steeringStepper.moveTo(0);
}

void TurnCorner()
{
  if(firstInstanceTurning)
  {
    isTurningCorner = true;
    firstInstanceTurning = false;
  }

  else if(!firstInstanceTurning)//some condition that must be met in regards to how many wheel steps it took while steering
  {
    isTurningCorner = false;
    firstInstanceTurning = true;
    cornersTurned++;
  }

  Serial.println("Turning corner");
  steeringStepper.moveTo(25 * serialDir); //25 steps represents 45 degrees; multiplying by serialDir (which is -1.o to 1.0), determines how much of 45 degrees the wheels must turn
}

void SteeringAroundTrafficLight()
{
  Serial.println("Passing t.light");
  steeringStepper.moveTo(25 * serialDir);
}

void LeaveParking()
{
  Serial.println("trying to leave parking");
  if(getDistance(L_TRIG_PIN, L_ECHO_PIN) <= 5)
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
  else if(getDistance(R_TRIG_PIN, R_ECHO_PIN) <= 5)
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
