#include <AccelStepper.h>

int numOfSteps = 0;
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

  drivingStepper.setMaxSpeed(1000);
  // Set the speed in steps per second:
  drivingStepper.setSpeed(1000);
  // Step the motor with a constant speed as set by setSpeed():
  drivingStepper.setAcceleration(100);
  drivingStepper.move(100);

  steeringStepper.setMaxSpeed(1000);
  // Set the speed in steps per second:
  steeringStepper.setSpeed(1000);
  // Step the motor with a constant speed as set by setSpeed():
  steeringStepper.setAcceleration(100);

}

void loop() {
  //Leaving Parking Zone
  if(getDistance(F_TRIG_PIN, F_ECHO_PIN) <= 5 && (getDistance(L_TRIG_PIN, L_ECHO_PIN) <= 5 || getDistance(R_TRIG_PIN, R_ECHO_PIN) <= 5)) //&&magenta
  {
    Serial.println("trying to leave parking");
  }

  //Driving Straight
  if(getDistance(F_TRIG_PIN, F_ECHO_PIN) >= 30 && (getDistance(L_TRIG_PIN, L_ECHO_PIN) <= 30 || getDistance(R_TRIG_PIN, R_ECHO_PIN) <= 30)) 
  {
    Serial.println("Driving straight");
  }

  //Turning Corner
  if(getDistance(F_TRIG_PIN, F_ECHO_PIN) < 30 && (getDistance(L_TRIG_PIN, L_ECHO_PIN) >= 30 || getDistance(R_TRIG_PIN, R_ECHO_PIN) >= 30)) //&&colour = blue || orange
  {
    Serial.println("Turning corner");
  }

  //Passing t.lights
  if(getDistance(F_TRIG_PIN, F_ECHO_PIN) < 10) //&&colour = "red" || "green"
  {
    Serial.println("Passing t.light");
  }
}

float getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int duration = pulseIn(echoPin, HIGH);
  return duration/58;
}
