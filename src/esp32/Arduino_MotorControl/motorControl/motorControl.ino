#include <Stepper.h>

int numOfSteps = 0;

#define STEERING_STEP_PIN 32
#define STEERING_DIRECTION_PIN 33

#define DRIVING_STEP_PIN 19
#define DRIVING_DIRECTION_PIN 18

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32!");

  pinMode(STEERING_STEP_PIN, OUTPUT);
  pinMode(STEERING_DIRECTION_PIN, OUTPUT);

  pinMode(DRIVING_STEP_PIN, OUTPUT);
  pinMode(DRIVING_DIRECTION_PIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  // this speeds up the simulation

  changeDirection();
  turnSteering();

  delay(3000);

  changeDirection();
  turnSteering();

  delay(3000);
  
  resetSteering();
  delay(3000);

}

void changeDirection()
{
  if (digitalRead(STEERING_DIRECTION_PIN) == LOW)
  {
    digitalWrite(STEERING_DIRECTION_PIN, HIGH);
  }
  else {
    digitalWrite(STEERING_DIRECTION_PIN, LOW);
  }
}

void turnSteering()
{

  if (numOfSteps == 0)
  {
    for (int i = 0; i < 90; i++)
    {
      digitalWrite(STEERING_STEP_PIN, HIGH);
      digitalWrite(STEERING_STEP_PIN, LOW);

      if (digitalRead(STEERING_DIRECTION_PIN) == HIGH)
      {
        numOfSteps++;
      }
      else
      {
        numOfSteps--;
      }

      delay(10);
    }
  }

  else
  {
    for (int i = 0; i < 180; i++)
    {
      digitalWrite(STEERING_STEP_PIN, HIGH);
      digitalWrite(STEERING_STEP_PIN, LOW);

      if (digitalRead(STEERING_DIRECTION_PIN) == HIGH)
      {
        numOfSteps++;
      }
      else
      {
        numOfSteps--;
      }

      delay(10);
    }
  }



  Serial.println(numOfSteps);

}

void resetSteering()
{
  changeDirection();
  for (int i = 0; i < 90; i++)
  {
    digitalWrite(STEERING_STEP_PIN, HIGH);
    digitalWrite(STEERING_STEP_PIN, LOW);

    if (digitalRead(STEERING_DIRECTION_PIN) == HIGH)
    {
      numOfSteps++;
    }
    else
    {
      numOfSteps--;
    }

    delay(10);
  }

}


