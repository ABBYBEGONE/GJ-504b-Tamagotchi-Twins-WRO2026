#include <ESP32Servo.h>
#include <HC_SR04.h> //This library allows for sensors to read data asynchronously

//Receive via serial
String serialChallengeMode = "";
float serialDir, serialSpd;
String serialBotState;
enum CommandIndex { LEAVING_PARKING, STRAIGHT, OBSTACLE, REVERSE, CORNER, PARKING, CMD_UNKNOWN};
const String commands[] = { "LEAVING_PARKING", "STRAIGHT", "OBSTACLE", "REVERSE", "CORNER", "PARKING"};

const int numCommands = sizeof(commands) / sizeof(commands[0]);

// Function to map string to enum
CommandIndex getCommandIndex(String input) {
  for (int i = 0; i < numCommands; i++) {
    if (input == commands[i]) {
      return static_cast<CommandIndex>(i);
    }
  }
  return CMD_UNKNOWN;
}
//

CommandIndex botState; // handled lower in loop(); used in the switch-case Statements

//edit this every round
int carDirection = 1;
// 1: car is driving counter clockwise
//-1: car is driving clockwise

//For open challenge only
bool frontDistanceChecked = false;
int initialFrontDistance;
//



//changes as round progresses

bool hasStarted = false; //or free to drive, consult tawana's serial
float frontDistance, leftDistance, rightDistance;
int currentAngle;
int targetAngle;
int numberOfCornersTurned = 0;
//

int asyncTimeForUSS = 100000;

//Motor A: DC Motor; Motor B: servo motor
#define Motor_A 34
#define DRIVING_1A_PIN 19
#define DRIVING_1B_PIN 21

#define SERVO_SIG_PIN 18
//#define B_1A_PIN 25
//#define B_1B_PIN 26

#define UNI_TRIG_PIN 2 //all USSs use same TRIG PIN
#define F_ECHO_PIN 15
#define L_ECHO_PIN 14
#define R_ECHO_PIN 4

Servo servoMotor;

HC_SR04_BASE *SideSensors[] = { new HC_SR04<L_ECHO_PIN>(), new HC_SR04<R_ECHO_PIN>()};
HC_SR04<F_ECHO_PIN> sonicMaster(UNI_TRIG_PIN, SideSensors, 2); //The master sensor is the Front USS



void setup() {
  Serial.begin(115200); //Turn on Serial communication with a baud rate of 115200

  //Set the sensors up for asynchronous (non-blocking) reading
  sonicMaster.beginAsync();
  //

  servoMotor.attach(SERVO_SIG_PIN);

  pinMode(UNI_TRIG_PIN, OUTPUT);
  pinMode(F_ECHO_PIN, INPUT);
  pinMode(L_ECHO_PIN, INPUT);
  pinMode(R_ECHO_PIN, INPUT);

  //Sends out pulse
  sonicMaster.startAsync(asyncTimeForUSS);
  // Only startAsync one of these bcuz the sensors use the same TRIGGER PIN therefore they pulse at the same time

  currentAngle = 0;
  targetAngle = 90;
  turnToTargetAngle(targetAngle); //initialize servo motor to look straight

  Serial.setTimeout(10); //sets how long the board should wait for each serial read to come through
  //A low time ensures that code does not get stalled for a long time because the board is waiting for serial input
}

void loop() {



  //Read sensor data asynchronously; allows other code to run while readings are being taken
  if (sonicMaster.isFinished())
  {
    frontDistance = sonicMaster.getDist_cm(0);
    leftDistance = sonicMaster.getDist_cm(1);
    rightDistance = sonicMaster.getDist_cm(2);

    sonicMaster.startAsync(asyncTimeForUSS);
  }

  //send USS data via serial means to Raspi board
  //Serial.print(frontDistance);
  //Serial.print(",");
  //Serial.print(leftDistance);
  //Serial.print(",");
  //Serial.println(rightDistance);
  //

  if (Serial.available() > 0) //Checks if there is data to be read from serial communication
  {
    if (serialChallengeMode != "") //checks if the serial challenge mode is not empty (the challenge has been set)
    {
      String cmd = Serial.readStringUntil('\n');
      int comma1 = cmd.indexOf(',');
      int comma2 = cmd.indexOf(',', comma1 + 1);
      int comma3 = cmd.indexOf(',', comma2 + 1);

      //Assigns serialDir, serialSpd, and serialBotState their values by using substrings to gather the necessary data
      if (comma1 > 0 && comma2 > 0) {

        serialDir = cmd.substring(0, comma1).toFloat();
        serialSpd = cmd.substring(comma1 + 1, comma2).toFloat();
        //Only take the 3rd value; if a 4th comma exists (i.e. more values follow), stop there, otherwise take the rest of the string
        serialBotState = (comma3 > 0) ? cmd.substring(comma2 + 1, comma3) : cmd.substring(comma2 + 1);

        targetAngle = 90 + (int)(round(45 * serialDir));
        botState = getCommandIndex(serialBotState);

        Serial.print(targetAngle);
        Serial.print(",");
        Serial.println(serialSpd);
      }
    }
    else{
      
      serialChallengeMode = Serial.readStringUntil('\n'); //read and set the challenge mode
      if(!(serialChallengeMode == "OPEN" || serialChallengeMode=="OBSTACLE")  )
      {
        serialChallengeMode = "";
      }
    }

  }

  turnToTargetAngle(targetAngle);
  setDrivingMotorSpeed((int)(round(255 * serialSpd)), 1);


  //skeleton for Round logic

  if (serialChallengeMode == "OPEN")
  {
    if (!frontDistanceChecked) //Check if this is really only called once
    { //call this in setup() rather, regardless of challenge mode
      initialFrontDistance = frontDistance;
      frontDistanceChecked = true;
    }

    switch (botState) //State switching with a switch statement XD
    {

      case STRAIGHT:
        turnToTargetAngle(targetAngle); //just drive straight or at whatever serial angle is provided
        setDrivingMotorSpeed((int)(round(255 * serialSpd)), 1);
        break;

      case REVERSE: //might not be needed for open
        turnToTargetAngle(90); // drive straight backwards
        setDrivingMotorSpeed((int)(round(255 * serialSpd)), -1); //swap direction to reverse
        break;

      case CORNER:
        turnToTargetAngle(targetAngle);
        driveForXMilliseconds(500, (int)(round(255 * serialSpd)), 1);
        //keep driving and stopping to make sure bot doesn't overshoot
        //review this against python logic
        break;

      case PARKING:
        if (frontDistance <= initialFrontDistance)
        {
          setDrivingMotorSpeed(0, 1);
        }
        else
        {
          turnToTargetAngle(90);
          setDrivingMotorSpeed((int)(round(255 * serialSpd)), 1);
        }
        break;

      default:
        setDrivingMotorSpeed(0, 1);
    }
  }

  ////////////
  else if (serialChallengeMode == "OBSTACLE")
  {
    switch (botState) //State switching with a switch statement XD
    {
      case LEAVING_PARKING:
        ExitParking();
        break;

      case STRAIGHT: OBSTACLE:
        turnToTargetAngle(targetAngle);
        setDrivingMotorSpeed((int)(round(255 * serialSpd)), 1);

        break;

      case REVERSE:
        turnToTargetAngle(90); // drive straight backwards
        setDrivingMotorSpeed((int)(round(255 * serialSpd)), -1); //swap direction to reverse
        break;

      case CORNER:
        turnToTargetAngle(targetAngle);
        driveForXMilliseconds(500, (int)(round(255 * serialSpd)), 1);
      //keep driving and stopping to make sure bot doesn't overshoot
      //review this against python logic

      case PARKING:
        turnToTargetAngle(targetAngle);
        driveForXMilliseconds(250, (int)(round(255 * serialSpd)), 1);
        //keep driving and stopping to make sure bot doesn't drive too far
        //review this against python logic
        break;

      default:
        setDrivingMotorSpeed(0, 1);

    }

  }

}

void setDrivingMotorSpeed(int speed, int dir) {

  speed = constrain(speed, 0, 255); // Ensure valid PWM range

  if (dir == 1) { // Forward
    analogWrite(DRIVING_1A_PIN, speed);
    digitalWrite(DRIVING_1B_PIN, LOW);
  } else if (dir == -1) { // Reverse
    analogWrite(DRIVING_1B_PIN, speed);
    digitalWrite(DRIVING_1A_PIN, LOW);
  } else { // Stop
    digitalWrite(DRIVING_1A_PIN, LOW);
    digitalWrite(DRIVING_1A_PIN, LOW);
  }
}

void driveForXMilliseconds(int milliseconds, int speed, int dir)
//used for drive then stop then drive logic; precise movements
//Uses blocking methods to lock car into set motion until the function is finshed
{
  if (dir == 1)
  { // Forward
    analogWrite(DRIVING_1A_PIN, speed);
    digitalWrite(DRIVING_1B_PIN, LOW);
  }
  else
  { // Reverse
    analogWrite(DRIVING_1B_PIN, speed);
    digitalWrite(DRIVING_1A_PIN, LOW);
  }
  delay(milliseconds);
  digitalWrite(DRIVING_1A_PIN, LOW);
  digitalWrite(DRIVING_1A_PIN, LOW);
  delay(100);

}

void turnToTargetAngle(int targetAngle)
{
  if (targetAngle > currentAngle) //Target angle is to the right of current angle
  {
    for (int pos = currentAngle; pos <= targetAngle; pos += 1)
    {
      servoMotor.write(pos);
      currentAngle = pos;
      delay(10);             // waits 10ms for the servo to reach the position
    }
  }
  else { //Target angle is to the left of current angle
    for (int pos = currentAngle; pos >= targetAngle; pos -= 1)
    {
      servoMotor.write(pos);
      currentAngle = pos;
      delay(10);             // waits 10ms for the servo to reach the position
    }
  }

}

void ExitParking()
{
  if (leftDistance > rightDistance) //wall to right of car therefore turn left
  {
    turnToTargetAngle(45);
    driveForXMilliseconds(1000, 255, 1); //adjust this time in house
    turnToTargetAngle(135);
    driveForXMilliseconds(1000, 255, 1); //car should theoretically be straight at this point

  }
  else //wall to left of car therefore turn right
  {
    turnToTargetAngle(135);
    driveForXMilliseconds(1000, 255, 1);
    turnToTargetAngle(45);
    driveForXMilliseconds(1000, 255, 1);
  }


  turnToTargetAngle(90); //straighten wheels
  delay(100);
  hasStarted = true;

}
