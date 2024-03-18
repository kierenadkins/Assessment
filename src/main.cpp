// This code was based on the border control code
#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4IMU.h>
#include <SD.h>
#include <SPI.h>

#define QTR_THRESHOLD 1000 // ms

#define REVERSE_SPEED 100    // ms
#define TURN_SPEED 100       // ms
#define FORWARD_SPEED 150    // ms
#define REVERSE_DURATION 200 // ms
#define TURN_DURATION 600    // ms
#define DROP_PARCEL_OFF 2000 // ms

#define HIT_INTERVAL 1000 // ms

unsigned long lastLeftHitTime = 0;  // Left Sensor Hit
unsigned long lastRightHitTime = 0; // Right Sensor Hit

Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4IMU imu;
Zumo32U4LCD display;

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

const uint8_t sensorThreshold = 6; // detect objects
uint8_t houseFinder = 2;           // Houses to be delivered too
uint8_t rightTurn = 0;             // count right turns
uint8_t leftTurn = 0;              // count left Turns

char path[1000];
uint8_t pathLength = 0;

// this function is simply used to count down before the robot explores a maze
void waitForButtonAndCountDown()
{
  display.clear();
  display.println(F("Press A"));

  buttonA.waitForButton();

  display.clear();

  // Play audible countdown.
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);
  delay(1000);
}

// This function is used to drop of the parcel when a house is discovered, then turns out of the room
void detectedObject()
{
  buzzer.playNote(NOTE_G(4), 500, 15);
  motors.setSpeeds(0, 0);
  display.print("Dropping");
  delay(DROP_PARCEL_OFF);

  motors.setSpeeds(-200, 200);                     // Rotate in place
  delay(TURN_DURATION);                            // Adjust this delay as needed
  motors.setSpeeds(FORWARD_SPEED, -FORWARD_SPEED); // Reverse after rotation
  delay(REVERSE_DURATION);                         // Delay to reverse
}

// this function uses the path history to try and navigate back out of the maze to the start by using the path and reversing.
void findMyWayHome()
{
  for (int i = pathLength - 1; i >= 0; --i)
  {
    Serial.print(path[i]);
    reverseDirection(path[i]);
  }
}

// this function is not correctly implemented but the idea is that the directions would be reversed to the orginal direction
void reverseDirection(char direction)
{
  switch (direction)
  {
  case 'S':
    motors.setSpeeds(-FORWARD_SPEED, -FORWARD_SPEED); // Reverse
    break;
  case 'L':
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED); // Forward
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED); // Right turn
    delay(TURN_DURATION);
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED); // Reverse
    break;
  case 'R':
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED); // Forward
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED); // Left turn
    delay(TURN_DURATION);
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED); // Reverse
    break;
  case 'T':
    motors.setSpeeds(200, -200);                     // Rotate in place (opposite direction)
    delay(TURN_DURATION);                            // Adjust this delay as needed
    motors.setSpeeds(-FORWARD_SPEED, FORWARD_SPEED); // Reverse after rotation
    delay(REVERSE_DURATION);                         // Delay to reverse
    break;
  default:
    break;
  }
}

// turn's the robot in the direction that is put in
void direction(char direction)
{
  switch (direction)
  {
  case 'S':
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    addDirection('S');
    break;
  case 'L':
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    rightTurn++;
    addDirection('L');
    lastLeftHitTime = millis();
    break;
  case 'R':
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    leftTurn++;
    addDirection('R');
    lastRightHitTime = millis();
    break;
  case 'T':
    motors.setSpeeds(-200, 200);                     // Rotate in place
    delay(TURN_DURATION);                            // Adjust this delay as needed
    motors.setSpeeds(FORWARD_SPEED, -FORWARD_SPEED); // Reverse after rotation
    delay(REVERSE_DURATION);                         // Delay to reverse
    leftTurn = 0;
    rightTurn = 0;
    addDirection('T');
    break;
  default:
    break;
  }
}

// add the direction to the path history
void addDirection(char direction)
{
  path[pathLength] = direction;
  pathLength++;
}

// informs the user that all parcels have been delivered
void printDelivered()
{
  display.print("All Parcels");

  // Go to the next line
  display.gotoXY(0, 1);

  // Print a number
  display.print("Delivered");
}

void setup()
{
  proxSensors.initFrontSensor();
  lineSensors.initThreeSensors();
  waitForButtonAndCountDown();
  Serial.begin(9600);
}

void loop()
{
  if (buttonA.isPressed())
  {
    // If button is pressed, stop and wait for another press to
    // go again.
    motors.setSpeeds(0, 0);
    buttonA.waitForRelease();
    waitForButtonAndCountDown();
  }

  proxSensors.read();

  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  bool objectSeen = leftValue >= sensorThreshold && rightValue >= sensorThreshold;

  lineSensors.read(lineSensorValues);

  if (objectSeen)
  {
    detectedObject();
    --houseFinder;
    if (houseFinder == 0)
    {
      motors.setSpeeds(0, 0);
      buzzer.playNote(NOTE_G(4), 500, 15);
      printDelivered();
      findMyWayHome();
    }
  }
  else if ((millis() - lastRightHitTime < HIT_INTERVAL) && (millis() - lastLeftHitTime < HIT_INTERVAL))
  {
    direction('T');
    lastRightHitTime = 0;
    lastLeftHitTime = 0;
  }
  else if (leftTurn >= 10 && rightTurn >= 10)
  {
    direction('T');
  }
  else if (lineSensorValues[0] > QTR_THRESHOLD)
  {
    direction('R');
  }
  else if (lineSensorValues[1] > QTR_THRESHOLD)
  {
    direction('T');
  }
  else if (lineSensorValues[2] > QTR_THRESHOLD)
  {
    direction('L');
  }
  else
  {
    direction('S');
  }
}
