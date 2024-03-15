/** This example uses the Zumo's line sensors to detect the white
border around a sumo ring.  When the border is detected, it
backs up and turns. */

#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4IMU.h>

// This might need to be tuned for different lighting conditions,
// surfaces, etc.
#define QTR_THRESHOLD 1000 // microseconds
#define HIT_INTERVAL = 1000;

#define REVERSE_SPEED 100 // 0 is stopped, 400 is full speed
#define TURN_SPEED 100
#define FORWARD_SPEED 150
#define REVERSE_DURATION 200 // ms
#define TURN_DURATION 600    // ms
#define DROP_PARCEL_OFF 2000 // ms

unsigned long lastLeftHitTime = 0;
unsigned long lastRightHitTime = 0;

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4IMU imu;
Zumo32U4LCD display;

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

const uint8_t sensorThreshold = 6;
uint8_t houseFinder = 2;
uint8_t rightTurn = 0;
uint8_t leftTurn = 0;

char path[1000];
uint8_t pathLength = 0;

void waitForButtonAndCountDown()
{
  ledYellow(1);
  display.clear();
  display.println(F("Press A"));

  buttonA.waitForButton();

  ledYellow(0);
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

void findMyWayHome()
{
  for (int i = pathLength - 1; i >= 0; --i)
  {
    Serial.print(path[i]);
    reverseDirection(path[i]);
  }
}

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
    break;
  default:
    break;
  }
}

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
    break;
  default:
    break;
  }
}

void addDirection(char direction)
{
  path[pathLength] = direction;
  pathLength++;
}

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

  bool objectSeen = 0;
  objectSeen = leftValue >= sensorThreshold && rightValue >= sensorThreshold;

  lineSensors.read(lineSensorValues);

  if (objectSeen)
  {
    detectedObject();
    --houseFinder;
    if (houseFinder == 0)
    {
      // findMyWayHome();
      motors.setSpeeds(0, 0);
      buzzer.playNote(NOTE_G(4), 500, 15);
      printDelivered();
      while (1 == 1)
      {
      }
    }
  }
  else if ((millis() - lastRightHitTime < 1000) && (millis() - lastLeftHitTime < 1000))
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
    // If leftmost sensor detects line, reverse and turn to the
    // right.
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
