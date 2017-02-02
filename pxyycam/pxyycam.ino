//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with
// Pixy and Arduino.  This program simply prints the detected object blocks
// (including color codes) through the serial console.  It uses the Arduino's
// ICSP port.  For more information go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino)
//
// It prints the detected blocks once per second because printing all of the
// blocks for all 50 frames per second would overwhelm the Arduino's serial port.
//

#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
double const FOCAL_LENGTH = 2.8e-3 * 39.3701; //meters to inches (f)
double const CHIP_HEIGHT = 2.43e-3 * 39.3701; //meters to inches hc(y)
double const CHIP_WIDTH = 3.888e-3 * 39.3701; // meters to inches hc(x)
double const CHIP_PIXEL_HEIGHT = 200; // pc(y)
double const CHIP_PIXEL_WIDTH = 320; // pc(x)
double const GEAR_TARGET_HEIGHT = 5; // ho
double const THETA = 0; //angle from vertical of camera (radians)
int const FRAME_WIDTH = 320;
int const FRAME_HEIGHT = 100;
// This is the main Pixy object
Pixy pixy;
bool gear = false;
bool goal = false;
double visionWidth = 0;
double visionHeight = 0;
double visionX = 0;
double visionY = 0;
float targetType = -1;
float angularOffset = 0;
float xOffset = 0;
float yOffset = 0;

void requestEvent(int howMany);
void detectTarget();
double goalDistance();
double goalAngle();
void calculateVisionPosition();
void calculateAngularOffset ();

void requestEvent(int howMany) {
  uint8_t data[4 * sizeof(float)];


  memcpy(&data[0], &targetType, sizeof(float));
  memcpy(&data[sizeof(float)], &angularOffset, sizeof(float));
  memcpy(&data[2 * sizeof(float)], &xOffset, sizeof(float));
  memcpy(&data[3 * sizeof(float)], &yOffset, sizeof(float));

  Wire.write(data, sizeof(data));
}


void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onRequest(requestEvent); // register event
  pixy.init();
}

void loop() {
  static int i = 0;
  if (i % 50 == 0) {
    detectTarget();
    
    if (goal) {
      targetType = 0;
      calculateVisionPosition();
      calculateGoalAngularOffset();
      calculateGoalDistance();
    }
    else if (gear) {
      targetType = 1;
      calculateVisionPosition();
      // calculateGearAngularOffset()
      // calculateGearDistnce()
    }
    else {
      targetType = -1;
    }
  }
  i++;
}

void detectTarget() {
  if (pixy.getBlocks() >= 2) {
    int xOffset = abs(pixy.blocks[0].x - pixy.blocks[1].x);
    int yOffset = abs(pixy.blocks[0].y - pixy.blocks[1].y);
    if (xOffset > yOffset) {
      gear = true;
      goal = false;
    }
    else {
      gear = false;
      goal = true;
    }
  }
  else {
    gear = false;
    goal = false;
  }
}

void calculateGoalDistance() {
  double p = (pixy.blocks[0].height + pixy.blocks[1].height) / 2;
  yOffset = (FOCAL_LENGTH * CHIP_PIXEL_HEIGHT * GEAR_TARGET_HEIGHT * cos(THETA)) / (CHIP_HEIGHT * p);
  xOffset =  0;
}


void calculateVisionPosition() {
  int leftPos = min(pixy.blocks[0].x - (.5 * pixy.blocks[0].width), pixy.blocks[1].x - (.5 * pixy.blocks[1].width));
  int rightPos = max(pixy.blocks[0].x + (.5 * pixy.blocks[0].width), pixy.blocks[1].x + (.5 * pixy.blocks[1].width));
  int topPos = max(pixy.blocks[0].y + (.5 * pixy.blocks[0].height), pixy.blocks[1].y + (.5 * pixy.blocks[1].height));
  int bottomPos = min(pixy.blocks[0].y - (.5 * pixy.blocks[0].height), pixy.blocks[1].y - (.5 * pixy.blocks[1].height));
  visionWidth = rightPos - leftPos;
  visionHeight = topPos - bottomPos;
  visionX = (leftPos + rightPos) / 2;
  visionY = (topPos + bottomPos) / 2;
}

void calculateGoalAngularOffset () {
  int rawOffset = (FRAME_WIDTH / 2) - visionX; //positive means move left
  angularOffset = rawOffset / visionHeight; //float for sending
}


