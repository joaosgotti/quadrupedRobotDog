/*
  Robot.cpp - Library for interacting with the quadruped robot dog
  Created by André Teixeira, João Jorge, João Veiga, May 1st, 2021
*/
#include "Robot.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <Adafruit_PWMServoDriver.h>

int SERVO_MIN[12] = {93, 100, 105, 110, 120, 120, 110, 110, 115, 110, 115, 115};
int SERVO_MAX[12] = {510, 520, 520, 522, 525, 530, 532, 530, 525, 525, 525, 530};
int HOME_ANGLE[12] = {40, 55, 125, 120, 105, 55, 125, 120, 80, 90, 90, 90};
int SERVO_CHANNEL[12] = {6, 4, 2, 0, 7, 5, 3, 1, 10, 11, 8, 9};

Robot::Leg::Leg()
{
  L1 = 3.0;
  L2 = 10.0;
  L3 = 8.0;
  q[0] = 90;
  q[1] = 90;
  q[2] = 90;
}

Robot::Leg::~Leg() {}

Robot::Robot()
{
  L = 20.0;
  W = 10.0;

  // For each leg define servo channels, minimum and maximum positions
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 3; j++){
      leg[i].servoChannel[j] = SERVO_CHANNEL[i + 4*j];
      leg[i].servoMin[j] = SERVO_MIN[leg[i].servoChannel[j]];
      leg[i].servoMax[j] = SERVO_MAX[leg[i].servoChannel[j]];
    }
  }
}

Robot::~Robot() {}

void Robot::initRobot(Adafruit_PWMServoDriver servoPwm)
{
  int i, j, pulseLength;
  int q[3];

  // Initialize all servos to home position
  for(i = 0; i < 4; i++){
    for(j = 0; j < 3; j++){
      q[j] = HOME_ANGLE[leg[i].servoChannel[j]];
    }
    setLeg(servoPwm, i, q);
  }
}

void Robot::setLeg(Adafruit_PWMServoDriver servoPwm, int legID, int q[3])
{
  int i, pulseLength;

  for(i = 0; i < 3; i++){
    pulseLength = map(q[i], 0, 180, leg[legID].servoMin[i], leg[legID].servoMax[i]);
    servoPwm.setPWM(leg[legID].servoChannel[i], 0, pulseLength);
    leg[legID].q[i] = q[i];
    delay(30);
  }
}

void Robot::inverseKinematics(int *q, int legID, float x, float y, float z)
{
  float D, aux;

  // Convert to radians
  for(int i = 0; i < 3; i++){
    q[i] = q[i] * M_PI / 180;
  }

  // Compute inverse kinematics
  switch(legID){
    case 0:
      aux = -1;
      break;
    case 1:
      aux = 1;
      break;
    case 2:
      aux = 1;
      break;
    case 3:
      aux = -1;
      break;
    default:
      printf("Error @inverseKinematics() - invalid leg ID...");
      exit(EXIT_FAILURE);
  }

  q[0] = -atan2f(-y, x) - atan2f(sqrtf(powf(x, 2.0) + powf(y, 2.0) - powf(leg[legID].L1, 2.0)), -leg[legID].L1);
  D = (powf(x, 2.0) + powf(y, 2.0) - powf(leg[legID].L1, 2.0) + powf(z, 2.0) - powf(leg[legID].L2, 2.0) - powf(leg[legID].L3, 2.0)) / (2 * leg[legID].L2 * leg[legID].L3);
  q[2] = atan2f(aux * sqrtf(1 - powf(D, 2.0)), D);
  q[1] = atan2f(z, sqrtf(powf(x, 2.0) + powf(y, 2.0) - powf(leg[legID].L1, 2.0))) - atan2(leg[legID].L3 * sinf(q[2]), leg[legID].L2 + (leg[legID].L3 * cosf(q[2])));

  // Round and convert to degrees
  q[0] = round(q[0] * 180 / M_PI);
  q[1] = round(q[1] * 180 / M_PI);
  q[2] = round(q[2] * 180 / M_PI);
}



