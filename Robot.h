/*
  Robot.h - Library for interacting with the quadruped robot dog
  Created by André Teixeira, João Jorge, João Veiga, May 1st, 2021
*/
#ifndef Robot_h
#define Robot_h

#include <Adafruit_PWMServoDriver.h>

class Robot
{
  /*
  * ====================== Private ================================
  */
  private:
    class Leg{
      private:

      public:
        float L1, L2, L3;             // Physical parameters
        int servoMin[3], servoMax[3]; // Servos minimum and maximum positions 
        int q[3];                     // Joint angles
        int servoChannel[3];          // Servos channel numbers 

        Leg();                    // Constructor
        ~Leg();                   // Destructor
    };


  /*
  * ====================== Public ================================
  */  
  public:  
    Leg leg[4];         // Legs
    float L, W; 

    Robot();
    ~Robot();

    //int degreesToPulseLength(int degrees);
    void initRobot(Adafruit_PWMServoDriver servoPwm);
    void inverseKinematics(int *q, int legID, float x, float y, float z);
    void setLeg(Adafruit_PWMServoDriver servoPwm, int legID, int q[3]);
};

#endif