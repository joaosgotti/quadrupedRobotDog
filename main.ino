#include "Robot.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define echoPinRight 7 // attach pin D2 Arduino to pin Echo of Right HC-SR04
#define trigPinRight 6 //attach pin D3 Arduino to pin Trig of Rigth HC-SR04
#define echoPinLeft 5 // attach pin D2 Arduino to pin Echo of Left HC-SR04
#define trigPinLeft 4 //attach pin D3 Arduino to pin Trig of Left HC-SR04
#define redLed 10
#define yellowLed 8
#define greenLed 9
#define buttonPin 2

/*
* ================== Variables and defines ==========================
*/
Adafruit_PWMServoDriver servoPwm = Adafruit_PWMServoDriver(0x40);
float gaitCycle = 2.0;      // in seconds
float dutyFactor = 0.875;   
Robot robot;
int distanceLeft, distanceRight; // variable for the distance measurement

/*
* ================ Functions declarations ===========================
*/
void buttonWait();
void down(int deg);
void up(int deg);
int readSonar(int trigPin, int echoPin);
void sonarLED(int distanceLeft, int distanceRight);

/*
* =========================== Setup ==================================
*/
void setup() {
  Serial.begin(9600);

  // Setup sonar pins
  pinMode(trigPinRight, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinRight, INPUT); // Sets the echoPin as an INPUT
  pinMode(trigPinLeft, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinLeft, INPUT); // Sets the echoPin as an INPUT
  pinMode(redLed,OUTPUT);
  pinMode(yellowLed,OUTPUT);
  pinMode(greenLed, OUTPUT);

  // Wait for start button press
  pinMode(buttonPin, INPUT);
  buttonWait();
  delay(2000);

  Serial.println("Program started ...");

  // Initialize PWM servo driver 
  servoPwm.begin();
  servoPwm.setPWMFreq(50);

  // Initialize robot
  robot.initRobot(servoPwm);
  delay(5000);

  // Move legs
  /*q[0] = robot.leg[0].q[0] - 100;
  q[1] = robot.leg[0].q[1] - 45;
  q[2] = robot.leg[0].q[2];
  robot.setLeg(servoPwm, 0, q);*/
  /*down(30);
  delay(4000);
  up(30);
  delay(4000);*/
}

/*
* =========================== Main loop ==============================
*/
void loop() {
  // Read sonars
  distanceLeft = readSonar(trigPinLeft, echoPinLeft);
  distanceRight = readSonar(trigPinRight, echoPinRight);
  sonarLED(distanceLeft, distanceRight);

  down(30);
  delay(2000);
  up(30);
  delay(2000);
}

// Stands down 
void up(int deg)
{
  int sign;
  int q[3];

  for(int i = 0; i < 4; i++){
    if((i == 0) || (i == 2)){
      sign = -1;
    }
    else{
      sign = 1;
    }
    q[0] = robot.leg[i].q[0] - (sign * (deg + 15));
    q[1] = robot.leg[i].q[1] - (sign * deg);
    q[2] = robot.leg[i].q[2];
    robot.setLeg(servoPwm, i, q);
  }
}

// Stands down 
void down(int deg)
{
  int sign;
  int q[3];

  for(int i = 0; i < 4; i++){
    if((i == 0) || (i == 2)){
      sign = -1;
    }
    else{
      sign = 1;
    }
    q[0] = robot.leg[i].q[0] + (sign * (deg + 15));
    q[1] = robot.leg[i].q[1] + (sign * deg);
    q[2] = robot.leg[i].q[2];
    robot.setLeg(servoPwm, i, q);
  }
}


// Stands up 


// Reads sonars and displays distance in serial monitor
int readSonar(int trigPin, int echoPin)
{
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  return distance;
}

// Turns on LEDS according to the minimum distance and prints distances
void sonarLED(int distanceLeft, int distanceRight)
{
  // Compute minimum of two distances and turn on LED accordingly
  int minDistance = min(distanceLeft, distanceRight);

  if(minDistance < 15){
    digitalWrite(redLed, HIGH);   
    digitalWrite(yellowLed, LOW);
    digitalWrite(greenLed, LOW);
  }
  else if(minDistance >= 15 && minDistance < 50){
    digitalWrite(yellowLed, HIGH); 
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, LOW); 
  }
  else{
    digitalWrite(greenLed, HIGH);
    digitalWrite(redLed, LOW);
    digitalWrite(yellowLed, LOW); 
  }
  // Displays the distance on the Serial Monitor
  Serial.print("Distance Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm \t Distance Right: ");
  Serial.print(distanceRight);
  Serial.println(" cm");
}


// Waits for button input
void buttonWait()
{
  int buttonState = 0;
  while(1){
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {
      return;
    }
  }
}

