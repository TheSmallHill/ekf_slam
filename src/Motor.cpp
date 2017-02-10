#include <iostream>
#include <string>
#include <stdint.h>
#include <unistd.h>

#include "../lib/BlackLib/v3_0/BlackLib.h"
#include "Motor.h"

using namespace std;
using namespace BlackLib;

Motor::~Motor()
{
}

int Motor::getPos()
{

  return pos;
  
}

float Motor::getAng()
{

  return (pos*stepSize)/msDivider;

}

int Motor::setDirection(int dir)
{

  if (dir == 0) { //clockwise
  
    directionPin.setValue(low);
    
  } else if (dir == 1) {
  
    directionPin.setValue(high);
  
  } else {
  
    cout << "Invalid direction" << endl;
    
  }
  
  if ((directionPin.isHigh() && (dir == 1)) || (!directionPin.isHigh() && (dir == 0))) {
  
    return(1);
    
  } else return(0);

}

int Motor::getDirection()
{

  return stoi(directionPin.getValue());

}

int Motor::setMicrostep(int ms[3])
{

 if (ms[0] == 0 && ms[1] == 0 && ms[2] == 0 ) {
  msDivider = 1; //full step
 } else if (ms[0] == 1 && ms[1] == 0 && ms[2] == 0 ) {
  msDivider = 2; //half step
 } else if (ms[0] == 0 && ms[1] == 1 && ms[2] == 0 ) {
  msDivider = 4; //quarter step
 } else if (ms[0] == 1 && ms[1] == 1 && ms[2] == 0 ) {
  msDivider = 8; //eighth step
 } else if (ms[0] == 1 && ms[1] == 1 && ms[2] == 1 ) {
  msDivider = 16; //sixteenth step
 }

  if (stoi(ms1Pin.getValue()) == ms[0] && stoi(ms2Pin.getValue()) == ms[1] && stoi(ms3Pin.getValue()) == ms[2]) {
   return(1); 
  } else return(0);
  
}

void Motor::incrementMotor(int steps)
{

  // figure out how to change speed  
  struct timespec tim, tim1;
  tim.tv_sec = 0;
  tim.tv_nsec = 1000000L;
  
  for (int iter = 0; iter < steps; iter++) {
  
    stepPin.setValue(BlackLib::high);
    nanosleep(&tim, &tim1); //figure out optimal pause length
    stepPin.setValue(BlackLib::low);
    nanosleep(&tim, &tim1);
    
    //change position counter
    if (directionPin.isHigh()) {
    
      pos--;
    
    } else if (!directionPin.isHigh()) {
    
      pos++;
    
    } else {}
    
  }

}
/*
void Motor::rotateToAng(float desiredAngle)
{

  float angle = (pos*stepSize)/msDivider;

  if (desiredAngle < angle) {
  
    directionPin.setValue(BlackLib::low);
  
  } else if (desiredAngle > angle) {
  
    directionPin.setValue(BlackLib::high);
  
  } else {} //exit function
  
  int steps = abs((desiredAngle-angle)/(stepSize/msDivider));
  
  this->incrementMotor(steps);
  
}
*/
void Motor::test()
{

  cout << "Test successful!" << endl;

}

BlackLib::BlackGPIO pulse(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);

Motor *M1 = new Motor(&pulse, &direc);
