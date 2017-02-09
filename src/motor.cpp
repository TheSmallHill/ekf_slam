#include <iostream>
#include <stdint.h>
#include <unistd.h>

#include "motor.h"

using namespace std;
using namespace BlackLib;

int motor::getPos()
{

  return pos;
  
}

float motor::getAng()
{

  return (pos*stepSize)/msDivider;

}

int motor::setDirection(int dir)
{

  if (dir == 0) { //clockwise
  
    directionPin.setValue(low);
    
  } else if (dir == 1) {
  
    directionPin.setValue(high);
  
  } else {
  
    cout << "Invalid direction" << endl;
    
  }
  
  if ((directionPin.isHigh && dir == 1) || (!directionPin.isHigh && dir == 0)) {
  
    return(1);
    
  } else return(0);

}

int motor::getDirection()
{

  return directionPin.getValue;

}

int motor::setMicrostep(int* ms[3])
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

  if (ms1Pin.getValue == ms[0] && ms2Pin.getValue == ms[1] && ms3Pin.getValue == ms[2]) {
   return(1); 
  } else return(0);
  
}

void motor::incrementMotor(int steps)
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
    if (directionPin.isHigh) {
    
      pos--;
    
    } else if (!directionPin.isHigh) {
    
      pos++;
    
    } else {}
    
  }

}
/*
void motor::rotateToAng(float desiredAngle)
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
void motor::test()
{

  cout << "Test successful!" << endl;

}

//BlackLib::BlackGPIO pulse(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
//BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);

//motor *M1 = new motor(&pulse, &direc);
