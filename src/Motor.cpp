#include <iostream>
#include <string>
#include <stdint.h>
#include <unistd.h>

#include "../lib/BlackLib/v3_0/BlackLib.h"
#include "Motor.h"

/* motor destructor */
Motor::~Motor()
{
}

/* get position of motor */
int Motor::getPos()
{

  return pos;
  
}

/* get angle of motor */
float Motor::getAng()
{

  return (pos*stepSize)/msDivider;

}

/* set the direction the motor will move in*/
int Motor::setDirection(int dir)
{

  if (dir == 0) { //clockwise
  
    directionPin.setValue(BlackLib::low);
    
  } else if (dir == 1) {
  
    directionPin.setValue(BlackLib::high);
  
  } else {
  
    std::cout << "Invalid direction" << std::endl;
    
  }
  
  if ((directionPin.isHigh() && (dir == 1)) || (!directionPin.isHigh() && (dir == 0))) {
  
    return(1);
    
  } else return(0);

}

/* get the value for what direction the motor is currently set to*/
int Motor::getDirection()
{

  return stoi(directionPin.getValue());

}

/* set the microstepping pins for the motor controller*/
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

/* increment the motor a certain number of steps*/
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
    
    } else {

	pos = pos;

	}
    
  }

}

/* test that the object was created successfully*/
void Motor::test()
{

  std::cout << "Test successful!" << std::endl;

}
