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

  return (pose*stepSize)/msDivider;

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

int motor::setMicrostep(int* ms[3])
{

  

}
