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
