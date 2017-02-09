#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include "../lib/BlackLib/v3_0/BlackLib.h"

using namespace std;

class motor{
  
  public:
  
    float position;
    float positionMax;
    float positionMin;
    float stepSize;
    int numSteps;
    int direction;
    float speed;
    float speedMax;
    int ms[3] = {0, 0, 0};
      
    int getPos();
    float getAng();
    void setDirection(int);
    void setMicrostep(int*);
    void incrementMotor(int);
    void rotateToPos(float);
    void test();
    
    BlackGPIO& stepPin;
    BlackGPIO& directionPin;
    BlackGPIO& enablePin;
    BlackGPIO& sleepPin;

    motor(BlackLib::BlackGPIO* stepPin, BlackLib::BlackGPIO* directionPin, BlackLib::BlackGPIO* enablePin, BlackLib::BlackGPIO* sleepPin) : stepPin(*stepPin), directionPin(*directionPin), enablePin(*enablePin), sleepPin(*sleepPin) 
    {
    }
    
    ~motor();
    
  private: //consider making other members private like position, speed, divisions, direction
    float msDivider;
    //bool checkLimits(float, float);
      
  
};

extern motor *M1;
