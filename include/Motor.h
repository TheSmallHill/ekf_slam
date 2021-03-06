#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include <xbeep.h>

#include "../lib/BlackLib/v3_0/BlackLib.h"

class Motor{
  
  float stepSize = 1.8;
  //int numSteps = 200;
  
  public:
    
    float pos;
    float posMax;
    float posMin;  
    int direction;
    int ms[3];
      
    int getPos();
    float getAng();
    int setDirection(int);
    int getDirection();
    int setMicrostep(int*);
    void incrementMotor(int);
    void test();
    
    BlackLib::BlackGPIO& stepPin;
    BlackLib::BlackGPIO& directionPin;
    
    BlackLib::BlackGPIO& ms1Pin;
    BlackLib::BlackGPIO& ms2Pin;
    BlackLib::BlackGPIO& ms3Pin;
  
    Motor(BlackLib::BlackGPIO* stepPin, BlackLib::BlackGPIO* directionPin, BlackLib::BlackGPIO* ms1Pin, BlackLib::BlackGPIO* ms2Pin, BlackLib::BlackGPIO* ms3Pin) : stepPin(*stepPin), directionPin(*directionPin), ms1Pin(*ms1Pin), ms2Pin(*ms2Pin), ms3Pin(*ms3Pin) {}
    
    ~Motor();
    
  private: //consider making other members private like position, speed, divisions, direction
    float msDivider;
    //bool checkLimits(float, float);
      
  
};
