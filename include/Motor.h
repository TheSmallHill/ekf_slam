#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include "../lib/BlackLib/v3_0/BlackLib.h"

using namespace std;
using namespace BlackLib;

class Motor{
  
  float stepSize = 1.8;
  //int numSteps = 200;
  
  public:
    
    float pos;
    float posMax;
    float posMin;  
    int direction;
    //float speed;
    //float speedMax;
    int ms[3];
      
    int getPos();
    float getAng();
    int setDirection(int);
    int getDirection();
    int setMicrostep(int*);
    void incrementMotor(int);
    //void rotateToAng(float);
    void test();
    
    BlackGPIO& stepPin;
    BlackGPIO& directionPin;
    
    BlackGPIO& ms1Pin;
    BlackGPIO& ms2Pin;
    BlackGPIO& ms3Pin;
  
    Motor(BlackLib::BlackGPIO* stepPin, BlackLib::BlackGPIO* directionPin, BlackLib::BlackGPIO* ms1Pin, BlackLib::BlackGPIO* ms2Pin, BlackLib::BlackGPIO* ms3Pin) : stepPin(*stepPin), directionPin(*directionPin), ms1Pin(*ms1Pin), ms2Pin(*ms2Pin), ms3Pin(*ms3Pin) {}
    
    ~Motor();
    
  private: //consider making other members private like position, speed, divisions, direction
    float msDivider;
    //bool checkLimits(float, float);
      
  
};

//extern Motor *M1;
