#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include "../lib/BlackLib/v3_0/BlackLib.h"

using namespace std;
using namespace BlackLib;

class motor{
  
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
    int setMicrostep(int);
    void incrementMotor(int);
    //void rotateToAng(float);
    void test();
    
    BlackGPIO& stepPin;
    BlackGPIO& directionPin;
  /*  
  BlackGPIO& ms1Pin(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
    BlackGPIO& ms2Pin(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
    BlackGPIO& ms3Pin(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);
  */
    motor(BlackLib::BlackGPIO* stepPin, BlackLib::BlackGPIO* directionPin) : stepPin(*stepPin), directionPin(*directionPin) { }
    
    ~motor();
    
  private: //consider making other members private like position, speed, divisions, direction
    float msDivider;
    //bool checkLimits(float, float);
      
  
};

extern motor *M1;
