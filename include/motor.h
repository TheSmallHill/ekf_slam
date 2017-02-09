#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include "..///BlackLib.h"

using namespace std;

class motor{
  
  public:
  
    float position;
    float positionMax;
    float positionMin;
    int direction;
    float speed;
    float speedMax;
    uint8_t microStepping;
     
    void setPos(float, float);
    void setDirection(int);
    void incrementMotor(int);
    void rotateToPos(float);
    void test();
    
    BlackGPIO& stepPin;
    BlackGPIO& directionPin;
    BlackGPIO& enablePin;
    BlackGPIO& resetPin;
    BlackGPIO& sleepPin;
    BlackGPIO& ms1Pin;
    BlackGPIO& ms2Pin;
    BlackGPIO& ms3Pin;

    motor(BlackLib::BlackGPIO* stepPin, BlackLib::BlackGPIO* directionPin, BlackLib::BlackGPIO* enablePin, BlackLib::BlackGPIO* resetPin, BlackLib::BlackGPIO* sleepPin, BlackLib::BlackGPIO* ms1Pin, BlackLib::BlackGPIO* ms2Pin, BlackLib::BlackGPIO* ms3Pin) : stepPin(*stepPin), directionPin(*directionPin), enablePin(*enablePin), resetPin(*resetPin), sleepPin(*sleepPin), ms1Pin(*ms1Pin), ms2Pin(*ms2Pin), ms3Pin(*ms3Pin) 
    {
    }
    
    ~motor();
    
  //private: //consider making other members private like position, speed, divisions, direction
    bool checkLimits(float, float);
    
};

extern motor *M1;
