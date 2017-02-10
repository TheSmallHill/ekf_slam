#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include <string.h>
#include <string>

#include "../lib/BlackLib/v3_0/BlackLib.h"
#include <xbeep.h>

using namespace std;
using namespace BlackLib;

// struct to hold a single observation
typedef struct observation{
  int ID;
  float angle;
  int RSSI;
  float distance;
};

class Observer {

  public:
    
    Oberver(); //constructor, start uart, connect to xbee, create results, figure out what values should be passed to constructor
    ~Observer(); //destructor, shutdown xbee, shutdown uart, delete dynamically allocated arrays
    void doOberservation(); //Start performing an observation, only one
    void newScan(); //prepare for a new set of observations (clear results, reset numObservations)
    void calibrate(int, int);
    void processData(); // go through results, convert to distance, find range and bearing for each beacon, save to results  
    observation** getResults(); //returns pointer to the final, processed array
  
  private:
  
    void observationCB(struct xbee*, struct xbee_con*, struct xbee_pkt**, void**);
    float findBearing(); //find bearing for 1 beacon
    float toDistance(int); //convert one RSSI to distance
    
  
  protected:
  
    int numObservations;  
    observation** observedData; //2D array of observations (array will still be organized by angle and ID but this will make processing easier)
  
}
