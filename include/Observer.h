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

class Observer {

  public:
    
    Oberver(); //constructor
    ~Observer(); //destructor
    doOberservation(); //Start performing an observation, only one
    newScan(); //prepare for a new set of observations (clear results, reset numObservations)
    calibrate();
    getResults();
    process  
  
  private:
  
    observationCB(struct xbee*, struct xbee_con*, struct xbee_pkt**, void**);
  
  protected:
  
    int numObservations;  
  
}
