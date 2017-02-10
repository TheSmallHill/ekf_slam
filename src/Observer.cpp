#include <iostream>
#include <cstdint>
#include <stdint.h>
#include <string.h>
#include <string>
#include <xbeep.h>

#include "../lib/BlackLib/v3_0/BlackLib.h"
#include "Observer.h"

using namespace std;
using namespace BlackLib;

Observer::Observer()
{
  //start uart specified by constructor arguments
  
  //start xbee (default xbee2 but could have argument passed to constructor
  
  //create new array for results to be stored
  observedData = new observation[1][1];
}

Observer::~Observer()
{

  //shutdown xbee

  //shutdown uart
  
  //delete dynamically allocated arrays
  
}

Observer::doObservation(float angle)
{

  //start observation where data is retrieved through callbacks (investigate way to not use callback functions)
  
  //using data from callback, resize the observedData array and store the data
  
  //increment numObservations
  numObservations++;
  
}

Observer::newScan()
{

  //reallocate the observedData array to an empty 1x1, ready for next observations at angle
  
  //reset numObservations
  numObservations = 0;

}

//Observer::calibrate(){}

Observer::processData()
{

  //iterate through array (convert each element's rssi to distance too) and find the bearing for each beacon
  

}

Observer::getResults()
{

  //return pointer to all the observed data
  return(observedData);

}

Observer::observationCB(/*fill in the arguments here*/)
{

  

}

Observer::findBearing(/*somehow pass a column?*/)
{

  //find the bearing of the beacon

}

Observer::toDistance(int rssi)
{

  //convert provided rssi to distance
  
}
