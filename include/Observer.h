#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include <cstring>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <semaphore.h>
#include <errno.h>
#include <time.h>

#include <xbeep.h>

#include "../lib/BlackLib/v3_0/BlackLib.h"
#include "remotenode.h"
#include "atcon.h"

//using namespace std;
//using namespace BlackLib;
//using namespace libxbee;

// struct to hold a single observation
typedef struct observation{
  int ID;
	std::string name;
  float angle;
  int RSSI;
  float distance;
} obs;

class Observer {

	//int numObservations;

  public:
    
    	Observer(); //constructor, start uart, connect to xbee, create results, figure out what values should be passed to constructor
    	~Observer(); //destructor, shutdown xbee, shutdown uart, delete dynamically allocated arrays	
    
//	virtual void observationCB(struct xbee*, struct xbee_con*, struct xbee_pkt**, void**);	
	void doObservation(float); //Start performing an observation, only one
	void newScan(); //prepare for a new set of observations (clear results, reset numObservations)
    	void calibrate(int, int);
    	void processData(); // go through results, convert to distance, find range and bearing for each beacon, save to results  
    	obs** getResults(); //returns pointer to the final, processed array
  
  private:
  
    	
    	float findBearing(); //find bearing for 1 beacon
    	float toDistance(int); //convert one RSSI to distance
  
	libxbee::XBee* xbee;
	BlackLib::BlackUART* uart;	
	atcon* con;
	  
    	

  protected:
  	obs** observedData; //2D array of observations (array will still be organized by angle and ID but this will make processing easier)
    	int numObs;
  
};

/*Observer: public ConCallback::ConCallback {

};*/
