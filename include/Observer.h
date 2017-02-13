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
#include <map>
//#include <vector>

#include <xbeep.h>

#include "../lib/BlackLib/v3_0/BlackLib.h"
#include "remotenode.h"
#include "atcon.h"

// struct to hold a single observation
typedef struct observation{
  	int ID;
	std::string name;
  	float angle;
  	int rssi;
  	float distance;
} obs;

//typedef std::vector<std::vector<obs> > obsArray;

class Observer {

	//int numObservations;

  public:
    
    	Observer(/* what should be passed to the constructor? */); 
    	~Observer(); //destructor, shutdown xbee, shutdown uart, delete dynamically allocated arrays	
    
	std::vector<obs> doObservation(float); //Start performing an observation, only one	
	void newScan(); //prepare for a new set of observations (clear results, reset numObservations)
    	void calibrate(int, int);
    	void processData(std::vector<std::vector<obs> >); // go through results, convert to distance, find range and bearing for each beacon, save to results  
     	void printData(std::vector<std::vector<obs> >); //print data to the screen
	void saveData(std::vector<std::vector<obs> >); //and possibly other arguments
  
  private:
  
    	
    	float findBearing(); //find bearing for 1 beacon
    	float toDistance(obs); //convert one RSSI to distance
  
	libxbee::XBee* xbee;
	BlackLib::BlackUART* uart;	
	atcon* con;
	  
    	

  protected:
  	/* no protected members yet */
  
};
