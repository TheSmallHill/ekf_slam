#include <iostream>
#include <cstdint>
#include <stdint.h>
#include <string.h>
#include <string>
#include <xbeep.h>

#include "../lib/BlackLib/v3_0/BlackLib.h"
#include "Observer.h"
#include "remotenode.h"
#include "atcon.h"

#define BAUDRATE 9600

using namespace std;
using namespace BlackLib;
using namespace libxbee;

Observer::Observer()
{
  	//start uart using BlackLib, specified by constructor arguments
	  
	
  	//start xbee (default xbee2 but could have argument passed to constructor)
	//transmitter = new XBee("xbee2", "/dev/ttyS1", BAUDRATE);
	//connection = new Con(*transmitter, "Local AT"); // AT connection with callback handler
	xbee = new XBee("xbee2", "dev/ttyS1", BAUDRATE);	
	con = new atcon(*xbee, "Local AT");	
	//callback = new ConCallback;  
	
	//connection->xbee_conCallback(*transmitter, "Local AT");

  	//create new array for results to be stored, only starting point since we do not know how many beacons there are
  	observedData = new obs*[1];
}

Observer::~Observer()
{

  	//shutdown xbee
	//delete transmitter;
	//delete connection;
  
	//shutdown uart
  
  
	//delete dynamically allocated arrays
  
}

void Observer::doObservation(float angle)
{

  //start observation where data is retrieved through callbacks (investigate way to not use callback functions)
	//connection << "ND"; 
	con->start_node_detect();

  //using data from callback, resize the observedData array and store the data
	/* lazy-wait for it to finish */
		while (!con->node_detect_complete) {
			usleep(100000);
		}

		/* print out a list of nodes */
		for (std::list<remotenode>::iterator n = con->node_list.begin(); n != con->node_list.end(); n++) {
			/* it's just easier to print nice-looking output using printf()... */
			printf("Node: %-20s  0x%04X  0x%08X 0x%08X\n",
			       n->getName().c_str(), n->getAddr16(), n->getAddr64Hi(), n->getAddr64Lo());
}  


  //increment numObservations
  numObservations++;
  
}

void Observer::newScan()
{

  //reallocate the observedData array to an empty 1x1, ready for next observations at angle
  
  //reset numObservations
  numObservations = 0;

}

//Observer::calibrate(){}

 void Observer::processData()
{

  //iterate through array (convert each element's rssi to distance too) and find the bearing for each beacon
  

}

obs** Observer::getResults()
{

  //return pointer to all the observed data
  return(observedData);

}

void Observer::observationCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data)
{

  

}

float Observer::findBearing(/*somehow pass a column?*/)
{

  //find the bearing of the beacon
	

return(0);
}

float Observer::toDistance(int rssi)
{

  //convert provided rssi to distance
 
return(0); 
}

void ObserverCB::observationCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data){



}
