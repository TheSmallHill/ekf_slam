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

Observer::Observer()
{
  	//start uart using BlackLib, specified by constructor arguments
	uart = new BlackLib::BlackUART(BlackLib::UART2, BlackLib::Baud9600, BlackLib::ParityEven, BlackLib::StopOne, BlackLib::Char8);	
		
	bool isOpened = uart->open(BlackLib::ReadWrite | BlackLib::NonBlock);

    	if( !isOpened )
    	{
        	std::cout << "UART DEVICE CAN\'T OPEN.;" << std::endl;
        	exit(1);
    	}

  	//start xbee (default xbee2 but could have argument passed to constructor)
	xbee = new libxbee::XBee("xbee2", "dev/ttyS1", BAUDRATE);	
	con = new atcon(*xbee, "Local AT");	

  	//create new array for results to be stored, only starting point since we do not know how many beacons there are
  	observedData = new obs*[1];
	//numObservations = 0;
}

Observer::~Observer()
{

  	//shutdown xbee
	delete xbee;
  
	//shutdown uart
	delete uart;
  
	//delete dynamically allocated arrays
  	delete observedData;
}

void Observer::doObservation(float angle)
{
	
	int numBeacons = 0;
	//string temp;

	/* start detecting nodes */ 
	con->start_node_detect();

	/* wait for it to finish */
	while (!con->node_detect_complete) {
		usleep(100000); /* wait for .1 second */
	}

	observedData[numObs] = new obs[con->node_list.size()];

	/* save and print out a list of nodes */
	for (std::list<remotenode>::iterator n = con->node_list.begin(); n != con->node_list.end(); n++) {

		/* it's just easier to print nice-looking output using printf()... */
		printf("Node: %-20s  0x%04X  0x%08X 0x%08X\n", n->getName().c_str(), n->getAddr16(), n->getAddr64Hi(), n->getAddr64Lo());	
		
		/* but also save to a temporary variable */
		numBeacons++;				
		observedData[numObs][numBeacons].name = n->getName().c_str();
		/* figure out how to extract rssi */	
		
	}  

	//for (std::list<string>::iterator n = con->node_list.begin(); n!= con->node_list.end(); n++) {

	//	temp = n->
	
	//}


  	//increment numObservations
  	numObs++;
  
}

void Observer::newScan()
{

  	//reallocate the observedData array to an empty 1x1, ready for next observations at angle
	  

  	//reset numObservations
  	this->numObs = 0;

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

/*void Observer::observationCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data)
{

  

}
*/
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

/*void ObserverCB::observationCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data){



}*/
