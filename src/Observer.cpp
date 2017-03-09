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

/* observer object constructor */
Observer::Observer() // @TODO add parameters such as UART, baudrate, and serial port
{
  	/* start uart using BlackLib, specified by constructor arguments */
	// may have to change tty port used in BlackLib::BlackUART (currently ttyO#, should be ttyS#)
	uart = new BlackLib::BlackUART(BlackLib::UART2, BlackLib::Baud9600, BlackLib::ParityEven, BlackLib::StopOne, BlackLib::Char8);			
	bool isOpened = uart->open(BlackLib::ReadWrite | BlackLib::NonBlock);

    	if(!isOpened)
    	{
        	std::cout << "UART DEVICE CAN\'T OPEN.;" << std::endl;
        	exit(1);
    	}

  	/* have some of these arguments passed to constructor */
	xbee = new libxbee::XBee("xbee2", "/dev/ttyS2", BAUDRATE);	
	std::cout << "xbee connected" << std::endl;
	con = new atcon(*xbee, "Local AT");	
	std::cout << "new connection made" << std::endl;
	
  	//create new array for results to be stored, only starting point since we do not know how many beacons there are
  	//observedData = new obs*[1];
	//observedData = new std::map<int,obs>;	

}

/* observer object destructor */
Observer::~Observer()
{

  	/* delete objects and arrays created by constructor */
	delete con;
	delete xbee;
	uart->close(); //close connection before deleting
	delete uart;
    	//delete observedData;

}

/* do an observation */
std::vector<obs> Observer::doObservation(float angle)
{
	
	std::vector<obs> tempObservations;
	obs temp;	
	int count = 0;
	
	/* start detecting nodes */ 
	con->start_node_detect();

	/* wait for it to finish */
	while (!con->node_detect_complete) {
		usleep(100000); /* wait for .1 second */
		std::cout << "still waiting" << std::endl;
		if (count > 2) {
		break;
		} else count++;
	}

	//observedData[numObs] = new obs[con->node_list.size()];

	/* save and print out a list of nodes */
	for (std::list<remotenode>::iterator n = con->node_list.begin(); n != con->node_list.end(); n++) {

		/* it's just easier to print nice-looking output using printf()... */
		printf("Node: %-20s  0x%04X  0x%08X 0x%08X\n", n->getName().c_str(), n->getAddr16(), n->getAddr64Hi(), n->getAddr64Lo());	
		
		/* save to another vector so it can be returned */	
		temp.name = n->getName().c_str();
		temp.rssi = n->getRssi();		
		tempObservations.push_back(temp);				

	}  
	
	return(tempObservations); // tempObservations out of scope once this method ends 	

}

/* prepare for a new scan of all angles, may be unnecessary for the Observer class since all results are not kept within the class */
void Observer::newScan()
{

  	//reallocate the observedData array to an empty 1x1, ready for next observations at angle
	//delete observedData;
	//obs** observedData = new obs*[1];  

  	//reset numObservations
  	//this->numObs = 0;

}

/* process the data (find range and bearing) */
 void Observer::processData(std::vector<std::vector<obs> > data)
{

  //iterate through array (convert each element's rssi to distance too) and find the bearing for each beacon
  

}

/* get the pointer to the results array */
/*obs** Observer::getResults()
{

  //return pointer to all the observed data
  return(observedData);

}*/

/* find the bearing for one column */
float Observer::findBearing(/*somehow pass a column?*/)
{

  //find the bearing of the beacon
	

return(0);
}

/* convert rssi to distance */
float Observer::toDistance(obs data)
{

  //convert provided rssi to distance
 
return(0); 
}
