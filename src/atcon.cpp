#include "atcon.h"

//using namespace libxbee;
//using namespace std;

void atcon::xbee_conCallback(libxbee::Pkt **pkt) {
	if (!this->node_detect_complete) {
		if ((*pkt)->getATCommand() != "ND") {
			/* print an error if we thought we were in the middle of an 'ND' command... */
			std::cout << "Early exit of Node Detect...\n";
			this->node_detect_complete = true;
		}
	}

	if (this->node_detect_complete) {
		/* don't print anything if we're not interested */
		return;
	}

	//std::vector<unsigned char> data = (*pkt)->getVector(); 
	//std::string data = (*pkt)->getData();

	//if (data.size() == 0) {
	if ((*pkt)->size() == 0) {	
		/* an AT response, with zero data length indicates that the scan is complete */
		std::cout << "Scan Complete!\n";
		this->node_detect_complete = true;
		return;
	}

	//if (data.size() < 11) {
	if ((*pkt)->size() < 11) {	
		/* ensure that we have enough data */
		std::cout << "Received small packet...\n";
		return;
	}

	/* push a new remotenode object into our list */
	//this->node_list.push_back(remotenode(data));
	//this->name_list.pushback(&((*pkt)->data[10]));
	//this->rssi_list.pushback((*pkt)->data[10]);
	printf("%20s - %02X\n", &((*pkt)[10]), (**pkt)[10]);
}

void atcon::start_node_detect(void) {
	/* clear the list of it's previous results */
	this->node_list.clear();

	/* mark the in-progress flag */
	this->node_detect_complete = false;

	/* and execute the 'ND' command */
	try {
		*this << "ND";
	} catch (xbee_err err) {
		/* we expect the ND command to return -17 / XBEE_ETIMEOUT because it's a long running operation */
		if (err != -17) throw;
	}
}
