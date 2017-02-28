#include <stdint.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <list>
#include <iomanip>

#include <xbeep.h>

#include "remotenode.h"
#include "atcon.h"

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

	//print entire packet
	for (int i = 0; i < (*pkt)->size(); i++) {
		//std::cout << (**pkt)[i];
		printf("%02x ", (**pkt)[i]);
	}
	std::cout << "\n";
	
	std::vector<unsigned char> data = (*pkt)->getVector(); 

	if (data.size() == 0) {	
		/* an AT response, with zero data length indicates that the scan is complete */
		std::cout << "Scan Complete!\n";
		this->node_detect_complete = true;
		return;
	}

	if (data.size() < 11) {	
		/* ensure that we have enough data */
		std::cout << "Received small packet...\n";
		return;
	}

	/* push a new remotenode object into our list */
	this->node_list.push_back(remotenode(data));

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
