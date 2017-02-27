#include <xbeep.h>

#include "remotenode.h"

remotenode::remotenode(std::vector<unsigned char> nd_payload) { 
	int i = 0;

	this->addr16  = (nd_payload[i + 0] << 8) & 0xFF00;
	this->addr16 |= (nd_payload[i + 1] << 0) & 0x00FF;
	i += 2;

	this->addr64 = nd_payload[i + 0] & 0xFF;
	this->addr64 = ((this->addr64 << 8) & ~0xFF) | (nd_payload[i + 1] & 0xFF);
	this->addr64 = ((this->addr64 << 8) & ~0xFF) | (nd_payload[i + 2] & 0xFF);
	this->addr64 = ((this->addr64 << 8) & ~0xFF) | (nd_payload[i + 3] & 0xFF);
	this->addr64 = ((this->addr64 << 8) & ~0xFF) | (nd_payload[i + 4] & 0xFF);
	this->addr64 = ((this->addr64 << 8) & ~0xFF) | (nd_payload[i + 5] & 0xFF);
	this->addr64 = ((this->addr64 << 8) & ~0xFF) | (nd_payload[i + 6] & 0xFF);
	this->addr64 = ((this->addr64 << 8) & ~0xFF) | (nd_payload[i + 7] & 0xFF);
	i += 8;

	i += 1;

	//something like this?
	this->rssi = nd_payload[i];
	
	this->name = "";
	for (; i < nd_payload.size(); i++) {
		this->name += char(nd_payload[i]);
	}

	/* find how to extract RSSI from nd_payload, this might work since it is
	 * essentially how it was done in the C implementation... */
	//this->rssi = std::stoi(this->name); 	

}
