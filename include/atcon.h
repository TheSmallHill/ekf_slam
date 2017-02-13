#pragma once
#include <stdint.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <list>
#include <iomanip>
//#include <string>

#include <xbeep.h>

#include "remotenode.h"

//using namespace libxbee;
//using namespace std;

class atcon: public libxbee::ConCallback {
	public:
		explicit atcon(libxbee::XBee &parent, std::string type, struct xbee_conAddress *address = NULL): libxbee::ConCallback(parent, type, address), node_detect_complete(true) { };

		void xbee_conCallback(libxbee::Pkt **pkt);

		void start_node_detect(void);
		bool node_detect_complete;
		std::list<remotenode> node_list; 
		//std::list<string> name_list;
		//std::list<int> rssi_list;
};
