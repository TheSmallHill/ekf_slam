#pragma once
#include <stdint.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <list>
#include <iomanip>

#include <xbeep.h>

#include "remotenode.h"

using namespace libxbee;
using namespace std;

class atcon: public ConCallback {
	public:
		explicit atcon(XBee &parent, string type, struct xbee_conAddress *address = NULL): ConCallback(parent, type, address), node_detect_complete(true) { };

		void xbee_conCallback(Pkt **pkt);

		void start_node_detect(void);
		bool node_detect_complete;
		list<remotenode> node_list;
};
