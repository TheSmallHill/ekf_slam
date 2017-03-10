/*******************************************************************************************************************************
 * ekf_slam - a work-in-progress ROS node for an Extended Kalman Filter SLAM (Simultaneous Localization and Mapping) algorithm *
 * on a Pioneer 3-DX                                                                                                           *
 *   Copyright (C) 2017  TheSmallHill                                                                                          *
 *                                                                                                                             *
 *   This program is free software: you can redistribute it and/or modify                                                      *
 *   it under the terms of the GNU General Public License as published by                                                      *
 *   the Free Software Foundation, either version 3 of the License, or                                                         *
 *   (at your option) any later version.                                                                                       *
 *                                                                                                                             *
 *   This program is distributed in the hope that it will be useful,                                                           *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of                                                            *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                                             *
 *   GNU General Public License for more details.                                                                              *
 *                                                                                                                             *
 *   You should have received a copy of the GNU General Public License                                                         *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.                                                     *
 *******************************************************************************************************************************/

#include <cstdlib>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <list>
#include <iomanip>
#include <iostream>
#include <xbeep.h>
#include <eigen3/Eigen/Eigen>

/* classes we made */
#include "Observer.h"
#include "Motor.h"
#include "remotenode.h"
#include "atcon.h"

/* include files for ROS */
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ekf_slam/RadioScan.h>
#include <ekf_slam/RadioObservation.h>
#include <ekf_slam/RadioResponse.h>
#include <ros/xmlrpc_manager.h>

class rosMatlab
{
	public:
	rosMatlab(BlackLib::BlackGPIO* stepPin, BlackLib::BlackGPIO* directionPin, BlackLib::BlackGPIO* ms1Pin, BlackLib::BlackGPIO* ms2Pin, BlackLib::BlackGPIO* ms3Pin) : stepPin(*stepPin), directionPin(*directionPin), ms1Pin(*ms1Pin), ms2Pin(*ms2Pin), ms3Pin(*ms3Pin)
	{

		ROS_INFO("constructor");
 		

		 /* Create Motor object */
 		M1 = new Motor(stepPin, directionPin, ms1Pin, ms2Pin, ms3Pin);

		ROS_INFO("motor");

		M1->pos=-180;
 		M1->posMax = 180;
  		M1->posMin = -180;
  		M1->direction = 0;
  		M1->ms[0] = 0;
  		M1->ms[1] = 0;
  		M1->ms[2] = 0;

		ROS_INFO("xbee setup");
		xbee = new Observer();

		ROS_INFO("initialization");

		observationResponse = thisNode.advertise<ekf_slam::RadioScan>("observeResponse",1);
		observationRequest = thisNode.subscribe("observeRequest", 1, &rosMatlab::observationRequestCB, this);
		shutdownRequest = thisNode.subscribe("shutdownRequest", 1, &rosMatlab::shutdownRequestCB, this);

		ROS_INFO("publisher and subscribers");

	}
	~rosMatlab(){
		ROS_INFO("destructed objects");
		delete M1;
		delete xbee;
	}

	void shutdownRequestCB(const std_msgs::Bool& msg) {

		ROS_INFO("Shutdown request received, shutting down");
	
		ros::shutdown();

	}

	void observationRequestCB(const std_msgs::Bool& msg) {

		ROS_INFO("callback start");
 		
		ekf_slam::RadioScan dataSet;
		ekf_slam::RadioResponse packet;
		ekf_slam::RadioObservation measurement;
		
		std::vector<obs> obsRow;
		std::vector<std::vector<obs>> obsArray;
		
		dataSet.angle_min = 0;
		dataSet.angle_max = 360;
		dataSet.angle_increment = 9;
		
		int rowInfo[41];
		int maxRowInfo = 0;

		 //do a full set of observations (using a while loop) and save data
		for (int i = 0; i < 41; i++) {
			obsRow = xbee->doObservation(9*i);
			rowInfo[i] = obsRow.size();
			obsArray.push_back(obsRow);
			ROS_INFO("incrementing motor");
	        	M1->incrementMotor(80);

			if(rowInfo[i] > maxRowInfo){
				maxRowInfo = rowInfo[i];
			}
			
			ekf_slam::RadioObservation measurement;
			std::vector<obs>::iterator iter;
			measurement.angle = i*9;
			for(iter =obsRow.begin(); iter !=obsRow.end();iter++){
				packet.name = iter->name;
				packet.rssi = iter->rssi;
				measurement.observation.push_back(packet);
			}
			dataSet.observations.push_back(measurement);
		}

		std::vector<std::vector<obs>>::iterator row;
		std::vector<obs>::iterator col;
		int angleCount = 0;
		for(row = obsArray.begin(); row!=obsArray.end();row++){
			for(col = row->begin(); col!=row->end(); col++){
				if(col == row->begin()){
					std::cout << "| Angle = " << angleCount;
					angleCount = angleCount + 9;
				}
				std::cout << "| ID = " << col->name << "| RSSI = " << std::dec << col->rssi;

			}
			std::cout << "" << std::endl;
		}

		ROS_INFO("sending");
		observationResponse.publish(dataSet);
		ROS_INFO("sent");


	}

	private:
		ros::NodeHandle thisNode;
		ros::Publisher observationResponse;
		ros::Subscriber observationRequest;
		ros::Subscriber shutdownRequest;
		BlackLib::BlackGPIO& stepPin;
        	BlackLib::BlackGPIO& directionPin;
        	BlackLib::BlackGPIO& ms1Pin;
        	BlackLib::BlackGPIO& ms2Pin;
        	BlackLib::BlackGPIO& ms3Pin;
		Motor *M1;
		Observer *xbee;

};

int main(int argc, char** argv) {

{
	ROS_INFO("start of main\n");

	// initialize the ros node
  	ros::init(argc, argv, "observerNode");

	ROS_INFO("ros::init done\n");
  	
	BlackLib::BlackGPIO step(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
	BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);
	BlackLib::BlackGPIO ms1(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
	BlackLib::BlackGPIO ms2(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
	BlackLib::BlackGPIO ms3(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);
	
	rosMatlab RMobject(&step, &direc, &ms1, &ms2, &ms3);

	ROS_INFO("object created\n");

	ros::spin();

} // all objects out of scope

std::cout << "returning 0" << std::endl;

return(0);
}
