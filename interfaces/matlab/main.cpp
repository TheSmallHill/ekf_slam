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

class rosMatlab
{
	public:
	/* Constructor */
	//rosMatlab(BlackLib::BlackGPIO* stepPin, BlackLib::BlackGPIO* directionPin, BlackLib::BlackGPIO* ms1Pin, BlackLib::BlackGPIO* ms2Pin, BlackLib::BlackGPIO* ms3Pin) : stepPin(*stepPin), directionPin(*directionPin), ms1Pin(*ms1Pin), ms2Pin(*ms2Pin), ms3Pin(*ms3Pin)
	rosMatlab(Motor* M1, Observer* xbee) : M1(*M1), xbee(*xbee)
	{

		 /* Create Motor object */
 		//M1 = new Motor(stepPin, directionPin, ms1Pin, ms2Pin, ms3Pin);

		//ROS_INFO("Motor object created");

		//M1->pos=-180;
 		//M1->posMax = 180;
  		//M1->posMin = -180;
  		//M1->direction = 0;
  		//M1->ms[0] = 0;
  		//M1->ms[1] = 0;
  		//M1->ms[2] = 0;

		/* Create Observer object */		
		//xbee = new Observer();
		//ROS_INFO("Observer object created");

		/* ROS publishers and subscribers */
		observationResponse = thisNode.advertise<ekf_slam::RadioScan>("observeResponse",1);
		observationRequest = thisNode.subscribe("observeRequest", 1, &rosMatlab::observationRequestCB, this);
		shutdownRequest = thisNode.subscribe("shutdownRequest", 1, &rosMatlab::shutdownRequestCB, this);
		ROS_INFO("Publisher and subscribers created");

	}

	/* Destructor */	
	//~rosMatlab(){
		//ROS_INFO("destructed objects");
		//delete M1;
		//delete xbee;
	//}

	/* Shutdown callback, to shutdown from remote node*/
	void shutdownRequestCB(const std_msgs::Bool& msg) {

		ROS_INFO("Shutdown request received, shutting down");
		ros::shutdown();

	}

	void observationRequestCB(const std_msgs::Bool& msg) {

		ROS_INFO("callback start");
 		ekf_slam::RadioScan dataSet;
		ekf_slam::RadioResponse packet;
		ekf_slam::RadioObservation measurement;
		//double tempData[40][4];
		std::vector<obs> obsRow;
		std::vector<std::vector<obs>> obsArray;
		dataSet.angle_min = 0;
		dataSet.angle_max = 360;
		dataSet.angle_increment = 9; 

		 //do a full set of observations (using a while loop) and save data
		for (int i = 0; i < 41; i++) {
			obsRow = xbee->doObservation(9*i);
			obsArray.push_back(obsRow);
			ROS_INFO("incrementing motor");
	        	M1->incrementMotor(80);

			if(M1->getAng() >= 180) break;
		}

		/* @TODO fix copying issues so data is actually transferred */
		std::vector<std::vector<obs>>::iterator row;
		std::vector<obs>::iterator col;
		int angleCount = 0;
		for(row = obsArray.begin(); row!=obsArray.end();row++){
			for(col = row->begin(); col!=row->end(); col++){
				if(col == row->begin()){
					std::cout << "| Angle = " << angleCount;
					measurement.angle = angleCount;
					angleCount = angleCount + 9;
				}
				std::cout << "| ID = " << col->name << "| RSSI = " << std::dec << col->rssi;
				ekf_slam::RadioResponse packet;
				packet.name = col->name;
				packet.rssi = col->rssi;

				measurement.observation.push_back(packet);
			}
			std::cout << "" << std::endl;
			dataSet.observations.push_back(measurement);
		}

		ROS_INFO("sending");
		observationResponse.publish(dataSet);
		ROS_INFO("sent");
	}

	void spin() {
	
		ros::spin();
	
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
		//Motor *M1;
		//Observer *xbee;

};

int main(int argc, char** argv) {

{ // open a scope within main

 	// initialize the ros node
  	ros::init(argc, argv, "observerNode");

	ROS_INFO("ROS initialized");

	/* Create motor object to pass */
	BlackLib::BlackGPIO step(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
	BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);
	BlackLib::BlackGPIO ms1(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
	BlackLib::BlackGPIO ms2(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
	BlackLib::BlackGPIO ms3(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);
	Motor *M1 = new M1(&step, &direc, &ms1, &ms2, &ms3);
	
	/* Create observer object to pass */
	Observer *xbee = new Observer();
	
	/* start ROS node */
	rosMatlab RMobject(M1, xbee);
	ROS_INFO("Node started successfully");

	/* start spinning threads */
	RMobject->spin();

} // all objects out of scope after this (destructors called and memory freed)

std::cout << "Destruction complete" << std::endl;
return(0);
}
