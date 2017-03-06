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

class rosMatlab
{
public:
	rosMatlab(BlackLib::BlackGPIO* stepPin, BlackLib::BlackGPIO* directionPin, BlackLib::BlackGPIO* ms1Pin, BlackLib::BlackGPIO* ms2Pin, BlackLib::BlackGPIO* ms3Pin) : stepPin(*stepPin), directionPin(*directionPin), ms1Pin(*ms1Pin), ms2Pin(*ms2Pin), ms3Pin(*ms3Pin)
	{

ROS_INFO("constructor");
 // BlackLib::BlackGPIO step(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
 // BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);
 // BlackLib::BlackGPIO ms1(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
 // BlackLib::BlackGPIO ms2(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
 // BlackLib::BlackGPIO ms3(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);
  
//ROS_INFO("pins");

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

ROS_INFO("initialization");

observationResponse = thisNode.advertise<ekf_slam::RadioScan>("observeResponse",1);
observationRequest = thisNode.subscribe("observeRequest", 1, &rosMatlab::observationRequestCB, this);

ROS_INFO("publisher and subscriber");


	}

	void observationRequestCB(const std_msgs::Bool& msg) {

//	ros::NodeHandle thisNode;

ROS_INFO("callback start");
 ekf_slam::RadioScan temp; 
double tempData[40][4];
	std::vector<obs> obsRow;
temp.angle_min = 0;
temp.angle_max = 360;

 //do a full set of observations (using a while loop) and save data
for (int i = 0; i < 4; i++) {
//      obsRow = xbee->doObservation(M1->getAng());

        int j = 0;

/*      for(std::vector<obs>::iterator it = obsRow.begin(); it != obsRow.end();)

                tempData[i][j] = it->rssi;      

        }       */

	ROS_INFO("incrementing motor");
        M1->incrementMotor(80);

}
ROS_INFO("sending");
observationResponse.publish(temp);
ROS_INFO("sent");
	}

private:
	ros::NodeHandle thisNode;
	ros::Publisher observationResponse;
	ros::Subscriber observationRequest;	
	BlackLib::BlackGPIO& stepPin;
        BlackLib::BlackGPIO& directionPin;
        BlackLib::BlackGPIO& ms1Pin;
        BlackLib::BlackGPIO& ms2Pin;
        BlackLib::BlackGPIO& ms3Pin;
	Motor *M1;

};

/* Define pins for Motor object */	
  //BlackLib::BlackGPIO step(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
  //BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);
  //BlackLib::BlackGPIO ms1(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
  //BlackLib::BlackGPIO ms2(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
  //BlackLib::BlackGPIO ms3(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);
  
  /* Create Motor object */
  //Motor *M1 = new Motor(&step, &direc, &ms1, &ms2, &ms3);

 // Observer *xbee = new Observer();

/* this callback will happen every time you publish to its associated topic */
/*void observationRequestCB(const std_msgs::Bool msg) {
ROS_INFO("Entered the callback\n"); 

//make a ros temp variable for data
//Eigen::Matrix<double, 40, 4, Eigen::RowMajor> tempData;
std::vector<obs> obsRow;
temp.header.angle_min = 0;
temp.header.angle_max = 360;

 //do a full set of observations (using a while loop) and save data
for (int i = 0; i < 40; i++) {

//	obsRow = xbee->doObservation(M1->getAng());
	
	int j = 0;

	for(std::vector<obs>::iterator it = obsRow.begin(); it != obsRow.end(); ++it) {

		tempData[i][j] = it->rssi;    	

	}	

	M1->incrementMotor(80);

} */
  
//reorder data as necessary (basic data association)
  

 //put data into ros temp variable 
//for( int v=0,i=0;v<40;v++)
//{
  //for ( int u = 0; u<4;u++,i++)
   //{
	//double temp2 = tempData[u][v];
	//fprintf(stdout, "%lf ", temp2 );
    	//fprintf(stdout,"data ");
	//pcl::PointXYZ result;
    //result.x = (u-cx)*(RawDepthtoMeters(depth[i]) + minDistance*scaleFactor*(cx/cy);
    //result.y = (v-cy)*(RawDepthtoMeters(depth[i]) + minDistance*scaleFactor;
    //result.z = RawDepethtoMeters(depth[i]);
    //msg->points.push.back(result);
   //}
//fprintf(stdout, "\n");
//}

 //publish data to the scans topic
//}

int main(int argc, char** argv) {

ROS_INFO("start of main\n");
  /* Motor object initializations */
  /*M1->pos=-180;
  M1->posMax = 180;
  M1->posMin = -180;
  M1->direction = 0;
  M1->ms[0] = 0;
  M1->ms[1] = 0;
  M1->ms[2] = 0;	
*/
  /* create a new observer object */

//ROS_INFO("motor setup\n");  

 // initialize the ros node
  ros::init(argc, argv, "observerNode");
  
ROS_INFO("ros::init done\n");
  // node handle
  //ros::NodeHandle thisNode;
  
  // publishers and subscribers
  //ros::Subscriber observationRequest = thisNode.subscribe("observeRequest", 1000, observationRequestCB);
  //ros::Publisher observationResponse = thisNode.advertise<ekf_slam::RadioScan>("observeResponse", 50);

BlackLib::BlackGPIO step(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);
BlackLib::BlackGPIO ms1(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
BlackLib::BlackGPIO ms2(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
BlackLib::BlackGPIO ms3(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);

	rosMatlab RMobject(&step, &direc, &ms1, &ms2, &ms3);

ROS_INFO("object created\n"); 

  // start doing things
  ros::spin();
 
  return(0);
}
