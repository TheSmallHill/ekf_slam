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

/* classes we made */
#include "Observer.h"
#include "Motor.h"

/* include files for ROS */
#include "ros/ros.h"
#include "std_msgs/

/* Define pins for Motor object */	
  BlackLib::BlackGPIO step(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
  BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);
  BlackLib::BlackGPIO ms1(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
  BlackLib::BlackGPIO ms2(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
  BlackLib::BlackGPIO ms3(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);
  
  /* Create Motor object */
  Motor *M1 = new Motor(&step, &direc, &ms1, &ms2, &ms3);

  /* Motor object initializations */
  M1->pos=-180;
  M1->posMax = 180;
  M1->posMin = -180;
  M1->direction = 0;
  M1->ms[0] = 0;
  M1->ms[1] = 0;
  M1->ms[2] = 0;	

  /* create a new observer object */
  Observer *xbee = new Observer();

/* this callback will happen every time you publish to its associated topic */
void observationRequestCB(const std_msgs::Bool msg) {
 //make a ros temp variable for data
  
 //do a full set of observations (using a while loop) and save data
  
 //reorder data as necessary (basic data association)
  
 //put data into ros temp variable 

 //publish data to the scans topic
  
}

int main(int argc, char **argv) {
/*
  // Define pins for Motor object 	
  BlackLib::BlackGPIO step(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
  BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);
  BlackLib::BlackGPIO ms1(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
  BlackLib::BlackGPIO ms2(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
  BlackLib::BlackGPIO ms3(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);
  
  // Create Motor object 
  Motor *M1 = new Motor(&step, &direc, &ms1, &ms2, &ms3);

  // Motor object initializations 
  M1->pos=-180;
  M1->posMax = 180;
  M1->posMin = -180;
  M1->direction = 0;
  M1->ms[0] = 0;
  M1->ms[1] = 0;
  M1->ms[2] = 0;	

  // create a new observer object 
  Observer *xbee = new Observer();
*/
  
 // initialize the ros node
  ros::init(argc, argv, "observerNode");
  
  // node handle
  ros::NodeHandle thisNode;
  
  // publishers and subscribers
  ros::Subscriber observationRequest = thisNode.subscribe("observerNode", 1000, observationRequestCB);
  ros::Publisher observationResponse = thisNode.advertise<RadioScan>("observation", 50);
 
  // start doing things
  ros::Spin();
 
  return(0);
}
