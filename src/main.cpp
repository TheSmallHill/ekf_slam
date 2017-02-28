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

#include <stdint.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <list>
#include <iomanip>

#include <xbeep.h>

#include "Motor.h"
#include "Observer.h"
#include "atcon.h"
#include "remotenode.h"

int main(void){

{
/* make a vector of vectors to store results, much easier than a dynamic array */
std::vector<obs> obsRow;
std::vector<std::vector<obs> > obsArray;

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

Observer *xbee = new Observer();

while(1){
	
	obsRow = xbee->doObservation(M1->getAng());
	obsArray.push_back(obsRow);

	M1->incrementMotor(1);

	if (M1->getAng() >= 180) break;

}

/* print out the results */
std::vector<std::vector<obs> >::iterator row;
std::vector<obs>::iterator col;
for(row = obsArray.begin(); row != obsArray.end(); row++){	

	for (col = row->begin(); col!=row->end(); col++) {

		if (col == row->begin()) {

			std::cout << "| Angle = " << col->angle;

		}

		std::cout << "| ID: " << col->name << " RSSI: " << col->rssi;

	}

	std::cout << "|" << std::endl;

}

} // all objects out of scope after this curly bracket

return(0);
	
}
