/*******************************************************************************************************************************
 * ekf_slam - a work-in-progress ROS node for an Extended Kalman Filter SLAM (Simultaneous Localization and Mapping) algorithm *
 * on a Pioneer 3-DX                                                                                                           *
 *   Copyright (C) 2017  TheSmallHill                                                                                    *
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

#include "slamLib.h"

using namespace std;

int main(void){

// do some stuff here
initializeGPIO();

//int steps = 1653;
//int dir = 1;
float angle = 0;

struct timespec tim, tim1;
tim.tv_sec = 0;
tim.tv_nsec = 500000000L;

int rtn;

for (int i = 0; i < 20; i++) {
	rtn = incrementMotor(80, 1, angle);
	cout << "step: " << i << " angle: " << angle << endl;
	rtn = nodeDetect();
	cout << "done detecting" << endl;
//	nanosleep(&tim, &tim1);
}	

/*for (int j = 0; j < 5; j++){
for (int i = 0; i<20; i++){
	rtn = incrementMotor(80, 0, angle);
	cout << "step: " << i << " angle: " << angle << endl;
	nanosleep(&tim, &tim1);
}

for (int i = 0; i <20;i++){
	rtn = incrementMotor(80,1,angle);
	cout << "step: " << i << " angle: " << angle << endl;
	nanosleep(&tim, &tim1);

}
}*/

//rtn = rotate2Angle(angle, 0);

return rtn;
}
