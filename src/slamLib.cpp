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

//when putting this onto beaglebone, rewrite these next 2 lines (don't include them and instead do it like BlackLib::BlackGPIO or whatever for each function, should avoid issues that way)
using namespace std;
//using namespace BlackLib;

//int countResp;
//int rows_temp = 1;
//extern unsigned char* temp[6];

//change these to whatever GPIO is desired

int initializeUART(){
	// initialized by user before running program? or use BlackLib if it does the same thing
	return(0);
}

int initializeGPIO(){

/*BlackLib::BlackGPIO step(BlackLib::GPIO_51,BlackLib::output,BlackLib::FastMode);
BlackLib::BlackGPIO enable(BlackLib::GPIO_22,BlackLib::output,BlackLib::FastMode);
BlackLib::BlackGPIO direction(BlackLib::GPIO_60,BlackLib::output,BlackLib::FastMode);
*/
//	BlackGPIO step(GPIO_39,output,fastmode);
//	BlackGPIO enable(GPIO_67,output,fastmode);
//	BlackGPIO direc(GPIO_35,output,fastmode);

BlackLib::BlackGPIO step(BlackLib::GPIO_30,BlackLib::output,BlackLib::FastMode);
BlackLib::BlackGPIO enable(BlackLib::GPIO_60,BlackLib::output,BlackLib::SecureMode);
BlackLib::BlackGPIO direc(BlackLib::GPIO_31,BlackLib::output,BlackLib::FastMode);

	enable.setValue(BlackLib::high);
	
	step.setValue(BlackLib::low);

	direc.setValue(BlackLib::low);	

//	string pinValue = enable.getValue();

//	cout << pinValue << endl;	

//	enable.setValue(BlackLib::low); //enable is active low on the motor driver

//	pinValue = enable.getValue();

//	cout << pinValue << endl;

	return(0);

}

int incrementMotor(int steps, int dir, float &angle) {
		
	BlackLib::BlackGPIO step(BlackLib::GPIO_30,BlackLib::output,BlackLib::FastMode);
	BlackLib::BlackGPIO enable(BlackLib::GPIO_60,BlackLib::output,BlackLib::SecureMode);
	BlackLib::BlackGPIO direc(BlackLib::GPIO_31,BlackLib::output,BlackLib::FastMode);

	struct timespec tim, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = 1000000L;

//set direction based on input
	if (dir == 0) {
		direc.setValue(BlackLib::low); //rotate clockwise
	} else if (dir == 1) {
		direc.setValue(BlackLib::high); //rotate counterclockwise
	}
	
	enable.setValue(BlackLib::low);

	for(int iter=0; iter<steps; iter++){

		//cout << iter << endl;

		step.setValue(BlackLib::high);
		nanosleep(&tim, &tim2); //must be at least 1 us according to datasheet, additional time added to account for rise time
		step.setValue(BlackLib::low);
		nanosleep(&tim, &tim2);
	
		//change counter accordingly so the angle is known
		if (dir == 0) {
	
			angle -= 0.1125;
	
		} else if (dir == 1) {
	
			angle += 0.1125;

		}

	}

	enable.setValue(BlackLib::high);

	return(0);

}

int rotate2Angle(float &angle, float desAngle) {

	int dir;

	if (desAngle < angle) {
		dir = 0;
	} else if (desAngle > angle) {
		dir = 1;
	} else {
	dir = 0;
	}

	int steps = abs((desAngle - angle)/0.1125);

	int rtn = incrementMotor(steps, dir, angle);

	return(rtn);

}

float calibrate(float eta, char* name){ // finish this so it retrieves all RSSIs and ids, then checks ids against one provided by the user so the calibration beacon is known

	//node detect
	//find specified ID from name
	//set power level of that beacon to pRef

	float pRef = -60;

	return(pRef);

}

int discoverNodes(float &angle, double* DAtable, double** results, int &results_r, int &results_c){

	double** temp = {0};	

	int rtn = nodeDetect(); //detect nodes, they will be saved to a matrix

	//temp is in this format:
/*
	serial number for beacon 1  |  RSSI for beacon 1
	serial number for beacon 2  |  RSSI for beacon 2
	serial number for beacon 3  |  RSSI for beacon 3	
	...                         |  ...

*/

	//do data association
	dataAssociate(DAtable, results, results_r, results_c, temp, angle);

// format of results matrix: first column is the angle, each column after that is a beacon measurement in rssi. Could also be the same but by row (probably faster this way)
/*
ang0  |  rssi for beacon 0  |  rssi for beacon 1  | ...  |   rssi for beacon M
ang1  |  rssi for beacon 0  |  rssi for beacon 1  | ...  |   rssi for beacon M
ang2  |  rssi for beacon 0  |  rssi for beacon 1  | ...  |   rssi for beacon M
...   |  rssi for beacon 0  |  rssi for beacon 1  | ...  |   rssi for beacon M
andN  |  rssi for beacon 0  |  rssi for beacon 1  | ...  |   rssi for beacon M
*/

//find size of new results matrix, do some math and return the number of beacon measurements added

return(rtn);

}

void dataAssociate(double* DAtable, double** results, int &results_r, int &results_c, double** temp, float &angle){

//find number of rows of temp (the number of beacons found)
int rows_temp = sizeof(temp)/sizeof(temp[0]);
int rows_DAtable = sizeof(DAtable)/sizeof(DAtable[0]);

//int test = length(**temp);

int flag = 0;

//iterate through rows of temp (all the IDs found for the most recent discover)
for (int i=0;i<rows_temp;i++){

	//iterate through rows of DAtable (all previously discovered IDs
	for (int j=0;j<rows_DAtable;j++){
			
		//compare ID from temp to ID in DAtable
		if (temp[i][1] == DAtable[j]) {

			flag = 1;
			break;
		
		} else {

		flag = 0;
		continue;

		}

	}

	//found a matching entry
	if (flag) {

		//do nothing
		continue;

	} else { //if ID in temp is a new discovery, add it to the end of the list, otherwise do not add it to the list

	
		//reallocate size of results (another column for the new feature) and increment column counter	
		results_c++;
		double** temp_results = results;		
		results = (double**)realloc(temp_results,results_r*results_c*sizeof(double));
		
		//initialize new column to a really negative number (so it won't interfere with the range or bearing estimates)
		for (int i=0;i<results_r;i++){

			results[i][results_c] = -1000;

		}

	}

}
	
//add another row to the results matrix	and increment row counter	
results_r++;
double** temp_results = results;
results = (double**)realloc(temp_results,results_r*results_c*sizeof(double));	

//make first column the current angle
results[results_r][0] = angle;

//initialize new row to a really negative value, except the first column which is the angle
for (int i=1;i<results_c;i++){

	results[results_r][i] = -1000;

}

//add measured rssis to new row of results matrix (if there was a new feature, a column must be added as well)
for (int j=1;j<results_c;j++){

	//find temp row that has same ID as first ID in DAtable
	for (int i=0;i<rows_temp;i++) {
	
		//if found, put in first column
		if (DAtable[j-1] == temp[i][1]) {
		
			results[results_r][j] = temp[i][2];	
			break; //move on to the next column of the results matrix		

		} else continue;//if not, continue to the next ID
	
	}

}
	

}

void RSSI2dist(double** results, int rows, int columns, float eta, float pRef){

for (int i = 0; i < rows; i++) { //through rows

	for (int j = 1; j < columns; j++) { //through columns, skip the first column

		// should work
		results[i][j] = pow(10,(pRef-results[i][j])/eta);	


	}

}

}

void findBearing(double** results, int rows, int columns, double* DAtable, double* bearings){

int maxRSSI[columns]; //matrix to save index where max rssi for a beacon is

for (int i=1;i<columns;i++){

	double max_val = -1000; //start at really low rssi so we can find the max
	int max_idx = -1;

	for (int j=0;j<rows;j++){
	
	//find max value in current column, store value and index
		if (results[j][i] > max_val) {
			max_val = results[j][i];
			max_idx = j;
		} else continue;


	}

	//after going through all rows, save index where the max rssi was to another array
	maxRSSI[i-1] = max_idx;

}

//once max values are found, go through matrix and use the previously saved indices to save the angle
for (int i=0;i<columns;i++){

	int idx = maxRSSI[i];	

	bearings[i] = results[idx][i]; // check this

}

//order of bearings array should be the same as the columns of results, CHECK this

}

void findRange(double** results, double* DAtable, double* ranges){

//similar to find bearing, just go through and once the row with the bearing estimate is reached, save the range estimate for the associated beacon

}

void printResults(double* ranges, double* bearings, double* DAtable){

int numElements = sizeof(ranges)/sizeof(*ranges); //check this and modify so it works with pointers

for (int i=0;i<numElements;i++){

printf("%lf	%lf	%lf\n", DAtable[i], ranges[i], bearings[i]);//check this to make sure pointers are correct

}

//print a row for each beacon in this format:
//beacon ID  |  range estimate  |  bearing estimate

}

void saveData(double* ranges, double* bearings, double* DAtable){

//very similar to the print results function except the results are saved to a file instead of printed to the screen

}

void nodeCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data) {
	int i;
//, countResp;


	//int** temp;

	if (strncasecmp(reinterpret_cast<const char*>((*pkt)->atCommand), "ND", 2)) return;
	if ((*pkt)->dataLen == 0) {
		printf("Scan complete!\n");
		sem_post(&ndComplete);
		return;
	}

	if ((*pkt)->dataLen < 11) {
		printf("Received small packet...\n");
		return;
	}

//figure out how to save these to a matrix instead of printing them
	/*                     v   v       v   v   v   v      v   v   v   v       */
	printf("Node: %-20s  0x%02X%02X  0x%02X%02X%02X%02X 0x%02X%02X%02X%02X RSSI: 0x%02X\n",
	       &((*pkt)->data[10]),

	       (*pkt)->data[0],
	       (*pkt)->data[1],

	       (*pkt)->data[2],
	       (*pkt)->data[3],
	       (*pkt)->data[4],
	       (*pkt)->data[5],
	       (*pkt)->data[6],
	       (*pkt)->data[7],
	       (*pkt)->data[8],
	       (*pkt)->data[9],
	       (*pkt)->data[10]);
			
	//maybe save to a global matrix (or extern?) here
//	countResp++;	
//	temp[countResp] = &((*pkt)->data[10]);
//	temp[countResp][2] = (*pkt)->data[10];	
	
	//reallocate the results array provided to the callback function
	//add the newly found node in a new row
	
}

int nodeDetect(void){
	
//	extern double temp[][2];
	//extern int countResp;
	//int** temp[1][2] = {0};

	struct timespec tim, tim2, tim3;
	tim.tv_sec = 7;
	tim.tv_nsec = 0;
	tim3.tv_sec = 10;
	tim3.tv_nsec = 0;

	void *d;
	struct xbee *xbee;
	struct xbee_con *con;
	xbee_err ret;
	unsigned char txRet;
	struct timespec to;

	if (sem_init(&ndComplete, 0, 0) != 0) {
		printf("sem_init() returned an error: %d - %s\n", errno, strerror(errno));
		return -1;
	}
	
//	cout << "first if statement done" << endl;

	if ((ret = xbee_setup(&xbee, "xbee2", "/dev/ttyS2", 9600)) != XBEE_ENONE) {
		printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		return ret;
	}

//	cout << "second if statement done" << endl;

	if ((ret = xbee_conNew(xbee, &con, "Local AT", NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	if ((ret = xbee_conCallbackSet(con, nodeCB, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conCallbackSet() returned: %d", ret);
		return ret;
	}

	if ((ret = xbee_conTx(con, &txRet, "ND")) != XBEE_ENONE && ret != XBEE_ETIMEOUT) {
		xbee_log(xbee, -1, "xbee_conTx() returned: %d-%d", ret, txRet);
		return ret;
	}

	//maybe clear a global matrix here (or extern?)
//	countResp = 0;	

	printf("ND Sent!... waiting for completion\n");
	

	clock_gettime(CLOCK_REALTIME, &to);
	to.tv_sec  += 30;
	if (sem_timedwait(&ndComplete, &to) != 0) {
		if (errno == ETIMEDOUT) {
			printf("Timeout while waiting for ND command to complete...\n");
		} else {
			printf("Error calling sem_timedwait()... sleeping for 30 seconds instead\n");
			nanosleep(&tim, &tim2);
		}
	}

	if ((ret = xbee_conEnd(con)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conEnd() returned: %d", ret);
		return ret;
	}

	xbee_shutdown(xbee);
	nanosleep(&tim3, &tim2);	

return 0;
}

