//get rid of duplicates in this list
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <ctype.h>
//#include <semaphore.h>
#include <errno.h>
#include <time.h>
//#include <xbee.h> //check this
//#include "slamLib.h"
//#include "xbeeLib.h"

//#include <stdio.h>
//#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <semaphore.h>
//#include <errno.h>
//#include <time.h>
//#include <unistd.h>

#include <xbeep.h>

#include "../lib/BlackLib/v3_0/BlackLib.h" 

sem_t ndComplete;

int nodeDetect(void); //check this
void nodeCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data); //needs modification so it saves results instead of printing



//sem_t ndComplete; //for using semaphores, check where this should be put

//initializations
int initializeUART(void); //probably can get rid of this, but try to find a way to implement
int initializeGPIO(void); //check this

//motor control functions
int incrementMotor(int steps, int dir, float &angle);//check
int rotate2Angle(float &angle, float desAngle); //check

//measuring distance functions
float calibrate(float eta); //need to finish
int discoverNodes(float &angle, double* DAtable, double** results, int &results_r, int &results_c); //need to finish

void dataAssociate(double* DAtable, double** results, int &results_r, int &results_c, double** temp, float &angle); //need to finish

//data processing functions
void RSSI2dist(double** results, float eta, float pRef);//need to finish
void findBearing(double** results, double* DAtable, double* bearings);//need to finish
void findRange(double** results, double* DAtable, double* ranges);//need to finish
void printResults(double* ranges, double* bearings, double* DAtable);//need to finish
void saveData(double* ranges, double* bearings, double* DAtable);//need to finish

//shutdown functions
int shutdownUART(void);//implement
int shutdownGPIO(void);//implement

//a convenient length function
/*template <signed T, unsigned N, unsigned N1>
int length(T (&)[N][N1])
{
    return N;
}*/
