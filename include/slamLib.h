// add #idndef statements where necessary
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <semaphore.h>
#include <xbeep.h>

#include "../lib/BlackLib/v3_0/BlackLib.h" 

// globals
sem_t ndComplete;

// externals
extern BlackLib::BlackGPIO step;
extern BlackLib::BlackGPIO enable;
extern BlackLib::BlackGPIO direc;

// Functions for getting measurements from XBees
int nodeDetect(void); //check this
void nodeCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data); // find a way to save data to a matrix or other data structure

// Initializations
int initializeUART(void); // Unfinished
int initializeGPIO(void); // Cleanup
int initializeXbee(void); // Unfinished, possibly unnecessary if combined with initializeGPIO()

// Motor control functions
int incrementMotor(int steps, int dir, float &angle); // Cleanup
int rotate2Angle(float &angle, float desAngle); // Cleanup

// Measuring distance functions
float calibrate(float eta); // Unfinished
int discoverNodes(float &angle, double* DAtable, double** results, int &results_r, int &results_c); // Unfinished

// Data association
void dataAssociate(double* DAtable, double** results, int &results_r, int &results_c, double** temp, float &angle); // Unfinished

//data processing functions
void RSSI2dist(double** results, float eta, float pRef); // Unfinished
void findBearing(double** results, double* DAtable, double* bearings); // Unfinished
void findRange(double** results, double* DAtable, double* ranges); // Unfinished
void printResults(double* ranges, double* bearings, double* DAtable); // Unfinished
void saveData(double* ranges, double* bearings, double* DAtable); // Unfinished

//shutdown functions
int shutdownUART(void); // Unfinished
int shutdownGPIO(void); // Unfinished
int shutdownXbee(void); // Unfinished
