#include "Motor.h"
#include "Observer.h"

using namespace std;
using namespace BlackLib;

int main(void){

/* Define pins for Motor object */	
BlackLib::BlackGPIO step(BlackLib::GPIO_30, BlackLib::output, BlackLib::FastMode);
BlackLib::BlackGPIO direc(BlackLib::GPIO_31, BlackLib::output, BlackLib::FastMode);
BlackLib::BlackGPIO ms1(BlackLib::GPIO_39,BlackLib::output,BlackLib::FastMode);
BlackLib::BlackGPIO ms2(BlackLib::GPIO_35,BlackLib::output,BlackLib::FastMode);
BlackLib::BlackGPIO ms3(BlackLib::GPIO_67,BlackLib::output,BlackLib::FastMode);

/* Create Motor object */
Motor *M1 = new Motor(&step, &direc, &ms1, &ms2, &ms3);

/* Motor object initializations */
M1->pos=0;
M1->posMax = 90;
M1->posMin = -90;
M1->direction = 0;
M1->ms[0] = 0;
M1->ms[1] = 0;
M1->ms[2] = 0;	



return(0);
	
}
