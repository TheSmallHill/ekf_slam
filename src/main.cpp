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
