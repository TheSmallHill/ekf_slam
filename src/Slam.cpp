#include <iostream>
#include <cstdint>
#include <stdint.h>
#include <string.h>
#include <string>

#include "Slam.h"

/* Constructor */
Slam::Slam(double pose[3], double length, double timeStep) //change this constructor so it is like the one for the motor 
{

	/* assume robot starts at exact position */
	Pcov << 0, 0, 0,
		0, 0, 0,
		0, 0, 0;


}

void Slam::getObservations()
{

}

void Slam::dataAssociateKnown(Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor> x, Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor> z, Eigen::Matrix<int, Eigen::Dynamic, 1, Eigen::RowMajor> idz)
{

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> idn;

zf << 0;
zn << 0;
idf << 0;
idn << 0;

for (int i = 0; i < idz.rows(); i++) {

	int ii = idz(i,0);
	if (da_table(0,ii) == 0) {

		zn(0,zn.cols()+1) = z(0,i);
		zn(1,zn.cols()) = z(1,i); 
		idn(0,idn.cols()+1) = ii;

	} else {

		zf(0,zf.cols()+1) = z(0,i);
		zf(1,zf.cols()) = z(1,i);
		idf(0,idf.cols()+1) = da_table(0,ii);

	}

} 

int Nxv = 3;
int Nf = (x.rows() - Nxv)/2;
int idn_curr;
for (int i = 0; i < idn.cols(); i++) {
	idn_curr = idn(0,i);
	for (int j = 1; j < zn.cols()+1; j++) {	
		da_table(0,idn_curr) = Nf + j;
	}
}
}

/* Predict the new state covariance*/
void Slam::predict(double G, double th)
{

	double s = sin(G + th);
	double c = cos(G + th);

	double vts = Vn*timeStep*s;
	double vtc = Vn*timeStep*c;

	Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Fk;
	Eigen::Matrix<double, 3, 2, Eigen::RowMajor> Lk;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp;

	Fk << 1, 0, -vts,
     	      0, 1, vtc,
	      0, 0, 1;

	Lk << timeStep*c, -vts,
     	      timeStep*s, vtc,
	      timeStep*sin(G)/length, Vn*timeStep*cos(G)/length;

	Pcov.block(0,0,3,3) = Fk*Pcov.block(0,0,3,3)*Fk.transpose() + Lk*Q*Lk.transpose();

	if (Pcov.rows() > 3) {

		Pcov.block(0,3,3,Pcov.cols()-3) = Fk*Pcov.block(0,3,3,Pcov.cols()-3);
		temp = Pcov.block(0,3,3,Pcov.cols()-3);
		Pcov.block(3,0,Pcov.rows()-3,3) = temp.transpose();

	}

}

void Slam::update(Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor> x, Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor> z)
{
/*
function [x,P] = batch_update(x,P,z,R,idf)

lenz = size(z,2); % number of measurements
lenx = length(x); % number of states for the robot (x,y,theta)
H = zeros(2*lenz, lenx);
v = zeros(2*lenz, 1);
RR = zeros(2*lenz);

% update everything
for i=1:lenz
    ii = 2*i + (-1:0);
    [zp,H(ii,:)] = observe_model(x, idf(i));
    
    v(ii) = [z(1,i)-zp(1);
    pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii) = R;
end
        
[x,P] = KF_cholesky_update(x,P,v,RR,H);
end
*/

/* fix this assignment, it is invalid*/
const int lenz = z.cols();
const int lenx = z.rows();

//Eigen::Matrix<double, 2*lenz, lenx, Eigen::RowMajor> H;
Eigen::Matrix<double, 1, 1, Eigen::RowMajor> H;
for (int i = 0; i < 2*lenz; i++) {
	for (int j = 0; j < lenx; j++) {
		H(i,j) = 0;
	}
}

//Eigen::Matrix<double, 2*lenz, 1, Eigen::RowMajor> v;
Eigen::Matrix<double, 1, 1, Eigen::RowMajor> v;
for (int k = 0; k < 2*lenz; k++) {
	v(k,0) = 0;	
}

//Eigen::Matrix<double, 2*lenz, 2*lenz, Eigen::RowMajor> RR;
Eigen::Matrix<double, 1, 1, Eigen::RowMajor> RR;
for (int i = 0; i < 2*lenz; i++) {
	for (int j = 0; j < 2*lenz; j++) {
		RR(i,j) = 0;
	} 
} 

int ii;
double zp;
for (int i = 0; i < lenz; i++) {

	for (int j = -1; j < 1; j++) {

		ii = 2*i + j;
		//    [zp,H(ii,:)] = observe_model(x, idf(i)); // write this function		
		H.row(ii) = observeModel(x, ii, idf(i), &zp);

		v(0,ii) = z(0,i) - zp[0];
		v(1,ii) = pi_to_pi(z(1,i) - zp[1]);

		RR(ii,ii) = R(j+1,j+1);	

	}

}

//[x,P] = KF_cholesky_update(x,P,v,RR,H); //write this function

}

Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor> Slam::observeModel(Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor> x, int ii, int idf_i, double* zp)
{

int Nxv = 3;
int fpos = Nxv + idf_i*2 - 1;

Eigen::Matrix<double, 2, x.rows(), Eigen::RowMajor> H;
for (int i = 0; i < 2; i++) {
	for (int j = 0; j < x.rows(); j++) {
		H(i,j) << 0;
	}
}

double dx = x(0,fpos) - x(0,0);
double dy = x(0,fpos+1) - x(0,1);
double d2 = pow(dx,2) + pow(dy,2);
double d = sqrt(d2);
double xd = dx/d;
double yd = dy/d;
double xd2 = dx/d2;
double yd2 = dy/d2;

//predict z
Eigen::Matrix<double, 2, 1, Eigen::RowMajor> z;
z(0,0) = d;
z(1,0) = atan2(dy,dx) - x(2,0);

H.block(0,0,2,3) << -xd, -yd, 0,
     		    yd, -xd2, -1;

H.block(0,fpos,2,2) << xd, yd,
		       -yd2, xd2;

return(H.col(ii));

}

void Slam::choleskyUpdate()
{

}
