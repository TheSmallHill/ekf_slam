#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>

#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"

class Slam{

	public:
		Slam(double*, double, double);
		~Slam();		
		void getObservations();
		void dataAssociateKnown(Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor>, Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>, Eigen::Matrix<int, Eigen::Dynamic, 1, Eigen::RowMajor>);
		void predict(double, double);		
		void update(Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor>, Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>);
		void augment();
		void storeData();

		Eigen::Matrix<int, 1, Eigen::Dynamic, Eigen::RowMajor> da_table;

	protected:
		
	private:
		void choleskyUpdate();
		Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor> observeModel(Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor>, int, int, double*);		
		
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Pcov;
		double sigmaD, sigmaV, sigmaW, sigmaR, sigmaB, length, timeStep, Vn, Wn;
		double* pose[3];
		Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Q, R;
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> zf, idf, zn; 

};
