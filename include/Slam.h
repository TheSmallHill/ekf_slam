#pragma once
#include <iostream>
#include <cstdint>
#include <stdint.h>
#include <Eigen/Eigen> /* Eigen library */

class Slam{
  
  public:
    void getObservations();
    void dataAssociateKnown();
    void update();
    void augment();
    void storeData();
  
  protected:  
    
  private: 
    Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> Pcov;  // Fully dynamic, row major (heap allocation) 
  
};
