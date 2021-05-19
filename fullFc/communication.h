#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "SBUS.h"
#include <Wire.h>

#include "Eigen.h"
#include <cmath>
#include <Eigen/LU>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// RC signal bounds
const double rcLow = 193.0;
const double rcMid = 992.0;
const double rcHigh = 1791.0;

// Drone attitude and thrust bounds
const double maxAngle = PI/8;
const double maxHeightChange = 0.1;
const double maxYawChange = PI/16;

class rcObject {
  uint16_t channels[16];
  bool failSafe;
  bool lostFrame;

  VectorXd rcOutput;

  VectorXd rcToAngles(const VectorXd &rcInput);
  int getMode(const VectorXd &rcInput){ return (rcInput(4) / 150) + 1; };
  
  public:

    bool rcInit();
    void rcUpdate();
    VectorXd rcOut(){ return rcOutput; };
};

#endif
