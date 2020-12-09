#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#define DEARM false

#include <Wire.h>

#include "Eigen.h"
#include <cmath>
#include <Eigen/LU>

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double throttleLimit = 0.5;

class motorObject {
  int motorPort;
  int motorValue;
  int offsetValue;
  
  public:

    bool motorInit(const int &portIn, const int &offsetIn);
    void motorUpdate(const double &throttleIn);
    int motorOut(){ return motorValue; };
};

class motorControllerObject {
  
  VectorXd motorsOut;
  MatrixXd T2M;

  motorObject mo1;
  motorObject mo2;
  motorObject mo3;
  motorObject mo4;

  VectorXd torquesToThrottle(const VectorXd &torques);
  
  public:

    bool motorControllerInit();
    void motorControllerUpdate(const VectorXd &torques, bool armed);
    VectorXd motorControllerOut(){ return motorsOut; };
    VectorXd motorPwmOut();
};

#endif
