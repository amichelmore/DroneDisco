#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Eigen.h"
#include <cmath>
#include <Eigen/LU>
#include <Wire.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

extern const double mass; // Mass of drone
const double takeoffGain = 50;

class attitudePidObject {

  //VectorXd stateX;
  
  double heightDesired_d; // height
  VectorXd anglesDesired; // roll, pitch, yaw

  double roll_i;
  double pitch_i;

  double yaw_ref;
  double height_ref;
  
  VectorXd torqueDesired; // roll, pitch, yaw, thrust (output)
  
  MatrixXd PID; // rollP, rollI, rollD; pitchP, pitchI, pitchD; yawP, yawI, yawD; thrustP, thrustI, thrustD

  MatrixXd Rgb; // Rotation from global to body

  unsigned long lastUpdateT; // Last time it was updated;
  
  public:
    
    bool attitudeInit(const VectorXd &stateX);
    void attitudeUpdate(const VectorXd &anglesDesiredIn, const VectorXd &stateX, const bool takeoff);
    VectorXd attitudeOut(){ return torqueDesired; };
    double getPids(int i, int j){ return PID(i,j); };
    void modifyPids(double newConst, int i, int j);

};


#endif
