#ifndef ADAPTIVEEKF_H
#define ADAPTIVEEKF_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "UBLOX.h"

#include "Eigen.h"
#include <cmath>
#include <Eigen/LU>

using Eigen::MatrixXd;
using Eigen::VectorXd;

extern const double R; // Radius of the earth

const double alpha = 0.1; // Decay const / learning rate
const double angOffsetR = 5; // IMU has a 5 degree offset in the roll direction (whoops)

class ekfObject {

//  Matrix members, commented matrices are defined within functions and destroyed after to save space
  VectorXd x_ka; // Optimal x at time k
  MatrixXd P_ka; // Optimal P at time k
//  MatrixXd K_k; // Khalman gain

  MatrixXd Q_k; // Process noise covariance
  MatrixXd R_k; // Measurement noise covariance
//  VectorXd d_k; // Innovation 
  VectorXd e_k; // Residual

//  MatrixXd A; // State transition matrix
//  MatrixXd B; // Control matrix
  MatrixXd C; // Output matrix
//  VectorXd G; // Gravity vector

//  VectorXd u_k; // Input vector
//  VectorXd Z_k; // Measurement vector

//  MatrixXd Rbgp; // Body to global rotation matrix
//  MatrixXd J_f; // Jacobian of the state transition matrix
  MatrixXd J_h; // Jacobian of the output matrix

//  VectorXd x_kf; // Forecast x value 
//  MatrixXd P_kf; // Forecast P value
  
  double lat0; // Lattitude at origin
  double lon0; // Longitude at origin

  VectorXd state; // Full state: x, y, z, xd, yd, zd, roll, pitch, yaw, p, q, r, zdd

//  double roll;
//  double pitch;
//  double yaw;
  
  unsigned long lastUpdateT; // Last time it was updated;

  public:
    bool ekfInit();
    bool ekfUpdate();
    VectorXd ekfGetState(){ return state; };
    
};

#endif
