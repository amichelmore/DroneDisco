#ifndef FULL_H
#define FULL_H

#define DEBUG true
#define PIDTUNE true
#define RPYTTUNE 0 // 0 for roll, 1 for pitch, 2 for yaw, 3 for height
const double alphaPID = 0.05; // learning rate for the PIDs

#include "adaptiveEkf.h"
#include "pidController.h"
#include "communication.h"
#include "motorControl.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Consts
extern const double mass = 0.388; // Mass of Drone (Kg?) !!! maybe need to convert units
extern const double R = 6371000; // Radius of the earth (m)

#endif
