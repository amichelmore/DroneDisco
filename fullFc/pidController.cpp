#include "pidController.h"

extern double grav; // Gravity

bool attitudePidObject::attitudeInit(const VectorXd &stateX){
  
  torqueDesired = VectorXd::Zero(4);

  roll_i = 0;
  pitch_i = 0;

  yaw_ref = stateX(8);
  height_ref = stateX(2);

  PID = MatrixXd::Zero(4,3);
  PID << .013, .001, 0.002,
         .01, .001, 0.0028,
         0.004, 0, 0.004*0.3,
         0.8, .01, 0.3;
//    PID << .13, .003, 0.04,
//         .1, .003, 0.028,
//         0.004, 0, 0.004*0.3,
//         8, .1, 3;

  lastUpdateT = millis();
  
  return true;
  
}

void attitudePidObject::attitudeUpdate(const VectorXd &anglesDesiredIn, const VectorXd &stateX, const bool takeoff){

  anglesDesired = anglesDesiredIn.head(3);
  heightDesired_d = anglesDesiredIn(3);

  // Get time elapsed dt
  float dt = (millis() - lastUpdateT)*0.001;
  lastUpdateT = millis();

  // Roll Pitch PID calcs
  double e_roll = anglesDesired(0) - stateX(6);
  roll_i += dt*e_roll - 0.001*roll_i;
  torqueDesired(0) = PID(0,0)*e_roll + PID(0,1)*roll_i - PID(0,2)*stateX(9);

  double e_pitch = anglesDesired(1) - stateX(7);
  pitch_i += dt*e_pitch - 0.001*pitch_i;
  torqueDesired(1) = PID(1,0)*e_pitch + PID(1,1)*pitch_i - PID(1,2)*stateX(10);
  
  // Yaw PID calcs
  yaw_ref += dt*anglesDesired(2);
  double e_yaw = yaw_ref - stateX(8); // yaw error
  if(e_yaw > PI){
    e_yaw -= 2*PI;
  }
  if(e_yaw < PI){
    e_yaw += 2*PI;
  }
  torqueDesired(2) = PID(2,0)*e_yaw - PID(2,2)*stateX(11);

  // Thrust PID calcs
  if(takeoff == 0){
    height_ref -= dt*heightDesired_d;
    double e_height = (height_ref - stateX(2));
    torqueDesired(3) = -((PID(3,0)*stateX(5) + PID(3,1)*e_height - PID(3,2)*stateX(12)) - mass*grav);
  } else if (takeoff == 1){
    torqueDesired(3) = heightDesired_d*takeoffGain;
  }
  
}

void attitudePidObject::modifyPids(double newConst, int i, int j){
  
  // i is the roll, pitch, or yaw, j is the p, i, or d
  PID(i, j) = newConst;
  
}
