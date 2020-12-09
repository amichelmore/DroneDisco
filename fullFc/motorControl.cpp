#include "motorControl.h"

// Motor functions
bool motorObject::motorInit(const int &portIn, const int &offsetIn){
  
  // Initialize motor
  motorPort = portIn;
  offsetValue = offsetIn;

  // Set resolution, set frequency, set speed to 0
  analogWriteResolution(12);
  analogWrite(motorPort, 0); // Set duty cycle to 0 before changing frequecy for safety
  analogWriteFrequency(motorPort, 12000);
  analogWrite(motorPort, 2047);

  return true;
}

void motorObject::motorUpdate(const double &throttleIn){

  // Set speed to desired speed
  int motorValue = map(throttleIn, 0, 1, 2047, 4095) + offsetValue; 
  analogWrite(motorPort, motorValue);
  
}

// Motor controller functions
bool motorControllerObject::motorControllerInit(){
  
  // Initialize matrices
  motorsOut = VectorXd::Zero(4);
  
  // Initialize motors (motors 1, 2, 4, have a offset for some reason
  mo1.motorInit(22, 284);
  mo2.motorInit(14, 284);
  mo3.motorInit(11, 0);
  mo4.motorInit(3, 284);

  // Initialize T2M matrix from parameters
  double k = 5; // in kg * m / s^2
  double b = 5;
  double lr1 = 0.043; // in meters
  double lr2 = 0.053;
  double lp1 = 0.057;
  double lp2 = 0.055;

  MatrixXd M2T(4,4);
  M2T << -k*lr1, -k*lr2, k*lr2, k*lr1,
         k*lp1, -k*lp2, -k*lp2, k*lp1,
         b, -b, b, -b,
         k, k, k, k;

  T2M = M2T.inverse();

  return true;
}

void motorControllerObject::motorControllerUpdate(const VectorXd &torques, bool armed){
  // convert torques to motor thrusts
  motorsOut = torquesToThrottle(torques);
  
  // Update motor speeds
  #if DEARM
    armed = 0;
  #endif

  // Check if the motors are armed
  if(!armed){
    motorsOut = VectorXd::Zero(4);
  }

  //Update motors
  mo1.motorUpdate(motorsOut(0));
  mo2.motorUpdate(motorsOut(1));
  mo3.motorUpdate(motorsOut(2));
  mo4.motorUpdate(motorsOut(3));
  
}

VectorXd motorControllerObject::torquesToThrottle(const VectorXd &torques){
  
  // Multiply by T2M matrix
  VectorXd throttle(4);
  throttle = T2M * torques;
  throttle = throttle.array() + 0.05; // adding y = mx + b offset in a really weird way 

  // Keep within throttle bounds.
  for(int i = 0; i < 4; i++){
    if( throttle(i) > 1 ){
      throttle(i) = 1;
    } else if( throttle(i) < 0 ){
      throttle(i) = 0;
    }
  }
  
  return throttle;
}

VectorXd motorControllerObject::motorPwmOut(){
  VectorXd pwmOut(4);
  pwmOut << mo1.motorOut(), mo2.motorOut(), mo3.motorOut(), mo4.motorOut();

  return pwmOut;
}
