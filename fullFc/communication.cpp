#include "communication.h"

SBUS evo(Serial4);

bool rcObject::rcInit(){
  Wire.begin();
  evo.begin();
  rcOutput = VectorXd::Zero(6);
  rcOutput(4) = 1;

  return true;
}

void rcObject::rcUpdate(){
  if(evo.read(&channels[0], &failSafe, &lostFrame)){
    VectorXd rcInput(6);
    rcInput << channels[0], channels[1], channels[2], channels[3], channels[4], channels[5];

    // Converting button input into binary out
    int buttonOut = 0;
    if(rcInput(5) > rcMid) {
      buttonOut = 1;
    }

    // Getting desired roll, pitch, yaw, thrust
    VectorXd anglesOut(4);
    anglesOut = rcToAngles(rcInput);
    
    rcOutput << anglesOut.head(2), anglesOut(3), anglesOut(2), getMode(rcInput), buttonOut;
  }
}

VectorXd rcObject::rcToAngles(const VectorXd &rcInput){

  VectorXd anglesOut(4);
  //anglesOut = ((rcInput.head(4).array() - rcMid) / 799);
  
  // Add a dead zone
  for(int i = 0; i < 4; ++i){
    anglesOut(i) = (rcInput(i) - rcMid) / 799;
    if(std::abs(anglesOut(i)) < 0.014){
      anglesOut(i) = 0;
    }
  }
  
  anglesOut(0) *= maxAngle;
  anglesOut(1) *= -maxAngle;
  anglesOut(2) *= maxHeightChange;
  anglesOut(3) *= maxYawChange;

  return anglesOut;
}
