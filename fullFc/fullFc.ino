#include "fullFc.h"

ekfObject eo;
attitudePidObject apo;
rcObject rco;
motorControllerObject mco;

unsigned long lastUpdateT = millis();
extern double grav = 9.81; // Gravity

void setup() {

  // Start Serial and give some time for esc startup
  Serial.begin(115200);
  delay(5000);

  // Initialize ekf, attitude controller, RC controller, and motor controls
  if(!eo.ekfInit()){
    Serial.println("False");
    while(1);
  }
  eo.ekfUpdate();

  // Initialize atttitude and rc
  apo.attitudeInit(eo.ekfGetState());
  rco.rcInit();
  mco.motorControllerInit();
  
}

void loop() {

  // Update ekf and retrieve state
  eo.ekfUpdate();
  VectorXd state(13);
  state = eo.ekfGetState();

  // Read & convert Rc to desired angles
  rco.rcUpdate();
  VectorXd rcOutput(5);
  rcOutput = rco.rcOut();

  // Some variable declarations for the flight modes
  int flightMode = rcOutput(4);
  VectorXd attitudeIn(4);
  bool armed = false;
  
  // Flight modes
  // Get torques / thrust from PID attitude controller
  switch(flightMode) {
    case 1: // Mode 1: Startup
      // Set motor speeds to 0 and print diagnostics (state, theoretical motor outputs)
      attitudeIn << 0, 0, 0, 0;
      armed = false;
      
      break;

    case 2: // Mode 2: Takeoff
      // set desired roll, pitch, and yaw to 0 and pass takeoff flag
      attitudeIn << 0, 0, 0, rcOutput(3);
      apo.attitudeUpdate(attitudeIn, state, 1);
      armed = true;
      
      break;

    case 3: // Mode 2: normal flight
    
      // set desired roll, pitch, and yaw 
      apo.attitudeUpdate(rcOutput.head(4), state, 0);
      armed = true;
      
      break;

    #if PIDTUNE

      case 6: // P tune
  
        attitudeIn << 0, 0, 0, rcOutput(3);
        apo.attitudeUpdate(attitudeIn, state, 1);
        armed = true;
  
        if((millis() - lastUpdateT) > 100){ // update P value according to user request every 0.1 sec
    
          #if (RPYTUNE == 0 || RPYTUNE == 1)
            if(rcOutput(2) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 0)*(1+alphaPID)), RPYTTUNE, 0);
            }
            if(rcOutput(2) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 0)*(1-alphaPID)), RPYTTUNE, 0);
            }
          #endif
    
          #if (RPYTUNE == 2 || RPYTUNE == 3)
            if(rcOutput(0) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 0)*(1+alphaPID)), RPYTUNE, 0);
            }
            if(rcOutput(0) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 0)*(1-alphaPID)), RPYTUNE, 0);
            }
          #endif 
        }
        armed = true;
        
        break;
        
      case 5: // I tune (if applicable)

        attitudeIn << 0, 0, 0, rcOutput(3);
        apo.attitudeUpdate(attitudeIn, state, 1);
        armed = true;
  
        if((millis() - lastUpdateT) > 100){ // update I value according to user request every 0.1 sec (for only roll and pitch)
    
          #if (RPYTUNE == 0 || RPYTUNE == 1)
            if(rcOutput(2) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 1)*(1+alphaPID)), RPYTTUNE, 1);
            }
            if(rcOutput(2) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 1)*(1-alphaPID)), RPYTTUNE, 1);
            }
          #endif
          
          #if (RPYTUNE == 3)
            if(rcOutput(0) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 1)*(1+alphaPID)), RPYTUNE, 1);
            }
            if(rcOutput(0) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 1)*(1-alphaPID)), RPYTUNE, 1);
            }
          #endif 
        }
        armed = true;
        
        break;
        
      case 4: // D tune

        attitudeIn << 0, 0, 0, rcOutput(3);
        apo.attitudeUpdate(attitudeIn, state, 1);
        armed = true;
  
        if((millis() - lastUpdateT) > 100){ // update D value according to user request every 0.1 sec
    
          #if (RPYTUNE == 0 || RPYTUNE == 1)
            if(rcOutput(2) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 2)*(1+alphaPID)), RPYTTUNE, 2);
            }
            if(rcOutput(2) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 2)*(1-alphaPID)), RPYTTUNE, 2);
            }
          #endif
    
          #if (RPYTUNE == 2 || RPYTUNE == 3)
            if(rcOutput(0) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 2)*(1+alphaPID)), RPYTUNE, 2);
            }
            if(rcOutput(0) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 2)*(1-alphaPID)), RPYTUNE, 2);
            }
          #endif 
        }
        armed = true;
        
        break;
      
      case 7: // P tune
  
        #if RPYTTUNE == 0
          attitudeIn << rcOutput(0), 0, 0, 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTTUNE == 1
          attitudeIn << 0, rcOutput(1), 0, 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTUNE == 2
          attitudeIn << 0, 0, rcOutput(2), 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTTUNE == 3
          attitudeIn << 0, 0, 0, rcOutput(3);
          apo.attitudeUpdate(attitudeIn, state, 0);
        #endif
  
        if((millis() - lastUpdateT) > 100){ // update P value according to user request every 0.1 sec
    
          #if (RPYTUNE == 0 || RPYTUNE == 1)
            if(rcOutput(2) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 0)*(1+alphaPID)), RPYTTUNE, 0);
            }
            if(rcOutput(2) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 0)*(1-alphaPID)), RPYTTUNE, 0);
            }
          #endif
    
          #if (RPYTUNE == 2 || RPYTUNE == 3)
            if(rcOutput(0) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 0)*(1+alphaPID)), RPYTUNE, 0);
            }
            if(rcOutput(0) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 0)*(1-alphaPID)), RPYTUNE, 0);
            }
          #endif 
        }
        armed = true;
        
        break;
        
      case 8: // I tune (if applicable)

        #if RPYTTUNE == 0
          attitudeIn << rcOutput(0), 0, 0, 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTTUNE == 1
          attitudeIn << 0, rcOutput(1), 0, 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTUNE == 2
          attitudeIn << 0, 0, rcOutput(2), 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTTUNE == 3
          attitudeIn << 0, 0, 0, rcOutput(3);
          apo.attitudeUpdate(attitudeIn, state, 0);
        #endif
  
        if((millis() - lastUpdateT) > 100){ // update I value according to user request every 0.1 sec (for only roll and pitch)
    
          #if (RPYTUNE == 0 || RPYTUNE == 1)
            if(rcOutput(2) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 1)*(1+alphaPID)), RPYTTUNE, 1);
            }
            if(rcOutput(2) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 1)*(1-alphaPID)), RPYTTUNE, 1);
            }
          #endif
          
          #if (RPYTUNE == 3)
            if(rcOutput(0) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 1)*(1+alphaPID)), RPYTUNE, 1);
            }
            if(rcOutput(0) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 1)*(1-alphaPID)), RPYTUNE, 1);
            }
          #endif 
        }
        armed = true;
        
        break;
        
      case 9: // D tune

        #if RPYTTUNE == 0
          attitudeIn << rcOutput(0), 0, 0, 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTTUNE == 1
          attitudeIn << 0, rcOutput(1), 0, 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTUNE == 2
          attitudeIn << 0, 0, rcOutput(2), 0;
          apo.attitudeUpdate(attitudeIn, state, 0);
        #elif RPYTTUNE == 3
          attitudeIn << 0, 0, 0, rcOutput(3);
          apo.attitudeUpdate(attitudeIn, state, 0);
        #endif
  
        if((millis() - lastUpdateT) > 100){ // update D value according to user request every 0.1 sec
    
          #if (RPYTUNE == 0 || RPYTUNE == 1)
            if(rcOutput(2) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 2)*(1+alphaPID)), RPYTTUNE, 2);
            }
            if(rcOutput(2) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 2)*(1-alphaPID)), RPYTTUNE, 2);
            }
          #endif
    
          #if (RPYTUNE == 2 || RPYTUNE == 3)
            if(rcOutput(0) > 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 2)*(1+alphaPID)), RPYTUNE, 2);
            }
            if(rcOutput(0) < 0){
              apo.modifyPids((apo.getPids(RPYTTUNE, 2)*(1-alphaPID)), RPYTUNE, 2);
            }
          #endif 
        }
        armed = true;
        
        break;
        
    #endif
    
    default:
      // Pass desired inputs normally (Later this will be an error mode)
      // Set desired roll, pitch, and yaw, and height change
      apo.attitudeUpdate(rcOutput.head(4), state, 0);
      armed = true;
      
      break; 
  }

  // Convert torques & thrust to Motor input and command motors
  VectorXd attitudeOut(4);
  attitudeOut = apo.attitudeOut();
  mco.motorControllerUpdate(attitudeOut, armed);

  // Print out debug messages if DEBUG is defined
  #if DEBUG
    VectorXd motorOutputs;
    motorOutputs = mco.motorControllerOut();
  
    if((millis() - lastUpdateT) > 100){
  
      Serial.print("x: ");
      Serial.print(state(0));
      Serial.print(", y: ");
      Serial.print(state(1));
      Serial.print(", z: ");
      Serial.print(state(2));
      Serial.print(", xd: ");
      Serial.print(state(3));
      Serial.print(", yd: ");
      Serial.print(state(4));
      Serial.print(", zd: ");
      Serial.print(state(5));
      Serial.println();
      
      Serial.print("roll: ");
      Serial.print(state(6));
      Serial.print(" pitch: ");
      Serial.print(state(7));
      Serial.print(" yaw: ");
      Serial.print(state(8));
      Serial.print(" p: ");
      Serial.print(state(9));
      Serial.print(" q: ");
      Serial.print(state(10));
      Serial.print(" r: ");
      Serial.print(state(11));
      Serial.println();

      Serial.print("m1: ");
      Serial.print(motorOutputs(0));
      Serial.print(" m2: ");
      Serial.print(motorOutputs(1));
      Serial.print(" m3: ");
      Serial.print(motorOutputs(2));
      Serial.print(" m4: ");
      Serial.print(motorOutputs(3));
      Serial.println();

      Serial.print("rollDes: ");
      Serial.print(rcOutput(0));
      Serial.print(" pitchDes: ");
      Serial.print(rcOutput(1));
      Serial.print(" yawDDes: ");
      Serial.print(rcOutput(2));
      Serial.print(" heightDDes: ");
      Serial.print(rcOutput(3));
      Serial.print(" fm: ");
      Serial.print(rcOutput(4));
      Serial.println();
      
      #if PIDTUNE
        Serial.print("tuning: ");
        Serial.print(RPYTTUNE);
        Serial.print(" P: ");
        Serial.print(apo.getPids(RPYTTUNE, 0));
        Serial.print(" I: ");
        Serial.print(apo.getPids(RPYTTUNE, 1));
        Serial.print(" D: ");
        Serial.print(apo.getPids(RPYTTUNE, 2));
        Serial.println();
  
      #endif
      
      lastUpdateT = millis();
    }
    
  #endif

}
