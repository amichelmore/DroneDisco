#include "fullFc.h"

ekfObject eo;
attitudePidObject apo;
rcObject rco;
motorControllerObject mco;

bool armed = false;

bool slowLoop = false;
int fastLoop = 0;
unsigned long lastUpdateSlow = millis();
unsigned long lastUpdateFast = millis();
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

  // Fast and slow loop flags
  if((millis() - lastUpdateSlow) > 100) {
    slowLoop = true;
  }

  // Update ekf and retrieve state
  eo.ekfUpdate();
  VectorXd state(13);
  state = eo.ekfGetState();

  // Read & convert Rc to desired angles
  rco.rcUpdate();
  VectorXd rcOutput(5);
  rcOutput = rco.rcOut();

  VectorXd attitudeDes(4);
  attitudeDes = flightMode(rcOutput);
  apo.attitudeUpdate(attitudeDes, state, (rcOutput(4) == 1)); // Takeoff flag only active if in flight_mode 1

  // Check arming
  armed = false;
  // DO ARMING CODE
  
  // Convert torques & thrust to Motor input and command motors
  VectorXd attitudeOut(4);
  attitudeOut = apo.attitudeOut();
  mco.motorControllerUpdate(attitudeOut, armed);

  // Print out debug messages if DEBUG is defined
  #if DEBUG

    printDebug();
    
  #endif

  // Loop timer updates
  // slowLoop bool is active every 100ms
  if(slowLoop) {
    lastUpdateSlow = millis();
    slowLoop = false;
  }
  // fastLoop int = loop time of the main loop
  fastLoop = millis() - lastUpdateFast;
  lastUpdateFast = millis();
}

VectorXd flightMode(VectorXd rcOutput) {

  VectorXd attitudeDes(4);
  
  // Flight modes (!!! Need to clean this up)
  // Get torques / thrust from PID attitude controller
  int mode = rcOutput(4);
  switch(mode) {
    case 1: // Mode 1: Startup
      // Set motor speeds to 0 and print diagnostics (state, theoretical motor outputs)
      attitudeDes << 0, 0, 0, 0;
      
      break;

    case 2: // Mode 2: Takeoff
      // set desired roll, pitch, and yaw to 0 and pass takeoff flag
      attitudeDes << 0, 0, 0, rcOutput(3);
      
      break;

    case 3: // Mode 2: normal flight
    
      // set desired roll, pitch, and yaw
      attitudeDes = rcOutput.head(4); 
      
      break;

    #if PIDTUNE

      case 6: // P tune
  
        attitudeDes << 0, 0, 0, rcOutput(3);
  
        if(slowLoop){ // update P value according to user request every 0.1 sec
    
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
        
        break;
        
      case 5: // I tune (if applicable)

        attitudeDes << 0, 0, 0, rcOutput(3);
  
        if(slowLoop){ // update I value according to user request every 0.1 sec (for only roll and pitch)
    
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
        
        break;
        
      case 4: // D tune

        attitudeDes << 0, 0, 0, rcOutput(3);
  
        if(slowLoop){ // update D value according to user request every 0.1 sec
    
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
        
        break;
      
      case 7: // P tune
  
        #if RPYTTUNE == 0
          attitudeDes << rcOutput(0), 0, 0, 0;
        #elif RPYTTUNE == 1
          attitudeDes << 0, rcOutput(1), 0, 0;
        #elif RPYTUNE == 2
          attitudeDes << 0, 0, rcOutput(2), 0;
        #elif RPYTTUNE == 3
          attitudeDes << 0, 0, 0, rcOutput(3);
        #endif
  
        if(slowLoop){ // update P value according to user request every 0.1 sec
    
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
        
        break;
        
      case 8: // I tune (if applicable)

        #if RPYTTUNE == 0
          attitudeDes << rcOutput(0), 0, 0, 0;
        #elif RPYTTUNE == 1
          attitudeDes << 0, rcOutput(1), 0, 0;
        #elif RPYTUNE == 2
          attitudeDes << 0, 0, rcOutput(2), 0;
        #elif RPYTTUNE == 3
          attitudeDes << 0, 0, 0, rcOutput(3);
        #endif
  
        if(slowLoop){ // update I value according to user request every 0.1 sec (for only roll and pitch)
    
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
        
        break;
        
      case 9: // D tune

        #if RPYTTUNE == 0
          attitudeDes << rcOutput(0), 0, 0, 0;
        #elif RPYTTUNE == 1
          attitudeDes << 0, rcOutput(1), 0, 0;
        #elif RPYTUNE == 2
          attitudeDes << 0, 0, rcOutput(2), 0;
        #elif RPYTTUNE == 3
          attitudeDes << 0, 0, 0, rcOutput(3);
        #endif
  
        if(slowLoop){ // update D value according to user request every 0.1 sec
    
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
        
        break;
        
    #endif
    
    default:

      // Return zeros and disarm;
      attitudeDes = VectorXd::Zero(4);
      armed = false;
      
      break; 
  }

  return attitudeDes;
}

void printDebug() {
  VectorXd state(13);
  state = eo.ekfGetState();

  VectorXd rcOutput(5);
  rcOutput = rco.rcOut();

  VectorXd motorOutputs(4);
  motorOutputs = mco.motorControllerOut();
  
  if(slowLoop){
  
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
  }
}
