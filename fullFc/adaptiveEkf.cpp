#include "adaptiveEkf.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55); // IMU
UBLOX gps(Serial2,115200); // GPS

butterFilter bxd; // xyz velocity butterworth filters
butterFilter byd;
butterFilter bzd;
butterFilter bp; // pqr bw filters
butterFilter bq;
butterFilter br;
butterFilter bxdd; // accelerometer bw filters
butterFilter bydd;
butterFilter bzdd;

extern double grav; // Gravity

bool ekfObject::ekfInit(){
  
  // Initialize the Matrices and Vectors
  x_ka = VectorXd::Zero(7);
  P_ka = MatrixXd::Zero(7,7);

  Q_k = MatrixXd::Identity(7,7);
  R_k = MatrixXd::Identity(7,7);
  e_k = VectorXd::Zero(7);

  C = MatrixXd::Identity(7,7);
  J_h = MatrixXd::Identity(7,7);

  state = VectorXd::Zero(13);

  // Filter inits
  bxd.butterInit();
  byd.butterInit();
  bzd.butterInit();
  bp.butterInit();
  bq.butterInit();
  br.butterInit();
  bxdd.butterInit();
  bydd.butterInit();
  bzdd.butterInit();

  // IMU Setup
  if(!bno.begin()){
    return false;
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // GPS Setup
  gps.begin();
  
  // Get initial GPS fix
  while(!gps.readSensor()){
    Serial.println("Getting GPS fix");
    delay(10);
    }
  while(gps.getHorizontalAccuracy_m() > 10){
    Serial.print(gps.getNumSatellites());
    Serial.print(" ");
    Serial.println(gps.getHorizontalAccuracy_m());
    gps.readSensor();
    delay(10);
  }
  lat0 = gps.getLatitude_rad();
  lon0 = gps.getLongitude_rad();

  // Initialize R_k measurement noise covariance
  R_k(0, 0) = pow(gps.getHorizontalAccuracy_m(), 2);
  R_k(1, 1) = R_k(0, 0);
  R_k(2, 2) = pow(gps.getVerticalAccuracy_m(), 2);
  R_k(3, 3) = pow(gps.getSpeedAccuracy_ms(), 2);
  R_k(4, 4) = R_k(3, 3);
  R_k(5, 5) = R_k(3, 3);
  R_k(6, 6) = pow((5*PI/180), 2);

  // Initialize state x_ka
  x_ka(2) = -gps.getMSLHeight_m();
  P_ka = ((x_ka - VectorXd::Constant(7, x_ka.mean()))*(x_ka - VectorXd::Constant(7, x_ka.mean())).transpose());

// Math is wrong fr grav calc, so just assuming 9.81
//  // Set gravity const
//  sensors_event_t orientationData; 
//  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);  
//  double roll = (orientationData.orientation.z)*(PI/180); // this is different than below for gravitational accuracy.
//  double pitch = (-orientationData.orientation.y)*(PI/180);
//
//  sensors_event_t gravityData; 
//  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
//  //grav = (gravityData.acceleration.z*cos(roll)*cos(pitch)) + (gravityData.acceleration.x*sin(roll)*cos(pitch)) + (gravityData.acceleration.y*cos(roll)*sin(pitch));
//  Serial.print("Gravity: ");
//  Serial.println(grav);

  lastUpdateT = millis();
  
  return true;
}

bool ekfObject::ekfUpdate(){
 
  // If ekf crashes, this shouldn't happen
  if(std::isnan(x_ka(0))){
    ekfInit();
  }
  
  // Read IMU for angles r p y, input u_k, and angVel
  sensors_event_t orientationData, linearAccelData, angVelocityData; 
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);  
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  double roll = (orientationData.orientation.z + angOffsetR)*(PI/180);
  double pitch = (-orientationData.orientation.y)*(PI/180);
  double yaw = (orientationData.orientation.x - 90)*(PI/180);

  // Check input yaw
  if(yaw > PI){
    yaw -= (2*PI);
  } else if(yaw < -PI){
    yaw += (2*PI);
  }
  
  // Butter filters for the gyro
  bp.butterUpdate(-(angVelocityData.gyro.x*(PI/180)));
  bq.butterUpdate(angVelocityData.gyro.y*(PI/180));
  br.butterUpdate(-(angVelocityData.gyro.z*(PI/180)));
  bxdd.butterUpdate(linearAccelData.acceleration.x);
  bydd.butterUpdate(-linearAccelData.acceleration.y);
  bzdd.butterUpdate(linearAccelData.acceleration.z);
  
  // Update full state
  state(6) = roll;
  state(7) = pitch;
  state(9) = bp.butterOut();
  state(10) = bq.butterOut();
  state(11) = br.butterOut();
  state(12) = bzdd.butterOut(); // This is only correct for near hover angles

  // U_t (control input) = x_dd, y_dd, z_dd, yaw_dd
  
  
  VectorXd u_k(4);
  u_k << bxdd.butterOut(), bydd.butterOut(), bzdd.butterOut(), br.butterOut();
  
  if(gps.readSensor()) {

    // Get time elapsed dt
    float dt = (millis() - lastUpdateT)*0.001;
    // Serial.print("dt2: ");
    // float dt2 = dt*1000;
    // Serial.println(dt2, 4);
    lastUpdateT = millis();

    // Declare temporary matrices
    VectorXd Z_k(7); // Measurement matrix
    MatrixXd A(7,7); // State transition matrix
    MatrixXd B(7,4); // Control matrix
    // VectorXd G(7); // Gravity vector
    MatrixXd Rbgp(3,3); // Rotation from body to global frame
    MatrixXd J_f(7,7); // Jacobian of state transition
    VectorXd x_kf(7); // State x forecast
    VectorXd d_k(7); // Innovation
    MatrixXd P_kf(7,7); // Error covariance forecast
    MatrixXd K_k(7,7); // Khalman gain
    
    // Read in measurements
    //Potentially add a filter to these velocities. Though they are already being filtered at the output of the EKF.
    Z_k << R*(gps.getLatitude_rad()-lat0), R*(gps.getLongitude_rad()-lon0)*cos(lat0), -gps.getMSLHeight_m(), gps.getNorthVelocity_ms(), gps.getEastVelocity_ms(), gps.getDownVelocity_ms(), yaw;
  
    // Set A, B, G, Rbgp, J_f @ dt, roll, pitch, and yaw.
    A << 1, 0, 0, dt, 0, 0, 0,
         0, 1, 0, 0, dt, 0, 0,
         0, 0, 1, 0, 0, dt, 0,
         0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 1;

    
    B << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0,
         cos(pitch)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw), cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw), 0,
         cos(pitch)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw), 0,
         -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), 0,
         0, 0, 0, 1;
    B *= dt;
  
    //G << 0, 0, 0, 0, 0, grav*dt, 0;
  
    Rbgp << -cos(pitch)*sin(yaw), -sin(roll)*sin(pitch)*sin(yaw) - cos(roll)*cos(yaw), -cos(roll)*sin(pitch)*sin(yaw) + sin(roll)*cos(yaw),
            cos(pitch)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw), cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
            0, 0, 0;

    
    J_f << 1, 0, 0, dt, 0, 0, 0,
           0, 1, 0, 0, dt, 0, 0,
           0, 0, 1, 0, 0, dt, 0,
           0, 0, 0, 1, 0, 0, Rbgp.row(0)*u_k.head(3)*dt,
           0, 0, 0, 0, 1, 0, Rbgp.row(1)*u_k.head(3)*dt,
           0, 0, 0, 0, 0, 1, Rbgp.row(2)*u_k.head(3)*dt,
           0, 0, 0, 0, 0, 0, 1;
  
    // Model Forecast
    //x_kf = A*(x_ka) + B*(u_k) + G;
    x_kf = A*(x_ka) + B*(u_k);
    P_kf = J_f*P_ka*J_f.transpose() + Q_k;

    // Check forecast yaw angle
    if(x_kf(6) > PI){
      x_kf(6) -= (2*PI);
    } else if(x_kf(6) < -PI){
      x_kf(6) += (2*PI);
    }
    
    // Data Assimilation
    R_k = alpha*R_k.eval() + (1-alpha)*(e_k*e_k.transpose() + J_h*P_kf*J_h.transpose());
    d_k = Z_k - C*x_kf;
    
    // Fix angle diff
    if(d_k(6) > PI){
      d_k(6) -= 2*PI;
    } else if(d_k(6) < -PI){
      d_k(6) += 2*PI;
    }

    K_k = P_kf*J_h.transpose()*(J_h*P_kf*J_h.transpose() + R_k).inverse();
    x_ka = x_kf + K_k*(d_k);
    P_ka = (MatrixXd::Identity(7,7) - K_k*J_h)*P_kf;

    // Check yaw angle
    if(x_ka(6) > PI){
      x_ka(6) -= (2*PI);
    } else if(x_ka(6) < -PI){
      x_ka(6) += (2*PI);
    }
    
    Q_k = alpha*Q_k.eval() + (1-alpha)*(K_k*d_k*d_k.transpose()*K_k.transpose());

    // Butter filters for the velocities
    bxd.butterUpdate(x_ka(3));
    byd.butterUpdate(x_ka(4));
    bzd.butterUpdate(x_ka(5));
    
    // Update full state
    state(0) = x_ka(0);
    state(1) = x_ka(1);
    state(2) = x_ka(2);
    state(3) = bxd.butterOut();
    state(4) = byd.butterOut();
    state(5) = bzd.butterOut();
    state(8) = x_ka(6);
    
    return true;
  }
  else{
    return false;
  }
}
