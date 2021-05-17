#include "butter.h"

void butterFilter::butterInit(){
  
  xn2 = 0;
  xn1 = 0;
  xn = 0;
  yn2 = 0;
  yn1 = 0;
  yn = 0;
  
}

void butterFilter::butterUpdate(const double &xin){

  // A discrete butterworth filter
  xn = xin;
  yn = b2*xn + b1*xn1 + b0*xn2 - a1*yn1 - a0*yn2;

  // Updating past states
  xn2 = xn1;
  xn1 = xn;
  yn2 = yn1;
  yn1 = yn;
  
}
