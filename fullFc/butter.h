// b2*z^2 + b1*z + b0
// ------------------
// a2*z^2 + a1*z + a0

//Filter Constants
const double b2 = 0.2929; 
const double b1 = 0.5858;
const double b0 = 0.2929;
// const double a2 = 1; // a2 always 0
const double a1 = -2.2e-16;
const double a0 = 0.1716;

class butterFilter {
  double xn2; // Stored values
  double xn1;
  double yn2;
  double yn1;
  
  double xn; // Input
  double yn; // Output

  public:
    void butterInit();
    void butterUpdate(const double &xin);
    double butterOut(){return yn;};
};
