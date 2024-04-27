#include "pointMass.hpp"
#include "eigen3/Eigen/Dense"

int main() {

  double theta = 30*M_PI/180.0;
  double psi = 0*M_PI/180.0;
  double V = 1000.0;

  Eom eom;

  eom.simulate(V, theta, psi, 0.01, 0.0001);

  return 0;
}