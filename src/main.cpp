#include "eom.hpp"
#include "eigen3/Eigen/Dense"

int main() {

  double theta = 3*M_PI/180.0;
  double psi = 45*M_PI/180.0;
  double V = 800;

  Eom eom;

  eom.simulate(V, theta, psi, 0.001, 0.1);

  return 0;
}