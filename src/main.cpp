#include "eigen3/Eigen/Dense"
#include "equations/pointMass.hpp"
#include "mission.hpp"

auto main() -> int {

  Mission mission;
  mission.readInputFile("mission.json");

  mission.initClasses();

  mission.zeroing();

  //double theta = 30 * M_PI / 180.0;
  //double psi = 0 * M_PI / 180.0;
  //double Vel = 1000.0;
//
  //PointMass eom;
//
  //eom.simulate(Vel, theta, psi, 0.01, 0.0001);

  return 0;
}