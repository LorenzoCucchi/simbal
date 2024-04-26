
#include "eigen3/Eigen/Dense"
#include "environment/atmosisa.hpp"
#include "environment/geodetic.hpp"
#include "projectile/projectile.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

typedef Eigen::Matrix<double, 19, 1> state_type;

class Eom {
public:
  void init(Eigen::Vector3d Vb, Eigen::Vector3d euler_angles,
            Eigen::Vector2d coord_angles, double h, double dt);
  void system(const state_type &x, state_type &dxdt, double t);
  void runge_kutta4(state_type &x, double t);
  void simulate(double V, double theta, double psi, double fin_alt, double dt);

private:
  double dt;

  double Re = 6378137.0;

  Eigen::Vector3d w_ei{0, 0, 7.2921159e-5};

  Geodetic geodetic;
  Atmosphere atmosphere;
  Projectile projectile;

  state_type initial_state;
};