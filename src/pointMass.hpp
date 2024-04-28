
#include "eigen3/Eigen/Dense"
#include "environment/atmosisa.hpp"
#include "environment/geodetic.hpp"
#include "projectile/projectile.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <queue>
#include <tuple>

typedef Eigen::Matrix<double, 18, 1> state_type;

class Eom {
public:
  void init(Eigen::Vector3d Vb, Eigen::Vector3d euler_angles,
            Eigen::Vector2d coord_angles, double h, double dt);
  void system(const state_type &x, state_type &dxdt, double t);
  void runge_kutta4(state_type &x, double t);
  void step();
  void simulate(double V, double theta, double psi, double fin_alt, double dt);

private:
  double dt;
  std::queue<std::tuple<double, state_type, state_type>> dataQueue;

  double Re = 6378137.0;

  Eigen::Vector3d w_ei{0, 0, 7.2921159e-5};

  Geodetic geodetic;
  Atmosphere atmosphere;
  Projectile projectile;

  state_type initial_state;
  state_type x;
  double t;

  // data
  Eigen::Vector3d pos_f;
  Eigen::Vector3d vel_f;
  Eigen::Vector3d acc_f;
  Eigen::Vector3d eulAng;
  Eigen::Vector2d coordAng;
  Eigen::Vector3d coordAcc;
  double h;
  Eigen::Vector3d Fa_n;
  Eigen::Vector3d r_e;
  Eigen::Vector3d w_fe;
  Eigen::Matrix3d Omega_fe;
  Eigen::Vector3d accCor;
  Eigen::Matrix3d R_fe;
  Eigen::Matrix3d R_dot_fe;
  Eigen::Vector3d rad_acc;
};