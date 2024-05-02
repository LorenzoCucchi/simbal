#ifndef POINTMASS_HPP
#define POINTMASS_HPP

#include <queue>
#include <tuple>

#include "eigen3/Eigen/Dense"
#include "environment/atmosisa.hpp"
#include "environment/geodetic.hpp"
#include "frames/frames.hpp"
#include "projectile/projectile.hpp"

using state_type = Eigen::Matrix<double, 18, 1>;

class PointMass {
 public:
  void init(Eigen::Vector3d& vel_b, Eigen::Vector3d& euler_angles,
            double in_alt, Eigen::Vector2d& coord_angles, double deltaTime);
  void initClasses(Geodetic* geodetic_ptr, Atmosphere* atmosphere_ptr,
                   Projectile* projectile_ptr);
  void system(const state_type& x_state, state_type& dxdt, double time);
  void runge_kutta4(state_type& x_state, double time);
  void step();
  void simulate(double vel, double theta, double psi, double fin_alt,
                double deltaTime);
  auto getState() -> state_type { return x_state; }
  auto getPos() -> Eigen::Vector3d { return pos_f; }

 private:
  double dt{};
  std::queue<std::tuple<double, state_type, state_type>> dataQueue;

  double Re{};
  Eigen::Vector3d w_ei{0, 0, 7.2921159e-5};

  Geodetic* geodetic;
  Atmosphere* atmosphere;
  Projectile* projectile{};

  state_type initial_state;
  state_type x_state;
  double time{};

  // data
  Eigen::Vector3d pos_f;
  Eigen::Vector3d vel_f;
  Eigen::Vector3d acc_f;
  Eigen::Vector3d eulAng;
  Eigen::Vector2d coordAng;
  Eigen::Vector3d coordAcc;
  double h{};
  Eigen::Vector3d Fa_n;
  Eigen::Vector3d r_e;
  Eigen::Vector3d w_fe;
  Eigen::Matrix3d Omega_fe;
  Eigen::Vector3d accCor;
  Eigen::Matrix3d R_fe;
  Eigen::Matrix3d R_dot_fe;
  Eigen::Vector3d rad_acc;
};

#endif