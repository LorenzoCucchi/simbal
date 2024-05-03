#ifndef POINTMASS_HPP
#define POINTMASS_HPP

#include <eigen3/Eigen/Core>
#include <string>
#include <vector>

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
  auto getState() -> state_type { return x_state; }
  auto getOldState() -> state_type { return old_state; }
  auto getPos() -> Eigen::Vector3d { return pos_f; }
  auto getOldPos() -> Eigen::Vector3d { return old_state.block<3, 1>(15, 0); }
  auto getDataName() -> std::vector<std::string> { return dataName; };
  auto getData() -> Eigen::Matrix<double, 37, 1> {
    logData();
    return data;
  }

 private:
  double dt{};
  Eigen::Matrix<double, 37, 1> data;
  std::vector<std::string> dataName{
      "time",      "vel_n",     "vel_e",       "vel_d",       "latitude",
      "longitude", "altitude",  "C_11",        "C_12",        "C_13",
      "C_21",      "C_22",      "C_23",        "C_31",        "C_32",
      "C_33",      "pos_n",     "pos_e",       "pos_d",       "der_acc_n",
      "der_acc_e", "der_acc_d", "der_acc_lat", "der_acc_lon", "der_acc_alt",
      "dot_C_11",  "dot_C_12",  "dot_C_13",    "dot_C_21",    "dot_C_22",
      "dot_C_23",  "dot_C_31",  "dot_C_32",    "dot_C_33",    "der_vel_n",
      "der_vel_e", "der_vel_d"};

  double Re{};
  Eigen::Vector3d w_ei{0, 0, 7.2921159e-5};

  void logData();

  Geodetic* geodetic;
  Atmosphere* atmosphere;
  Projectile* projectile{};

  state_type initial_state;
  state_type x_state;
  state_type old_state;
  state_type dxdt;
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