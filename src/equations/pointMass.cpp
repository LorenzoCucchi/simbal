#include "pointMass.hpp"

#include <eigen3/Eigen/Core>

#include "eigen3/Eigen/Dense"

using state_type = Eigen::Matrix<double, 18, 1>;

void PointMass::initClasses(Geodetic* geodetic_ptr, Atmosphere* atmosphere_ptr,
                            Projectile* projectile_ptr) {
  this->geodetic = geodetic_ptr;
  this->atmosphere = atmosphere_ptr;
  this->projectile = projectile_ptr;
};

void PointMass::init(Eigen::Vector3d& vel_b, Eigen::Vector3d& euler_angles,
                     double in_alt, Eigen::Vector2d& coord_angles,
                     double deltaTime) {
  this->dt = deltaTime;
  initial_state.setZero();

  initial_state.block<3, 1>(0, 0) = Frames::R_fb(euler_angles) * vel_b;
  initial_state.block<2, 1>(3, 0) = coord_angles;
  initial_state(5) = in_alt;
  initial_state.block<9, 1>(6, 0) = Eigen::Map<Eigen::Matrix<double, 9, 1>>(
      Frames::R_fe(coord_angles).data());
  initial_state.block<3, 1>(15, 0) = Eigen::Vector3d{0, 0, 0};

  this->Re = this->geodetic->get_radiusE();
  this->atmosphere->update(h);
  this->x_state = initial_state;
};

void PointMass::system(const state_type& x_state, state_type& dxdt,
                       double /*t*/) {

  this->vel_f = {x_state(0), x_state(1), x_state(2)};
  this->pos_f = {x_state(15), x_state(16), -x_state(17)};
  this->coordAng = {x_state(3), x_state(4)};
  double mu_temp = x_state(3);
  double l_temp = x_state(4);
  double h_temp = -x_state(5);

  this->geodetic->update(mu_temp, h_temp);
  this->atmosphere->update(h_temp);

  Eigen::Vector3d V_w{0, 0, 0};

  // Aero forces
  this->Fa_n =
      -0.5 * this->atmosphere->get_density() * (vel_f - V_w).norm() *
      (vel_f - V_w) * this->projectile->get_Area() *
      Projectile::get_Drag(vel_f.norm() / this->atmosphere->get_sound());

  // Position
  double mu_dot = 1 / (this->Re + h_temp) * vel_f(0);
  double l_dot = 1 / (this->Re + h_temp) * cos(x_state(3)) * vel_f(1);
  double h_dot = vel_f(2);

  this->coordAcc = {mu_dot, l_dot, h_dot};

  this->r_e = {(this->Re + h_temp) * cos(mu_temp) * cos(l_temp),
               (this->Re + h_temp) * cos(mu_temp) * sin(l_temp),
               (this->Re + h) * sin(mu_temp)};

  this->w_fe = {l_dot * cos(mu_temp), -mu_dot, -l_dot * sin(mu_temp)};

  this->Omega_fe = Frames::Om_fe(w_fe);

  this->R_fe = Frames::R_fe(Eigen::Vector2d{mu_temp, l_temp});
  this->accCor = 2 * this->R_fe * this->w_ei.cross(vel_f);

  this->R_dot_fe = -Omega_fe * this->R_fe;

  this->rad_acc = -Frames::R_fe(Eigen::Vector2d{mu_temp, l_temp}) *
                  this->w_ei.cross(this->w_ei.cross(r_e));
  this->acc_f = Fa_n / this->projectile->get_Mass() +
                Eigen::Vector3d{0, 0, this->geodetic->get_gravity()} + accCor +
                rad_acc;

  // populate dxdt
  dxdt.block<3, 1>(0, 0) = this->acc_f;
  dxdt.block<3, 1>(3, 0) = Eigen::Vector3d{mu_dot, l_dot, h_dot};
  dxdt.block<9, 1>(6, 0) =
      Eigen::Map<Eigen::Matrix<double, 9, 1>>(R_dot_fe.data());
  dxdt.block<3, 1>(15, 0) = this->vel_f;
};

void PointMass::runge_kutta4(state_type& x_state, double time) {

  state_type k_1;
  state_type k_2;
  state_type k_3;
  state_type k_4;
  state_type temp;

  system(x_state, k_1, time);
  temp = x_state + 0.5 * dt * k_1;

  system(temp, k_2, time + 0.5 * dt);
  temp = x_state + 0.5 * dt * k_2;

  system(temp, k_3, time + 0.5 * dt);
  temp = x_state + dt * k_3;

  system(temp, k_4, time + dt);
  x_state += dt / 6.0 * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4);

  dxdt = (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4) / 6.0;

}

void PointMass::step() {
  this->old_state = this->x_state;
  this->time += dt;
  runge_kutta4(this->x_state, time);
}

void PointMass::logData() {
  data << time, x_state, dxdt;
}

