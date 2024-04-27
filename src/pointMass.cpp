#include "pointMass.hpp"
#include "eigen3/Eigen/Dense"
#include "frames/frames.hpp"
#include "fstream"
#include "iostream"

typedef Eigen::Matrix<double, 18, 1> state_type;

void Eom::init(Eigen::Vector3d Vb, Eigen::Vector3d euler_angles,
               Eigen::Vector2d coord_angles, double h, double dt) {
  this->dt = dt;
  initial_state.setZero();

  initial_state.block<3, 1>(0, 0) = Frames::R_fb(euler_angles) * Vb;
  initial_state.block<2, 1>(3, 0) = coord_angles;
  initial_state(5) = h;
  initial_state.block<9, 1>(6, 0) = Eigen::Map<Eigen::Matrix<double, 9, 1>>(
      Frames::R_fe(coord_angles).data());
  initial_state.block<3, 1>(15, 0) = Eigen::Vector3d{0, 0, 0};
  std::cout << initial_state << std::endl;

  this->geodetic.init(coord_angles(0), h);
  this->Re = this->geodetic.get_radiusE();
  this->atmosphere.update(h);
};

void Eom::system(const state_type &x, state_type &dxdt, double t) {

  Eigen::Vector3d vel_f{x(0), x(1), x(2)};
  this->pos_f = {x(15), x(16), x(17)};
  this->coordAng = {x(3), x(4)};
  double mu = x(3);
  double l = x(4);
  double h = -x(5);

  this->geodetic.update(mu, h);
  this->geodetic.get_radiusE();
  this->atmosphere.update(h);

  Eigen::Vector3d V_w{0, 0, 0};

  // Aero forces
  this->Fa_n = -0.5 * this->atmosphere.get_density() * (vel_f - V_w).norm() *
               (vel_f - V_w) * this->projectile.get_Area() *
               this->projectile.get_Drag(vel_f.norm() /
                                         this->atmosphere.get_speed_of_sound());

  // Position
  double mu_dot = 1 / (this->Re + h) * vel_f(0);
  double l_dot = 1 / (this->Re + h) * cos(x(3)) * vel_f(1);
  double h_dot = vel_f(2);

  this->coordAcc = {mu_dot, l_dot, h_dot};

  this->r_e = {(this->Re + h) * cos(mu) * cos(l),
               (this->Re + h) * cos(mu) * sin(l), (this->Re + h) * sin(mu)};

  this->w_fe = {l_dot * cos(mu), -mu_dot, -l_dot * sin(mu)};

  this->Omega_fe = Frames::Om_fe(w_fe);

  this->R_fe = Frames::R_fe(Eigen::Vector2d{mu, l});
  this->accCor = 2 * this->R_fe * w_ei.cross(vel_f);

  this->R_dot_fe = -Omega_fe * this->R_fe;

  this->rad_acc =
      -Frames::R_fe(Eigen::Vector2d{mu, l}) * w_ei.cross(w_ei.cross(r_e));
  this->acc_f = Fa_n / this->projectile.get_Mass() +
                           Eigen::Vector3d{0, 0, this->geodetic.get_gravity()} +
                           accCor + rad_acc;

  // populate dxdt
  dxdt.block<3, 1>(0, 0) = this->acc_f;
  dxdt.block<3, 1>(3, 0) = Eigen::Vector3d{mu_dot, l_dot, h_dot};
  dxdt.block<9, 1>(6, 0) =
      Eigen::Map<Eigen::Matrix<double, 9, 1>>(R_dot_fe.data());
  dxdt.block<3, 1>(15, 0) = vel_f;
};

void Eom::runge_kutta4(state_type &x, double t) {

  state_type k1, k2, k3, k4;
  state_type temp;

  system(x, k1, t);
  temp = x + 0.5 * dt * k1;

  system(temp, k2, t + 0.5 * dt);
  temp = x + 0.5 * dt * k2;

  system(temp, k3, t + 0.5 * dt);
  temp = x + dt * k3;

  system(temp, k4, t + dt);
  x += dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

}

void Eom::simulate(double V, double theta, double psi, double fin_alt,
                   double dt) {
  double t0 = 0.0;
  double t = t0;
  this->dt = dt;
  this->projectile.init(0.105, 43.0, 0.1);

  init({V, 0, 0}, {0, theta, psi}, {0, 0}, 0, dt);

  state_type x = this->initial_state;
  state_type dxdt;

  auto start = std::chrono::high_resolution_clock::now();

  while (x(5) <= fin_alt) {
    t += dt;
    runge_kutta4(x, t);

    dataQueue.push(std::make_tuple(t, x, dxdt));
  }

  std::cout << dataQueue.size() << std::endl;
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Time taken by while loop: " << duration.count()
            << " milliseconds" << std::endl;

  return;
}
