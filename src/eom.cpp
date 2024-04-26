#include "eom.hpp"
#include "eigen3/Eigen/Dense"
#include "frames/frames.hpp"
#include "fstream"
#include "iostream"

typedef Eigen::Matrix<double, 19, 1> state_type;

void Eom::init(Eigen::Vector3d Vb, Eigen::Vector3d euler_angles,
               Eigen::Vector2d coord_angles, double h, double dt) {
  this->dt = dt;
  initial_state.setZero();

  initial_state.block<3, 1>(0, 0) = Frames::R_fb(euler_angles) * Vb;
  initial_state.block<3, 1>(3, 0) = euler_angles;
  initial_state(6) = h;
  initial_state.block<9, 1>(7, 0) = Eigen::Map<Eigen::Matrix<double, 9, 1>>(
      Frames::R_fe(coord_angles).data());
  initial_state.block<3, 1>(16, 0) = Eigen::Vector3d{0, 0, 0};
  std::cout << initial_state << std::endl;

  this->geodetic.init(coord_angles(0), h);
  this->Re = this->geodetic.get_radiusE();
  this->atmosphere.update(h);
};

void Eom::system(const state_type &x, state_type &dxdt, double t) {

  Eigen::Vector3d vel_f{x(0), x(1), x(2)};

  double mu = x(3);
  double l = x(4);
  double h = -x(5);

  this->geodetic.update(mu, h);
  this->geodetic.get_radiusE();
  this->atmosphere.update(h);

  Eigen::Vector3d Fa_n;

  // Aero forces
  Fa_n = -0.5 * this->atmosphere.get_density() * vel_f.norm() * vel_f * this->projectile.get_Area()* 0.25;

  // Position
  double mu_dot = 1 / (this->Re + x(6)) * vel_f(0);
  double l_dot = 1 / (this->Re + x(6)) * cos(x(3)) * vel_f(1);
  double h_dot = vel_f(2);

  Eigen::Vector3d r_e{(this->Re + h) * cos(mu) * cos(l),
                      (this->Re + h) * cos(mu) * sin(l),
                      (this->Re + h) * sin(mu)};

  Eigen::Vector3d w_fe{l_dot * cos(mu), -mu_dot, -l_dot * sin(mu)};

  Eigen::Matrix3d Omega_fe = Frames::Om_fe(w_fe);

  Eigen::Vector3d coriolis =
      -2 * Frames::R_fe(Eigen::Vector2d{mu, l}) * w_ei.cross(vel_f);

  Eigen::Matrix3d R_dot_fe = -Omega_fe * Frames::R_fe(Eigen::Vector2d{mu, l});

  Eigen::Vector3d rad_acc =
      -Frames::R_fe(Eigen::Vector2d{mu, l}) * w_ei.cross(w_ei.cross(r_e));

  dxdt.block<3, 1>(0, 0) = Fa_n / 0.010 +
                           Eigen::Vector3d{0, 0, this->geodetic.get_gravity()} +
                           coriolis;

  dxdt.block<3, 1>(3, 0) = Eigen::Vector3d{mu_dot, l_dot, h_dot};

  dxdt.block<9, 1>(6, 0) =
      Eigen::Map<Eigen::Matrix<double, 9, 1>>(R_dot_fe.data());

  dxdt.block<3, 1>(16, 0) = vel_f;
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
  this->projectile.init(0.0067, 0.011, 0.1);
  std::vector<double> t_s, mu_c, l_c, h_c, v_x, v_y, v_z, x_c, y_c, z_c;
  double z_land = 0.0;
  double z_ref = 0.00;

  init({V, 0, 0}, {0, theta, psi}, {0, 0}, 0, dt);

  state_type x = this->initial_state;

  while (z_ref <= fin_alt) {
    t += dt;
    runge_kutta4(x, t);

    v_x.push_back(x(0));
    v_y.push_back(x(1));
    v_z.push_back(x(2));
    mu_c.push_back(x(3));
    l_c.push_back(x(4));
    h_c.push_back(-x(5));
    x_c.push_back(x(16));
    y_c.push_back(x(17));
    z_c.push_back(-x(18));

    t_s.push_back(t);
    z_ref = x(5);
    // std::cout << t << std::endl;
  }

  std::cout << t_s.size() << std::endl;
  std::fstream file;
  file.open("simulation_results.txt", std::ios_base::out);
  file << "t_s" << " " << "x_c" << " " << "y_c" << " " << "z_c" << " " << "v_x"
       << " " << "v_y" << " " << "v_z" << std::endl;

  file << "0" << " " << initial_state(0) << " " << initial_state(1) << " "
       << initial_state(2) << " " << initial_state(3) << " " << initial_state(4)
       << " " << initial_state(5) << " " << initial_state(16) << " "
       << initial_state(17) << " " << initial_state(18) << std::endl;

  for (int i = 0; i < t_s.size(); i++) {
    file << t_s[i] << " " << v_x[i] << " " << v_y[i] << " " << v_z[i] << " "
         << mu_c[i] << " " << l_c[i] << " " << h_c[i] << " " << x_c[i] << " "
         << y_c[i] << " " << z_c[i] << std::endl;
  }
  file.close();

  return;
}
