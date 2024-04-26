#include <cmath>

#include "eigen3/Eigen/Dense"

#include "frames.hpp"

void Frames::initNed(double inp_Vb, double inp_theta, double inp_psi) {
  this->theta = inp_theta;
  this->phi = 0;
  this->psi = inp_psi;

  this->mu = 0;
  this->l = 0;
}

Eigen::Matrix3d Frames::R_bf(const Eigen::Vector3d &eul) {

  Eigen::Matrix3d R_bf;

  double c_ph = cos(eul(0));
  double s_ph = sin(eul(0));
  double c_th = cos(eul(1));
  double s_th = sin(eul(1));
  double c_ps = cos(eul(2));
  double s_ps = sin(eul(2));

  R_bf.setZero();

  R_bf(0, 0) = c_th * c_ps;
  R_bf(0, 1) = c_th * s_ps;
  R_bf(0, 2) = -s_th;
  R_bf(1, 0) = s_ph * s_th * c_ps - c_ph * s_ps;
  R_bf(1, 1) = s_ph * s_th * s_ps + c_ph * c_ps;
  R_bf(1, 2) = s_ph * c_th;
  R_bf(2, 0) = c_ph * s_th * c_ps + s_ph * s_ps;
  R_bf(2, 1) = c_ph * s_th * s_ps - s_ph * c_ps;
  R_bf(2, 2) = c_ph * c_th;

  return R_bf;
}

Eigen::Matrix3d Frames::R_fb(const Eigen::Vector3d &eul) {

  return R_bf(eul).inverse();
}

Eigen::Matrix3d Frames::R_fe(const Eigen::Vector2d &coord_angles) {
  Eigen::Matrix3d R_fe;

  double c_mu = cos(coord_angles(0));
  double s_mu = sin(coord_angles(0));
  double c_l = cos(coord_angles(1));
  double s_l = sin(coord_angles(1));

  R_fe.setZero();

  R_fe(0, 0) = -c_l * s_mu;
  R_fe(0, 1) = -s_l * s_mu;
  R_fe(0, 2) = c_mu;
  R_fe(1, 0) = -s_l;
  R_fe(1, 1) = c_l;
  R_fe(1, 2) = 0;
  R_fe(2, 0) = -c_mu * c_l;
  R_fe(2, 1) = -c_mu * s_l;
  R_fe(2, 2) = -s_mu;

  return R_fe;
}

Eigen::Matrix3d Frames::R_ef(const Eigen::Vector2d &coord_angles) {
  return R_fe(coord_angles).inverse();
}

Eigen::Matrix3d Frames::Om_fe(const Eigen::Vector3d &w_fe) {
  Eigen::Matrix3d Om_fe;

  Om_fe.setZero();

  Om_fe(0, 1) = -w_fe(2);
  Om_fe(0, 2) = w_fe(1);
  Om_fe(1, 0) = w_fe(2);
  Om_fe(1, 2) = -w_fe(0);
  Om_fe(2, 0) = -w_fe(1);
  Om_fe(2, 1) = w_fe(0);

  return Om_fe;
}