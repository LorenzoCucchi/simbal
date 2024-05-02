#include <cmath>

#include "eigen3/Eigen/Dense"

class Frames {
 public:
  void initNed(double inp_Vb, double inp_theta, double inp_psi);

  static auto R_bf(const Eigen::Vector3d &euler_angles) -> Eigen::Matrix3d;

  static auto R_fb(const Eigen::Vector3d &euler_angles) -> Eigen::Matrix3d;
  
  static auto R_fe(const Eigen::Vector2d &coord_angles) -> Eigen::Matrix3d;

  static auto R_ef(const Eigen::Vector2d &coord_angles) -> Eigen::Matrix3d;

  static auto Om_fe(const Eigen::Vector3d &w_fe) -> Eigen::Matrix3d;

 private:
  double theta;
  double phi;
  double psi;

  double mu;
  double l;
};