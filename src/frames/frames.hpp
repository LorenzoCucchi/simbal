#include <cmath>

#include "eigen3/Eigen/Dense"

class Frames {
 public:
  void initNed(double inp_Vb, double inp_theta, double inp_psi);

  static Eigen::Matrix3d R_bf(const Eigen::Vector3d &euler_angles);

  static Eigen::Matrix3d R_fb(const Eigen::Vector3d &euler_angles);
  
  static Eigen::Matrix3d R_fe(const Eigen::Vector2d &coord_angles);

  static Eigen::Matrix3d R_ef(const Eigen::Vector2d &coord_angles);

  static Eigen::Matrix3d Om_fe(const Eigen::Vector3d &w_fe);

 private:
  double theta;
  double phi;
  double psi;

  double mu;
  double l;
};