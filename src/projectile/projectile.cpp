#include "projectile/projectile.hpp"
#include "eigen3/Eigen/Dense"

void Projectile::init(double diam, double mass, double /*length*/) {
  this->diam = diam;
  this->mass = mass;
  this->area = M_PI * pow(diam / 2, 2);
  this->I = Eigen::Matrix3d::Identity();
}

auto Projectile::get_Drag(double mach) -> double {
  if (mach > 1.9) {
    return 0.439493 + mach * (-0.0793543 + mach * 0.00448477);
  }
  if (mach > 1.05) {
    return 0.642743 + mach * (-0.272545 + mach * 0.0492475);
  }
  if (mach > 0.9) {
    return 0.353384 + mach * (-0.692406 + mach * 0.509469);
  }
  return 0.119775 + mach * (-0.00231118 + mach * 0.002867112);
}
