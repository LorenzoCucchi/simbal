#include "projectile/projectile.hpp"
#include "eigen3/Eigen/Dense"

void Projectile::init(double diam, double mass, double length) {
  this->diam = diam;
  this->mass = mass;
  this->area = M_PI * pow(diam / 2, 2);
  this->I = Eigen::Matrix3d::Identity();
}