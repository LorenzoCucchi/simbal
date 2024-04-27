#include "eigen3/Eigen/Dense"

class Projectile {
public:
  void init(double diam, double mass, double length);
  double get_Drag(double mach);
  double get_Area() { return this->area; }
  double get_Mass() { return this->mass; }
  Eigen::Matrix3d get_I() { return this->I; }

private:
  double area;
  double diam;
  double mass;
  Eigen::Matrix3d I;
};