#include "eigen3/Eigen/Dense"

class Projectile {
 public:
  void init(double diam, double mass, double length);
  static auto get_Drag(double mach) -> double;
  [[nodiscard]] auto get_Area() const -> double { return this->area; }
  [[nodiscard]] auto get_Mass() const -> double { return this->mass; }
  [[nodiscard]] auto get_I() const -> Eigen::Matrix3d { return this->I; }

 private:
  double area;
  double diam;
  double mass;
  Eigen::Matrix3d I;
};