#include <filesystem>

#include "equations/pointMass.hpp"

class Mission {
 public:
  auto readInputFile(const std::filesystem::path& path) -> bool;
  void initClasses();
  auto zeroing() -> bool;

 private:
  struct coord_struct {
    std::string type;
    std::string unit;
    double lat;
    double lon;
    double alt;
  };

  struct target_struct {
    double distance;
    double azimuth;
    double los_angle;
    double altitude;
    double easting;
    double northing;
    Eigen::Vector3d position;
  };

  struct atmos_struct {
    double temperature;
    double pressure;
    double humidity;
  };

  struct proj_struct {
    std::string name;
    double twist;
    double velocity;
    double sight_height;
    double zero_range;
  };

  coord_struct coordinates_{};
  target_struct target_{};
  atmos_struct atmosphere_{};
  proj_struct projcetile_{};

  PointMass eom;
  Geodetic geodetic;
  Atmosphere atmosphere;
  Projectile projectile;
};