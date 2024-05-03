#include "mission.hpp"

#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "equations/pointMass.hpp"
#include "json.hpp"
#include "utils/logger.hpp"

auto Mission::readInputFile(const std::filesystem::path& path) -> bool {
  nlohmann::json data{};
  try {
    std::ifstream inputFile(path);
    if (!inputFile) {
      throw std::runtime_error("File not found");
    }
    data = nlohmann::json::parse(inputFile);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return false;
  }

  coord_struct coords{};
  try {
    nlohmann::json Coordinates = data["Coordinates"];
    coords.type = Coordinates["Type"].get<std::string>();
    coords.unit = Coordinates["Unit"].get<std::string>();
    coords.lat = Coordinates["Latitude"].get<double>();
    coords.lon = Coordinates["Longitude"].get<double>();
    coords.alt = Coordinates["Altitude"].get<double>();
  } catch (const std::exception& e) {
    std::cerr << "Error parsing coordinates: " << e.what() << std::endl;
    return false;
  }

  target_struct target{};
  try {
    nlohmann::json Target = data["Target"];
    target.distance = Target["Distance"].get<double>();
    target.azimuth = Target["Azimuth"].get<double>();
    target.los_angle = Target["LOS_angle"].get<double>();
    target.altitude =
        target.distance * std::sin(target.los_angle * M_PI / 180.0);
    target.easting =
        target.distance * std::cos(target.los_angle) * std::sin(target.azimuth);
    target.northing =
        target.distance * std::cos(target.los_angle) * std::cos(target.azimuth);
    target.position =
        Eigen::Vector3d{target.northing, target.easting, target.altitude};
  } catch (const std::exception& e) {
    std::cerr << "Error parsing target: " << e.what() << std::endl;
    return false;
  }

  atmos_struct atmosph{};
  try {
    nlohmann::json Atmosphere = data["Atmosphere"];
    atmosph.temperature = Atmosphere["Temperature"].get<double>();
    atmosph.pressure = Atmosphere["Pressure"].get<double>();
    atmosph.humidity = Atmosphere["Humidity"].get<double>();
  } catch (const std::exception& e) {
    std::cerr << "Error parsing atmosphere: " << e.what() << std::endl;
    return false;
  }

  proj_struct proj{};
  try {
    nlohmann::json Projectile = data["Gun"];
    proj.name = Projectile["Projectile_name"].get<std::string>();
    proj.twist = Projectile["Twist"].get<double>();
    proj.velocity = Projectile["Velocity"].get<double>();
    proj.sight_height = Projectile["Sight_height"].get<double>();
    proj.zero_range = Projectile["Zero_range"].get<double>();
  } catch (const std::exception& e) {
    std::cerr << "Error parsing projectile: " << e.what() << std::endl;
    return false;
  }

  coordinates_ = coords;
  target_ = target;
  atmosphere_ = atmosph;
  projcetile_ = proj;

  return true;
}

void Mission::initClasses() {
  geodetic.init(coordinates_.lat, coordinates_.alt);
  atmosphere.update(coordinates_.alt);
  projectile.init(0.0067, 0.01, 0.1);
  eom.initClasses(&geodetic, &atmosphere, &projectile);
}

auto Mission::zeroing() -> bool {

  double initial_theta =
      std::atan(projcetile_.sight_height / target_.distance) +
      target_.los_angle * M_PI / 180.0;
  Eigen::Vector3d vel_b{projcetile_.velocity, 0, 0};
  Eigen::Vector3d euler_angles{0, initial_theta,
                               target_.azimuth * M_PI / 180.0};
  Eigen::Vector2d coord_angles{coordinates_.lat, coordinates_.lon};

  eom.init(vel_b, euler_angles, coordinates_.alt, coord_angles, 0.0001);
  Eigen::Vector3d pos_b{0.0, 0.0, 0.0};

  double distance_init = (target_.position - pos_b).norm();

  std::array<Eigen::Vector3d, 2> pos_data = {pos_b, pos_b};
  std::array<double, 2> distance = {distance_init, distance_init};

  LoggerState<Eigen::Matrix<double, 37, 1>> logEom(
      0.5 * 1024 * 1024 * 1024, "logEom.txt", eom.getDataName());
  logEom.initLog();

  while (pos_b.block<2, 1>(0, 0).norm() < target_.distance) {
    eom.step();
    pos_b = eom.getPos();
    //std::cout << "pos_b:\n " << pos_b << std::endl;
    distance.at(0) = distance.at(1);
    pos_data.at(0) = pos_data.at(1);
    pos_data.at(1) = pos_b;
    distance.at(1) = (target_.position - pos_b).norm();
    //std::cout << "Distance: " << distance.at(1) << std::endl;
    logEom.addData(eom.getData());
    if (distance.at(1) > distance.at(0)) {
      logEom.flushData();
      break;
    }
  }

  Eigen::Vector3d passing_point = missDistance(pos_data);
  std::cout << "Passing point:\n " << passing_point << std::endl;

  return true;
}

auto Mission::missDistance(std::array<Eigen::Vector3d, 2>& pos_data)
    -> Eigen::Vector3d {
  double t_interp =
      (target_.position.block<2, 1>(0, 0) - pos_data.at(0).block<2, 1>(0, 0))
          .norm() /
      (pos_data.at(1).block<2, 1>(0, 0) - pos_data.at(0).block<2, 1>(0, 0))
          .norm();
  return Eigen::Vector3d{pos_data.at(0) +
                         t_interp * (pos_data.at(1) - pos_data.at(0))};
}

auto Mission::calcCorrection(Eigen::Vector3d& miss) -> Eigen::Vector3d {
  return Eigen::Vector3d{0, 0, 0};
}
// crea funzione che calcoli distanza da bersaglio
// crea funzione che calcoli azzeramento
// crea funzione che calcoli angolo di tiro e aggiustamento torrette
// crea funzione che chiami il salvataggio dei dati