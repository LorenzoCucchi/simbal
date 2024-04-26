#include <catch2/catch_all.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"

#include "frames.hpp"

TEST_CASE("Frames initNed") {

  Frames frames;
  frames.initNed(1, 0, 0);
  SECTION("Test ned to body with phi = 0, theta = 0, psi = 0") {
    Eigen::Matrix3d expected;
    expected.setIdentity();

    REQUIRE(frames.R_bf(Eigen::Vector3d::Zero()) == expected);
  }
  SECTION("Test body to ned with phi = 0, theta = 0, psi = 0") {
    Eigen::Matrix3d expected;
    expected.setIdentity();

    REQUIRE(frames.R_bf(Eigen::Vector3d::Zero()) == expected);
  }
}

TEST_CASE("Frames ecef") {

  Frames frames;
  frames.initNed(1, 0, 0);
  SECTION("Test longitude 0 and latitude 0") {
    Eigen::Matrix3d expected;
    expected << 0, 0, 1, 0, 1, 0, -1, 0, 0;

    REQUIRE(frames.R_fe(Eigen::Vector2d::Zero()) == expected);
  }
  SECTION("Test longitude pi/2 and latitude 0") {
    Eigen::Matrix3d expected;
    expected << -1, 0, 0, 0, 1, 0, 0, 0, -1;
    Eigen::Matrix3d R_fe = frames.R_fe(Eigen::Vector2d{M_PI / 2, 0});

    for (int i = 0; i < expected.rows(); ++i) {
      for (int j = 0; j < expected.cols(); ++j) {
        REQUIRE(std::abs(R_fe(i, j) -expected(i, j)) < 1e-10);
      }
    }
  }
}