#include <catch2/catch_all.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>

#include "atmosisa.hpp"
#include "catch2/catch_test_macros.hpp"

TEST_CASE("Atmosphere test") {
  Atmosphere atmosphere;
  atmosphere.update(0);
  SECTION("Test pressure at 0 altitude") {
    REQUIRE(atmosphere.get_pressure() == Catch::Approx(101325));
  }
  SECTION("Test temperature at 0 altitude") {
    REQUIRE(atmosphere.get_temperature() == Catch::Approx(288.15));
  }
  SECTION("Test density at 0 altitude") {
    REQUIRE(atmosphere.get_density() == Catch::Approx(1.225));
  }
  SECTION("Test speed of sound at 0 altitude") {
    REQUIRE(atmosphere.get_speed_of_sound() == Catch::Approx(340.294));
  }
  SECTION("Test pressure at 11 km altitude") {
    atmosphere.update(11000);
    REQUIRE(atmosphere.get_pressure() == Catch::Approx(22632.1));
  }
  SECTION("Test temperature at 11 km altitude") {
    atmosphere.update(11000);
    REQUIRE(atmosphere.get_temperature() == Catch::Approx(216.65));
  }
  SECTION("Test density at 11 km altitude") {
    atmosphere.update(11000);
    REQUIRE(atmosphere.get_density() == Catch::Approx(0.363918));
  }
  SECTION("Test speed of sound at 11 km altitude") {
    atmosphere.update(11000);
    REQUIRE(atmosphere.get_speed_of_sound() == Catch::Approx(295.0695));
  }
  SECTION("Test pressure at 20 km altitude") {
    atmosphere.update(20000);
    REQUIRE(atmosphere.get_pressure() == Catch::Approx(5474.89));
  }
  SECTION("Test temperature at 20 km altitude") {
    atmosphere.update(20000);
    REQUIRE(atmosphere.get_temperature() == Catch::Approx(216.65));
  }
  SECTION("Test density at 20 km altitude") {
    atmosphere.update(20000);
    REQUIRE(atmosphere.get_density() == Catch::Approx(0.088035));
  }
  SECTION("Test speed of sound at 20 km altitude") {
    atmosphere.update(20000);
    REQUIRE(atmosphere.get_speed_of_sound() == Catch::Approx(295.07));
  }
  SECTION("Test pressure at 32 km altitude") {
    atmosphere.update(32000);
    REQUIRE(atmosphere.get_pressure() == Catch::Approx(868.015));
  }
  SECTION("Test temperature at 32 km altitude") {
    atmosphere.update(32000);
    REQUIRE(atmosphere.get_temperature() == Catch::Approx(228.65));
  }
  SECTION("Test density at 32 km altitude") {
    atmosphere.update(32000);
    REQUIRE(atmosphere.get_density() == Catch::Approx(0.013225));
}
};