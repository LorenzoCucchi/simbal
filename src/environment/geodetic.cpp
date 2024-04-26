#include "geodetic.hpp"
#include <cmath>

void Geodetic::init(double latitude, double altitude) {
  this->latitude = latitude;
  this->altitudeMSL = altitude;
  this->a2 = this->a * this->a;
  this->b2 = this->b * this->b;
  this->a4 = this->a2 * this->a2;
  this->b4 = this->b2 * this->b2;
  this->e2 = this->e * this->e;
  this->wgs84();
}

void Geodetic::update(double latitude, double altitude) {
  this->latitude = latitude;
  this->altitudeMSL = altitude;
  this->wgs84();
}

void Geodetic::wgs84() {
  double sin2phi = sin(this->latitude) * sin(this->latitude);
  double cos2phi = cos(this->latitude) * cos(this->latitude);

  this->gamma =
      this->gammaE * (1 + this->k * sin2phi) / sqrt(1 - this->e2 * sin2phi);

  this->radiusE = sqrt((this->a4 * cos2phi + this->b4 * sin2phi) /
                       (this->a2 * cos2phi + this->b2 * sin2phi));

  this->gravity =
      this->gamma *
      (1 -
       (2 / this->a) * (1 + this->f + this->m - 2 * this->f * sin2phi) *
           this->altitudeMSL +
       (3 / this->a2) * pow(this->altitudeMSL, 2));
}
