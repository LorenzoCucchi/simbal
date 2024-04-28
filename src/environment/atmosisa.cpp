#include "atmosisa.hpp"
#include <cmath>

void Atmosphere::update(double target_altitude) {
  this->altitude = target_altitude / 1e3;
  atmos_press_temp();
  atmos_density();
  atmos_speed_of_sound();
}

void Atmosphere::atmos_press_temp() {
  double start_pressure = this->P_0;
  double start_temperature = this->T_0;
  double start_gradient = this->L_Mb[0];
  double start_altitude = this->H[0];

  for (int i = 1; i <= 7; i++) {
    if (this->altitude <= this->H[i]) {
      this->pressure =
          compute_pressure(start_temperature, start_gradient, this->altitude,
                           start_altitude, start_pressure);
      this->temperature = start_temperature +
                          start_gradient * (this->altitude - start_altitude);
      break;
    } else {
      start_pressure =
          compute_pressure(start_temperature, start_gradient, this->H[i],
                           start_altitude, start_pressure);
      start_temperature =
          start_temperature + start_gradient * (this->H[i] - start_altitude);
      start_gradient = this->L_Mb[i];
      start_altitude = this->H[i];
    }
  }
};

void Atmosphere::atmos_density() {
  this->density = this->pressure / (this->r * this->temperature);
};

void Atmosphere::atmos_speed_of_sound() {
  this->speed_of_sound =
      sqrt(this->gamma * this->R_star * this->temperature / this->M_0);
};

double Atmosphere::compute_pressure(double start_temperature,
                                    double start_gradient, double next_altitude,
                                    double start_altitude,
                                    double start_pressure) {
  if (start_gradient != 0.0) {
    return start_pressure *
           std::pow(start_temperature /
                        (start_temperature +
                         start_gradient * (next_altitude - start_altitude)),
                    ((this->g_0 * this->M_0) /
                     (this->R_star * start_gradient / 1e3)));
  } else {
    return start_pressure *
           std::exp(-this->g_0 * this->M_0 * (next_altitude - start_altitude) /
                    (this->R_star * start_temperature / 1e3));
  }
};