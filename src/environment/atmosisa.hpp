/* U.S. Standard Atmosphere (1976) model from sea-level to 86Km. */
#include <array>
class Atmosphere {

 public:
  /**
   * @brief Construct a new Atmosphere object
   *
   * @param altitude altitude in meters
   */
  explicit Atmosphere(double altitude = 0.0) : altitude(altitude) {
    update(altitude);
  }
  /**
   * @brief Updates the atmospheric data based on the altitude using the
   * ATMOSISA model
   *
   * @param target_altitude altitude in meters
   * @return * void
   */
  void update(double target_altitude);

  /**
   * @brief getter function for the temperature
   *
   * @return double
   */
  [[nodiscard]] auto get_temperature() const -> double { return temperature; }

  /**
   * @brief getter function for the pressure
   *
   * @return double
   */
  [[nodiscard]] auto get_pressure() const -> double { return pressure; }

  /**
   * @brief getter function for the density
   *
   * @return double
   */
  [[nodiscard]] auto get_density() const -> double { return density; }

  /**
   * @brief getter function for the speed of sound
   *
   * @return double
   */
  [[nodiscard]] auto get_sound() const -> double { return speed_of_sound; }

 private:
  /**
   * @brief Private variable that stores the altitude in km
   *
   */
  double altitude;

  /**
   * @brief Private variables that store the temperature in K
   *
   */
  double temperature{};

  /**
   * @brief Private variables that store the pressure in Pa
   *
   */
  double pressure{};

  /**
   * @brief Private variables that store the density in kg/m^3
   *
   */
  double density{};

  /**
   * @brief Private variables that store the speed of sound in m/s
   *
   */
  double speed_of_sound{};

  /**
   * @brief Private functions that calculates the pressure and temperature based
   * on the altitude
   *
   * @return void
   */
  void atmos_press_temp();

  /**
   * @brief Private function that calculates the density based on the pressure
   * and temperature
   *
   * @return void
   */
  void atmos_density();

  /**
   * @brief Private function that calculates the speed of sound based on the
   * temperature
   *
   * @return void
   */
  void atmos_speed_of_sound();

  /**
   * @brief Private function that computes the pressure based on the temperature
   * and altitude
   *
   * @return double
   */
  [[nodiscard]] auto compute_pressure(double start_temperature, double start_gradient,
                        double next_altitude, double start_altitude,
                        double start_pressure) const -> double;

  /* Boltzmann constant (N*m/K) */
  double k = 1.380622e-23;

  /* Molecular weights values of air components */
  std::array<double, 10> M = {28.0134, 31.9988, 39.948, 44.00995, 20.183,
                              4.0026,  83.80,   131.30, 16.04303, 2.01594};

  /* Avogadro's number */
  double N_A = 6.022169e23;

  /* Gas constant (N*m/kmol/K) */
  double R_star = 8.31432e3;

  /* Fractional volume concentrations (dimensionless) */
  std::array<double, 10> F = {0.78084,    0.209476,   0.00934,    0.000314,
                              0.00001818, 0.00000524, 0.00000114, 0.000000087,
                              0.000002,   0.0000005};

  /* Mean-Sea-level value of the acceleration of gravity (m/s^2) */
  double g_0 = 9.80665;

  /* Base of the successive atmospheric layers (km) */
  std::array<double, 8> H = {0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.8520};

  /* Gradients of molecular scale temperature w.r.t. the geopotential height
   * (K/km) */
  std::array<double, 7> L_Mb = {-6.5, 0.0, 1.0, 2.8, 0.0, -2.8, -2.0};

  /* Sea-level atmospheric pressure (Pa) */
  double P_0 = 1.013250e5;

  /* Effective earth's radius (Km) */
  double r_0 = 6.356766e6;

  /* Sea-level temperature (K) */
  double T_0 = 288.15;

  /* Sutherland constant (K) */
  double S = 110.0;

  /* Dynamic viscosity constant (Kg/s/m/K^(1/2)) */
  double beta = 1.458e-6;

  /* Ratio of specific heat of air at constant pressure to the specific heat of
   * air at constant volume (dimensionless) */
  double gamma = 1.4;

  /* Mean effective collision diameter of molecules (m) */
  double sigma = 3.65e-10;

  /* Air gas constant */
  double r = 287.05;

  /* Molecular weight of the atmosphere, from reference table 3. */
  double M_0 = 28.0134 * 0.78084 + 31.9988 * 0.209476 + 39.948 * 0.00934 +
               44.00995 * 0.000314 + 20.183 * 0.00001818 + 4.0026 * 0.00000524 +
               83.80 * 0.00000114 + 131.30 * 0.000000087 + 16.04303 * 0.000002 +
               2.01594 * 0.0000005;
};