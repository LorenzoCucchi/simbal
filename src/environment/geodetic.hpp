class Geodetic {

 public:
  Geodetic();
  void init(double latitude, double altitude);

  void update(double latitude, double altitude);

  [[nodiscard]] auto get_gravity() const -> double { return gravity; }

  [[nodiscard]] auto get_radiusE() const -> double { return radiusE; }

 private:
  void wgs84();

  // State variables
  double latitude = 0.0 ;     // [rad]
  double altitudeMSL = 0.0;  // [m]
  double gravity{};      // [m/s^2]

  // Miscellaneous
  double a2;       // [m^2]
  double b2;       // [m^2]
  double a4;       // [m^4]
  double b4;       // [m^4]
  double e2;       // [-]
  double radiusE;  // [m]
  double gamma;    // [m/s^2]

  double gammaE =
      9.7803253359;  // [m/s^2] Normal gravity at the equator (on the ellispoid)
  double k = 1.931852652458e-03;  // [-]     Somigliana’s Formula - normal
                                  // gravity formula constant
  double e =
      8.1819190842622e-02;  // [-]     First eccentricity of the ellispoid
  double a = 6378137.0;     // [m]     Semi-major axis of the ellipsoid
  double b = 6356752.3142;  // [m]     Semi-minor axis of the ellipsoid
  double f = 3.3528106647475e-03;  // [-]     WGS 84 flattening (reduced)
  double m = 3.449786506841e-03;   // [-]     Normal gravity formula
                                   // constant (w^2*a^2*b/GM)
};