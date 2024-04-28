#include "pointMass.hpp"


class Mission {
  public:
    void init();
    
  private:
    Eom eom;
    Geodetic geodetic;
    Atmosphere atmosphere;
    Projectile projectile;


};