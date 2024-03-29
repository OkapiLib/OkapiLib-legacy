#ifndef OKAPI_POTENTIOMETER
#define OKAPI_POTENTIOMETER

#include "PAL/PAL.h"

namespace okapi {
  class Potentiometer {
  public:
    explicit constexpr Potentiometer(const unsigned char iport, const bool iinverted = false):
      port(iport),
      inverted(iinverted) {}

    int get() const { return inverted ? 4095 - PAL::analogRead(port) : PAL::analogRead(port); }
  private:
    const unsigned char port;
    const bool inverted;
  };

  inline namespace literals {
    constexpr Potentiometer operator"" _p(const unsigned long long int p) { return Potentiometer(static_cast<unsigned char>(p), false); }
    constexpr Potentiometer operator"" _ip(const unsigned long long int p) { return Potentiometer(static_cast<unsigned char>(p), true); }
  }
}

#endif /* end of include guard: OKAPI_POTENTIOMETER */