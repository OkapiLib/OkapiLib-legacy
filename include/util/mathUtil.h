#ifndef OKAPI_MATHUTIL
#define OKAPI_MATHUTIL

namespace okapi {
  static constexpr float analogInToV = 286.0;
  static constexpr float inchToMM = 25.4;
  static constexpr float degreeToRadian = 0.01745;
  static constexpr float radianToDegree = 57.2958;
  static constexpr float imeHighTorTPR = 627.2;
  static constexpr float imeHighStrTPR = 392.0;
  static constexpr float imeTurboTPR = 261.333;
  static constexpr float ime269TPR = 240.448;
  static constexpr float quadEncoderTPR = 360.0;
  static constexpr float pi = 3.14159265358979323846;

  constexpr float ipow(float x, int n) {
    return (n == 0) ? 1 :
      n == 1 ? x :
        n > 1 ? ((n & 1) ? x * ipow(x, n-1) : ipow(x, n/2) * ipow(x, n/2)) :
        1 / ipow(x, -n);
  }
}

#endif /* end of include guard: OKAPI_MATHUTIL */
