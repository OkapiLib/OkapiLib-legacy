#ifndef OKAPI_AVGFILTER
#define OKAPI_AVGFILTER

#include "filter/filter.h"
#include "util/mathUtil.h"

namespace okapi {
  class EKFFilter final : public Filter {
  public:
    EKFFilter(const float iQ = 0.0001,  const float iR = ipow(0.2, 2)):
      Q(iQ),
      R(iR),
      xHat(0),
      xHatPrev(0),
      xHatMinus(0),
      P(0),
      Pprev(1),
      Pminus(0),
      K(0) {}

    virtual ~EKFFilter() = default;

    float filter(const float ireading) override {
      //Time update
      xHatMinus = xHatPrev;
      Pminus = Pprev + Q;

      //Measurement update
      K = Pminus / (Pminus + R);
      xHat = xHatMinus + K * (ireading - xHatMinus);
      P = (1 - K) * Pminus;

      return xHat;
    }

    float getOutput() const override { return xHat; }
  private:
    const float Q, R;
    float xHat, xHatPrev, xHatMinus, P, Pprev, Pminus, K;
  };
}

#endif /* end of include guard: OKAPI_AVGFILTER */
