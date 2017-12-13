#ifndef OKAPI_VELOCITY
#define OKAPI_VELOCITY

#include "filter/ekfFilter.h"
#include "util/mathUtil.h"
#include "PAL/PAL.h"

namespace okapi {
  class VelMathParams {
    public:
      VelMathParams(const float iticksPerRev, const float iQ = 0.0001, const float iR = ipow(0.2, 2)):
        ticksPerRev(iticksPerRev),
        Q(iQ),
        R(iR) {}

      float ticksPerRev, Q, R;
  };

  class VelMath {
  public:
    VelMath(const float iticksPerRev, const float iQ = 0.0001, const float iR = ipow(0.2, 2)):
      lastTime(0),
      vel(0),
      lastVel(0),
      lastPos(0),
      ticksPerRev(iticksPerRev),
      filter(iQ, iR) {}

    VelMath(const VelMathParams& iparams):
      lastTime(0),
      vel(0),
      lastVel(0),
      lastPos(0),
      ticksPerRev(iparams.ticksPerRev),
      filter(iparams.Q, iparams.R) {}

    /**
     * Calculate new velocity
     * @param  inewPos New position
     * @return         New velocity
     */
    float step(const float inewPos) {
      const long now = PAL::millis();

      vel = static_cast<float>((1000 / (now - lastTime))) * (inewPos - lastPos) * (60 / ticksPerRev);
      vel = filter.filter(vel);

      lastPos = inewPos;
      lastTime = now;

      return vel;
    }

    void setTicksPerRev(const float iTPR) { ticksPerRev = iTPR; }

    float getOutput() const { return vel; }

    float getDiff() const { return vel - lastVel; }
  private:
    long lastTime;
    float vel, lastVel, lastPos, ticksPerRev;
    EKFFilter filter;
  };
}

#endif /* end of include guard: OKAPI_VELOCITY */
