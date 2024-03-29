#include <cmath>
#include "control/velPid.h"
#include "PAL/PAL.h"

namespace okapi {
  void VelPid::setGains(const float ikP, const float ikD) {
    kP = ikP;
    kD = ikD * static_cast<float>(sampleTime) / 1000.0;
  }

  void VelPid::setSampleTime(const int isampleTime) {
    if (isampleTime > 0) {
      kD /= static_cast<float>(isampleTime) / static_cast<float>(sampleTime);
      sampleTime = isampleTime;
    }
  }

  void VelPid::setOutputLimits(float imax, float imin) {
    //Always use larger value as max
    if (imin > imax) {
      const float temp = imax;
      imax = imin;
      imin = temp;
    }

    outputMax = imax;
    outputMin = imin;

    //Fix output
    if (output > outputMax)
      output = outputMax;
    else if (output < outputMin)
      output = outputMin;
  }

  float VelPid::stepVel(const float inewReading) {
    return velMath.step(inewReading);
  }

  float VelPid::step(const float inewReading) {
    if (isOn) {
      const long now = PAL::millis();
      if (now - lastTime >= sampleTime) {
        stepVel(inewReading);
        const float error = target - velMath.getOutput();

        const float derivative = velMath.getDiff(); //Derivative over measurement to eliminate derivative kick on setpoint change

        output += kP * error - kD * derivative;

        if (output > outputMax)
          output = outputMax;
        else if (output < outputMin)
          output = outputMin;

        lastError = error;
        lastTime = now; //Important that we only assign lastTime if dt >= sampleTime
      }

      return output;
    }

    return 0;
  }

  void VelPid::reset() {
    error = 0;
    lastError = 0;
    output = 0;
  }
}
