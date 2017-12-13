#ifndef OKAPI_MOTOR
#define OKAPI_MOTOR

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2012                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     SmartMotorLib.c                                              */
/*    Author:     James Pearman                                                */
/*    Created:    2 Oct 2012                                                   */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  21 Oct 2012 - Initial release                         */
/*                V1.01   7 Dec 2012                                           */
/*                       small bug in SmartMotorLinkMotors                     */
/*                       fix for High Speed 393 Ke constant                    */
/*                       kNumbOfTotalMotors replaced with kNumbOfRealMotors    */
/*                       _Target_Emulator_ defined for versions of ROBOTC      */
/*                       prior to 3.55                                         */
/*                       change to motor enums for V3.60 ROBOTC compatibility  */
/*               V1.02  27 Jan 2013                                            */
/*                      Linking an encoded and non-encoded motor was not       */
/*                      working correctly, added new field to the structure    */
/*                      eport to allow one motor to access the encoder for     */
/*                      another correctly.                                     */
/*               V1.03  10 March 2013                                          */
/*                      Due to new version of ROBOTC (V3.60) detection of PID  */
/*                      version changed. V3.60 was originally planned to have  */
/*                      different motor definitions.                           */
/*                      Added the ability to assign any sensor to be used      */
/*                      for rpm calculation, a bit of a kludge as I didn't     */
/*                      want to add a new variable so reused encoder_id        */
/*               V1.04  27 June 2013                                           */
/*                      Change license (added) to the Apache License           */
/*               V1.05  11 Nov 2013                                            */
/*                      Fix bug when speed limited and changing directions     */
/*                      quickly.                                               */
/*               V1.06  3 Sept 2014                                            */
/*                      Added support for the 393 Turbo Gears and ROBOTC V4.26 */
/*               V1.07  17 April 2016                                          */
/*                      Allow motor control disable using a slew rate of 0     */
/*               V1.08  8 May 2016                                             */
/*                      Change function calls to be compatible with V4.XX      */
/*                      Add _NO_WARNING macro                                  */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    Portions of this code are based on work by Chris Siegert aka vamfun on   */
/*    the Vex forums.                                                          */
/*    blog:  vamfun.wordpress.com for model details and vex forum threads      */
/*    email: vamfun_at_yahoo_dot_com                                           */
/*    Mentor for team 599 Robodox and team 1508 Lancer Bots                    */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    email: jbpearman_at_mac_dot_com                                          */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Description:                                                             */
/*                                                                             */
/*    This library is designed to work with encoded motors and provides        */
/*    functions to obtain velocity, instantaneous current and estimated PTC    */
/*    temperature.  This data is then used to allow fixed threshold or         */
/*    temperature based current limiting.                                      */
/*                                                                             */
/*    The algorithms used are based on experiments and work done during the    */
/*    summer of 2012 by Chris Siegert and James Pearman.                       */
/*                                                                             */
/*    This library makes extensive use of pointers and therefore needs ROBOTC  */
/*    V3.51 or later.                                                          */
/*                                                                             */
/*    The following vexforum threads have much of the background information   */
/*    used in this library.                                                    */
/*    http://www.vexforum.com/showthread.php?t=72100                           */
/*    http://www.vexforum.com/showthread.php?t=73318                           */
/*    http://www.vexforum.com/showthread.php?t=73960                           */
/*    http://www.vexforum.com/showthread.php?t=74594                           */
/*                                                                             */
/*    Global Memory use for V1.02 is 1404 bytes                                */
/*      1240 for motor data                                                    */
/*       156 for controller data                                               */
/*         8 misc                                                              */
/*                                                                             */
/*    CPU time for SmartMotorTask                                              */
/*    Motor calculations ~ 530uS,  approx 5% cpu bandwidth                     */
/*    Controller calculations with LED status ~ 1.25mS                         */
/*    Worse case is therefore about 1.8mS which occurs every 100mS             */
/*                                                                             */
/*    CPU time for SmartMotorSlewRateTask                                      */
/*    approx 400uS per 15mS loop, about 3% cpu bandwidth                       */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include "PAL/PAL.h"

namespace okapi {
  namespace motor {
    constexpr int trueSpeed[128] = {
      0,18,21,21,22,23,24,24,24,25,25,26,26,26,27,27,27,28,28,28,28,29,29,29,
      30,30,30,30,31,31,31,31,32,32,33,33,33,33,34,34,34,35,35,35,36,36,36,37,
      37,37,38,38,38,38,39,39,39,40,40,40,41,41,41,42,42,42,43,43,43,44,44,44,
      45,45,45,46,46,46,47,47,48,48,49,49,50,50,51,51,51,52,52,53,53,54,54,55,
      55,56,57,58,59,59,60,61,62,64,65,66,67,67,68,71,72,73,73,76,78,79,81,82,
      83,90,95,115,122,123,124,127
    };
    constexpr int cubicSpeed[128] = {
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,
      3,3,3,4,4,4,5,5,5,6,6,6,7,7,8,8,9,9,10,10,11,11,12,13,13,14,15,16,16,17,
      18,19,19,20,21,22,23,24,25,26,27,28,29,31,32,33,34,35,37,38,39,41,42,44,
      45,47,48,50,51,53,55,57,58,60,62,64,66,68,70,72,74,76,78,80,83,85,87,89,
      92,94,97,99,102,104,107,110,113,115,118,121,124,127
    };
  }

  class Motor {
  public:
    explicit constexpr Motor(const unsigned char iport, const int *its):
      port(iport),
      sign(1),
      ts(its) {}

    explicit constexpr Motor(const unsigned char iport, const int isign = 1, const int *its = motor::trueSpeed):
      port(iport),
      sign(isign),
      ts(its) {}

    constexpr Motor(const Motor& other):
      port(other.port),
      sign(other.sign),
      ts(motor::trueSpeed) {}

    virtual void set(const int val) const { PAL::motorSet(port, val * sign); }

    virtual void setTS(const int val) const {
      if (val > 127)
        PAL::motorSet(port, ts[127] * sign);
      else if (val < -127)
        PAL::motorSet(port, ts[127] * -1 * sign);
      else if (val < 0)
        PAL::motorSet(port, ts[-1 * val] * -1 * sign);
      else
        PAL::motorSet(port, ts[val] * sign);
    }

  protected:
    const unsigned char port;
    const int sign;
    const int *ts;
  };

  class CubicMotor : public Motor {
  public:
    explicit constexpr CubicMotor(const unsigned char iport, const int *its):
      Motor(iport, 1, its) {}
    
    explicit constexpr CubicMotor(const unsigned char iport, const int isign = 1, const int *its = motor::trueSpeed):
      Motor(iport, isign, its) {}
      
    constexpr CubicMotor(const CubicMotor& other):
      CubicMotor(other.port, other.sign) {}

    virtual void set(const int val) const override {
      if (val > 127)
        PAL::motorSet(port, motor::cubicSpeed[127] * sign);
      else if (val < -127)
        PAL::motorSet(port, motor::cubicSpeed[127] * -1 * sign);
      else if (val < 0)
        PAL::motorSet(port, motor::cubicSpeed[-1 * val] * -1 * sign);
      else
        PAL::motorSet(port, motor::cubicSpeed[val] * sign);
    }

    virtual void setTS(const int val) const override { Motor::setTS(val); }
  };

  class SlewMotor : public Motor {
  public:
    SlewMotor(const Motor& imotor, const float islewRate):
      Motor(imotor),
      slewRate(islewRate) {}

    virtual void set(const int val) {
      slew(val);
      Motor::set((int)artSpeed);
    }

    virtual void setTS(const int val) {
      slew(val);
      Motor::setTS((int)artSpeed);
    }
  protected:
    float slewRate, artSpeed = 0;

    __attribute__((always_inline))
    void slew(const int val) {
      if (artSpeed != val) {
        if (val > artSpeed) {
          artSpeed += slewRate;
          if (artSpeed > val)
            artSpeed = static_cast<float>(val);
        } else if (val < artSpeed) {
          artSpeed -= slewRate;
          if (artSpeed < val)
            artSpeed = static_cast<float>(val);
        }
      }
    }
  };

  class CubicSlewMotor : public CubicMotor {
  public:
    CubicSlewMotor(const CubicMotor& imotor, const float islewRate):
      CubicMotor(imotor),
      slewRate(islewRate) {}
    
    virtual void set(const int val) {
      slew(val);
      CubicMotor::set((int)artSpeed);
    }

    virtual void setTS(const int val) {
      slew(val);
      CubicMotor::setTS((int)artSpeed);
    }
  protected:
    float slewRate, artSpeed = 0;
    
    __attribute__((always_inline))
    void slew(const int val) {
      if (artSpeed != val) {
        if (val > artSpeed) {
          artSpeed += slewRate;
          if (artSpeed > val)
            artSpeed = static_cast<float>(val);
        } else if (val < artSpeed) {
          artSpeed -= slewRate;
          if (artSpeed < val)
            artSpeed = static_cast<float>(val);
        }
      }
    }
  };

  class SmartMotor : public Motor {
  public:
    SmartMotor(const RotarySensor& iRS, const float itpr):
      t_const_1(c1393),
      t_const_2(c2393),
      t_ambient(tempAmbient),
      current(0),
      peak_current(0),
      safe_current(iSafeCortex),
      i_free(iFree393),
      i_stall(iStall393),
      r_motor(r393),
      l_motor(l393),
      ke_motor(ke393),
      rpm_free(rpmFree393),
      ticks_per_rev(itpr),
      safe_current(iSafe393),
      enc(0),
      oldenc(0),
      t_ambient(tempAmbient),
      temperature(t_ambient),
      target_current(safe_current),
      limit_current(safe_current),
      limit_tripped(0),
      ptc_tripped(0),
      limit_cmd(motorMaxCmdUndefined),
      v_bemf_max(ke_motor * rpm_free) {}
  
  protected:
    constexpr int motorMaxCmdUndefined = 255;
    constexpr float rSys = 0.3, pwmFreq = 1150, vDiode = 0.75;
    constexpr float iFree393 = 0.2, iStall393 = 4.8, rpmFree393 = 110, r393 = 7.2 / iStall393, l393 = 0.00065,
      ke393 = 7.2 * (1 - iFree393 / iStall393) / rpmFree393, iSafe393 = 0.9;
    constexpr float iSafeCortex = 3.0, iSafePE = 3.0;
    constexpr float tempAmbient = (72.0 - 32.0) * 5 / 9, tempTrip = 100.0, tempHyst = 10.0, tempRef = 25.0;
    constexpr float iHoldCortex = 3.0, tTripCortex = 1.7, kTauCortex = 0.5, tauCortex = kTauCortex * tTripCortex * 5.0 * 5.0,
      c1Cortex = (tempTrip - tempRef) / (iHoldCortex * iHoldCortex), c2Cortex = 1.0 / (tauCortex * 1000.0);
    constexpr float iHold393 = 1.0/* 0.9 */, tTrip393 = 7.1, kTau393 = 0.5, tau393 = kTau393 * tTrip393 * 5.0 * 5.0,
      c1393 = (tempTrip - tempRef) / (iHold393 * iHold393), c2393 = 1.0 / (tau393 * 1000.0);

    short motor_cmd, motor_req, motor_slew;
    short limit_tripped, limit_cmd, limit_current;
    RotarySensor encoder;
    float ticks_per_rev;
    long enc, oldenc;
    float delta, rpm;
    float i_free, i_stall, r_motor, l_motor, ke_motor, rpm_free, v_bemf_max;
    float current, filtered_current, peak_current, safe_current, target_current;
    float temperature, t_const_1, t_const_2, t_ambient, ptc_tripped;
    unsigned long lastTime;
  }

  inline namespace literals {
    constexpr Motor operator"" _m(const unsigned long long int m) { return Motor(static_cast<unsigned char>(m), 1); }
    constexpr Motor operator"" _rm(const unsigned long long int m) { return Motor(static_cast<unsigned char>(m), -1); }
    constexpr CubicMotor operator"" _m3(const unsigned long long int m) { return CubicMotor(static_cast<unsigned char>(m), 1); }
    constexpr CubicMotor operator"" _rm3(const unsigned long long int m) { return CubicMotor(static_cast<unsigned char>(m), -1); }
  }
}

#endif /* end of include guard: OKAPI_MOTOR */
