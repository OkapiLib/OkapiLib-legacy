#include "control/autoPid.h"

#include <cmath>

namespace okapi {
    AutoPid::AutoPid() {}
    AutoPid::~AutoPid() {}

    void AutoPid::init(float Kp, float Ki, float Kd) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;

        this->p_error = 0;
        this->i_error = 0;
        this->d_error = 0;

        counter_ = 0;

        epochCumulativeError_ = 0;
        previousEpochError_ = 0;
        currentEpochError_ = 0;
        needsTraining_ = true;

        i_e_fabs_ = 0;
    }

    void AutoPid::updateEpochError(float cte) {
        using namespace std;
        i_e_fabs_ += abs(cte);
        epochCumulativeError_ += pow(cte, 2);
    }

    void AutoPid::resetEpochError() {
        i_e_fabs_ = 0;
        epochCumulativeError_ = 0;
    }

    void AutoPid::evaluate() {
        if (needsTraining_) {
            currentEpochError_ = sqrt(epochCumulativeError_ / epochLength_);
            needsTraining_ = currentEpochError_ > errorThreshold_;
        }
    }

    void AutoPid::adjust(float &Kx, float dx, float dE) {
        float partialDKx = Kx * dx * dE * learnRate_;
        Kx -= partialDKx;
    }

    float AutoPid::getKp() const { return Kp; }
    float AutoPid::getKi() const { return Ki; }
    float AutoPid::getKd() const { return Kd; }

    void AutoPid::backProp() {
        float deltaError = previousEpochError_ - currentEpochError_;
        previousEpochError_ = currentEpochError_;

        adjust(Kp, -p_error, deltaError);
        adjust(Ki, -i_e_fabs_, deltaError);
        adjust(Kd, -d_error, deltaError);
    }

    void AutoPid::updateError(float cte) {
        d_error = cte - p_error;
        p_error = cte;
        i_error += cte;

        updateEpochError(cte);
    }

    float AutoPid::totalError() {
        return -Kp * p_error - Ki * i_error - Kd * d_error;
    }
}