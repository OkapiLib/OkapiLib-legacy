#ifndef OKAPI_AUTOPID
#define OKAPI_AUTOPID

#include <string>

namespace okapi {
    class AutoPid {
    public:
        AutoPid();
        virtual ~AutoPid();

        void init(float Kp, float Ki, float Kd);
        void updateError(float cte);
        float totalError();
        void evaluate();
        void backProp();
        void resetEpochError();

        float getKp() const;
        float getKi() const;
        float getKd() const;

        int counter_;
        bool needsTraining_;
        float currentEpochError_;
        const int epochLength_ = 200;
    private:
        float p_error, i_error, d_error, i_e_fabs_, Kp, Ki, Kd, epochCumulativeError_, previousEpochError_;
        const float errorThreshold_ = 0.0005, learnRate_ = 0.000001;

        void updateEpochError(float cte);
        void adjust(float &Kx, float dx, float dE);
        void printMessage(const std::string& message, float value);
    };
}

#endif /* end of include guard: OKAPI_AUTOPID */
