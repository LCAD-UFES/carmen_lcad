#include "LocalizationUtil.h"
#include <vector>

#ifndef LOCALIZATIONPARTICLE_H_
#define LOCALIZATIONPARTICLE_H_

class LocalizationParticle
{
public:
        struct State
        {
                Point2D<float> mPoint;
                float mAngle; //theta in degrees
                float mWeight; // weight[k]
        };

        LocalizationParticle();
        LocalizationParticle(State p_mState);
        LocalizationParticle(const LocalizationParticle &p);

        void updatePosition(float m, float a);
        void updatePosition(Point2D<float> p, float a);
        void Print();
        void drift_position(float stdDev, float vel, float dt, ParamData pd);
        void drift_angle(float stdDev, float vel, float dt, ParamData pd);
        static float getTime(float p, ParamData pd);
        static float getVelocity(float p, ParamData pd);
        static float getDistance(float p, float dt, ParamData pd);
        static float getStdDev(float p);
        static float getChangeInRotation(float p, float dt, ParamData pd);
        static vector<LocalizationParticle> stepParticles(vector<LocalizationParticle> p, float dt, int i_pX, int i_pY, int i_angle, ParamData pd);
        static float normalizeAngle(float a);

        State mState;
};

#endif /* LOCALIZATIONPARTICLE_H_ */
