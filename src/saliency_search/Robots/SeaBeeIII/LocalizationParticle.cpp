#include "LocalizationParticle.h"
#include <vector>

LocalizationParticle::LocalizationParticle()
{
        //
}

LocalizationParticle::LocalizationParticle(State p_mState)
{
        mState = p_mState;
}

LocalizationParticle::LocalizationParticle(const LocalizationParticle &p)
{
        mState = p.mState;
}

void LocalizationParticle::updatePosition(float m, float a)
{
        mState.mPoint.i += m * cos(D_DEGREE * a);
        mState.mPoint.j += m * sin(D_DEGREE * a);
        //mState.mAngle = a;
}

void LocalizationParticle::updatePosition(Point2D<float> p, float a)
{
        mState.mPoint.i += p.i;
        mState.mPoint.j += p.j;
        mState.mAngle = a;
}

void LocalizationParticle::Print()
{
        cout << "[" << mState.mAngle << "," << mState.mPoint.i << "," << mState.mPoint.j << "]; w=" << mState.mWeight << endl;
}

// Updates the position
void LocalizationParticle::drift_position(float stdDev, float vel, float dt, ParamData pd)
{
        //simple fix; randomDoubleFromNormal() needs to be scaled; before it was just 1 ie 1 foot/s velocity

        //1.5 in/second max sustained velocity due to drift
        float randomDriftDistanceX = (vel + pd["trans_drift"]) * randomDoubleFromNormal(stdDev) * dt;
        float randomDriftDistanceY = (vel + pd["trans_drift"]) * randomDoubleFromNormal(stdDev) * dt;
        updatePosition(Point2D<float>(randomDriftDistanceX, randomDriftDistanceY), mState.mAngle);
}

void LocalizationParticle::drift_angle(float stdDev, float vel, float dt, ParamData pd)
{
        //[vel] degrees/second max sustained angular velocity due to drift
        float randomDriftAngle = (vel + pd["ang_drift"]) * randomDoubleFromNormal(stdDev) * dt;
        updatePosition(Point2D<float>(0.0, 0.0), mState.mAngle + randomDriftAngle);
}

//amt of time it takes to go 1 ft at p% thrusters
float LocalizationParticle::getTime(float p, ParamData pd) //static
{
        //time it takes to go 8 feet
        //return 5.53 / p;

        //time it takes to go 1 foot
        return pd["speed_scalar"] / abs(p);
}

float LocalizationParticle::getVelocity(float p, ParamData pd) //static
{
        //sub goes 1 foot in t(p) seconds
        //return 1.0 / getTime(p);
        return abs(p) / pd["speed_scalar"];
}

float LocalizationParticle::getDistance(float p, float dt, ParamData pd) //static
{
        return getVelocity(p, pd) * dt;
}

float LocalizationParticle::getStdDev(float p)
{
        //samples:
        //[0] : 6.48
        //[1] : 0.2592
        return 6.48 * std::pow((double)0.04, (double)abs(p));
}

float LocalizationParticle::getChangeInRotation(float p, float dt, ParamData pd) //static
{
        //t(p) = amt of time it takes to turn 90 degrees at motor value p
        //90.0 * dt / t(p)
        // 360 / (pi * D) [D = 1.5 ft = distance between thrusters]
        static const float FT_TO_DEG = -360 / (acos(-1.0) * pd["rotate_scalar"]); //degrees rotated when thrusters move one foot
        float scale = 1.0;
        if(p < 0)
        {
                scale = -1.0;
        }
        float theResult = getDistance(p, dt, pd) * FT_TO_DEG;
        return scale * theResult;
}

vector<LocalizationParticle> LocalizationParticle::stepParticles(vector<LocalizationParticle> p, float dt, int i_pX, int i_pY, int i_angle, ParamData pd) //static
{
        float pX = (float)i_pX / 100.0;
        float pY = (float)i_pY / 100.0;

        float distance = getDistance(pY, dt, pd);
        float angle = getChangeInRotation(pX, dt, pd);
        float t_vel = getVelocity(pY, pd);
        float a_vel = getVelocity(pX, pd);
        float dist_stdDev = getStdDev(pY);
        float ang_stdDev = getStdDev(pX);

        for (unsigned int i = 0; i < p.size(); i++)
        {
                //new idea: move to the position, then apply the drift that may have occurred over that time

                p[i].updatePosition(distance, p[i].mState.mAngle + angle + i_angle);
                p[i].mState.mAngle += angle;

                p[i].drift_angle(ang_stdDev, a_vel, dt, pd);
                p[i].drift_position(dist_stdDev, t_vel, dt, pd);
                //p[i].mState.mAngle = normalizeAngle(p[i].mState.mAngle);
        }
        return p;
}

float LocalizationParticle::normalizeAngle(float a)
{
	while(a < 0)
	{
		a += 360;
	}
	while(a >= 360)
	{
		a -= 360;
	}
	return a;
}
