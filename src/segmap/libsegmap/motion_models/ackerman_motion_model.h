

#ifndef __ACKERMAN_MOTION_MODEL_H__
#define __ACKERMAN_MOTION_MODEL_H__

#include <carmen/segmap_pose2d.h>

void ackerman_motion_model(double &x, double &y, double &th, double v, double phi, double dt);
void ackerman_motion_model(Pose2d &pose, double v, double phi, double dt);

#endif
