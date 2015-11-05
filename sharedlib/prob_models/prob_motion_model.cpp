#include "prob_motion_model.h"
#include <assert.h>
#include <math.h>

static MotionModelTypes motion_model_type = OdometryMotionModel;
static OdometryMotionModelParams odometry_model_params;
static VelocityMotionModelParams velocity_model_params;
static AckermanMotionModelParams ackerman_model_params;

void init_odometry_motion_model(OdometryMotionModelParams params)
{
	motion_model_type = OdometryMotionModel;

	odometry_model_params.alpha1 = params.alpha1;
	odometry_model_params.alpha2 = params.alpha2;
	odometry_model_params.alpha3 = params.alpha3;
	odometry_model_params.alpha4 = params.alpha4;
}

void init_velocity_motion_model(VelocityMotionModelParams params)
{
	motion_model_type = VelocityMotionModel;

	velocity_model_params.alpha1 = params.alpha1;
	velocity_model_params.alpha2 = params.alpha2;
	velocity_model_params.alpha3 = params.alpha3;
	velocity_model_params.alpha4 = params.alpha4;
	velocity_model_params.alpha5 = params.alpha5;
	velocity_model_params.alpha6 = params.alpha6;
}

void init_ackerman_motion_model(AckermanMotionModelParams params)
{
	motion_model_type = AckermanMotionModel;

	ackerman_model_params.L = params.L;
	ackerman_model_params.alpha1 = params.alpha1;
	ackerman_model_params.alpha2 = params.alpha2;
	ackerman_model_params.alpha3 = params.alpha3;
	ackerman_model_params.alpha4 = params.alpha4;
}

/**
 * This algorithm accepts as an input the robot's command ut and pose xt at time t-1 (xt_1) and
 * outputs a random pose and time t, xt, distributed according p(xt | ut, xt_1).
 * @param ut a vector of two robot pose values, o_xt_1, o_xt, provided by the robot's odometry at time t-1 and t.
 * @param xt_1 a previous robot pose in the real world at time t-1 (expected).
 */
carmen_point_t sample_motion_model_odometry(const OdometryMotionCommand *ut, carmen_point_t xt_1)
{
	int backwards;

	double dx, dy;
	double delta_rot1, delta_rot2, delta_trans;
	double dhat_rot1, dhat_trans, dhat_rot2;
	double std_rot1, std_rot2, std_trans;

	double odom_a1 = odometry_model_params.alpha1;
	double odom_a2 = odometry_model_params.alpha2;
	double odom_a3 = odometry_model_params.alpha3;
	double odom_a4 = odometry_model_params.alpha4;

	carmen_point_t xt = xt_1;

	assert(motion_model_type == OdometryMotionModel);

	dx = ut->final.x - ut->initial.x;
	dy = ut->final.y - ut->initial.y;

	delta_trans = sqrt(dx * dx + dy * dy);

	backwards = ( dx * cos(ut->final.theta) +
			dy * sin(ut->final.theta) < 0);

	/* The dr1/dr2 code becomes unstable if delta_t is too small. */
	if(delta_trans < 0.05)
	{
		delta_rot1 = carmen_normalize_theta(ut->final.theta - ut->initial.theta) / 2.0;
		delta_rot2 = delta_rot1;
	}
	else
	{
		if(backwards)
			delta_rot1 = carmen_normalize_theta(
					atan2(ut->initial.y - ut->final.y, ut->initial.x - ut->final.x) - ut->initial.theta);
		else
			delta_rot1 = carmen_normalize_theta(
					atan2(ut->final.y - ut->initial.y, ut->final.x - ut->initial.x) - ut->initial.theta);

		delta_rot2 = carmen_normalize_theta(ut->final.theta - ut->initial.theta - delta_rot1);
	}

	/* compute motion model parameters */
	std_rot1 = odom_a1 * fabs(delta_rot1) + odom_a2 * delta_trans;
	std_trans = odom_a3 * delta_trans + odom_a4 * fabs(delta_rot1 + delta_rot2);
	std_rot2 = odom_a1 * fabs(delta_rot2) + odom_a2 * delta_trans;

	dhat_rot1 = carmen_gaussian_random(delta_rot1, std_rot1);
	dhat_trans = carmen_gaussian_random(delta_trans, std_trans);
	dhat_rot2 = carmen_gaussian_random(delta_rot2, std_rot2);

	if(backwards)
	{
		xt.x -= dhat_trans * cos(xt.theta + dhat_rot1);
		xt.y -= dhat_trans * sin(xt.theta + dhat_rot1);
	}
	else
	{
		xt.x += dhat_trans * cos(xt.theta + dhat_rot1);
		xt.y += dhat_trans * sin(xt.theta + dhat_rot1);
	}
	xt.theta = carmen_normalize_theta(xt.theta + dhat_rot1 + dhat_rot2);

	return xt;
}

carmen_point_t sample_motion_model_velocity(const VelocityMotionCommand *ut, carmen_point_t xt_1)
{
	carmen_point_t xt = {0.0, 0.0, 0.0};

	assert(motion_model_type == VelocityMotionModel);

	double alpha1 = velocity_model_params.alpha1;
	double alpha2 = velocity_model_params.alpha2;
	double alpha3 = velocity_model_params.alpha3;
	double alpha4 = velocity_model_params.alpha4;
	double alpha5 = velocity_model_params.alpha5;
	double alpha6 = velocity_model_params.alpha6;

	double v = ut->v;
	double w = ut->w;
	double dt = ut->delta_t;

	double _v = v + carmen_normal_distribution(alpha1 * v * v + alpha2 * w * w);
	double _w = w + carmen_normal_distribution(alpha3 * v * v + alpha4 * w * w);
	double gamma = carmen_normal_distribution(alpha5 * v * v + alpha6 * w * w);

	double v_divided_by_w = _v / _w;

	double cos_theta = cos(xt_1.theta);
	double sin_theta = sin(xt_1.theta);

	double sin_theta_dt = sin(xt_1.theta + _w * dt);
	double cos_theta_dt = cos(xt_1.theta + _w * dt);

	if ( isnan(v_divided_by_w) || isinf(v_divided_by_w) )
	{
		xt.x = xt_1.x + _v * cos_theta * dt;
		xt.y = xt_1.y + _v * sin_theta * dt;
		xt.theta = xt_1.theta;
	}
	else
	{
		xt.x = xt_1.x - v_divided_by_w * sin_theta + v_divided_by_w * sin_theta_dt;
		xt.y = xt_1.y + v_divided_by_w * cos_theta - v_divided_by_w * cos_theta_dt;
		xt.theta = carmen_normalize_theta(xt_1.theta + _w * dt + gamma * dt);
	}
	return xt;
}

carmen_point_t sample_motion_model_ackerman(const AckermanMotionCommand *ut, carmen_point_t xt_1, int converged)
{
	carmen_point_t xt = {0.0, 0.0, 0.0};

	assert(motion_model_type == AckermanMotionModel);

	double alpha1 = ackerman_model_params.alpha1;
	double alpha2 = ackerman_model_params.alpha2;
	double alpha3 = ackerman_model_params.alpha3;
	double alpha4 = ackerman_model_params.alpha4;

	if (converged)
	{
		double v = ut->v;
		double phi = ut->phi;
		double dt = ut->delta_t/2.0;
		double L = ackerman_model_params.L;

		double _v_step1 = v + carmen_gaussian_random(0.0, alpha1 * v * v + alpha2 * phi * phi);
		double _phi_step1 = phi + carmen_gaussian_random(0.0, alpha3 * phi * phi + alpha4 * v * v);

		if (_phi_step1 > M_PI/4.0)
		{
			_phi_step1 = M_PI/4.0;
		}
		else if (_phi_step1 < -M_PI/4.0)
		{
			_phi_step1 = -M_PI/4.0;
		}

		xt.x = xt_1.x + _v_step1 * dt * cos(xt_1.theta);
		xt.y = xt_1.y + _v_step1 * dt * sin(xt_1.theta);
		xt.theta = carmen_normalize_theta(xt_1.theta + _v_step1 * dt * tan(_phi_step1) / L);

		double _v_step2 = _v_step1 + carmen_gaussian_random(0.0, alpha1 * _v_step1 * _v_step1 + alpha2 * _phi_step1 * _phi_step1);
		double _phi_step2 = _phi_step1 + carmen_gaussian_random(0.0, alpha3 * _phi_step1 * _phi_step1 + alpha4 * _v_step1 * _v_step1);

		if (_phi_step2 > M_PI/4.0)
		{
			_phi_step2 = M_PI/4.0;
		}
		else if (_phi_step2 < -M_PI/4.0)
		{
			_phi_step2 = -M_PI/4.0;
		}

		xt.x = xt.x + _v_step2 * dt * cos(xt.theta);
		xt.y = xt.y + _v_step2 * dt * sin(xt.theta);
		xt.theta = carmen_normalize_theta(xt.theta + _v_step2 * dt * tan(_phi_step2) / L);
	}
	else
	{	// moves like a free-flying-object
		double v = ut->v + carmen_gaussian_random(0.0, (alpha1 + alpha4) * ut->v * ut->v + (alpha2 + alpha3) * ut->phi * ut->phi);
		double theta = carmen_normalize_theta(xt_1.theta + carmen_gaussian_random(0.0, (alpha1 + alpha4) * v * v));
		double dt = ut->delta_t;

		xt.x = xt_1.x + v * dt * cos(theta);
		xt.y = xt_1.y + v * dt * sin(theta);
		xt.theta = theta;
	}
	
	return xt;
}

/**
 * When timestamp > 0.0 indicate that velocity command was initialized.
 */
void
init_velocity_motion_command(VelocityMotionCommand *ut,
		const double timestamp)
{
	ut->last_timestamp = timestamp;
}

void
init_ackerman_motion_command(AckermanMotionCommand *ut,
		const double timestamp)
{
	ut->last_timestamp = timestamp;
}

int
update_velocity_motion_command(VelocityMotionCommand *ut,
		const double v, const double w, const double timestamp)
{
	if (ut->last_timestamp <= 0.0)
	{
		init_velocity_motion_command(ut, timestamp);
		return 0;
	}
	if (isnan(ut->v) || isnan(ut->w) ||
			isinf(ut->v) || isinf(ut->w))
	{
		ut->last_timestamp = 0.0;
		return 0;
	}

	ut->delta_t = timestamp - ut->last_timestamp;
	ut->w = w;
	ut->v = v;

	ut->last_timestamp = timestamp;

	return 1;
}

int
update_ackerman_motion_command(AckermanMotionCommand *ut,
		const double v, const double phi, const double timestamp)
{
	if (ut->last_timestamp <= 0.0)
	{
		init_ackerman_motion_command(ut, timestamp);
		return 0;
	}

	ut->delta_t = timestamp - ut->last_timestamp;
	ut->phi = phi;
	ut->v = v;

	ut->last_timestamp = timestamp;

	return 1;
}

void
update_odometry_motion_command(OdometryMotionCommand *ut, carmen_point_t odometry)
{
	ut->initial = ut->final;
	ut->final = odometry;
}
