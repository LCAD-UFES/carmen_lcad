#include <carmen/carmen.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/velodyne_interface.h>
#include <carmen/matrix.h>
#include <iostream>
#include "virtual_scan.h"

using namespace std;


/* This function draws an ellipse with a given mean and covariance parameters.
 It does this using PCA.  It figures out the eigenvectors and eigenvalues
 (major and minor axes) and draws the ellipse using this transformation
 of coordinates */

void
compute_error_ellipse(double &angle, double &major_axis, double &minor_axis,
		double x_var, double xy_cov, double y_var, double k)
{
	double len, discriminant, eigval1, eigval2, eigvec1x, eigvec1y, eigvec2x, eigvec2y;

	/* check for special case of axis-aligned */
	if (fabs(xy_cov) < (fabs(x_var) + fabs(y_var) + 1e-4) * 1e-4)
	{
		eigval1 = x_var;
		eigval2 = y_var;
		eigvec1x = 1.;
		eigvec1y = 0.;
		eigvec2x = 0.;
		eigvec2y = 1.;
	}
	else
	{
		/* compute axes and scales of ellipse */
		discriminant = sqrt(4 * carmen_square(xy_cov) + carmen_square(x_var - y_var));
		eigval1 = .5 * (x_var + y_var - discriminant);
		eigval2 = .5 * (x_var + y_var + discriminant);
		eigvec1x = (x_var - y_var - discriminant) / (2.0 * xy_cov);
		eigvec1y = 1.0;
		eigvec2x = (x_var - y_var + discriminant) / (2.0 * xy_cov);
		eigvec2y = 1.0;
		/* normalize eigenvectors */
		len = sqrt(carmen_square(eigvec1x) + 1.0);
		eigvec1x /= len;
		eigvec1y /= len;
		len = sqrt(carmen_square(eigvec2x) + 1.0);
		eigvec2x /= len;
		eigvec2y /= len;
	}

	/* take square root of eigenvalues and scale -- once this is
	 done, eigvecs are unit vectors along axes and eigvals are
	 corresponding radii */
	if (eigval1 < 0 || eigval2 < 0)
	{
		eigval1 = 0.001;
		eigval2 = 0.001;
	}
	eigval1 = sqrt(eigval1) * k;
	eigval2 = sqrt(eigval2) * k;
	if (eigval1 < 0.001)
		eigval1 = 0.001;
	if (eigval2 < 0.001)
		eigval2 = 0.001;

	if (eigval1 >= eigval2)
	{
		angle = atan2(eigvec1y, eigvec1x);
		major_axis = eigval1;
		minor_axis = eigval2;
	}
	else
	{
		angle = atan2(eigvec2y, eigvec2x);
		major_axis = eigval2;
		minor_axis = eigval1;
	}
}


void
set_constant_acceleration_system_matrixes(Matrix &F_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double delta_t, double sigma_vx, double sigma_vy, double sigma_r, double sigma_theta)
{
	double aux_data1[6 * 6] =
	{
		1.0,			delta_t, 		(delta_t*delta_t)/2.0, 	0.0,		0.0, 		0.0,
		0.0, 			1.0,	 		delta_t, 				0.0,		0.0,		0.0,
		0.0, 			0.0, 			1.0,					0.0,		0.0,		0.0,
		0.0, 			0.0, 			0.0, 					1.0,		delta_t,	(delta_t*delta_t)/2.0,
		0.0, 			0.0, 			0.0, 					0.0,		1.0,		delta_t,
		0.0, 			0.0, 			0.0, 					0.0,		0.0,		1.0,
	};
	Matrix aux_matrix1(6, 6, aux_data1);
	F_k_1 = aux_matrix1;  // eq. 2.17, pg 11

	double aux_data22[2 * 2] =
	{
		sigma_vx*sigma_vx, 	0.0,
		0.0, 				sigma_vy*sigma_vy,
	};
	Matrix aux_matrix22(2, 2, aux_data22);
	double aux_data23[6 * 2] =
	{
		(delta_t*delta_t)/2.0,	0.0,
		delta_t, 				0.0,
		1.0,					0.0,
		0.0, 					(delta_t*delta_t)/2.0,
		0.0,					delta_t,
		0.0,					1.0,
	};
	Matrix aux_matrix23(6, 2, aux_data23);
	Q_k_1 = aux_matrix23 * aux_matrix22 * ~aux_matrix23;

	double aux_data3[2 * 6] =
	{
		1.0,			0.0, 			0.0,			0.0, 			0.0,			0.0,
		0.0, 			0.0,	 		0.0,			1.0, 			0.0,			0.0,
	};
	Matrix aux_matrix3(2, 6, aux_data3);
	H_k = aux_matrix3;  // eq. 2.38, pg 15

	double aux_data4[2 * 2] =
	{
		sigma_r,		0.0,
		0.0, 			sigma_theta,
	};
	Matrix aux_matrix4(2, 2, aux_data4);
	R_p_k = aux_matrix4;  // eq. 2.34, pg 15
}


void
constant_acceleration_system_setup(double &x, double &y, double &phi, double &v, Matrix &x_k_k, Matrix &P_k_k,
		Matrix &F_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double &delta_t, double &sigma_vx, double &sigma_vy, double &sigma_r, double &sigma_theta)
{
	x = -170.0;
	y = -150.0;
	phi = carmen_degrees_to_radians(45.0);
	v = 5.0;

	delta_t = 1.0;
	sigma_vx = 0.02;
	sigma_vy = 0.02;
	sigma_r = 2.0;
	sigma_theta = carmen_degrees_to_radians(1.0);

	// Initial state
	double X[6 * 1] =
	{
		x - 5.0,		// x + adicao de erro inicial
		v * cos(phi),	// x' (v_x)
		0.0,			// x''
		y + 10.0,		// y + adicao de erro inicial
		v * sin(phi),	// y'
		0.0,			// y''
	};
	Matrix aux_matrix1(6, 1, X);
	x_k_k = aux_matrix1; // eq. 3.27, pg 24

	double P[6 * 6] =
	{
		10.0 * 10.0,	0.0, 			0.0, 			0.0,			0.0,			0.0,
		0.0, 			0.2 * 0.2, 		0.0, 			0.0,			0.0,			0.0,
		0.0, 			0.0, 			1.0 * 1.0,		0.0,			0.0,			0.0,
		0.0, 			0.0, 			0.0,			10.0 * 10.0,	0.0,			0.0,
		0.0, 			0.0, 			0.0, 			0.0,			0.2 * 0.2,		0.0,
		0.0, 			0.0, 			0.0, 			0.0,			0.0,			1.0 * 1.0,
	};
	Matrix aux_matrix2(6, 6, P);
	P_k_k = aux_matrix2;  // eq. 3.33, pg 25

	// Constant Velocity System setup
	set_constant_acceleration_system_matrixes(F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_vx, sigma_vy, sigma_r, sigma_theta);
}


void
set_constant_angular_velocity_system_matrixes(Matrix &F_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double delta_t, double w, double sigma_x, double sigma_y, double sigma_r, double sigma_theta)
{
	double aux_data1[4 * 4] =
	{
		1.0,			sin(w*delta_t)/w, 		0.0, 			-(1.0-cos(w*delta_t))/w,
		0.0, 			cos(w*delta_t),	 		0.0, 			-sin(w*delta_t),
		0.0, 			(1.0-cos(w*delta_t))/w,	1.0,			sin(w*delta_t)/w,
		0.0, 			sin(w*delta_t), 		0.0, 			cos(w*delta_t),
	};
	Matrix aux_matrix1(4, 4, aux_data1);
	F_k_1 = aux_matrix1;  // eq. 2.17, pg 11

	double aux_data22[2 * 2] =
	{
		sigma_x*sigma_x, 	0.0,
		0.0, 				sigma_y*sigma_y,
	};
	Matrix aux_matrix22(2, 2, aux_data22);
	double aux_data23[4 * 2] =
	{
		(delta_t*delta_t)/2.0,	0.0,
		delta_t, 				0.0,
		0.0, 					(delta_t*delta_t)/2.0,
		0.0,					delta_t,
	};
	Matrix aux_matrix23(4, 2, aux_data23);
	Q_k_1 = aux_matrix23 * aux_matrix22 * ~aux_matrix23;

	double aux_data3[2 * 4] =
	{
		1.0,			0.0, 			0.0, 			0.0,
		0.0, 			0.0,	 		1.0, 			0.0,
	};
	Matrix aux_matrix3(2, 4, aux_data3);
	H_k = aux_matrix3;  // eq. 2.38, pg 15

	double aux_data4[2 * 2] =
	{
		sigma_r,		0.0,
		0.0, 			sigma_theta,
	};
	Matrix aux_matrix4(2, 2, aux_data4);
	R_p_k = aux_matrix4;  // eq. 2.34, pg 15
}


void
constant_angular_velocity_system_setup(double &x, double &y, double &phi, double &v, double &w, Matrix &x_k_k, Matrix &P_k_k,
		Matrix &F_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double &delta_t, double &sigma_x, double &sigma_y, double &sigma_r, double &sigma_theta)
{
	x = -170.0;
	y = -150.0;
	phi = carmen_degrees_to_radians(45.0);
	v = 5.0;
	w = -0.01;

	delta_t = 1.0;
	sigma_x = 2.0;
	sigma_y = 2.0;
	sigma_r = 2.0;
	sigma_theta = carmen_degrees_to_radians(1.0);

// Initial state
	double X[4 * 1] =
	{
		x - 5.0,		// x + adicao de erro inicial
		v * cos(phi),	// x' (v_x)
		y + 10.0,		// y + adicao de erro inicial
		v * sin(phi)	// y' (v_y)
	};
	Matrix aux_matrix1(4, 1, X);
	x_k_k = aux_matrix1; // eq. 3.27, pg 24


	double P[4 * 4] =
	{
		10.0 * 10.0,	0.0, 			0.0, 			0.0,
		0.0, 			0.2 * 0.2, 		0.0, 			0.0,
		0.0, 			0.0, 			10.0 * 10.0,	0.0,
		0.0, 			0.0, 			0.0, 			0.2 * 0.2
	};
	Matrix aux_matrix2(4, 4, P);
	P_k_k = aux_matrix2;  // eq. 3.33, pg 25

	// Constant Velocity System setup
	set_constant_angular_velocity_system_matrixes(F_k_1, Q_k_1, H_k, R_p_k, delta_t, w, sigma_x, sigma_y, sigma_r, sigma_theta);
}


void
set_constant_velocity_system_matrixes(Matrix &F_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double delta_t, double sigma_x, double sigma_y, double sigma_r, double sigma_theta)
{
	double aux_data1[4 * 4] =
	{
		1.0,			delta_t, 		0.0, 			0.0,
		0.0, 			1.0,	 		0.0, 			0.0,
		0.0, 			0.0, 			1.0,			delta_t,
		0.0, 			0.0, 			0.0, 			1.0,
	};
	Matrix aux_matrix1(4, 4, aux_data1);
	F_k_1 = aux_matrix1;  // eq. 2.17, pg 11

	double aux_data22[2 * 2] =
	{
		sigma_x*sigma_x, 	0.0,
		0.0, 				sigma_y*sigma_y,
	};
	Matrix aux_matrix22(2, 2, aux_data22);
	double aux_data23[4 * 2] =
	{
		(delta_t*delta_t)/2.0,	0.0,
		delta_t, 				0.0,
		0.0, 					(delta_t*delta_t)/2.0,
		0.0,					delta_t,
	};
	Matrix aux_matrix23(4, 2, aux_data23);
	Q_k_1 = aux_matrix23 * aux_matrix22 * ~aux_matrix23;

	double aux_data3[2 * 4] =
	{
		1.0,			0.0, 			0.0, 			0.0,
		0.0, 			0.0,	 		1.0, 			0.0,
	};
	Matrix aux_matrix3(2, 4, aux_data3);
	H_k = aux_matrix3;  // eq. 2.38, pg 15

	double aux_data4[2 * 2] =
	{
		sigma_r,		0.0,
		0.0, 			sigma_theta,
	};
	Matrix aux_matrix4(2, 2, aux_data4);
	R_p_k = aux_matrix4;  // eq. 2.34, pg 15
}


void
constant_velocity_system_setup(double &x, double &y, double &phi, double &v, Matrix &x_k_k, Matrix &P_k_k,
		Matrix &F_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double &delta_t, double &sigma_x, double &sigma_y, double &sigma_r, double &sigma_theta)
{
	x = -170.0;
	y = -150.0;
	phi = carmen_degrees_to_radians(45.0);
	v = 5.0;

	delta_t = 1.0;
	sigma_x = 0.2;
	sigma_y = 0.2;
	sigma_r = 2.0;
	sigma_theta = carmen_degrees_to_radians(1.0);

// Initial state
	double X[4 * 1] =
	{
		x - 5.0,		// x + adicao de erro inicial
		v * cos(phi),	// x' (v_x)
		y + 10.0,		// y + adicao de erro inicial
		v * sin(phi)	// y' (v_y)
	};
	Matrix aux_matrix1(4, 1, X);
	x_k_k = aux_matrix1; // eq. 3.27, pg 24


	double P[4 * 4] =
	{
		10.0 * 10.0,	0.0, 			0.0, 			0.0,
		0.0, 			0.2 * 0.2, 		0.0, 			0.0,
		0.0, 			0.0, 			10.0 * 10.0,	0.0,
		0.0, 			0.0, 			0.0, 			0.2 * 0.2
	};
	Matrix aux_matrix2(4, 4, P);
	P_k_k = aux_matrix2;  // eq. 3.33, pg 25

	// Constant Velocity System setup
	set_constant_velocity_system_matrixes(F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_x, sigma_y, sigma_r, sigma_theta);
}


void
true_position_observation(Matrix &z_k, Matrix &R_k, Matrix R_p_k, double x, double y, double sigma_r, double sigma_theta)
{
	double v_p_k_[2 * 1] =
	{
		carmen_gaussian_random(0.0, sigma_r / 2),
		carmen_gaussian_random(0.0, sigma_theta / 2),
	};
	Matrix v_p_k(2, 1, v_p_k_);  // eqs. 2.33, 2.34, pg 14

	double r = sqrt(x * x + y * y) + v_p_k.val[0][0]; 	// perfect observation + noise
	double theta = atan2(y, x) +  + v_p_k.val[1][0]; 	// perfect observation + noise

	double _z_k[2 * 1] =
	{
		r * cos(theta),
		r * sin(theta),
	};
	Matrix z_k_(2, 1, _z_k);	// eq. 2.37, 2.38, pg 15
	z_k = z_k_;

	double j_z_p_k[2 * 2] =
	{
		cos(theta), -r * sin(theta),
		sin(theta), r * cos(theta),
	};
	Matrix J_z_p_k(2, 2, j_z_p_k);  // eq. 2.40, pg 15

	R_k = J_z_p_k * R_p_k * ~J_z_p_k;		// eq. 2.42, pg 15
}


void
kalman_filter(Matrix &x_k_k, Matrix &P_k_k, Matrix z_k,
		Matrix F_k_1, Matrix Q_k_1, Matrix H_k, Matrix R_k)
{	// table 3.1 pg 23
	Matrix x_k_1_k_1 = x_k_k;
	Matrix P_k_1_k_1 = P_k_k;

	// Prediction
	Matrix x_k_k_1 = F_k_1 * x_k_1_k_1; // Traget tracking nao tem entrada de controle
	Matrix P_k_k_1 = F_k_1 * P_k_1_k_1 * ~F_k_1 + Q_k_1;

	// Correction
	Matrix S_k = H_k * P_k_k_1 * ~H_k + R_k;
	Matrix K_k = P_k_k_1 * ~H_k * Matrix::inv(S_k);
	x_k_k = x_k_k_1 + K_k * (z_k - H_k * x_k_k_1); // pequena mudancca aqui para evitar a necessidade da variavel ~zk
//	P_k_k = P_k_k_1 - K_k * S_k * ~K_k;
	P_k_k = (Matrix::eye(x_k_k.m) - K_k * H_k) * P_k_k_1;
//	Matrix aux = Matrix::eye(x_k_k.m) - K_k * H_k;
//	P_k_k = aux * P_k_k_1 * ~aux + K_k * R_k * ~K_k;
}


int
main()
{
	double angle, major_axis, minor_axis;
	Matrix x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k;

//	double x, y, phi, v, delta_t, sigma_x, sigma_y, sigma_r, sigma_theta;
//	constant_velocity_system_setup(x, y, phi, v, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_x, sigma_y, sigma_r, sigma_theta);
//	compute_error_ellipse(angle, major_axis, minor_axis,
//			P_k_k.val[0][0], P_k_k.val[0][2], P_k_k.val[2][2], 2.4477);
//	printf("%lf %lf %lf %lf %lf %lf %lf\n", x, y, x_k_k.val[0][0], x_k_k.val[2][0], major_axis, minor_axis, angle * 180.0 / M_PI);

	double x, y, phi, v, w, delta_t, sigma_x, sigma_y, sigma_r, sigma_theta;
	constant_angular_velocity_system_setup(x, y, phi, v, w, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_x, sigma_y, sigma_r, sigma_theta);
	compute_error_ellipse(angle, major_axis, minor_axis,
			P_k_k.val[0][0], P_k_k.val[0][2], P_k_k.val[2][2], 2.4477);
	printf("%lf %lf %lf %lf %lf %lf %lf\n", x, y, x_k_k.val[0][0], x_k_k.val[2][0], major_axis, minor_axis, angle * 180.0 / M_PI);

	//	double x, y, phi, v, delta_t, sigma_vx, sigma_vy, sigma_r, sigma_theta;
//	constant_acceleration_system_setup(x, y, phi, v, x_k_k, P_k_k, F_k_1, Q_k_1, H_k, R_p_k, delta_t, sigma_vx, sigma_vy, sigma_r, sigma_theta);
//	compute_error_ellipse(angle, major_axis, minor_axis,
//			P_k_k.val[0][0], P_k_k.val[0][3], P_k_k.val[3][3], 2.4477);
//	printf("%lf %lf %lf %lf %lf %lf %lf\n", x, y, x_k_k.val[0][0], x_k_k.val[3][0], major_axis, minor_axis, angle * 180.0 / M_PI);

	Matrix z_k, R_k;
	double t = 0.0;
	do
	{
		true_position_observation(z_k, R_k, R_p_k, x, y, sigma_r, sigma_theta);

		kalman_filter(x_k_k, P_k_k, z_k, F_k_1, Q_k_1, H_k, R_k);

		compute_error_ellipse(angle, major_axis, minor_axis,
				P_k_k.val[0][0], P_k_k.val[0][2], P_k_k.val[2][2], 2.4477);
		printf("%lf %lf %lf %lf %lf %lf %lf\n", x, y, x_k_k.val[0][0], x_k_k.val[2][0], major_axis, minor_axis, angle * 180.0 / M_PI);

//		compute_error_ellipse(angle, major_axis, minor_axis,
//				P_k_k.val[0][0], P_k_k.val[0][3], P_k_k.val[3][3], 2.4477);
//		printf("%lf %lf %lf %lf %lf %lf %lf\n", x, y, x_k_k.val[0][0], x_k_k.val[3][0], major_axis, minor_axis, angle * 180.0 / M_PI);

		x = x + v * cos(phi) * delta_t;	// true position update
		y = y + v * sin(phi) * delta_t;	// true position update

		t += delta_t;
		phi += 0.02;
	} while (t < 100.0);
}
