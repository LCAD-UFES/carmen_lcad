#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>

int
main(void)
{
	int i;
	double xi, yi;
	double x[5] = {0.0, 0.5, 25.0, 69.0, 70.0};
	double y[5] = {0.0, 0.0,  3.0,  6.0,  6.0};

	printf("#m=0,S=4\n");

	for (i = 0; i < 5; i++)
		printf("%g %g\n", x[i], y[i]);

	printf("#m=1,S=0\n");

	///////////////////////////////////////////////////////
	// spline 1
	gsl_interp_accel *acc_original = gsl_interp_accel_alloc();
	gsl_spline *spline_original = gsl_spline_alloc(gsl_interp_cspline, 5);

	gsl_spline_init(spline_original, x, y, 5);

	for (xi = x[0]; xi < x[4]; xi += 0.01)
	{
		yi = gsl_spline_eval(spline_original, xi, acc_original);
		printf("%g %g\n", xi, yi);
	}

	///////////////////////////////////////////////////////
	// spline 2
	x[0] = 20.0;
	x[1] = 20.5; // 0.5m ahead
	x[2] = x[0] + 25.0;
	x[3] = x[0] + 69.0;
	x[4] = x[0] + 70.0;

	y[0] = gsl_spline_eval(spline_original, x[0], acc_original);
	y[1] = gsl_spline_eval(spline_original, x[1], acc_original);
	y[2] = (gsl_spline_eval(spline_original, x[2], acc_original) - 0.0) / 2.0;
	y[3] = 0.0;
	y[4] = 0.0;

	printf("#m=0,S=2\n");

	for (i = 0; i < 5; i++)
		printf("%g %g\n", x[i], y[i]);

	printf("#m=2,S=0\n");

	gsl_interp_accel *acc2 = gsl_interp_accel_alloc();
	gsl_spline *spline2 = gsl_spline_alloc(gsl_interp_cspline, 5);
	gsl_spline_init(spline2, x, y, 5);

	for (xi = x[0]; xi < x[4]; xi += 0.01)
	{
		yi = gsl_spline_eval(spline2, xi, acc2);
		printf("%g %g\n", xi, yi);
	}

	///////////////////////////////////////////////////////
	// spline 3
	x[0] = 30.0;
	x[1] = 30.5; // 0.5m ahead
	x[2] = x[0] + (70.0 - x[0]) / 2.0;
	x[3] = 69.0;
	x[4] = 70.0;

	y[0] = gsl_spline_eval(spline_original, x[0], acc_original);
	y[1] = gsl_spline_eval(spline_original, x[1], acc_original);
	y[2] = gsl_spline_eval(spline_original, x[2], acc_original);
	y[3] = 6.0;
	y[4] = 6.0;

	printf("#m=0,S=3\n");

	for (i = 0; i < 5; i++)
		printf("%g %g\n", x[i], y[i]);

	printf("#m=3,S=0\n");

	gsl_interp_accel *acc3 = gsl_interp_accel_alloc();
	gsl_spline *spline3 = gsl_spline_alloc(gsl_interp_cspline, 5);
	gsl_spline_init(spline3, x, y, 5);

	for (xi = x[0]; xi < x[4]; xi += 0.01)
	{
		yi = gsl_spline_eval(spline3, xi, acc3);
		printf("%g %g\n", xi, yi);
	}

	gsl_spline_free(spline_original);
	gsl_interp_accel_free(acc_original);

	gsl_spline_free(spline2);
	gsl_interp_accel_free(acc2);

	gsl_spline_free(spline3);
	gsl_interp_accel_free(acc3);

	return 0;
}
