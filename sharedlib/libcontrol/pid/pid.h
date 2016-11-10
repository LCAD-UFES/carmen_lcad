#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif


double carmen_libpid_steering_PID_controler(double atan_desired_curvature, double atan_current_curvature, double delta_t);


void carmen_libpid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command,
											double desired_velocity, double current_velocity, double delta_t);


void carmen_libpid_read_PID_parameters(int argc, char *argv[]);


void
pid_plot_curvature(double current_phi, double desired_phi, double steering_effort, double v);


void
printa_test();


#ifdef __cplusplus
}
#endif

#endif // PID_H
