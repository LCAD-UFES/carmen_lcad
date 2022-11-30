#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

double
carmen_libpid_steering_PID_controler_publish_data(steering_pid_data_message * msg, double atan_desired_curvature, double atan_current_curvature, double plan_size,
		int manual_override, double kp, double kd, double ki);
		
double
carmen_libpid_steering_PID_controler(double atan_desired_curvature, double atan_current_curvature, double plan_size, int tune_pid_mode,
		int manual_override);


double
carmen_libpid_steering_PID_controler_FUZZY_publish_data(steering_pid_data_message *msg, double atan_desired_curvature, double atan_current_curvature, double delta_t_old __attribute__ ((unused)),
		int manual_override, double v, double steer_kp, double steer_kd, double steer_ki);

double carmen_libpid_steering_PID_controler_FUZZY(double atan_desired_curvature, double atan_current_curvature,
		double delta_t_old __attribute__ ((unused)), int manual_override, double v);


void carmen_libpid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command,
		double desired_velocity, double current_velocity, double delta_t, int manual_override, double gear_ratio);

void
carmen_libpid_velocity_PID_controler_publish_data(velocity_pid_data_message *msg, double *throttle_command, double *brakes_command, int *gear_command,
										double desired_velocity, double current_velocity, double delta_t_old __attribute__ ((unused)),
										int manual_override, double kp, double kd, double ki);

void carmen_libpid_read_PID_parameters(int argc, char *argv[]);


#ifdef __cplusplus
}
#endif

#endif // PID_H
