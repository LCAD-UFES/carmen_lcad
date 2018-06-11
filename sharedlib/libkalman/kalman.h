
//#define EKF_SENSING

#define NUM_MODELS 3

using namespace std;


void
compute_error_ellipse(double &angle, double &major_axis, double &minor_axis,
		double x_var, double xy_cov, double y_var, double k);

void
set_CT_system_matrixes(Matrix &F_k_1, Matrix &fx_k_1, Matrix &Q_k_1,
		double T, double w, double vx, double vy, double sigma_w, double sigma_vct);

void
CT_system_setup(double x, double y, double yaw, double v, double w, Matrix &x_k_k, Matrix &P_k_k,
		Matrix &F_k_1, Matrix &fx_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double T, double sigma_w, double sigma_vct, double sigma_r, double sigma_theta);

void
set_CA_system_matrixes(Matrix &F_k_1, Matrix &Q_k_1, double T, double sigma_vca);

void
CA_system_setup(double x, double y, double yaw, double v, Matrix &x_k_k, Matrix &P_k_k,
		Matrix &F_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double T, double sigma_vca, double sigma_r, double sigma_theta);

void
set_CV_system_matrixes(Matrix &F_k_1, Matrix &Q_k_1, double T, double sigma_s);

void
CV_system_setup(double x, double y, double yaw, double v, Matrix &x_k_k, Matrix &P_k_k,
		Matrix &F_k_1, Matrix &Q_k_1, Matrix &H_k, Matrix &R_p_k,
		double T, double sigma_s, double sigma_r, double sigma_theta);

void
set_R_p_k_matriz(Matrix &R_p_k, double sigma_r, double sigma_theta);

void
position_observation(Matrix &z_k, Matrix &R_k, Matrix R_p_k, double radius, double theta, double sigma_r, double sigma_theta);

void
kalman_filter(Matrix &x_k_k, Matrix &P_k_k, Matrix &delta_zk, Matrix &S_k,
		Matrix z_k, Matrix F_k_1, Matrix Q_k_1, Matrix H_k, Matrix R_k);

void
extended_kalman_filter(Matrix &x_k_k, Matrix &P_k_k, Matrix &delta_zk, Matrix &S_k,
		Matrix z_k, Matrix F_k_1, Matrix fx_k_1, Matrix Q_k_1, Matrix H_k, Matrix R_k);

vector<Matrix>
extend_vector_dimensions(vector<Matrix> x);

vector<Matrix>
extend_matrix_dimensions(vector<Matrix> x, double max_a, double max_w);

void
mode_estimate_and_covariance_combination(Matrix &x_k_k, Matrix &P_k_k, vector<Matrix> x_k_k_1, vector<Matrix> P_k_k_1, double u_k[NUM_MODELS]);

void
imm_filter(Matrix &x_k_k, Matrix &P_k_k, vector<Matrix> &x_k_1_k_1, vector<Matrix> &P_k_1_k_1,
		Matrix z_k, Matrix R_k,
		vector<Matrix> F_k_1, vector<Matrix> Q_k_1, vector<Matrix> H_k,
		double T, double sigma_w, double sigma_vct, double max_a, double max_w,
		double p[NUM_MODELS][NUM_MODELS], double u_k[NUM_MODELS]);
