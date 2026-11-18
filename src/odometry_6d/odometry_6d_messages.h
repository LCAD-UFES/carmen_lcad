/*
 * odometry_6d -- odometria do robo em 6 graus de liberdade, so' em termos de VELOCIDADE.
 *
 * Diferente da odometria "de pose" (x, y, theta integrados por quem publica), esta mensagem
 * carrega o estado de movimento instantaneo no referencial do CORPO do robo:
 *   - velocidade linear      (vx, vy, vz)                 m/s
 *   - velocidade angular     (wx, wy, wz)                 rad/s   (wz = taxa de guinada)
 *   - aceleracao angular     (alpha_x, alpha_y, alpha_z)  rad/s^2
 * Quem consome integra no proprio relogio (ver README.md). Nao ha' pose: a deriva do
 * referencial de odometria de quem publica nunca chega ao consumidor.
 */
#ifndef CARMEN_ODOMETRY_6D_MESSAGES_H
#define CARMEN_ODOMETRY_6D_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	double vx, vy, vz;                  // velocidade linear no corpo (m/s)
	double wx, wy, wz;                  // velocidade angular no corpo (rad/s)
	double alpha_x, alpha_y, alpha_z;   // aceleracao angular no corpo (rad/s^2)
	double timestamp;                   // carmen_get_time() da amostra
	char *host;
} carmen_odometry_6d_message;

#define CARMEN_ODOMETRY_6D_NAME "carmen_odometry_6d"
#define CARMEN_ODOMETRY_6D_FMT  "{double,double,double,double,double,double,double,double,double,double,string}"

#ifdef __cplusplus
}
#endif

#endif
