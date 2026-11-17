#include <carmen/carmen.h>
#include <carmen/fused_odometry_messages.h>


/*
 * Define as mensagens de fused_odometry neste processo.
 *
 * Chamada de dentro do subscribe e do publish, e nao so' de quem publica. O motivo:
 * IPC_subscribe exige que a mensagem ja' esteja DEFINIDA. Se o assinante sobe antes do
 * publicador -- que e' o caso quando o localize_ackerman sobe em ~2 s e o publicador da
 * pose vem de um roslaunch que leva dezenas de segundos -- a assinatura falha, e falha
 * EM SILENCIO: o handler simplesmente nunca e' chamado, sem erro, sem aviso, para sempre.
 * O sintoma e' o modulo rodando normalmente e nao publicando nada que dependa da pose.
 *
 * Definir dos dois lados quebra essa dependencia de ordem de subida: quem chegar primeiro
 * define, o segundo encontra a definicao pronta. Idempotente por causa do 'first'.
 */
void
carmen_fused_odometry_define_messages(void)
{
	static int first = 1;
	IPC_RETURN_TYPE err;

	if (!first)
		return;

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_FUSED_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, IPC_VARIABLE_LENGTH, CARMEN_FUSED_ODOMETRY_PARTICLE_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);

	first = 0;
}

void
carmen_fused_odometry_subscribe_fused_odometry_message(	carmen_fused_odometry_message *fused_odometry_message,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how)
{
  carmen_fused_odometry_define_messages();

  carmen_subscribe_message(CARMEN_FUSED_ODOMETRY_NAME, 
                           CARMEN_FUSED_ODOMETRY_FMT,
                           fused_odometry_message, sizeof(carmen_fused_odometry_message), 
			   handler, subscribe_how);
}

void
carmen_fused_odometry_unsubscribe_fused_odometry_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_FUSED_ODOMETRY_NAME, handler);
}


void
carmen_fused_odometry_subscribe_fused_odometry_particle_message(carmen_fused_odometry_particle_message *fused_odometry_particle_message,
									carmen_handler_t handler,
									carmen_subscribe_t subscribe_how)
{
	carmen_fused_odometry_define_messages();

	carmen_subscribe_message(	CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, 
                           		CARMEN_FUSED_ODOMETRY_PARTICLE_FMT,
                          		fused_odometry_particle_message, sizeof(carmen_fused_odometry_particle_message), 
					handler, subscribe_how);
}

void
carmen_fused_odometry_unsubscribe_fused_odometry_particle_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, handler);
}

void
carmen_fused_odometry_publish_message(carmen_fused_odometry_message *message)
{
	IPC_RETURN_TYPE err;

	carmen_fused_odometry_define_messages();

	err = IPC_publishData(CARMEN_FUSED_ODOMETRY_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FUSED_ODOMETRY_NAME);
}

void
carmen_fused_odometry_publish_particles(carmen_fused_odometry_particle_message *message)
{
	IPC_RETURN_TYPE err;

	carmen_fused_odometry_define_messages();

	err = IPC_publishData(CARMEN_FUSED_ODOMETRY_PARTICLE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_FUSED_ODOMETRY_PARTICLE_NAME);
}

