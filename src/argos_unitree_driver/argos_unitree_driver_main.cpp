
/*
 * argos_unitree_driver -- ponte entre o ARGOS (Unitree Go2, DDS) e o CAN virtual
 * que o ojArgos/ford_escape_hybrid usam para falar com o carmen.
 *
 * Este modulo NAO publica IPC. Ele e' so' a ponta de baixo da cadeia; quem publica
 * carmen_robot_ackerman_odometry para o carmen e' o ford_escape_hybrid
 * (vehicle_driver_new.c:452), a partir do JAUS que o ojArgos monta.
 *
 * Cadeia completa (a mesma do IARA/XGV, com o Go2 no lugar do carro):
 *
 *   obstacle_avoider --carmen_base_ackerman_motion_command--> ford_escape_hybrid
 *        (roda o PID de velocidade e de direcao)
 *              --JAUS SetWrenchEffort--> ojArgos --CAN 0x100--> ESTE MODULO
 *                                                                    |
 *                                                       SportClient::Move()
 *                                                                    v
 *                                                                  Go2
 *                                                                    |
 *                                  DDS rt/sportmodestate <-----------+
 *                                                                    |
 *   ford_escape_hybrid <--JAUS-- ojArgos <--CAN 0x425 + 0x80-- ESTE MODULO
 *              |
 *   carmen_robot_ackerman_odometry --> base_ackerman --> localize_ackerman --> ...
 *
 * POR QUE PASSAR PELO CAN E NAO FALAR IPC DIRETO
 * ----------------------------------------------
 * A versao anterior deste modulo publicava carmen_robot_ackerman_odometry por conta
 * propria e comandava o robo direto do motion_command, pulando ojArgos e
 * ford_escape_hybrid. Funcionava, mas jogava fora o PID do ford_escape_hybrid: o
 * comando do planejador ia cru para o robo, sem malha fechada contra a velocidade
 * medida. Passar pelo CAN devolve o controle para onde ele sempre esteve no carmen.
 *
 * FORMATO DOS FRAMES
 * ------------------
 *   0x425  (driver -> ojArgos)  float32 LE   velocidade v, m/s, COM SINAL
 *   0x80   (driver -> ojArgos)  float32 LE   phi, rad, COM SINAL
 *   0x100  (ojArgos -> driver)  int16 LE x2  [0..1] = velocidade, [2..3] = direcao
 *
 * Os dois primeiros sao float por decisao nossa -- ver o comentario em
 * ojArgos/src/main.c:update_car_speed(). O formato original do XGV le' os bytes como
 * inteiro SEM SINAL, entao nao representa re' nem esterco para a esquerda; num carro
 * isso passa (a re' vem da marcha), no Go2 nao.
 *
 * O 0x100 e' gerado por ojArgos/src/pd.c:send_efforts(), que ja' vinha pronto do
 * ojWheeltec:
 *     int_velocity = (throttle_effort / 100) * 5000 * 2.3   -> effort% = raw / 115
 *     int_phi      = (steering_effort / 100) * 2000         -> effort% = raw / 20
 *
 * SOBRE O can_to_wirelesscontroller.cpp DO I2CA
 * ---------------------------------------------
 * O exemplo de referencia (I2CA/argos/src/argos_drivers_cpp) faz
 *
 *     phi_ += steering_effort;                       // a cada frame CAN
 *     ang_vel = tan(phi_) * v_ / L_;
 *
 * que e' um integrador SEM SATURACAO. A 60 Hz de CAN com esterco de fundo de escala,
 * phi_ passa de pi em menos de um segundo -- e tan() tem periodo pi, entao o sinal do
 * comando inverte sozinho e volta a crescer. E' exatamente o defeito que o proprio
 * tutorial do I2CA descreve no cabecalho ("devido a overflows o robo apenas anda para
 * tras e apenas gira no sentido horario"). Aqui NAO integramos: o esforco de direcao
 * vira phi por mapeamento direto e saturado (ver can_effort_to_command()).
 *
 * Uso:
 *   ./argos_unitree_driver -network_interface enp130s0
 *   ./argos_unitree_driver -network_interface enp130s0 -can_interface vcan0
 *   ./argos_unitree_driver -network_interface enp130s0 -dry_run on   # so' le odometria
 */

#include <atomic>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <signal.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <carmen/carmen.h>
#include <carmen/ford_escape_hybrid_interface.h>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

// Topico DDS do estado do modo esporte do Go2 (unitree_sdk2, example/go2).
// Existe tambem rt/lf/sportmodestate, que traz os MESMOS campos a 20 Hz em vez de
// 500 Hz (medido). Ficamos com o de alta taxa: filtramos aqui embaixo e nao ha' motivo
// para abrir mao da resolucao.
#define TOPIC_SPORT_MODE_STATE "rt/sportmodestate"

// mode() do SportModeState_ quando o robo esta em pe e aceita Move().
// 1 = idle/balance stand, 3 = locomotion.
#define GO2_MODE_BALANCE_STAND 1
#define GO2_MODE_LOCOMOTION    3

// Ids dos frames -- tem que casar com ojArgos/src/main.c e ojArgos/src/pd.c.
#define CAN_ID_VELOCITY 0x425
#define CAN_ID_STEERING 0x80
#define CAN_ID_COMMAND  0x100

#define EXEC_COOLDOWN_LOCAL(execute, cooldown)                       \
{                                                                    \
	static double ec_last = 0.0;                                     \
	if ((carmen_get_time() - ec_last) > (cooldown))                  \
	{                                                                \
		execute;                                                     \
		fflush(stdout);                                              \
		fflush(stderr);                                              \
		ec_last = carmen_get_time();                                 \
	}                                                                \
}

// Escala do send_efforts() do ojArgos (CAN_CONVERSION_CONSTANT = 256 em
// ojArgos/src/pd.c). O effort vem em [-100, 100], entao o raw vem em [-25600, 25600]
// e effort = raw / 256.
//
// E' a MESMA escala do driver do I2CA (~/I2CA/argos/src/argos_drivers_cpp/src/
// can_to_wirelesscontroller.cpp:99), que faz raw / (256.0 * 100.0) para ir direto a'
// fracao [-1, 1]; aqui a divisao por 100 fica em can_effort_to_command(). A conta e' a
// mesma, so' muda onde ela e' feita.
//
// >>> ESTE VALOR E' CONTRATO COM O ojArgos. Mudar aqui sem mudar o pd.c (ou vice-versa)
// >>> faz o driver interpretar todo comando com a escala errada, em silencio.
#define CAN_VELOCITY_EFFORT_SCALE 256.0
#define CAN_STEERING_EFFORT_SCALE 256.0


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Parametros                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


// Vindos do param_daemon (secao robot, ja' existentes no carmen-argos.ini).
static double g_distance_between_axles = 0.35;
static double g_max_v = 0.4;
static double g_max_v_reverse = -0.4;
static double g_max_phi = 0.8;

// Vindos da linha de comando (nao precisam existir no .ini).
static char  *g_network_interface = NULL;
static char  *g_can_interface = NULL;
static double g_command_rate = 50.0;
static double g_odometry_rate = 60.0;		// 60 Hz e' a taxa que o ojArgos espera no 0x425
static double g_command_timeout = 0.3;
static double g_state_timeout = 0.3;
static double g_v_min_for_phi = 0.05;
static double g_v_deadband = 0.02;
// Zona morta do COMANDO -- ver o comentario em robot_command_loop(). Abaixo destes
// valores o robo e' parado de verdade (StopMove) em vez de receber Move(0, 0, 0).
static double g_v_command_deadband = 0.02;			// m/s
static double g_vyaw_command_deadband = 0.05;		// rad/s
static double g_v_filter_gain = 0.3;
static double g_phi_filter_gain = 0.5;
static double g_max_vyaw = 1.5;
static int    g_auto_stand = 1;
static int    g_dry_run = 0;
static int    g_verbose = 0;

// Esterco em rad/s com 100% de esforco. Este e' o GANHO da malha de esterco, entao o default
// sai do Kp do proprio PID do ford_escape_hybrid, e nao de um numero solto:
//
//   phi_ponto = -(Kp * e / 100) * phi_rate  =>  constante de tempo T = 100 / (Kp * phi_rate)
//
// Com o robot_PID_steering_kp = 12,0 do carmen-argos.ini e T = 0,4 s da' ~21 rad/s. Com o
// valor antigo (1,6 rad/s) T dava 5 SEGUNDOS -- o robo terminava a manobra antes de esterçar.
// Se ele oscilar na curva, baixe; se demorar a entrar, suba.
#define PHI_LOOP_TIME_CONSTANT 0.4		// s

static double g_phi_rate = 0.0;			// 0 = derivado do Kp em read_parameters
static double g_steering_kp = 12.0;		// robot_PID_steering_kp, so' para derivar o default
static double g_steering_angle = 0.0;	// integral do esforco de esterco, em rad

// Postura do robo no fim da missao. O task_manager nao tem comando de "deitar" -- o
// vocabulario dele e' de caminhao -- mas tem `set parking brake on|off` e
// `set engine on|off`, que viram um carmen_ford_escape_engine_and_parking_brake com dois
// inteiros. E' o gancho: a missao pede o freio de mao, e aqui isso vira postura do Go2.
enum posture_t { POSTURE_NONE = 0, POSTURE_STAND, POSTURE_SIT, POSTURE_STAND_DOWN, POSTURE_DAMP };

static int    g_posture = 1;					// -posture off desliga tudo isto
static char  *g_park_posture_name = NULL;		// -park_posture       (freio de mao ligado)
static char  *g_engine_off_posture_name = NULL;	// -engine_off_posture (motor desligado)
static posture_t g_park_posture = POSTURE_STAND_DOWN;
static posture_t g_engine_off_posture = POSTURE_DAMP;

// O handler do IPC so' registra o pedido: as chamadas do SportClient sao RPC com timeout e
// nao podem rodar dentro do laco do IPC, pelo mesmo motivo das outras threads deste modulo.
static std::atomic<int>  g_posture_request(POSTURE_NONE);
static std::atomic<bool> g_posture_held(false);   // true = robo pousado, ignora motion command


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Estado compartilhado entre as threads                                                        //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Quatro threads, cada uma so' toca o que e' dela:
//
//   1. thread do DDS (criada pelo SDK)  -- recebe SportModeState do robo
//   2. thread de odometria (aqui)       -- filtra e escreve 0x425 / 0x80 no CAN
//   3. thread de leitura do CAN (aqui)  -- bloqueia em read() do 0x100
//   4. thread de controle (aqui)        -- envia Move()/StopMove() pro robo
//
// A thread 4 e' separada da 3 de proposito: SportClient::Move() e' um RPC com timeout
// sobre DDS, e chamar isso de dentro do laco de leitura do CAN faria o socket encher
// e descartar comando toda vez que o robo demorasse a responder.

static std::mutex g_state_mutex;
static double  g_state_v = 0.0;             // velocity[0], m/s, no corpo do robo
static double  g_state_yaw_rate = 0.0;      // yaw_speed, rad/s
static double  g_state_timestamp = 0.0;
static uint8_t g_state_mode = 0;
static bool    g_state_valid = false;

static std::mutex g_command_mutex;
static double g_command_v = 0.0;
static double g_command_phi = 0.0;
static double g_command_timestamp = 0.0;
static bool   g_command_valid = false;

static std::atomic<bool> g_running(true);
static std::atomic<bool> g_robot_engaged(false);   // robo em pe e aceitando Move()

static int g_can_socket = -1;

// Precisa ser alcancavel pelo shutdown_module(): o Go2 MANTEM a ultima velocidade ate'
// receber outra ordem, entao um driver que morre sem mandar StopMove() deixa o robo
// andando sozinho.
static unitree::robot::go2::SportClient *g_sport_client = NULL;


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// CAN                                                                                          //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static int
open_can_socket(const char *interface_name)
{
	struct ifreq ifr;
	struct sockaddr_can addr;
	int can_socket;

	can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (can_socket < 0)
	{
		fprintf(stderr, "argos_unitree_driver: nao consegui abrir o socket CAN: %s\n", strerror(errno));
		return (-1);
	}

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
	if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0)
	{
		fprintf(stderr, "argos_unitree_driver: interface CAN '%s' nao existe (%s).\n"
				"  Crie a vcan com:\n"
				"    sudo modprobe vcan\n"
				"    sudo ip link add dev %s type vcan\n"
				"    sudo ip link set up %s\n",
				interface_name, strerror(errno), interface_name, interface_name);
		close(can_socket);
		return (-1);
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(can_socket, (struct sockaddr *) &addr, sizeof(addr)) < 0)
	{
		fprintf(stderr, "argos_unitree_driver: nao consegui associar o socket a '%s': %s\n",
				interface_name, strerror(errno));
		close(can_socket);
		return (-1);
	}

	return (can_socket);
}


static void
send_can_float(int can_id, float value)
{
	struct can_frame frame;

	if (g_can_socket < 0)
		return;

	memset(&frame, 0, sizeof(frame));
	frame.can_id = can_id;
	frame.can_dlc = 4;
	memcpy(frame.data, &value, sizeof(float));

	if (write(g_can_socket, &frame, sizeof(frame)) != (ssize_t) sizeof(frame))
		fprintf(stderr, "argos_unitree_driver: falha ao enviar o frame 0x%X.\n", can_id);
}


static posture_t
posture_from_name(const char *name, posture_t fallback)
{
	if (name == NULL)
		return (fallback);
	if (strcasecmp(name, "sit") == 0)			return (POSTURE_SIT);
	if (strcasecmp(name, "stand_down") == 0)	return (POSTURE_STAND_DOWN);
	if (strcasecmp(name, "damp") == 0)			return (POSTURE_DAMP);
	if (strcasecmp(name, "stand") == 0)			return (POSTURE_STAND);
	if (strcasecmp(name, "none") == 0)			return (POSTURE_NONE);

	fprintf(stderr, "argos_unitree_driver: postura '%s' desconhecida "
			"(use sit, stand_down, damp, stand ou none).\n", name);
	return (fallback);
}


static const char *
posture_name(posture_t posture)
{
	switch (posture)
	{
		case POSTURE_STAND:      return ("stand");
		case POSTURE_SIT:        return ("sit");
		case POSTURE_STAND_DOWN: return ("stand_down");
		case POSTURE_DAMP:       return ("damp");
		default:                 return ("none");
	}
}


static void
engine_and_parking_brake_handler(carmen_ford_escape_engine_and_parking_brake_message *message)
{
	// Comecam no estado normal de operacao -- motor ligado, freio solto -- e nao num
	// sentinela: com -1 a PRIMEIRA mensagem sempre parecia mudanca de motor, e um
	// `set parking brake on` logo na largada era lido como "motor ligado, fique de pe".
	static int last_engine = 1;
	static int last_parking_brake = 0;

	if (!g_posture)
		return;

	// Motor desligado manda mais do que o freio de mao: se os dois mudarem no mesmo
	// instante, o desligamento vence.
	if (message->engine != last_engine)
	{
		last_engine = message->engine;
		g_posture_request.store(message->engine ? POSTURE_STAND : g_engine_off_posture);
	}
	else if (message->parking_brake != last_parking_brake)
	{
		g_posture_request.store(message->parking_brake ? g_park_posture : POSTURE_STAND);
	}

	last_parking_brake = message->parking_brake;
}


// Converte os esforcos que o ford_escape_hybrid produziu (via ojArgos) no par (v, phi).
//
// VELOCIDADE: mapeamento direto e saturado. 100% = g_max_v. Quem fecha a malha e' o PID
// de velocidade do ford_escape_hybrid, que ja' recebe a velocidade medida pelo 0x425.
//
// ESTERCO: INTEGRADO, com saturacao. O propulsiveRotationalEffortZPercent do XGV nao e'
// uma posicao de volante, e' o esforco no MOTOR do volante -- o angulo e' a integral dele.
// Era esse o defeito: com mapeamento direto, o PID de esterco pedia -4,87% de esforco e o
// robo esterçava 0,047 rad. Como quase nao virava, o 0x80 continuava reportando ~0, o erro
// nao fechava, e o robo andava reto com o integral do PID subindo 0,2% por quadro.
//
// O can_to_wirelesscontroller.cpp do I2CA integra tambem -- e acerta nisso. O que ele erra
// e' integrar SEM saturacao (`phi_ += steering_effort` a cada quadro): phi_ passa de pi em
// menos de um segundo e o tan() inverte o sinal sozinho. Aqui integramos com dt real e
// saturamos em +-g_max_phi.
//
// SINAL: o ford_escape_hybrid gera o desejado como `-atan(curvature)`
// (ford_escape_hybrid.c:258) e o ojArgos converte a realimentacao do 0x80 com a MESMA
// formula (ojArgos/src/main.c:443-444). Nessa convencao esforco NEGATIVO tem que AUMENTAR
// o phi do carmen -- dai o sinal trocado na integracao.
static void
can_effort_to_command(double velocity_effort, double steering_effort, double dt, double *v, double *phi)
{
	if (velocity_effort > 100.0)
		velocity_effort = 100.0;
	else if (velocity_effort < -100.0)
		velocity_effort = -100.0;

	if (steering_effort > 100.0)
		steering_effort = 100.0;
	else if (steering_effort < -100.0)
		steering_effort = -100.0;

	*v = (velocity_effort / 100.0) * ((velocity_effort >= 0.0) ? g_max_v : -g_max_v_reverse);

	g_steering_angle -= (steering_effort / 100.0) * g_phi_rate * dt;

	if (g_steering_angle > g_max_phi)
		g_steering_angle = g_max_phi;
	else if (g_steering_angle < -g_max_phi)
		g_steering_angle = -g_max_phi;

	*phi = g_steering_angle;
}


// Thread 3. Bloqueia em read() -- o socket CAN e' o relogio deste laco.
static void
can_read_loop(void)
{
	struct can_frame frame;

	while (g_running.load())
	{
		ssize_t nbytes = read(g_can_socket, &frame, sizeof(frame));

		if (nbytes < 0)
		{
			if (errno == EINTR)
				continue;
			fprintf(stderr, "argos_unitree_driver: erro lendo o CAN: %s\n", strerror(errno));
			break;
		}

		if (nbytes < (ssize_t) sizeof(frame) || frame.can_id != CAN_ID_COMMAND || frame.can_dlc < 4)
			continue;

		// int16 little endian, como o ojArgos escreve em pd.c:send_efforts().
		int16_t velocity_raw = (int16_t) ((frame.data[1] << 8) | frame.data[0]);
		int16_t steering_raw = (int16_t) ((frame.data[3] << 8) | frame.data[2]);

		double velocity_effort = velocity_raw / CAN_VELOCITY_EFFORT_SCALE;
		double steering_effort = steering_raw / CAN_STEERING_EFFORT_SCALE;

		static double last_can_timestamp = 0.0;
		const double now = carmen_get_time();
		double dt = (last_can_timestamp > 0.0) ? (now - last_can_timestamp) : 0.0;
		last_can_timestamp = now;

		// Uma pausa longa (modulo suspenso, ojArgos reiniciado) nao pode virar um degrau
		// gigante de esterco na primeira amostra depois dela.
		if (dt > 0.1)
			dt = 0.1;

		double v = 0.0, phi = 0.0;
		can_effort_to_command(velocity_effort, steering_effort, dt, &v, &phi);

		static bool had_command_before = false;
		if (!had_command_before && ((fabs(v) > 0.0) || (fabs(phi) > 0.0)))
		{
			printf("argos_unitree_driver: primeiro 0x100 nao-nulo do ojArgos "
					"(effort v %.1f%%, phi %.1f%% -> v %.3f m/s, phi %.3f rad).\n",
					velocity_effort, steering_effort, v, phi);
			fflush(stdout);
			had_command_before = true;
		}

		if (g_verbose)
		{
			static int can_heartbeat = 0;
			if ((can_heartbeat++ % 40) == 0)
			{
				printf("argos_unitree_driver: 0x100 effort v %.1f%% phi %.1f%% -> v %.3f phi %.3f\n",
						velocity_effort, steering_effort, v, phi);
				fflush(stdout);
			}
		}

		{
			std::lock_guard<std::mutex> lock(g_command_mutex);
			g_command_v = v;
			g_command_phi = phi;
			g_command_timestamp = carmen_get_time();
			g_command_valid = true;
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Odometria: Go2 -> CAN                                                                        //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


// Thread 1 (DDS, criada pelo unitree_sdk2).
static void
sport_mode_state_handler(const void *dds_message)
{
	const unitree_go::msg::dds_::SportModeState_ *state =
			(const unitree_go::msg::dds_::SportModeState_ *) dds_message;

	std::lock_guard<std::mutex> lock(g_state_mutex);

	g_state_v = state->velocity()[0];
	g_state_yaw_rate = state->yaw_speed();
	g_state_mode = state->mode();
	g_state_timestamp = carmen_get_time();
	g_state_valid = true;

	g_robot_engaged.store((g_state_mode == GO2_MODE_BALANCE_STAND) ||
			(g_state_mode == GO2_MODE_LOCOMOTION));
}


// Thread 2.
static void
odometry_loop(void)
{
	const useconds_t period_us = (useconds_t) (1000000.0 / g_odometry_rate);
	static double filtered_v = 0.0;
	static double filtered_phi = 0.0;
	bool had_state_before = false;
	int heartbeat = 0;

	while (g_running.load())
	{
		double v = 0.0, yaw_rate = 0.0, state_timestamp = 0.0;
		bool valid = false;

		{
			std::lock_guard<std::mutex> lock(g_state_mutex);
			v = g_state_v;
			yaw_rate = g_state_yaw_rate;
			valid = g_state_valid;
			state_timestamp = g_state_timestamp;
		}

		// Estado velho conta como estado ausente. Rodando pelo cabo, perder o robo no
		// meio da operacao (cabo solto, robo desligado) e' bem mais provavel do que
		// rodando embarcado, e o SportModeState_ nao tem timestamp proprio -- sem esta
		// checagem republicariamos a ultima velocidade para sempre, e o base_ackerman
		// integraria uma viagem que o robo nao esta' fazendo.
		if (valid && (carmen_get_time() - state_timestamp) > g_state_timeout)
		{
			valid = false;
			g_robot_engaged.store(false);
		}

		if (!valid)
		{
			// Deixar de escrever no CAN e' o certo aqui: o ojArgos/ford_escape_hybrid
			// enxergam o silencio, enquanto um 0.0 seria lido como "parado" e a pose
			// congelaria com o robo andando.
			if ((heartbeat % (int) g_odometry_rate) == 0)
				printf("argos_unitree_driver: %s SportModeState do robo em '%s' (interface '%s').\n",
						had_state_before ? "PERDI o" : "ainda sem",
						TOPIC_SPORT_MODE_STATE, g_network_interface);
			fflush(stdout);
			heartbeat++;
			usleep(period_us);
			continue;
		}

		if (!had_state_before)
		{
			printf("argos_unitree_driver: recebendo SportModeState do robo (mode=%d).\n", (int) g_state_mode);
			fflush(stdout);
			had_state_before = true;
		}

		// Zona morta. Com o robo PARADO o Go2 reporta velocity[0] oscilando em torno de
		// -0.008 m/s (medido). Sem cortar isso, o base_ackerman integra ~0,5 m de deriva
		// por minuto com o robo imovel. O exemplo do I2CA faz o mesmo corte
		// (ros_to_carmen_odometry.cpp:88).
		if (fabs(v) < g_v_deadband)
			v = 0.0;

		// phi so' e' observavel quando ha' translacao: com v ~ 0 a razao yaw_rate/v
		// explode e o atan satura em +-pi/2, o que faria a pose girar parada.
		double phi = 0.0;
		if (fabs(v) > g_v_min_for_phi)
			phi = atan((yaw_rate * g_distance_between_axles) / v);

		// Passa-baixa de primeira ordem. O DDS entrega ~500 Hz e escrevemos a 60 Hz;
		// sem filtro cada frame seria uma amostra instantanea de um sinal ruidoso, e o
		// que chegaria no ford_escape_hybrid seria aliasing, nao odometria.
		filtered_v += g_v_filter_gain * (v - filtered_v);
		filtered_phi += g_phi_filter_gain * (phi - filtered_phi);

		if (filtered_phi > g_max_phi)
			filtered_phi = g_max_phi;
		else if (filtered_phi < -g_max_phi)
			filtered_phi = -g_max_phi;

		send_can_float(CAN_ID_VELOCITY, (float) filtered_v);
		send_can_float(CAN_ID_STEERING, (float) filtered_phi);

		heartbeat++;
		usleep(period_us);
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Controle do robo                                                                             //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


// Converte (v, phi) de Ackermann no par (vx, vyaw) que o Go2 aceita.
static void
ackermann_to_go2(double v, double phi, float *vx, float *vyaw)
{
	if (v > g_max_v)
		v = g_max_v;
	else if (v < g_max_v_reverse)
		v = g_max_v_reverse;

	if (phi > g_max_phi)
		phi = g_max_phi;
	else if (phi < -g_max_phi)
		phi = -g_max_phi;

	// Modelo de bicicleta: a taxa de guinada e' proporcional a' velocidade. Com v = 0
	// isso da' vyaw = 0 -- de proposito. O Go2 gira parado, mas o planejador do carmen
	// planeja em Ackermann e nunca pede giro sem translacao; deixar o robo girar aqui
	// criaria movimento que a odometria (que tambem e' Ackermann) nao representa, e a
	// pose divergiria em silencio.
	double yaw_rate = (v * tan(phi)) / g_distance_between_axles;

	if (yaw_rate > g_max_vyaw)
		yaw_rate = g_max_vyaw;
	else if (yaw_rate < -g_max_vyaw)
		yaw_rate = -g_max_vyaw;

	*vx = (float) v;
	*vyaw = (float) yaw_rate;
}


// Thread 4.
static void
robot_command_loop(unitree::robot::go2::SportClient *sport_client)
{
	const useconds_t period_us = (useconds_t) (1000000.0 / g_command_rate);
	bool stopped = true;

	while (g_running.load())
	{
		double v, phi, command_timestamp;
		bool valid;

		{
			std::lock_guard<std::mutex> lock(g_command_mutex);
			v = g_command_v;
			phi = g_command_phi;
			command_timestamp = g_command_timestamp;
			valid = g_command_valid;
		}

		// Watchdog: se o ford_escape_hybrid parar de emitir 0x100 (modulo caiu, ojArgos
		// caiu, vcan sumiu), o robo NAO pode continuar com o ultimo comando. O Go2
		// mantem a ultima velocidade ate' receber outra ordem, entao o silencio tem que
		// virar StopMove() explicito.
		// Postura pedida pela missao (via set parking brake / set engine). Roda AQUI, e nao
		// no handler do IPC, porque cada uma destas chamadas e' um RPC com timeout.
		const int requested = g_posture_request.exchange(POSTURE_NONE);
		if ((requested != POSTURE_NONE) && !g_dry_run)
		{
			int32_t ret = 0;

			// Sair de qualquer postura passa por parar de andar primeiro: o Go2 nao
			// aceita StandDown/Sit enquanto esta' em locomocao (mode=3).
			sport_client->StopMove();
			stopped = true;

			switch (requested)
			{
				case POSTURE_SIT:        ret = sport_client->Sit();          break;
				case POSTURE_STAND_DOWN: ret = sport_client->StandDown();    break;
				case POSTURE_DAMP:       ret = sport_client->Damp();         break;
				case POSTURE_STAND:      ret = sport_client->RecoveryStand(); break;
				default:                 break;
			}

			g_posture_held.store(requested != POSTURE_STAND);

			printf("argos_unitree_driver: postura '%s' %s (ret=%d).%s\n",
					posture_name((posture_t) requested),
					(ret == 0) ? "aceita" : "RECUSADA", (int) ret,
					g_posture_held.load() ? " Motion command ignorado ate' levantar." : "");
			fflush(stdout);
		}
		else if ((requested != POSTURE_NONE) && g_dry_run)
		{
			g_posture_held.store(requested != POSTURE_STAND);
			printf("argos_unitree_driver: DRY RUN, postura '%s' apenas registrada.\n",
					posture_name((posture_t) requested));
			fflush(stdout);
		}

		// Pousado, o robo ignora o motion command. Sem isto, o primeiro 0x100 nao-nulo que
		// chegasse depois do `stop` da missao levantaria o robo e o poria para andar.
		if (g_posture_held.load())
		{
			usleep(period_us);
			continue;
		}

		bool command_is_fresh = valid && ((carmen_get_time() - command_timestamp) <= g_command_timeout);

		// O volante e' integrado, entao ele GUARDA o ultimo angulo. Se o comando expirou
		// (ford_escape_hybrid caiu, cabo do CAN sumiu), zerar: senao, quando o comando
		// voltar, o robo retoma a curva de antes do silencio, que ninguem pediu.
		if (!command_is_fresh)
			g_steering_angle = 0.0;

		float vx = 0.0f, vyaw = 0.0f;

		if (command_is_fresh)
			ackermann_to_go2(v, phi, &vx, &vyaw);

		// Comando nulo tem que virar StopMove(), NAO Move(0, 0, 0).
		//
		// Esta distincao e' a diferenca entre o robo parado e o robo "sapateando".
		// Move() -- ate' com os tres eixos zerados -- coloca e MANTEM o Go2 em
		// locomocao (mode=3), onde ele marcha no lugar em vez de ficar imovel. Marchando
		// ele escorrega: medimos velocity[1] (lateral) e yaw_speed oscilando em +-0,1
		// com o robo supostamente parado, e ~2 m de deslocamento real acumulado. Pior,
		// esse movimento parasita volta pela odometria, entra no 0x425/0x80 e o
		// ford_escape_hybrid passa a fechar a malha em cima de um movimento que ninguem
		// pediu.
		//
		// StopMove() tira o robo da locomocao e devolve ele para balance stand (mode=1),
		// onde velocity fica em ~0,003 e a pose para de andar sozinha.
		bool command_is_null = (fabs(vx) < g_v_command_deadband) &&
				(fabs(vyaw) < g_vyaw_command_deadband);

		if (!g_dry_run)
		{
			if (command_is_fresh && !command_is_null)
			{
				// vy = 0 SEMPRE: o Go2 anda de lado se este campo for diferente de zero,
				// e um Ackermann nao tem esse grau de liberdade -- a odometria nao
				// conseguiria representar o deslocamento lateral e a pose derivaria.
				const int32_t ret = sport_client->Move(vx, 0.0f, vyaw);

				// O robo pode RECUSAR o RPC (servico de sport ocupado pelo app ou pelo
				// controle remoto, modo errado, api lock). Sem olhar o retorno o driver
				// fica publicando no vazio e parece que "nao recebe comando".
				if (ret != 0)
				{
					EXEC_COOLDOWN_LOCAL(fprintf(stderr,
							"argos_unitree_driver: Move(%.3f, 0, %.3f) RECUSADO pelo robo, ret=%d. "
							"Desligue o app/controle remoto do Go2 ou use -control_mode wireless.\n",
							vx, vyaw, (int) ret), 1.0);
				}
				else if (g_verbose)
				{
					EXEC_COOLDOWN_LOCAL(printf(
							"argos_unitree_driver: Move(%.3f, 0, %.3f) aceito.\n", vx, vyaw), 1.0);
				}

				stopped = false;
			}
			else if (!stopped)
			{
				const int32_t ret = sport_client->StopMove();
				if (ret != 0)
					fprintf(stderr, "argos_unitree_driver: StopMove() recusado, ret=%d.\n", (int) ret);
				stopped = true;
			}
		}

		usleep(period_us);
	}

	if (!g_dry_run && !stopped)
		sport_client->StopMove();
}


static void
shutdown_module(int sig)
{
	static int done = 0;

	if (!done)
	{
		done = 1;
		g_running.store(false);

		// Antes de qualquer outra coisa: tirar o robo de locomocao. Sem isto, matar o
		// driver com o robo andando deixa ele andando -- foi assim que o robo continuou
		// se deslocando depois de um pkill durante os testes.
		if ((g_sport_client != NULL) && !g_dry_run)
		{
			g_sport_client->StopMove();
			g_sport_client->BalanceStand();
		}

		if (g_can_socket >= 0)
			close(g_can_socket);
		carmen_ipc_disconnect();
		printf("\nargos_unitree_driver: desconectado do IPC. Sig: %d.\n", sig);
		fflush(stdout);
	}

	exit(0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Inicializacoes                                                                               //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
read_parameters(int argc, char **argv)
{
	// Geometria e limites: reaproveita os parametros de robot que o ARGOS ja' tem no
	// carmen-argos.ini -- nada de parametro novo no .ini pra este modulo.
	carmen_param_t robot_param_list[] =
	{
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &g_distance_between_axles, 0, NULL},
		{(char *) "robot", (char *) "max_velocity",                          CARMEN_PARAM_DOUBLE, &g_max_v,                  0, NULL},
		{(char *) "robot", (char *) "max_velocity_reverse",                  CARMEN_PARAM_DOUBLE, &g_max_v_reverse,          0, NULL},
		{(char *) "robot", (char *) "max_steering_angle",                    CARMEN_PARAM_DOUBLE, &g_max_phi,                0, NULL},
		{(char *) "robot", (char *) "PID_steering_kp",                       CARMEN_PARAM_DOUBLE, &g_steering_kp,            0, NULL},
	};
	carmen_param_install_params(argc, argv, robot_param_list,
			sizeof(robot_param_list) / sizeof(robot_param_list[0]));

	// Os do driver sao so' de linha de comando: allow_unfound deixa o default valer
	// quando nao vem nada, sem exigir entrada no .ini.
	carmen_param_allow_unfound_variables(1);
	carmen_param_t driver_param_list[] =
	{
		{(char *) "commandline", (char *) "network_interface", CARMEN_PARAM_STRING, &g_network_interface, 0, NULL},
		{(char *) "commandline", (char *) "can_interface",     CARMEN_PARAM_STRING, &g_can_interface,     0, NULL},
		{(char *) "commandline", (char *) "command_rate",      CARMEN_PARAM_DOUBLE, &g_command_rate,      0, NULL},
		{(char *) "commandline", (char *) "odometry_rate",     CARMEN_PARAM_DOUBLE, &g_odometry_rate,     0, NULL},
		{(char *) "commandline", (char *) "command_timeout",   CARMEN_PARAM_DOUBLE, &g_command_timeout,   0, NULL},
		{(char *) "commandline", (char *) "state_timeout",     CARMEN_PARAM_DOUBLE, &g_state_timeout,     0, NULL},
		{(char *) "commandline", (char *) "v_min_for_phi",     CARMEN_PARAM_DOUBLE, &g_v_min_for_phi,     0, NULL},
		{(char *) "commandline", (char *) "v_deadband",        CARMEN_PARAM_DOUBLE, &g_v_deadband,        0, NULL},
		{(char *) "commandline", (char *) "v_command_deadband",    CARMEN_PARAM_DOUBLE, &g_v_command_deadband,    0, NULL},
		{(char *) "commandline", (char *) "vyaw_command_deadband", CARMEN_PARAM_DOUBLE, &g_vyaw_command_deadband, 0, NULL},
		{(char *) "commandline", (char *) "v_filter_gain",     CARMEN_PARAM_DOUBLE, &g_v_filter_gain,     0, NULL},
		{(char *) "commandline", (char *) "phi_filter_gain",   CARMEN_PARAM_DOUBLE, &g_phi_filter_gain,   0, NULL},
		{(char *) "commandline", (char *) "max_vyaw",          CARMEN_PARAM_DOUBLE, &g_max_vyaw,          0, NULL},
		{(char *) "commandline", (char *) "auto_stand",        CARMEN_PARAM_ONOFF,  &g_auto_stand,        0, NULL},
		{(char *) "commandline", (char *) "dry_run",           CARMEN_PARAM_ONOFF,  &g_dry_run,           0, NULL},
		{(char *) "commandline", (char *) "verbose",           CARMEN_PARAM_ONOFF,  &g_verbose,           0, NULL},
		{(char *) "commandline", (char *) "phi_rate",          CARMEN_PARAM_DOUBLE, &g_phi_rate,          0, NULL},
		{(char *) "commandline", (char *) "posture",           CARMEN_PARAM_ONOFF,  &g_posture,           0, NULL},
		{(char *) "commandline", (char *) "park_posture",      CARMEN_PARAM_STRING, &g_park_posture_name, 0, NULL},
		{(char *) "commandline", (char *) "engine_off_posture", CARMEN_PARAM_STRING, &g_engine_off_posture_name, 0, NULL},
	};
	carmen_param_install_params(argc, argv, driver_param_list,
			sizeof(driver_param_list) / sizeof(driver_param_list[0]));
	carmen_param_allow_unfound_variables(0);

	if (g_network_interface == NULL)
		g_network_interface = (char *) "eth0";

	if (g_can_interface == NULL)
		g_can_interface = (char *) "vcan0";

	if (g_distance_between_axles <= 0.0)
	{
		fprintf(stderr, "argos_unitree_driver: robot_distance_between_front_and_rear_axles = %lf invalido.\n",
				g_distance_between_axles);
		exit(1);
	}

	g_park_posture = posture_from_name(g_park_posture_name, POSTURE_STAND_DOWN);
	g_engine_off_posture = posture_from_name(g_engine_off_posture_name, POSTURE_DAMP);

	if (g_phi_rate <= 0.0)
	{
		if (g_steering_kp > 0.0)
			g_phi_rate = 100.0 / (g_steering_kp * PHI_LOOP_TIME_CONSTANT);
		else
			g_phi_rate = g_max_phi / 0.5;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////



// O CycloneDDS LANCA excecao quando a interface nao existe ou esta' sem carrier (robo
// desligado, cabo fora). Sem capturar, o std::terminate derruba o processo com core dump
// e a unica pista fica no stderr -- que o proccontrol nao mostra. Vira loop de respawn
// sem explicacao. Aqui a mensagem e' explicita e a saida e' limpa.
static void
init_dds_or_die(const char *module, int domain_id, const char *network_interface)
{
	try
	{
		unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
	}
	catch (const std::exception &e)
	{
		fprintf(stderr, "\n%s: NAO consegui subir o DDS na interface '%s'.\n", module, network_interface);
		fprintf(stderr, "%s: confira com `ip -br link show %s` -- precisa estar UP e com carrier\n",
				module, network_interface);
		fprintf(stderr, "%s: (robo desligado ou cabo fora da' exatamente este erro).\n", module);
		fprintf(stderr, "%s: detalhe do CycloneDDS: %s\n\n", module, e.what());
		exit(1);
	}
}


int
main(int argc, char **argv)
{
	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	printf("argos_unitree_driver: rede '%s', CAN '%s', entre-eixos %.3lf m, "
			"v em [%.2lf, %.2lf] m/s, phi max %.2lf rad (esterco %.2lf rad/s a 100%%)%s.\n",
			g_network_interface, g_can_interface, g_distance_between_axles,
			g_max_v_reverse, g_max_v, g_max_phi, g_phi_rate,
			g_dry_run ? ", DRY RUN (nao envia comando ao robo)" : "");
	fflush(stdout);

	g_can_socket = open_can_socket(g_can_interface);
	if (g_can_socket < 0)
		exit(1);

	// Conecta no DDS do robo pela interface de rede indicada (o cabo). Tem que vir
	// antes de qualquer ChannelSubscriber/Client.
	init_dds_or_die("argos_unitree_driver", 0, g_network_interface);

	unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> state_subscriber;
	state_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
			TOPIC_SPORT_MODE_STATE));
	state_subscriber->InitChannel(sport_mode_state_handler, 1);

	unitree::robot::go2::SportClient sport_client;
	sport_client.SetTimeout(1.0f);
	sport_client.Init();
	g_sport_client = &sport_client;

	if (!g_dry_run && g_auto_stand)
	{
		// Sem estar em pe o robo ignora Move(). Um sleep curto antes deixa o primeiro
		// SportModeState chegar, pra decidir com base no mode() real.
		sleep(1);
		if (!g_robot_engaged.load())
		{
			printf("argos_unitree_driver: robo em mode=%d, mandando StandUp + BalanceStand.\n",
					(int) g_state_mode);
			fflush(stdout);
			sport_client.StandUp();
			sleep(1);
			sport_client.BalanceStand();
		}
	}

	std::thread odometry_thread(odometry_loop);
	std::thread can_thread(can_read_loop);
	std::thread command_thread(robot_command_loop, &sport_client);

	// Este modulo nao publica nem assina IPC -- quem fala com o carmen e' o
	// ford_escape_hybrid, do outro lado do ojArgos. O dispatch fica aqui so' para o
	// processo continuar vivo e visivel para o proccontrol.
	if (g_posture)
	{
		carmen_ford_escape_subscribe_engine_and_parking_brake_message(NULL,
				(carmen_handler_t) engine_and_parking_brake_handler, CARMEN_SUBSCRIBE_LATEST);
		printf("argos_unitree_driver: postura ligada -- freio de mao -> '%s', motor desligado -> '%s'.\n",
				posture_name(g_park_posture), posture_name(g_engine_off_posture));
		fflush(stdout);
	}

	carmen_ipc_dispatch();

	g_running.store(false);
	odometry_thread.join();
	can_thread.join();
	command_thread.join();

	return (0);
}
