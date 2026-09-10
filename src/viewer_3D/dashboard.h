#ifndef VIEWER_3D_DASHBOARD_H
#define VIEWER_3D_DASHBOARD_H

/*
 * Painel do carro para o viewer_3D.
 *
 * Desenha com cairo numa superficie em memoria, sobe como textura e cola por cima da cena 3D.
 * O redesenho so' acontece quando algum valor muda (ou a cada 200 ms), entao o custo em regime
 * permanente e' o de um quad texturizado.
 *
 * Nao depende de nada do viewer_3D: quem preenche o dashboard_data_t e' o programa hospedeiro.
 */

#ifdef __cplusplus
extern "C"
{
#endif

#define DASHBOARD_MAX_MISSIONS		64
#define DASHBOARD_MISSION_NAME_SIZE	128
#define DASHBOARD_HISTORY_SIZE		240	// ~24 s a 10 Hz

// Marchas. O painel desenha a regua P R N D L com a corrente acesa.
#define DASHBOARD_GEAR_UNKNOWN		0
#define DASHBOARD_GEAR_P			1
#define DASHBOARD_GEAR_R			2
#define DASHBOARD_GEAR_N			3
#define DASHBOARD_GEAR_D			4
#define DASHBOARD_GEAR_L			5

typedef struct
{
	// ---- movimento -------------------------------------------------------------------
	double v;					// m/s (positivo para frente)
	double phi;					// rad, esterccamento das RODAS (do base_ackerman / globalpos)
	double steering_wheel;		// GRAUS do VOLANTE = graus(phi) * relaccao de direccao
	double steering_wheel_pct;	// -100..100 %, do ford_escape (g_XGV_steering); 0 = sem CAN
	double a_long;				// m/s^2, aceleraccao longitudinal
	double a_lat;				// m/s^2, aceleraccao lateral
	double rpm;

	// Marcha: DASHBOARD_GEAR_*, nunca o codigo cru do CAN. Quem preenche traduz.
	int    gear;
	int    gear_estimated;		// 1 = deduzida do movimento porque o CAN nao disse (simulador)
	int    gear_raw;			// codigo que veio do CAN, exibido quando nao se sabe traduzir

	// ---- comandos --------------------------------------------------------------------
	double throttle;			// 0..100 %
	double brake;				// 0..100 %
	double v_command;			// m/s pedido pelo obstacle_avoider / MPP
	double phi_command;			// rad pedido

	// ---- estado do veiculo -----------------------------------------------------------
	int    autonomous;
	int    engine;
	int    parking_brake;
	int    turn_signal;			// 0 nenhum, 1 esquerda, 2 direita, 3 alerta
	int    headlights;
	int    horn;

	// ---- planejamento ----------------------------------------------------------------
	char   behavior_state[128];
	char   route_planner_state[64];
	char   mission_state[64];
	char   mission_name[DASHBOARD_MISSION_NAME_SIZE];
	char   localize_state[64];
	int    frenet_num_paths;
	int    frenet_selected;		// indice do caminho escolhido, -1 se nenhum
	double goal_distance;		// m ate' o goal corrente, < 0 se nao ha'
	double max_speed;			// m/s
	double max_phi;				// rad, fundo de escala da regua de direccao
	double steering_ratio;		// voltas do volante por unidade de angulo da roda (~16)

	// ---- pose ------------------------------------------------------------------------
	double x, y, theta;

	// ---- saude (timestamps do ultimo recebimento; 0 = nunca) -------------------------
	double t_globalpos;
	double t_can;
	double t_camera;
	double t_route;
	double t_frenet;
} dashboard_data_t;


// Cria as estruturas do painel. Chamar depois de ter contexto GL corrente.
void dashboard_init(void);
void dashboard_destroy(void);

// Ponteiro para os dados: os handlers de IPC escrevem direto aqui.
dashboard_data_t *dashboard_data(void);

// Lista de missoes mostrada no painel lateral (copia os nomes).
void dashboard_set_missions(char **names, int num_missions);

// Lugares para onde a camera pode ser apontada (as anotaccoes do mapa). Copia os nomes.
void dashboard_set_camera_targets(char **names, int num_targets);

// 1 enquanto a camera esta presa a um lugar; 0 quando esta seguindo o carro. So' muda o texto.
void dashboard_set_camera_following(int following);

// Ultimo quadro da camera. 'channels' 1 ou 3; 3 canais sao tratados como BGR.
void dashboard_set_camera_image(const unsigned char *data, int width, int height, int channels);

// Quantos pixels o painel come na base da janela. Serve para o programa hospedeiro subir o
// proprio menu e nao deixa-lo escondido embaixo do painel. Zero quando o painel esta oculto ou
// nao esta encostado na base.
double dashboard_menu_bottom_offset(int window_width, int window_height);

// Desenha o painel por cima da cena 3D. Chamar por ultimo, depois de tudo.
void dashboard_draw(int window_width, int window_height);

// Liga/desliga a lista de missoes e a miniatura da camera.
void dashboard_toggle_missions(void);
void dashboard_toggle_camera(void);
void dashboard_toggle_camera_panel(void);
void dashboard_toggle_panel(void);
int  dashboard_missions_visible(void);
int  dashboard_camera_panel_visible(void);

// ---- aparencia -------------------------------------------------------------------------------
// posiccao do painel
#define DASHBOARD_POSITION_BOTTOM		0	// barra colada embaixo, largura toda
#define DASHBOARD_POSITION_TOP			1	// barra colada em cima, largura toda
#define DASHBOARD_POSITION_BOTTOM_LEFT	2	// flutuante, canto inferior esquerdo
#define DASHBOARD_POSITION_BOTTOM_RIGHT	3	// flutuante, canto inferior direito
#define DASHBOARD_POSITION_TOP_LEFT		4
#define DASHBOARD_POSITION_TOP_RIGHT	5
#define DASHBOARD_NUM_POSITIONS			6

// escala 0 = automatica (acompanha a largura da janela); opacidade 0.15..1.0
void dashboard_set_style(double scale, double opacity, int position);
void dashboard_cycle_position(void);
void dashboard_add_scale(double delta);
void dashboard_add_opacity(double delta);
double dashboard_get_scale(void);
double dashboard_get_opacity(void);
int    dashboard_get_position(void);

// ---- acoes -----------------------------------------------------------------------------------
#define DASHBOARD_ACTION_NONE			0	// o clique nao era no painel
#define DASHBOARD_ACTION_CONSUMED		1	// era no painel, nada a fazer
#define DASHBOARD_ACTION_MISSION		2	// executar a missao que ficou em mission_out
#define DASHBOARD_ACTION_ABORT			3	// abortar a missao
#define DASHBOARD_ACTION_SET_POSE		4	// entrar no modo "clique no mapa para por o robo"
#define DASHBOARD_ACTION_SET_GOAL		5	// entrar no modo "clique no mapa para por o destino"
#define DASHBOARD_ACTION_GO				6
#define DASHBOARD_ACTION_STOP			7
#define DASHBOARD_ACTION_MISSIONS		8	// abrir/fechar a lista de missoes
#define DASHBOARD_ACTION_CAMERA_PANEL	9	// abrir/fechar a lista de lugares da camera
#define DASHBOARD_ACTION_CAMERA_TARGET	10	// mirar a camera no lugar que ficou em mission_out
#define DASHBOARD_ACTION_CAMERA_FOLLOW	11	// voltar a camera a seguir o carro

// modo corrente do mouse sobre o mapa
#define DASHBOARD_MODE_NORMAL			0
#define DASHBOARD_MODE_PICK_POSE		1
#define DASHBOARD_MODE_PICK_GOAL		2

void dashboard_set_mode(int mode);
int  dashboard_get_mode(void);

// Trata um clique; devolve um DASHBOARD_ACTION_*.
int dashboard_mouse(int type, int button, int x, int y, int window_width, int window_height,
		char *mission_out, int mission_out_size);

#ifdef __cplusplus
}
#endif

#endif
