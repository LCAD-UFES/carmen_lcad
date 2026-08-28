/*
 * imu_teste - painel de bancada para conferir se uma IMU esta' funcionando.
 *
 * Assina as mensagens do modulo xsens e desenha, em tempo real, os tres vetores
 * do sensor (acelerometro, giroscopio, magnetometro) com as suas unidades, a
 * orientacao dada pelo filtro de Kalman, e a taxa de mensagens.
 *
 * O ponto do modulo e' o veredito: uma IMU parada e saudavel tem |acc| = 9.81 m/s^2
 * (so' a gravidade), |gyr| ~ 0 rad/s e |mag| ~ 1 a.u. Cada painel compara a norma
 * medida com o valor esperado e mostra OK / ATENCAO / RUIM, para nao ser preciso
 * ficar interpretando numero solto.
 *
 * Teclas: q / Esc = sair, espaco = congelar a tela, z = zerar os picos das escalas.
 */

#include <carmen/carmen.h>
#include <carmen/xsens_interface.h>
#include <carmen/rotation_geometry.h>

#include <gtk/gtk.h>
#include <math.h>
#include <string.h>

#define WIN_W 1020
#define WIN_H 700

/* Valores esperados de uma IMU parada e sadia. */
#define GRAVIDADE          9.80665   /* m/s^2  */
#define CAMPO_TERRESTRE    1.0       /* a.u. -- o MTi normaliza o magnetometro */
#define GYR_PARADO         0.02      /* rad/s -- acima disto ja' nao esta' parada */

typedef struct
{
	double x, y, z;
} vec3_t;

/* ------------------------------------------------------------------ estado */

static struct
{
	int    recebeu_algo;
	vec3_t acc, gyr, mag;          /* m/s^2, rad/s, a.u.                     */
	double roll, pitch, yaw;       /* rad                                    */
	double quat[4];
	int    tem_quat;
	double temp;                   /* graus C                                */
	int    count;                  /* contador de amostra do sensor          */
	double timestamp;
	char   origem[32];             /* qual das mensagens do xsens chegou     */
	char   host[64];
} imu;

static struct
{
	double taxa_hz;                /* mensagens publicadas por segundo       */
	double taxa_sensor_hz;         /* amostras geradas pelo sensor           */
	double perda_pct;
	int    msgs;
	long   amostras;
	int    count_anterior;
	int    tem_count_anterior;
	double t0;
} est;

static int    congelado = 0;
static double pico_acc = 0.0, pico_gyr = 0.0, pico_mag = 0.0;

static GtkWidget *area_de_desenho = NULL;

/* ------------------------------------------------------------- utilitarios */

static double
norma(vec3_t v)
{
	return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}


static double
agora(void)
{
	return carmen_get_time();
}

/* ------------------------------------------------------------ cores do tema */

typedef struct { double r, g, b; } cor_t;

static const cor_t COR_FUNDO   = {0.078, 0.090, 0.110};
static const cor_t COR_PAINEL  = {0.110, 0.125, 0.153};
static const cor_t COR_BORDA   = {0.165, 0.184, 0.220};
static const cor_t COR_TEXTO   = {0.847, 0.867, 0.898};
static const cor_t COR_FRACA   = {0.541, 0.573, 0.620};
static const cor_t COR_X       = {0.898, 0.325, 0.294};
static const cor_t COR_Y       = {0.341, 0.671, 0.353};
static const cor_t COR_Z       = {0.290, 0.561, 0.878};
static const cor_t COR_OK      = {0.247, 0.725, 0.314};
static const cor_t COR_ATENCAO = {0.824, 0.600, 0.133};
static const cor_t COR_RUIM    = {0.973, 0.318, 0.286};

static void
set_cor(cairo_t *cr, cor_t c)
{
	cairo_set_source_rgb(cr, c.r, c.g, c.b);
}


static void
set_cor_alpha(cairo_t *cr, cor_t c, double a)
{
	cairo_set_source_rgba(cr, c.r, c.g, c.b, a);
}

/* ------------------------------------------------------------ texto e caixas */

static void
fonte(cairo_t *cr, double tamanho, int negrito, int mono)
{
	cairo_select_font_face(cr, mono ? "monospace" : "Sans", CAIRO_FONT_SLANT_NORMAL,
			negrito ? CAIRO_FONT_WEIGHT_BOLD : CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size(cr, tamanho);
}


static void
texto(cairo_t *cr, double x, double y, const char *s)
{
	cairo_move_to(cr, x, y);
	cairo_show_text(cr, s);
}


static double
largura_texto(cairo_t *cr, const char *s)
{
	cairo_text_extents_t ext;

	cairo_text_extents(cr, s, &ext);

	return ext.x_advance;
}


static void
texto_direita(cairo_t *cr, double x_direita, double y, const char *s)
{
	texto(cr, x_direita - largura_texto(cr, s), y, s);
}


static void
retangulo_arredondado(cairo_t *cr, double x, double y, double w, double h, double r)
{
	cairo_new_sub_path(cr);
	cairo_arc(cr, x + w - r, y + r,     r, -M_PI / 2.0,  0.0);
	cairo_arc(cr, x + w - r, y + h - r, r,  0.0,         M_PI / 2.0);
	cairo_arc(cr, x + r,     y + h - r, r,  M_PI / 2.0,  M_PI);
	cairo_arc(cr, x + r,     y + r,     r,  M_PI,        1.5 * M_PI);
	cairo_close_path(cr);
}


static void
painel(cairo_t *cr, double x, double y, double w, double h, const char *titulo, const char *unidade)
{
	retangulo_arredondado(cr, x, y, w, h, 8.0);
	set_cor(cr, COR_PAINEL);
	cairo_fill_preserve(cr);
	set_cor(cr, COR_BORDA);
	cairo_set_line_width(cr, 1.0);
	cairo_stroke(cr);

	if (titulo != NULL)
	{
		fonte(cr, 13.0, 1, 0);
		set_cor(cr, COR_TEXTO);
		texto(cr, x + 16.0, y + 26.0, titulo);
	}

	if (unidade != NULL)
	{
		double lw;

		fonte(cr, 12.0, 0, 1);
		lw = largura_texto(cr, unidade);

		retangulo_arredondado(cr, x + w - lw - 30.0, y + 12.0, lw + 16.0, 20.0, 4.0);
		set_cor(cr, COR_BORDA);
		cairo_fill(cr);

		set_cor(cr, COR_FRACA);
		texto(cr, x + w - lw - 22.0, y + 26.0, unidade);
	}
}


/* Chip colorido com o veredito do painel. */
static void
chip(cairo_t *cr, double x_direita, double y, const char *rotulo, cor_t c)
{
	double lw;

	fonte(cr, 11.0, 1, 0);
	lw = largura_texto(cr, rotulo);

	retangulo_arredondado(cr, x_direita - lw - 16.0, y - 13.0, lw + 16.0, 19.0, 9.5);
	set_cor_alpha(cr, c, 0.18);
	cairo_fill(cr);

	set_cor(cr, c);
	texto(cr, x_direita - lw - 8.0, y, rotulo);
}

/* --------------------------------------------------------- barra de um eixo */

/*
 * Barra com zero no centro: da' para ver sinal e magnitude de um golpe de olho,
 * que e' o que interessa quando se esta' girando o sensor na mao.
 */
static void
barra_eixo(cairo_t *cr, double x, double y, double w, double h,
		const char *rotulo, double valor, double escala, cor_t c, const char *unidade)
{
	double centro = x + w / 2.0;
	double comprimento;
	char buf[64];

	/* trilho */
	retangulo_arredondado(cr, x, y, w, h, h / 2.0);
	set_cor(cr, COR_FUNDO);
	cairo_fill(cr);

	/* marca do zero */
	set_cor(cr, COR_BORDA);
	cairo_set_line_width(cr, 1.0);
	cairo_move_to(cr, centro, y - 3.0);
	cairo_line_to(cr, centro, y + h + 3.0);
	cairo_stroke(cr);

	/* a barra em si, saturando no fim do trilho */
	comprimento = (valor / escala) * (w / 2.0);
	if (comprimento >  w / 2.0) comprimento =  w / 2.0;
	if (comprimento < -w / 2.0) comprimento = -w / 2.0;

	if (fabs(comprimento) > 1.0)
	{
		if (comprimento >= 0.0)
			retangulo_arredondado(cr, centro, y, comprimento, h, h / 2.0);
		else
			retangulo_arredondado(cr, centro + comprimento, y, -comprimento, h, h / 2.0);

		set_cor(cr, c);
		cairo_fill(cr);
	}

	/* rotulo do eixo, a' esquerda do trilho */
	fonte(cr, 13.0, 1, 0);
	set_cor(cr, c);
	texto(cr, x - 22.0, y + h - 2.0, rotulo);

	/* valor numerico, a' direita */
	snprintf(buf, sizeof(buf), "%+9.4f %s", valor, unidade);
	fonte(cr, 13.0, 0, 1);
	set_cor(cr, COR_TEXTO);
	texto(cr, x + w + 14.0, y + h - 2.0, buf);
}

/* ------------------------------------------------- painel de um sensor (vetor) */

/*
 * A escala das barras acompanha o pico recente, com decaimento, para o painel
 * servir tanto para o sensor parado quanto para o sensor sendo sacudido.
 */
static double
atualiza_escala(double *pico, vec3_t v, double escala_minima)
{
	double m = fabs(v.x);

	if (fabs(v.y) > m) m = fabs(v.y);
	if (fabs(v.z) > m) m = fabs(v.z);

	if (!congelado)
	{
		*pico *= 0.995;
		if (m > *pico)
			*pico = m;
	}

	return (*pico * 1.15 > escala_minima) ? *pico * 1.15 : escala_minima;
}


static void
painel_vetor(cairo_t *cr, double x, double y, double w, double h,
		const char *titulo, const char *unidade, vec3_t v, double escala,
		double esperado, double tolerancia, const char *nota_norma)
{
	double bx = x + 46.0;
	double bw = w - 46.0 - 132.0;
	double n = norma(v);
	char buf[128];

	painel(cr, x, y, w, h, titulo, unidade);

	barra_eixo(cr, bx, y + 44.0, bw, 14.0, "X", v.x, escala, COR_X, unidade);
	barra_eixo(cr, bx, y + 72.0, bw, 14.0, "Y", v.y, escala, COR_Y, unidade);
	barra_eixo(cr, bx, y + 100.0, bw, 14.0, "Z", v.z, escala, COR_Z, unidade);

	/* fundo de escala, para o tamanho da barra significar alguma coisa */
	fonte(cr, 10.0, 0, 1);
	set_cor(cr, COR_FRACA);
	snprintf(buf, sizeof(buf), "-%.2f", escala);
	texto(cr, bx, y + 132.0, buf);
	snprintf(buf, sizeof(buf), "+%.2f", escala);
	texto_direita(cr, bx + bw, y + 132.0, buf);
	texto(cr, bx + bw / 2.0 - 3.0, y + 132.0, "0");

	/* norma e veredito */
	fonte(cr, 13.0, 0, 1);
	set_cor(cr, COR_TEXTO);
	snprintf(buf, sizeof(buf), "|v| = %8.4f %s", n, unidade);
	texto(cr, x + 16.0, y + h - 14.0, buf);

	fonte(cr, 11.0, 0, 0);
	set_cor(cr, COR_FRACA);
	texto(cr, x + 190.0, y + h - 14.0, nota_norma);

	if (!imu.recebeu_algo)
		chip(cr, x + w - 16.0, y + h - 14.0, "SEM DADO", COR_FRACA);
	else if (fabs(n - esperado) <= tolerancia)
		chip(cr, x + w - 16.0, y + h - 14.0, "OK", COR_OK);
	else if (fabs(n - esperado) <= tolerancia * 3.0)
		chip(cr, x + w - 16.0, y + h - 14.0, "ATENCAO", COR_ATENCAO);
	else
		chip(cr, x + w - 16.0, y + h - 14.0, "FORA", COR_RUIM);
}

/* ------------------------------------------------- horizonte artificial + bussola */

static void
horizonte(cairo_t *cr, double cx, double cy, double raio, double roll, double pitch)
{
	double px = raio * (pitch / (M_PI / 2.0));   /* 90 graus = borda do circulo */
	int i;

	cairo_save(cr);

	cairo_arc(cr, cx, cy, raio, 0.0, 2.0 * M_PI);
	cairo_clip(cr);

	cairo_translate(cr, cx, cy);
	cairo_rotate(cr, roll);
	cairo_translate(cr, 0.0, px);

	/* ceu */
	cairo_rectangle(cr, -raio * 2.0, -raio * 2.0, raio * 4.0, raio * 2.0);
	cairo_set_source_rgb(cr, 0.180, 0.376, 0.588);
	cairo_fill(cr);

	/* solo */
	cairo_rectangle(cr, -raio * 2.0, 0.0, raio * 4.0, raio * 2.0);
	cairo_set_source_rgb(cr, 0.400, 0.290, 0.180);
	cairo_fill(cr);

	/* linha do horizonte */
	cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
	cairo_set_line_width(cr, 1.5);
	cairo_move_to(cr, -raio * 2.0, 0.0);
	cairo_line_to(cr,  raio * 2.0, 0.0);
	cairo_stroke(cr);

	/* escada de pitch de 10 em 10 graus */
	fonte(cr, 9.0, 0, 1);
	for (i = -60; i <= 60; i += 10)
	{
		double yy, meia;
		char buf[16];

		if (i == 0)
			continue;

		yy = -raio * (carmen_degrees_to_radians((double) i) / (M_PI / 2.0));
		meia = (i % 20 == 0) ? raio * 0.30 : raio * 0.16;

		cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.75);
		cairo_set_line_width(cr, 1.0);
		cairo_move_to(cr, -meia, yy);
		cairo_line_to(cr,  meia, yy);
		cairo_stroke(cr);

		if (i % 20 == 0)
		{
			snprintf(buf, sizeof(buf), "%d", i);
			texto(cr, meia + 4.0, yy + 3.0, buf);
		}
	}

	cairo_restore(cr);

	/* aro. cairo_save/restore guarda o estado grafico mas NAO o path: sem o
	   new_path o arco abaixo seria ligado por uma reta ao ponto corrente que o
	   cairo_show_text da escada de pitch deixou para tras. */
	cairo_new_path(cr);
	set_cor(cr, COR_BORDA);
	cairo_set_line_width(cr, 2.0);
	cairo_arc(cr, cx, cy, raio, 0.0, 2.0 * M_PI);
	cairo_stroke(cr);

	/* mira fixa do "veiculo" */
	cairo_set_source_rgb(cr, 1.0, 0.85, 0.2);
	cairo_set_line_width(cr, 2.0);
	cairo_move_to(cr, cx - raio * 0.5, cy);
	cairo_line_to(cr, cx - raio * 0.15, cy);
	cairo_move_to(cr, cx + raio * 0.15, cy);
	cairo_line_to(cr, cx + raio * 0.5, cy);
	cairo_move_to(cr, cx, cy - 4.0);
	cairo_line_to(cr, cx, cy + 4.0);
	cairo_stroke(cr);
}


/*
 * Mostrador do yaw. Convencao do carmen: angulo cresce no sentido anti-horario
 * a partir do eixo X do sensor -- NAO e' azimute magnetico (nao ha' correcao de
 * declinacao aqui), por isso o mostrador e' rotulado em graus e nao em N/S/L/O.
 */
static void
mostrador_yaw(cairo_t *cr, double cx, double cy, double raio, double yaw)
{
	int i;

	cairo_new_path(cr);
	set_cor(cr, COR_FUNDO);
	cairo_arc(cr, cx, cy, raio, 0.0, 2.0 * M_PI);
	cairo_fill(cr);

	set_cor(cr, COR_BORDA);
	cairo_set_line_width(cr, 2.0);
	cairo_arc(cr, cx, cy, raio, 0.0, 2.0 * M_PI);
	cairo_stroke(cr);

	fonte(cr, 9.0, 0, 1);
	for (i = 0; i < 360; i += 30)
	{
		double a = carmen_degrees_to_radians((double) i);
		double r0 = (i % 90 == 0) ? raio * 0.78 : raio * 0.87;

		set_cor(cr, (i % 90 == 0) ? COR_TEXTO : COR_FRACA);
		cairo_set_line_width(cr, (i % 90 == 0) ? 2.0 : 1.0);
		cairo_move_to(cr, cx + r0 * cos(-a), cy + r0 * sin(-a));
		cairo_line_to(cr, cx + raio * 0.96 * cos(-a), cy + raio * 0.96 * sin(-a));
		cairo_stroke(cr);

		if (i % 90 == 0)
		{
			char buf[16];

			snprintf(buf, sizeof(buf), "%d", i);
			set_cor(cr, COR_FRACA);
			texto(cr, cx + raio * 0.62 * cos(-a) - 6.0, cy + raio * 0.62 * sin(-a) + 3.0, buf);
		}
	}

	/* agulha */
	cairo_save(cr);
	cairo_translate(cr, cx, cy);
	cairo_rotate(cr, -yaw);

	set_cor(cr, COR_X);
	cairo_move_to(cr, raio * 0.80, 0.0);
	cairo_line_to(cr, -raio * 0.12, -raio * 0.14);
	cairo_line_to(cr, -raio * 0.12,  raio * 0.14);
	cairo_close_path(cr);
	cairo_fill(cr);

	cairo_restore(cr);

	cairo_new_path(cr);
	set_cor(cr, COR_TEXTO);
	cairo_arc(cr, cx, cy, 3.0, 0.0, 2.0 * M_PI);
	cairo_fill(cr);
}

/* --------------------------------------------------------------- cabecalho */

static void
cabecalho(cairo_t *cr, double x, double y, double w, double h)
{
	double idade = imu.recebeu_algo ? agora() - imu.timestamp : 0.0;
	char buf[256];
	cor_t c;
	const char *estado;

	painel(cr, x, y, w, h, NULL, NULL);

	fonte(cr, 17.0, 1, 0);
	set_cor(cr, COR_TEXTO);
	texto(cr, x + 16.0, y + 30.0, "imu_teste");

	fonte(cr, 12.0, 0, 0);
	set_cor(cr, COR_FRACA);
	texto(cr, x + 110.0, y + 30.0, "painel de conferencia da IMU (mensagens do modulo xsens)");

	fonte(cr, 11.0, 0, 0);
	set_cor(cr, COR_FRACA);
	texto(cr, x + 16.0, y + 50.0, "q/Esc = sair    espaco = congelar    z = zerar escalas");

	if (!imu.recebeu_algo)
	{
		estado = "AGUARDANDO MENSAGEM";
		c = COR_ATENCAO;
	}
	else if (idade > 1.0)
	{
		estado = "MENSAGENS PARARAM";
		c = COR_RUIM;
	}
	else if (congelado)
	{
		estado = "CONGELADO";
		c = COR_ATENCAO;
	}
	else
	{
		estado = "RECEBENDO";
		c = COR_OK;
	}
	chip(cr, x + w - 16.0, y + 28.0, estado, c);

	fonte(cr, 11.0, 0, 1);
	set_cor(cr, COR_FRACA);
	if (imu.recebeu_algo)
	{
		snprintf(buf, sizeof(buf), "%s   host %s   ultima ha %.2f s", imu.origem, imu.host, idade);
		texto_direita(cr, x + w - 16.0, y + 50.0, buf);
	}
	else
	{
		texto_direita(cr, x + w - 16.0, y + 50.0,
				"o modulo xsens esta' rodando? ./xsens -xsens_mti_dev /dev/ttyUSB0");
	}
}

/* ------------------------------------------------- painel de numeros crus */

static void
linha_valor(cairo_t *cr, double x, double y, double x_valor, const char *rotulo,
		const char *valor, cor_t cor_valor)
{
	fonte(cr, 11.0, 0, 0);
	set_cor(cr, COR_FRACA);
	texto(cr, x, y, rotulo);

	fonte(cr, 12.0, 0, 1);
	set_cor(cr, cor_valor);
	texto_direita(cr, x_valor, y, valor);
}


static void
painel_numeros(cairo_t *cr, double x, double y, double w, double h)
{
	double xv = x + w - 16.0;
	double yy = y + 46.0;
	char buf[128];
	cor_t c;

	painel(cr, x, y, w, h, "LEITURAS", NULL);

	snprintf(buf, sizeof(buf), "%+8.3f deg", carmen_radians_to_degrees(imu.roll));
	linha_valor(cr, x + 16.0, yy, xv, "roll", buf, COR_TEXTO);   yy += 22.0;
	snprintf(buf, sizeof(buf), "%+8.3f deg", carmen_radians_to_degrees(imu.pitch));
	linha_valor(cr, x + 16.0, yy, xv, "pitch", buf, COR_TEXTO);  yy += 22.0;
	snprintf(buf, sizeof(buf), "%+8.3f deg", carmen_radians_to_degrees(imu.yaw));
	linha_valor(cr, x + 16.0, yy, xv, "yaw", buf, COR_TEXTO);    yy += 28.0;

	if (imu.tem_quat)
	{
		snprintf(buf, sizeof(buf), "%+.4f %+.4f", imu.quat[0], imu.quat[1]);
		linha_valor(cr, x + 16.0, yy, xv, "quat q0 q1", buf, COR_FRACA); yy += 20.0;
		snprintf(buf, sizeof(buf), "%+.4f %+.4f", imu.quat[2], imu.quat[3]);
		linha_valor(cr, x + 16.0, yy, xv, "quat q2 q3", buf, COR_FRACA); yy += 28.0;
	}
	else
	{
		linha_valor(cr, x + 16.0, yy, xv, "quat", "(mensagem sem quaternion)", COR_FRACA);
		yy += 28.0;
	}

	snprintf(buf, sizeof(buf), "%.1f C", imu.temp);
	linha_valor(cr, x + 16.0, yy, xv, "temperatura", buf, COR_TEXTO); yy += 22.0;

	snprintf(buf, sizeof(buf), "%d", imu.count);
	linha_valor(cr, x + 16.0, yy, xv, "contador de amostra", buf, COR_TEXTO); yy += 28.0;

	snprintf(buf, sizeof(buf), "%.1f Hz", est.taxa_hz);
	linha_valor(cr, x + 16.0, yy, xv, "mensagens publicadas", buf, COR_TEXTO); yy += 22.0;

	snprintf(buf, sizeof(buf), "%.1f Hz", est.taxa_sensor_hz);
	linha_valor(cr, x + 16.0, yy, xv, "amostras do sensor", buf, COR_TEXTO); yy += 22.0;

	if (est.perda_pct < 2.0)
		c = COR_OK;
	else if (est.perda_pct < 20.0)
		c = COR_ATENCAO;
	else
		c = COR_RUIM;

	snprintf(buf, sizeof(buf), "%.1f %%", est.perda_pct);
	linha_valor(cr, x + 16.0, yy, xv, "amostras perdidas", buf, c);
}

/* ------------------------------------------------------------- desenho geral */

static gboolean
desenha(GtkWidget *widget, cairo_t *cr, gpointer dados)
{
	double m = 16.0;
	double lw, rw, rx;
	double esc_acc, esc_gyr, esc_mag;
	double y;
	GtkAllocation aloc;

	(void) dados;

	esc_acc = atualiza_escala(&pico_acc, imu.acc, 12.0);
	esc_gyr = atualiza_escala(&pico_gyr, imu.gyr, 0.5);
	esc_mag = atualiza_escala(&pico_mag, imu.mag, 1.2);

	/* pinta o fundo na area toda antes de escalar, senao sobra faixa suja
	   quando as proporcoes da janela nao batem com as do layout */
	set_cor(cr, COR_FUNDO);
	cairo_paint(cr);

	/*
	 * O layout abaixo e' todo em coordenadas fixas de WIN_W x WIN_H. O gerenciador
	 * de janelas nao respeita esse tamanho (o GNOME abriu em 1072x789 aqui), entao
	 * encaixamos o desenho na area real com escala uniforme e centralizado, em vez
	 * de deixar faixa morta de um lado e cortar do outro.
	 */
	if (widget != NULL)
	{
		double escala;

		gtk_widget_get_allocation(widget, &aloc);

		if (aloc.width > 0 && aloc.height > 0)
		{
			escala = (double) aloc.width / (double) WIN_W;
			if ((double) aloc.height / (double) WIN_H < escala)
				escala = (double) aloc.height / (double) WIN_H;

			cairo_translate(cr,
					(aloc.width  - WIN_W * escala) / 2.0,
					(aloc.height - WIN_H * escala) / 2.0);
			cairo_scale(cr, escala, escala);
		}
	}

	lw = 596.0;
	rx = m + lw + 12.0;
	rw = WIN_W - rx - m;

	cabecalho(cr, m, m, WIN_W - 2.0 * m, 64.0);

	y = m + 64.0 + 12.0;

	painel_vetor(cr, m, y, lw, 178.0, "ACELEROMETRO", "m/s2", imu.acc, esc_acc,
			GRAVIDADE, 0.5, "parado deve dar ~9.81 (gravidade)");

	painel_vetor(cr, m, y + 190.0, lw, 178.0, "GIROSCOPIO", "rad/s", imu.gyr, esc_gyr,
			0.0, GYR_PARADO, "parado deve dar ~0 (so' ruido de bias)");

	painel_vetor(cr, m, y + 380.0, lw, 178.0, "MAGNETOMETRO", "a.u.", imu.mag, esc_mag,
			CAMPO_TERRESTRE, 0.30, "normalizado: ~1.0 = campo terrestre");

	/* coluna da direita: orientacao e numeros */
	painel(cr, rx, y, rw, 276.0, "ORIENTACAO (filtro de Kalman)", "deg");
	horizonte(cr, rx + rw * 0.31, y + 130.0, 78.0, imu.roll, imu.pitch);
	mostrador_yaw(cr, rx + rw * 0.74, y + 130.0, 62.0, imu.yaw);

	fonte(cr, 10.0, 0, 0);
	set_cor(cr, COR_FRACA);
	texto(cr, rx + 16.0, y + 240.0, "roll / pitch");
	texto(cr, rx + rw * 0.74 - 46.0, y + 240.0, "yaw (nao e' azimute:");
	texto(cr, rx + rw * 0.74 - 46.0, y + 252.0, "sem declinacao magnetica)");

	painel_numeros(cr, rx, y + 288.0, rw, 270.0);

	return FALSE;
}

/* ------------------------------------------------------ estatisticas de taxa */

static void
contabiliza(int count, double timestamp)
{
	double dt;

	est.msgs++;

	if (est.tem_count_anterior)
	{
		int delta = (count - est.count_anterior) & 0xFFFF;

		/* delta 0 e' repeticao, delta enorme e' contador reiniciado: nao contam */
		if (delta > 0 && delta < 1000)
			est.amostras += delta;
	}
	est.count_anterior = count;
	est.tem_count_anterior = 1;

	if (est.t0 == 0.0)
		est.t0 = timestamp;

	dt = timestamp - est.t0;
	if (dt >= 1.0)
	{
		est.taxa_hz = est.msgs / dt;
		est.taxa_sensor_hz = est.amostras / dt;
		est.perda_pct = (est.amostras > 0)
				? 100.0 * (double) (est.amostras - est.msgs) / (double) est.amostras
				: 0.0;
		if (est.perda_pct < 0.0)
			est.perda_pct = 0.0;

		est.msgs = 0;
		est.amostras = 0;
		est.t0 = timestamp;
	}
}


static void
guarda_comum(carmen_xsens_axis acc, carmen_xsens_axis gyr, carmen_xsens_axis mag,
		double temp, unsigned short count, double timestamp, char *host, const char *origem)
{
	if (congelado)
		return;

	imu.acc.x = acc.x; imu.acc.y = acc.y; imu.acc.z = acc.z;
	imu.gyr.x = gyr.x; imu.gyr.y = gyr.y; imu.gyr.z = gyr.z;
	imu.mag.x = mag.x; imu.mag.y = mag.y; imu.mag.z = mag.z;

	imu.temp = temp;
	imu.count = (int) count;
	imu.timestamp = timestamp;
	imu.recebeu_algo = 1;

	strncpy(imu.origem, origem, sizeof(imu.origem) - 1);
	imu.origem[sizeof(imu.origem) - 1] = '\0';

	if (host != NULL)
	{
		strncpy(imu.host, host, sizeof(imu.host) - 1);
		imu.host[sizeof(imu.host) - 1] = '\0';
	}
}

/* ------------------------------------------------------------- handlers IPC */

static carmen_xsens_global_quat_message   msg_quat;
static carmen_xsens_global_euler_message  msg_euler;
static carmen_xsens_global_matrix_message msg_matrix;

static void
quat_handler(void)
{
	carmen_quaternion_t q;
	rotation_matrix *r;
	carmen_orientation_3D_t o;

	contabiliza((int) msg_quat.m_count, msg_quat.timestamp);

	if (congelado)
		return;

	guarda_comum(msg_quat.m_acc, msg_quat.m_gyr, msg_quat.m_mag, msg_quat.m_temp,
			msg_quat.m_count, msg_quat.timestamp, msg_quat.host, "xsens_global_quat");

	imu.quat[0] = msg_quat.quat_data.m_data[0];
	imu.quat[1] = msg_quat.quat_data.m_data[1];
	imu.quat[2] = msg_quat.quat_data.m_data[2];
	imu.quat[3] = msg_quat.quat_data.m_data[3];
	imu.tem_quat = 1;

	q.q0 = imu.quat[0];
	q.q1 = imu.quat[1];
	q.q2 = imu.quat[2];
	q.q3 = imu.quat[3];

	r = create_rotation_matrix_from_quaternions(q);
	o = get_angles_from_rotation_matrix(r);
	destroy_rotation_matrix(r);

	imu.roll  = o.roll;
	imu.pitch = o.pitch;
	imu.yaw   = o.yaw;
}


static void
euler_handler(void)
{
	contabiliza((int) msg_euler.m_count, msg_euler.timestamp);

	if (congelado)
		return;

	guarda_comum(msg_euler.m_acc, msg_euler.m_gyr, msg_euler.m_mag, msg_euler.m_temp,
			msg_euler.m_count, msg_euler.timestamp, msg_euler.host, "xsens_global_euler");

	/* o xsens publica os angulos de Euler em graus */
	imu.roll  = carmen_degrees_to_radians(msg_euler.euler_data.m_roll);
	imu.pitch = carmen_degrees_to_radians(msg_euler.euler_data.m_pitch);
	imu.yaw   = carmen_degrees_to_radians(msg_euler.euler_data.m_yaw);
	imu.tem_quat = 0;
}


static void
matrix_handler(void)
{
	carmen_orientation_3D_t o;

	contabiliza((int) msg_matrix.m_count, msg_matrix.timestamp);

	if (congelado)
		return;

	guarda_comum(msg_matrix.m_acc, msg_matrix.m_gyr, msg_matrix.m_mag, msg_matrix.m_temp,
			msg_matrix.m_count, msg_matrix.timestamp, msg_matrix.host, "xsens_global_matrix");

	o = xsens_get_pose_from_message(msg_matrix);

	imu.roll  = o.roll;
	imu.pitch = o.pitch;
	imu.yaw   = o.yaw;
	imu.tem_quat = 0;
}

/* -------------------------------------------------------- laco IPC e teclado */

/*
 * Integracao do IPC com o laco principal do GTK3.
 *
 * NAO fique chamando carmen_ipc_dispatch_nonblocking() num g_timeout: apesar do
 * comentario "wait 1 micro second" em global/ipc_wrapper.c, o IPC_listen() por
 * baixo recebe MILISSEGUNDOS, e cada chamada com a fila vazia custa ~1.1 ms
 * medido. Um punhado delas por tick satura o laco do GTK, o draw nunca roda, e a
 * janela abre mapeada (aparece na barra de tarefas) mas fica sem pintar nada.
 *
 * Em vez disso penduramos os descritores do IPC no laco do GLib: so' entramos no
 * IPC quando o socket tem dado de verdade, e ai' o dispatch retorna na hora.
 * E' o mesmo que o carmen_graphics_update_ipc_callbacks() faz no GTK2, que nao
 * da' para usar aqui por depender de GdkInputFunction.
 */
extern fd_set *x_ipcGetConnections(void);
extern int x_ipcGetMaxConnection(void);

static gboolean
ipc_tem_dado(GIOChannel *canal, GIOCondition condicao, gpointer dados)
{
	(void) canal;
	(void) condicao;
	(void) dados;

	carmen_ipc_dispatch_nonblocking();

	return TRUE;
}


/*
 * Registra os descritores que ainda nao foram vistos. Roda tambem de tempos em
 * tempos porque o IPC pode abrir conexao nova depois (reconexao com a central).
 */
static gboolean
registra_fds_do_ipc(gpointer dados)
{
	static char registrado[FD_SETSIZE];
	fd_set *fds;
	int max, fd;

	(void) dados;

	fds = x_ipcGetConnections();
	max = x_ipcGetMaxConnection();

	if (fds == NULL)
		return TRUE;

	for (fd = 0; fd <= max && fd < FD_SETSIZE; fd++)
	{
		GIOChannel *canal;

		if (!FD_ISSET(fd, fds) || registrado[fd])
			continue;

		canal = g_io_channel_unix_new(fd);
		g_io_add_watch(canal, G_IO_IN | G_IO_HUP | G_IO_ERR, ipc_tem_dado, NULL);
		g_io_channel_unref(canal);
		registrado[fd] = 1;
	}

	return TRUE;
}


static gboolean
redesenha(gpointer dados)
{
	(void) dados;

	if (area_de_desenho != NULL)
		gtk_widget_queue_draw(area_de_desenho);

	return TRUE;
}


static gboolean
tecla(GtkWidget *widget, GdkEventKey *evento, gpointer dados)
{
	(void) widget;
	(void) dados;

	switch (evento->keyval)
	{
		case GDK_KEY_q:
		case GDK_KEY_Q:
		case GDK_KEY_Escape:
			gtk_main_quit();
			break;

		case GDK_KEY_space:
			congelado = !congelado;
			break;

		case GDK_KEY_z:
		case GDK_KEY_Z:
			pico_acc = pico_gyr = pico_mag = 0.0;
			break;

		default:
			break;
	}

	return TRUE;
}


static void
encerra(int sinal)
{
	if (sinal == SIGINT)
	{
		carmen_ipc_disconnect();
		exit(0);
	}
}

/* ---------------------------------------------------------------------- main */

int
main(int argc, char **argv)
{
	GtkWidget *janela;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, encerra);

	memset(&imu, 0, sizeof(imu));
	memset(&est, 0, sizeof(est));
	strcpy(imu.host, "?");
	strcpy(imu.origem, "-");

	carmen_xsens_subscribe_xsens_global_quat_message(&msg_quat,
			(carmen_handler_t) quat_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_xsens_subscribe_xsens_global_euler_message(&msg_euler,
			(carmen_handler_t) euler_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_xsens_subscribe_xsens_global_matrix_message(&msg_matrix,
			(carmen_handler_t) matrix_handler, CARMEN_SUBSCRIBE_LATEST);

	gtk_init(&argc, &argv);

	janela = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW(janela), "imu_teste - conferencia da IMU");
	gtk_window_set_default_size(GTK_WINDOW(janela), WIN_W, WIN_H);

	area_de_desenho = gtk_drawing_area_new();
	gtk_widget_set_size_request(area_de_desenho, WIN_W / 2, WIN_H / 2);
	gtk_container_add(GTK_CONTAINER(janela), area_de_desenho);

	g_signal_connect(area_de_desenho, "draw", G_CALLBACK(desenha), NULL);
	g_signal_connect(janela, "key-press-event", G_CALLBACK(tecla), NULL);
	g_signal_connect(janela, "destroy", G_CALLBACK(gtk_main_quit), NULL);

	registra_fds_do_ipc(NULL);
	g_timeout_add(1000, registra_fds_do_ipc, NULL); /* pega conexao nova do IPC */
	g_timeout_add(33, redesenha, NULL);             /* ~30 quadros por segundo  */

	gtk_widget_show_all(janela);
	gtk_main();

	carmen_ipc_disconnect();

	return 0;
}
