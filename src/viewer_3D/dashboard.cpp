/*
 * Painel do carro para o viewer_3D -- desenho em cairo, colado como textura sobre a cena 3D.
 *
 * Ver dashboard.h. Este arquivo nao conhece o viewer_3D nem o carmen: recebe numeros e desenha.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>

#include <GL/glew.h>
#include <GL/gl.h>

#include <cairo/cairo.h>

#include "dashboard.h"

// ---------------------------------------------------------------------------------------------
// Medidas (coordenadas de projeto; o painel inteiro e' escalado por cairo_scale)
// ---------------------------------------------------------------------------------------------
#define PANEL_HEIGHT		168
#define MISSION_PANEL_WIDTH	308
#define CAMERA_WIDTH		320
#define MARGIN				14
#define PAD					16		// respiro interno de cada secccao
#define LABEL_BASELINE		22		// linha de base do rotulo da secccao

// largura de cada secccao. A ordem de encolhimento esta' em plan_sections().
#define W_SPEED				180
#define W_GEAR				132
#define W_STEER				156
#define W_CMD				150
#define W_DYN				140
#define W_HIST				212
#define W_STATE_MIN			224
#define W_STATE_MAX			470
#define W_ACTIONS			152
#define W_KEYS				150

// ---------------------------------------------------------------------------------------------
// Paleta -- grafite, um unico azul de acento e tres cores de estado dessaturadas. Nada de neon:
// o painel fica horas na tela ao lado da cena 3D e nao pode disputar atenccao com ela.
// ---------------------------------------------------------------------------------------------
#define C_BG			0.043, 0.051, 0.063, 0.93
#define C_RULE			1.000, 1.000, 1.000, 0.070
#define C_RULE_TOP		1.000, 1.000, 1.000, 0.150
#define C_TEXT			0.906, 0.925, 0.945, 1.000
#define C_LABEL			0.404, 0.455, 0.514, 1.000
#define C_DIM			0.545, 0.600, 0.655, 1.000
#define C_ACCENT		0.290, 0.620, 0.855, 1.000
#define C_OK			0.310, 0.706, 0.467, 1.000
#define C_WARN			0.851, 0.643, 0.255, 1.000
#define C_BAD			0.851, 0.325, 0.310, 1.000
#define C_TRACK			1.000, 1.000, 1.000, 0.065

// Tres familias: condensada para rotulos e numeros grandes, monoespaccada para valores que
// mudam a toda hora (senao a largura do digito faz o texto tremer), normal para o resto.
#define FONT_COND		"DejaVu Sans Condensed"
#define FONT_MONO		"DejaVu Sans Mono"
#define FONT_UI			"DejaVu Sans"


typedef struct
{
	cairo_surface_t *surface;
	cairo_t *cr;
	GLuint texture;
	int width;
	int height;
	int uploaded;
} layer_t;


static dashboard_data_t g_data;
static dashboard_data_t g_drawn;		// copia do que ja' esta' na textura, para detectar mudanca

static layer_t g_bar;

// Os dois paineis laterais -- missoes e lugares da camera -- sao a mesma coisa desenhada com
// titulos diferentes, entao compartilham a estrutura e a funccao de desenho.
#define PANEL_MAX_ITEMS		DASHBOARD_MAX_MISSIONS

typedef struct
{
	layer_t layer;
	const char *title;
	const char *hint;
	const char *footer;
	double footer_r, footer_g, footer_b;

	char items[PANEL_MAX_ITEMS][DASHBOARD_MISSION_NAME_SIZE];
	int num_items;
	int selected;			// item em destaque permanente (-1 = nenhum)

	int visible;
	int dirty;
	int hover;

	// area clicavel de cada linha, ja' em coordenadas de tela
	int row_x0, row_x1;
	int row_y0[PANEL_MAX_ITEMS];
	int row_y1[PANEL_MAX_ITEMS];
	int footer_y0, footer_y1;
} list_panel_t;

static list_panel_t g_missions_panel;
static list_panel_t g_camera_panel;

static GLuint g_camera_texture = 0;
static int g_camera_width = 0;
static int g_camera_height = 0;
static unsigned char *g_camera_rgb = NULL;
static int g_camera_rgb_size = 0;
static int g_camera_dirty = 0;

static int g_panel_visible = 1;
static int g_camera_visible = 1;			// a miniatura da camera, nao o painel de lugares
static int g_camera_following = 1;

// historico para os graficos
static float g_hist_v[DASHBOARD_HISTORY_SIZE];
static float g_hist_phi[DASHBOARD_HISTORY_SIZE];
static int g_hist_index = 0;
static int g_hist_count = 0;
static double g_hist_last_time = 0.0;

static double g_last_render_time = 0.0;

// aparencia
static double g_scale_setting = 0.0;		// 0 = automatica
static double g_scale = 1.0;				// a que esta' valendo agora
static double g_opacity = 1.0;
static int g_position = DASHBOARD_POSITION_BOTTOM;

// retangulo do painel na tela, atualizado a cada quadro (para o teste de clique)
static int g_bar_x = 0, g_bar_y = 0, g_bar_w = 0, g_bar_h = 0;

static int g_mode = DASHBOARD_MODE_NORMAL;

// botoes do card COMANDOS, em coordenadas de PROJETO (convertidas na hora do clique)
#define DASHBOARD_MAX_BUTTONS	8
typedef struct
{
	double x, y, w, h;
	int action;
} button_t;

static button_t g_buttons[DASHBOARD_MAX_BUTTONS];
static int g_num_buttons = 0;
static int g_button_hover = -1;


static double
now_seconds(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);

	return (tv.tv_sec + 1e-6 * tv.tv_usec);
}


// ---------------------------------------------------------------------------------------------
// Camada cairo -> textura GL
// ---------------------------------------------------------------------------------------------
static void
layer_ensure(layer_t *layer, int width, int height)
{
	if (layer->surface != NULL && layer->width == width && layer->height == height)
		return;

	if (layer->cr != NULL)
		cairo_destroy(layer->cr);
	if (layer->surface != NULL)
		cairo_surface_destroy(layer->surface);

	layer->surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, width, height);
	layer->cr = cairo_create(layer->surface);
	layer->width = width;
	layer->height = height;
	layer->uploaded = 0;

	if (layer->texture == 0)
		glGenTextures(1, &layer->texture);
}


static void
layer_clear(layer_t *layer)
{
	cairo_save(layer->cr);
	cairo_set_operator(layer->cr, CAIRO_OPERATOR_CLEAR);
	cairo_paint(layer->cr);
	cairo_restore(layer->cr);
}


static void
layer_upload(layer_t *layer)
{
	cairo_surface_flush(layer->surface);

	// O estado de empacotamento de pixel e' CLIENTE: o glPushAttrib(GL_ALL_ATTRIB_BITS) do
	// begin_2d() NAO o salva. Sem este par, o GL_UNPACK_ALIGNMENT/ROW_LENGTH daqui vazaria para
	// todo glTexImage2D do viewer_3D (mapa de imagem, download_map, texturas do carro) e a
	// imagem sairia enviesada -- as linhas lidas com o passo errado.
	glPushClientAttrib(GL_CLIENT_PIXEL_STORE_BIT);

	glBindTexture(GL_TEXTURE_2D, layer->texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, cairo_image_surface_get_stride(layer->surface) / 4);

	// cairo ARGB32 em little endian fica na memoria como B,G,R,A -- e' exatamente GL_BGRA
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, layer->width, layer->height, 0,
			GL_BGRA, GL_UNSIGNED_BYTE, cairo_image_surface_get_data(layer->surface));

	// ESTA linha e' o conserto do carro preto. layer_upload() e' chamada pela render_bar() e
	// pela render_list_panel(), que rodam FORA do begin_2d()/end_2d() -- entao a textura do
	// painel ficava amarrada em GL_TEXTURE_2D para sempre. O glmDraw() do modelo do carro, quando
	// o modelo nao tem coordenada de textura, nao liga nem desliga nada: herda o que estiver
	// amarrado. Herdando a superficie do cairo (ARGB pre-multiplicado, quase toda transparente),
	// o carro sai modulado por ela -- preto.
	glBindTexture(GL_TEXTURE_2D, 0);

	glPopClientAttrib();

	layer->uploaded = 1;
}


static void
blit_texture(GLuint texture, int x, int y, int width, int height)
{
	// a textura vem com alfa pre-multiplicado; modular os 4 canais pelo mesmo fator preserva isso
	glColor4f((GLfloat) g_opacity, (GLfloat) g_opacity, (GLfloat) g_opacity, (GLfloat) g_opacity);
	glBindTexture(GL_TEXTURE_2D, texture);
	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 0.0f); glVertex2i(x,         y);
		glTexCoord2f(1.0f, 0.0f); glVertex2i(x + width, y);
		glTexCoord2f(1.0f, 1.0f); glVertex2i(x + width, y + height);
		glTexCoord2f(0.0f, 1.0f); glVertex2i(x,         y + height);
	glEnd();
}


// ---------------------------------------------------------------------------------------------
// Primitivas de desenho
// ---------------------------------------------------------------------------------------------
static void
rounded_rect(cairo_t *cr, double x, double y, double w, double h, double r)
{
	if (r < 0.5)
	{
		cairo_rectangle(cr, x, y, w, h);
		return;
	}

	cairo_new_sub_path(cr);
	cairo_arc(cr, x + w - r, y + r,     r, -M_PI / 2.0, 0.0);
	cairo_arc(cr, x + w - r, y + h - r, r, 0.0,         M_PI / 2.0);
	cairo_arc(cr, x + r,     y + h - r, r, M_PI / 2.0,  M_PI);
	cairo_arc(cr, x + r,     y + r,     r, M_PI,        1.5 * M_PI);
	cairo_close_path(cr);
}


static void
set_font(cairo_t *cr, const char *family, double size, int bold)
{
	cairo_select_font_face(cr, family, CAIRO_FONT_SLANT_NORMAL,
			bold ? CAIRO_FONT_WEIGHT_BOLD : CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size(cr, size);
}


// Devolve a largura escrita, para quem precisa encostar outra coisa ao lado.
static double
text_at(cairo_t *cr, const char *family, double x, double y, double size, int bold,
		const char *str)
{
	cairo_text_extents_t extents;

	set_font(cr, family, size, bold);
	cairo_text_extents(cr, str, &extents);
	cairo_move_to(cr, x, y);
	cairo_show_text(cr, str);

	return (extents.x_advance);
}


static void
text_centered(cairo_t *cr, const char *family, double cx, double y, double size, int bold,
		const char *str)
{
	cairo_text_extents_t extents;

	set_font(cr, family, size, bold);
	cairo_text_extents(cr, str, &extents);
	cairo_move_to(cr, cx - extents.width / 2.0 - extents.x_bearing, y);
	cairo_show_text(cr, str);
}


static void
text_right(cairo_t *cr, const char *family, double x_right, double y, double size, int bold,
		const char *str)
{
	cairo_text_extents_t extents;

	set_font(cr, family, size, bold);
	cairo_text_extents(cr, str, &extents);
	cairo_move_to(cr, x_right - extents.width - extents.x_bearing, y);
	cairo_show_text(cr, str);
}


// Texto com entrelinha horizontal. O cairo nao tem tracking, entao vai caractere a caractere.
// E' o que da' aos rotulos o ar de placa de instrumento em vez de legenda de jogo.
static double
text_tracked(cairo_t *cr, double x, double y, double size, double spacing, const char *str)
{
	char one[8];
	double pen = x;

	set_font(cr, FONT_COND, size, 1);

	for (const char *p = str; *p != '\0'; )
	{
		int len = 1;
		cairo_text_extents_t extents;

		// UTF-8: leva a sequencia inteira, senao um 'Ç' vira dois quadradinhos
		if ((*p & 0x80) != 0)
		{
			if ((*p & 0xE0) == 0xC0)
				len = 2;
			else if ((*p & 0xF0) == 0xE0)
				len = 3;
			else if ((*p & 0xF8) == 0xF0)
				len = 4;
		}

		memcpy(one, p, len);
		one[len] = '\0';

		cairo_text_extents(cr, one, &extents);
		cairo_move_to(cr, pen, y);
		cairo_show_text(cr, one);

		pen += extents.x_advance + spacing;
		p += len;
	}

	return (pen - x);
}


static void
hairline(cairo_t *cr, double x, double y, double w, double alpha)
{
	cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, alpha);
	cairo_rectangle(cr, x, y, w, 1.0);
	cairo_fill(cr);
}


// Regua vertical que separa duas secccoes
static void
vrule(cairo_t *cr, double x, double y0, double y1)
{
	cairo_set_source_rgba(cr, C_RULE);
	cairo_rectangle(cr, x, y0, 1.0, y1 - y0);
	cairo_fill(cr);
}


// Rotulo da secccao, em maiusculas espacadas
static void
section_label(cairo_t *cr, double x, const char *label)
{
	cairo_set_source_rgba(cr, C_LABEL);
	text_tracked(cr, x, LABEL_BASELINE, 9.5, 1.3, label);
}


// Barra horizontal de 0 a 1, cantos retos (1 px de raio). Sem brilho, sem gradiente.
static void
bar_h(cairo_t *cr, double x, double y, double w, double h, double fraction,
		double r, double g, double b)
{
	if (fraction < 0.0)
		fraction = 0.0;
	if (fraction > 1.0)
		fraction = 1.0;

	cairo_set_source_rgba(cr, C_TRACK);
	rounded_rect(cr, x, y, w, h, 1.0);
	cairo_fill(cr);

	if (fraction > 0.002)
	{
		cairo_set_source_rgba(cr, r, g, b, 1.0);
		rounded_rect(cr, x, y, w * fraction, h, 1.0);
		cairo_fill(cr);
	}
}


// Barra com o zero no meio: cresce para a esquerda ou para a direita. 'ghost' e' um segundo
// valor (o comando) marcado so' com um tracinho.
static void
bar_bipolar(cairo_t *cr, double x, double y, double w, double h, double value, double full_scale,
		double r, double g, double b, int has_ghost, double ghost)
{
	double center = x + w / 2.0;
	double fraction = (full_scale > 1e-6) ? (value / full_scale) : 0.0;

	if (fraction < -1.0)
		fraction = -1.0;
	if (fraction > 1.0)
		fraction = 1.0;

	cairo_set_source_rgba(cr, C_TRACK);
	rounded_rect(cr, x, y, w, h, 1.0);
	cairo_fill(cr);

	if (fabs(fraction) > 0.004)
	{
		double length = (w / 2.0) * fabs(fraction);

		cairo_set_source_rgba(cr, r, g, b, 1.0);
		if (fraction > 0.0)
			cairo_rectangle(cr, center, y, length, h);
		else
			cairo_rectangle(cr, center - length, y, length, h);
		cairo_fill(cr);
	}

	// marca do zero
	cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.34);
	cairo_rectangle(cr, center - 0.5, y - 2.0, 1.0, h + 4.0);
	cairo_fill(cr);

	if (has_ghost)
	{
		double gf = (full_scale > 1e-6) ? (ghost / full_scale) : 0.0;

		if (gf < -1.0)
			gf = -1.0;
		if (gf > 1.0)
			gf = 1.0;

		cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.55);
		cairo_rectangle(cr, center + (w / 2.0) * gf - 0.5, y - 3.0, 1.0, h + 6.0);
		cairo_fill(cr);
	}
}


// Regua P R N D L, como num painel de carro: so' a marcha corrente fica acesa.
static void
gear_strip(cairo_t *cr, double x, double y, double w, double h, int gear)
{
	static const char *names[5] = { "P", "R", "N", "D", "L" };
	static const int codes[5] =
	{
		DASHBOARD_GEAR_P, DASHBOARD_GEAR_R, DASHBOARD_GEAR_N,
		DASHBOARD_GEAR_D, DASHBOARD_GEAR_L
	};
	double cell = w / 5.0;

	for (int i = 0; i < 5; i++)
	{
		double cx = x + i * cell;
		int on = (gear == codes[i]);
		double r = 0.290, g = 0.620, b = 0.855;

		if (codes[i] == DASHBOARD_GEAR_R)
		{
			r = 0.851; g = 0.643; b = 0.255;		// a re' merece cor propria
		}

		if (on)
		{
			cairo_set_source_rgba(cr, r, g, b, 0.20);
			rounded_rect(cr, cx + 1.0, y, cell - 2.0, h, 2.0);
			cairo_fill(cr);

			cairo_set_source_rgba(cr, r, g, b, 0.85);
			cairo_set_line_width(cr, 1.0);
			rounded_rect(cr, cx + 1.5, y + 0.5, cell - 3.0, h - 1.0, 2.0);
			cairo_stroke(cr);

			cairo_set_source_rgba(cr, C_TEXT);
		}
		else
		{
			cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.045);
			rounded_rect(cr, cx + 1.0, y, cell - 2.0, h, 2.0);
			cairo_fill(cr);

			cairo_set_source_rgba(cr, 0.330, 0.375, 0.425, 1.0);
		}

		text_centered(cr, FONT_COND, cx + cell / 2.0, y + h / 2.0 + 6.5, 17.0, 1, names[i]);
	}
}


static void
chip(cairo_t *cr, double x, double y, double w, double h, const char *label, int on,
		double r, double g, double b)
{
	if (on)
	{
		cairo_set_source_rgba(cr, r, g, b, 0.18);
		rounded_rect(cr, x, y, w, h, 2.0);
		cairo_fill(cr);

		cairo_set_source_rgba(cr, r, g, b, 0.80);
		cairo_set_line_width(cr, 1.0);
		rounded_rect(cr, x + 0.5, y + 0.5, w - 1.0, h - 1.0, 2.0);
		cairo_stroke(cr);

		cairo_set_source_rgba(cr, r, g, b, 1.0);
	}
	else
	{
		cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.040);
		rounded_rect(cr, x, y, w, h, 2.0);
		cairo_fill(cr);

		cairo_set_source_rgba(cr, 0.345, 0.385, 0.430, 1.0);
	}

	cairo_save(cr);
	cairo_rectangle(cr, x, y, w, h);
	cairo_clip(cr);
	text_tracked(cr, x + 7.0, y + h / 2.0 + 3.2, 8.5, 0.7, label);
	cairo_restore(cr);
}


static void
add_button(cairo_t *cr, double x, double y, double w, double h, const char *label, int action,
		int active, double r, double g, double b)
{
	int hover = 0;

	if (g_num_buttons < DASHBOARD_MAX_BUTTONS)
	{
		g_buttons[g_num_buttons].x = x;
		g_buttons[g_num_buttons].y = y;
		g_buttons[g_num_buttons].w = w;
		g_buttons[g_num_buttons].h = h;
		g_buttons[g_num_buttons].action = action;
		if (g_num_buttons == g_button_hover)
			hover = 1;
		g_num_buttons++;
	}

	cairo_set_source_rgba(cr, r, g, b, active ? 0.22 : (hover ? 0.12 : 0.045));
	rounded_rect(cr, x, y, w, h, 2.0);
	cairo_fill(cr);

	cairo_set_source_rgba(cr, r, g, b, active ? 0.95 : (hover ? 0.70 : 0.38));
	cairo_set_line_width(cr, 1.0);
	rounded_rect(cr, x + 0.5, y + 0.5, w - 1.0, h - 1.0, 2.0);
	cairo_stroke(cr);

	cairo_set_source_rgba(cr, r, g, b, active ? 1.0 : 0.88);

	// centraliza o rotulo espaccado: mede primeiro, escreve depois
	{
		double width;

		cairo_save(cr);
		cairo_push_group(cr);
		width = text_tracked(cr, 0.0, 0.0, 9.5, 1.1, label);
		cairo_pattern_destroy(cairo_pop_group(cr));
		cairo_restore(cr);

		cairo_set_source_rgba(cr, r, g, b, active ? 1.0 : 0.88);
		text_tracked(cr, x + (w - width) / 2.0, y + h / 2.0 + 3.6, 9.5, 1.1, label);
	}
}


static void
sparkline(cairo_t *cr, double x, double y, double w, double h, const float *history,
		int count, int index, double vmin, double vmax, double r, double g, double b,
		const char *label, const char *value)
{
	int n = (count < DASHBOARD_HISTORY_SIZE) ? count : DASHBOARD_HISTORY_SIZE;

	cairo_set_source_rgba(cr, C_LABEL);
	text_tracked(cr, x, y - 6.0, 8.5, 0.9, label);

	cairo_set_source_rgba(cr, C_DIM);
	text_right(cr, FONT_MONO, x + w, y - 6.0, 9.5, 0, value);

	cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.028);
	cairo_rectangle(cr, x, y, w, h);
	cairo_fill(cr);

	// linha do zero, quando o intervalo cruza o zero
	if (vmin < 0.0 && vmax > 0.0)
		hairline(cr, x, y + h * (vmax / (vmax - vmin)), w, 0.10);

	if (n >= 2)
	{
		cairo_set_line_width(cr, 1.4);
		cairo_set_source_rgba(cr, r, g, b, 0.90);
		cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);

		for (int i = 0; i < n; i++)
		{
			int slot = (index - n + i + 2 * DASHBOARD_HISTORY_SIZE) % DASHBOARD_HISTORY_SIZE;
			double value_i = history[slot];
			double fraction = (value_i - vmin) / (vmax - vmin);
			double px = x + w * i / (double) (n - 1);
			double py = y + h * (1.0 - fraction);

			if (py < y + 1.0)
				py = y + 1.0;
			if (py > y + h - 1.0)
				py = y + h - 1.0;

			if (i == 0)
				cairo_move_to(cr, px, py);
			else
				cairo_line_to(cr, px, py);
		}
		cairo_stroke(cr);
	}
}


// Leque de caminhos do frenet_path_planner: um tracinho por caminho, o escolhido inteiro.
static void
frenet_strip(cairo_t *cr, double x, double y, double w, double h, int num_paths, int selected)
{
	char buffer[64];

	if (num_paths <= 0)
	{
		cairo_set_source_rgba(cr, 0.345, 0.385, 0.430, 1.0);
		text_at(cr, FONT_MONO, x, y + h, 9.5, 0, "sem caminhos");
		return;
	}

	double slot = w / num_paths;
	double bar_w = (slot > 4.0) ? (slot - 2.0) : slot;

	for (int i = 0; i < num_paths; i++)
	{
		double bx = x + i * slot;
		int is_selected = (i == selected);
		double bh = is_selected ? h : (h * 0.45);

		if (is_selected)
			cairo_set_source_rgba(cr, C_ACCENT);
		else
			cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.16);

		cairo_rectangle(cr, bx, y + h - bh, bar_w, bh);
		cairo_fill(cr);
	}

	cairo_set_source_rgba(cr, C_DIM);
	if (selected >= 0)
		sprintf(buffer, "%d/%d", selected + 1, num_paths);
	else
		sprintf(buffer, "%d", num_paths);
	text_at(cr, FONT_MONO, x + w + 8.0, y + h, 9.5, 0, buffer);
}


// Linha chave/valor: chave a esquerda em cinza, valor a direita em monoespaccado.
static void
kv_row(cairo_t *cr, double x, double y, double width, const char *key, const char *value,
		double r, double g, double b)
{
	char shortened[256];
	cairo_text_extents_t extents;
	double room = width - 92.0;

	cairo_set_source_rgba(cr, C_LABEL);
	text_at(cr, FONT_UI, x, y, 9.5, 0, key);

	// valor comprido perde o fim com reticencias; cortar no clip escondia justamente o comeco,
	// que e' a parte que identifica a missao
	strncpy(shortened, value, sizeof(shortened) - 4);
	shortened[sizeof(shortened) - 4] = '\0';

	set_font(cr, FONT_MONO, 10.0, 0);
	cairo_text_extents(cr, shortened, &extents);

	while (extents.width > room && strlen(shortened) > 2)
	{
		int len = (int) strlen(shortened);

		// nao corta no meio de um caractere UTF-8
		do
			len--;
		while (len > 0 && (shortened[len] & 0xC0) == 0x80);

		strcpy(shortened + len, "…");
		cairo_text_extents(cr, shortened, &extents);

		if (extents.width <= room)
			break;

		shortened[len] = '\0';
	}

	cairo_set_source_rgba(cr, r, g, b, 1.0);
	text_right(cr, FONT_MONO, x + width, y, 10.0, 0, shortened);
}


static const char *
gear_name(int gear)
{
	switch (gear)
	{
	case DASHBOARD_GEAR_P:	return ("P");
	case DASHBOARD_GEAR_R:	return ("R");
	case DASHBOARD_GEAR_N:	return ("N");
	case DASHBOARD_GEAR_D:	return ("D");
	case DASHBOARD_GEAR_L:	return ("L");
	}

	return ("—");
}


static void
age_color(double timestamp, double *r, double *g, double *b)
{
	double age = (timestamp > 0.0) ? (now_seconds() - timestamp) : 1e9;

	if (age < 1.0)
	{
		*r = 0.310; *g = 0.706; *b = 0.467;
	}
	else if (age < 5.0)
	{
		*r = 0.851; *g = 0.643; *b = 0.255;
	}
	else
	{
		*r = 0.851; *g = 0.325; *b = 0.310;
	}
}


// ---------------------------------------------------------------------------------------------
// Barra inferior
// ---------------------------------------------------------------------------------------------

// Decide quais secccoes cabem. A ordem abaixo e' a ordem de importancia: VELOCIDADE,
// TRANSMISSAO, COMANDO e ACCOES estao sempre na tela; o resto entra conforme sobra largura.
static void
plan_sections(double width, int *show_steer, int *show_state, int *show_dyn, int *show_hist,
		int *show_keys, double *state_w, double *right_gap)
{
	double available = width - 2.0 * MARGIN - (W_SPEED + W_GEAR + W_CMD + W_ACTIONS);

	*show_steer = *show_state = *show_dyn = *show_hist = *show_keys = 0;
	*state_w = 0.0;

	if (available >= W_STEER)
	{
		*show_steer = 1;
		available -= W_STEER;
	}
	if (available >= W_STATE_MIN)
	{
		*show_state = 1;
		*state_w = W_STATE_MIN;
		available -= W_STATE_MIN;
	}
	if (available >= W_DYN)
	{
		*show_dyn = 1;
		available -= W_DYN;
	}
	if (available >= W_HIST)
	{
		*show_hist = 1;
		available -= W_HIST;
	}
	if (available >= W_KEYS)
	{
		*show_keys = 1;
		available -= W_KEYS;
	}

	// o ESTADO cresce ate um limite -- passar disso so' afasta o valor da sua chave. O que
	// ainda sobrar vira folga, empurrando ACCOES e OPCCOES para a borda direita.
	if (*show_state && available > 0.0)
	{
		double extra = W_STATE_MAX - W_STATE_MIN;

		if (extra > available)
			extra = available;

		*state_w += extra;
		available -= extra;
	}

	*right_gap = (available > 0.0) ? available : 0.0;
}


static void
render_bar(int window_width, int rounded)
{
	cairo_t *cr = g_bar.cr;
	dashboard_data_t *d = &g_data;
	char buffer[256];
	double x;
	double r, g, b;
	int show_steer, show_state, show_dyn, show_hist, show_keys;
	double state_w;
	double right_gap;
	double rule_top = 14.0;
	double rule_bottom = PANEL_HEIGHT - 14.0;
	int first_section = 1;

	layer_clear(&g_bar);
	g_num_buttons = 0;

	plan_sections(window_width, &show_steer, &show_state, &show_dyn, &show_hist, &show_keys,
			&state_w, &right_gap);

	// ---- fundo -----------------------------------------------------------------------
	cairo_set_source_rgba(cr, C_BG);
	rounded_rect(cr, 0.0, 0.0, window_width, PANEL_HEIGHT, rounded ? 3.0 : 0.0);
	cairo_fill(cr);

	if (rounded)
	{
		cairo_set_source_rgba(cr, C_RULE);
		cairo_set_line_width(cr, 1.0);
		rounded_rect(cr, 0.5, 0.5, window_width - 1.0, PANEL_HEIGHT - 1.0, 3.0);
		cairo_stroke(cr);
	}

	// filete do topo: cinza quando esta' so' olhando, colorido quando espera um clique no mapa
	if (g_mode == DASHBOARD_MODE_PICK_POSE)
		cairo_set_source_rgba(cr, C_WARN);
	else if (g_mode == DASHBOARD_MODE_PICK_GOAL)
		cairo_set_source_rgba(cr, C_ACCENT);
	else
		cairo_set_source_rgba(cr, C_RULE_TOP);
	cairo_rectangle(cr, 0.0, 0.0, window_width, (g_mode == DASHBOARD_MODE_NORMAL) ? 1.0 : 2.0);
	cairo_fill(cr);

	x = MARGIN;

	// ---- velocidade ------------------------------------------------------------------
	{
		double width = W_SPEED;
		double speed_kmh = fabs(d->v) * 3.6;
		double max_kmh = (d->max_speed > 0.5) ? (d->max_speed * 3.6) : 60.0;
		double advance;

		section_label(cr, x + PAD, "VELOCIDADE");

		sprintf(buffer, "%.1f", speed_kmh);
		cairo_set_source_rgba(cr, C_TEXT);
		advance = text_at(cr, FONT_COND, x + PAD, 92.0, 46.0, 0, buffer);

		cairo_set_source_rgba(cr, C_DIM);
		text_at(cr, FONT_UI, x + PAD + advance + 7.0, 92.0, 11.5, 0, "km/h");

		if (d->v < -0.05)
		{
			cairo_set_source_rgba(cr, C_WARN);
			text_tracked(cr, x + PAD + advance + 7.0, 74.0, 9.0, 1.0, "RÉ");
		}

		bar_h(cr, x + PAD, 106.0, width - 2.0 * PAD, 5.0, speed_kmh / max_kmh,
				(d->v < -0.05) ? 0.851 : 0.290, (d->v < -0.05) ? 0.643 : 0.620,
				(d->v < -0.05) ? 0.255 : 0.855);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 130.0, 9.5, 0, "comando");
		cairo_set_source_rgba(cr, C_DIM);
		sprintf(buffer, "%+.1f", d->v_command * 3.6);
		text_right(cr, FONT_MONO, x + width - PAD, 130.0, 10.0, 0, buffer);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 148.0, 9.5, 0, "máxima");
		cairo_set_source_rgba(cr, C_DIM);
		sprintf(buffer, "%.0f", max_kmh);
		text_right(cr, FONT_MONO, x + width - PAD, 148.0, 10.0, 0, buffer);

		x += width;
		first_section = 0;
	}

	// ---- transmissao -----------------------------------------------------------------
	{
		double width = W_GEAR;

		if (!first_section)
			vrule(cr, x, rule_top, rule_bottom);

		section_label(cr, x + PAD, "TRANSMISSÃO");

		gear_strip(cr, x + PAD, 46.0, width - 2.0 * PAD, 34.0, d->gear);

		// de onde veio a marcha. Sem CAN (o simulador manda o status zerado) ela e' deduzida
		// do movimento -- e isso tem que estar escrito, senao vira invenccao.
		cairo_set_source_rgba(cr, C_LABEL);
		if (!d->gear_estimated)
			text_at(cr, FONT_UI, x + PAD, 102.0, 9.5, 0, "do CAN");
		else if (d->gear_raw != 0)
		{
			cairo_set_source_rgba(cr, C_WARN);
			sprintf(buffer, "código %d ?", d->gear_raw);
			text_at(cr, FONT_UI, x + PAD, 102.0, 9.5, 0, buffer);
		}
		else
			text_at(cr, FONT_UI, x + PAD, 102.0, 9.5, 0, "deduzida");

		hairline(cr, x + PAD, 112.0, width - 2.0 * PAD, 0.06);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 132.0, 9.5, 0, "rpm");
		cairo_set_source_rgba(cr, C_DIM);
		if (d->rpm > 1.0)
			sprintf(buffer, "%.0f", d->rpm);
		else
			sprintf(buffer, "—");
		text_right(cr, FONT_MONO, x + width - PAD, 132.0, 10.0, 0, buffer);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 150.0, 9.5, 0, "marcha");
		cairo_set_source_rgba(cr, C_TEXT);
		text_right(cr, FONT_MONO, x + width - PAD, 150.0, 10.0, 1, gear_name(d->gear));

		x += width;
	}

	// ---- direccao --------------------------------------------------------------------
	if (show_steer)
	{
		double width = W_STEER;
		double full = (d->max_phi > 0.05) ? d->max_phi : 0.5;
		double degrees = d->phi * 180.0 / M_PI;
		int left = (d->turn_signal == 1 || d->turn_signal == 3);
		int right = (d->turn_signal == 2 || d->turn_signal == 3);
		int blink = ((int) (now_seconds() * 2.0)) % 2;

		vrule(cr, x, rule_top, rule_bottom);
		section_label(cr, x + PAD, "DIREÇÃO");

		// O numero grande e' o angulo do VOLANTE, nao o da roda: e' o que o motorista ve e o
		// que o painel do astro mostra (graus(phi) x relaccao de direccao). O angulo da roda
		// vai embaixo, porque e' ele que os planejadores usam.
		double wheel_degrees = d->steering_wheel;

		if (fabs(wheel_degrees) < 0.5)
			wheel_degrees = 0.0;
		sprintf(buffer, "%+.0f°", wheel_degrees);
		cairo_set_source_rgba(cr, C_TEXT);
		text_at(cr, FONT_COND, x + PAD, 76.0, 30.0, 0, buffer);

		cairo_set_source_rgba(cr, C_LABEL);
		text_right(cr, FONT_UI, x + width - PAD, 76.0, 9.5, 0, "volante");

		bar_bipolar(cr, x + PAD, 90.0, width - 2.0 * PAD, 7.0, d->phi, full,
				0.290, 0.620, 0.855, 1, d->phi_command);

		// as pontas da regua sao o batente do volante, na mesma unidade do numero grande
		cairo_set_source_rgba(cr, C_LABEL);
		sprintf(buffer, "-%.0f°", full * 180.0 / M_PI * d->steering_ratio);
		text_at(cr, FONT_MONO, x + PAD, 112.0, 8.5, 0, buffer);
		sprintf(buffer, "+%.0f°", full * 180.0 / M_PI * d->steering_ratio);
		text_right(cr, FONT_MONO, x + width - PAD, 112.0, 8.5, 0, buffer);

		hairline(cr, x + PAD, 122.0, width - 2.0 * PAD, 0.06);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 138.0, 9.5, 0, "roda");
		cairo_set_source_rgba(cr, C_DIM);
		if (fabs(degrees) < 0.05)
			degrees = 0.0;
		sprintf(buffer, "%+.1f°", degrees);
		text_right(cr, FONT_MONO, x + width - PAD, 138.0, 10.0, 0, buffer);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 156.0, 9.5, 0, "CAN");
		cairo_set_source_rgba(cr, C_DIM);
		if (fabs(d->steering_wheel_pct) > 1e-6)
			sprintf(buffer, "%+.0f %%", d->steering_wheel_pct);
		else
			sprintf(buffer, "—");
		text_right(cr, FONT_MONO, x + width - PAD, 156.0, 10.0, 0, buffer);

		// setas de seta: dois triangulos discretos no alto, ao lado do rotulo da secccao.
		// So' piscam quando ha' sinal de verdade; no resto do tempo sao dois vultos.
		{
			double ax = x + width - PAD;

			cairo_set_source_rgba(cr, 0.310, 0.706, 0.467, (right && blink) ? 1.0 : 0.11);
			cairo_move_to(cr, ax, 19.0);
			cairo_line_to(cr, ax - 9.0, 13.0);
			cairo_line_to(cr, ax - 9.0, 25.0);
			cairo_close_path(cr);
			cairo_fill(cr);

			cairo_set_source_rgba(cr, 0.310, 0.706, 0.467, (left && blink) ? 1.0 : 0.11);
			cairo_move_to(cr, ax - 15.0, 19.0);
			cairo_line_to(cr, ax - 24.0, 13.0);
			cairo_line_to(cr, ax - 24.0, 25.0);
			cairo_close_path(cr);
			cairo_fill(cr);
		}

		x += width;
	}

	// ---- comando ---------------------------------------------------------------------
	{
		double width = W_CMD;
		double inner = width - 2.0 * PAD;

		vrule(cr, x, rule_top, rule_bottom);
		section_label(cr, x + PAD, "COMANDO");

		cairo_set_source_rgba(cr, C_LABEL);
		text_tracked(cr, x + PAD, 56.0, 8.5, 0.9, "ACEL");
		cairo_set_source_rgba(cr, C_DIM);
		sprintf(buffer, "%.0f %%", d->throttle);
		text_right(cr, FONT_MONO, x + width - PAD, 56.0, 10.0, 0, buffer);
		bar_h(cr, x + PAD, 62.0, inner, 7.0, d->throttle / 100.0, 0.310, 0.706, 0.467);

		cairo_set_source_rgba(cr, C_LABEL);
		text_tracked(cr, x + PAD, 94.0, 8.5, 0.9, "FREIO");
		cairo_set_source_rgba(cr, C_DIM);
		sprintf(buffer, "%.0f %%", d->brake);
		text_right(cr, FONT_MONO, x + width - PAD, 94.0, 10.0, 0, buffer);
		bar_h(cr, x + PAD, 100.0, inner, 7.0, d->brake / 100.0, 0.851, 0.325, 0.310);

		hairline(cr, x + PAD, 118.0, inner, 0.06);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 136.0, 9.5, 0, "v pedida");
		cairo_set_source_rgba(cr, C_DIM);
		sprintf(buffer, "%+.2f", d->v_command);
		text_right(cr, FONT_MONO, x + width - PAD, 136.0, 10.0, 0, buffer);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 154.0, 9.5, 0, "φ pedido");
		cairo_set_source_rgba(cr, C_DIM);
		sprintf(buffer, "%+.3f", d->phi_command);
		text_right(cr, FONT_MONO, x + width - PAD, 154.0, 10.0, 0, buffer);

		x += width;
	}

	// ---- dinamica --------------------------------------------------------------------
	if (show_dyn)
	{
		double width = W_DYN;
		double inner = width - 2.0 * PAD;

		vrule(cr, x, rule_top, rule_bottom);
		section_label(cr, x + PAD, "ACELERAÇÃO");

		cairo_set_source_rgba(cr, C_LABEL);
		text_tracked(cr, x + PAD, 56.0, 8.5, 0.9, "LONGIT.");
		cairo_set_source_rgba(cr, C_DIM);
		sprintf(buffer, "%+.2f", d->a_long);
		text_right(cr, FONT_MONO, x + width - PAD, 56.0, 10.0, 0, buffer);
		bar_bipolar(cr, x + PAD, 62.0, inner, 7.0, d->a_long, 4.0, 0.290, 0.620, 0.855, 0, 0.0);

		cairo_set_source_rgba(cr, C_LABEL);
		text_tracked(cr, x + PAD, 100.0, 8.5, 0.9, "LATERAL");
		cairo_set_source_rgba(cr, C_DIM);
		sprintf(buffer, "%+.2f", d->a_lat);
		text_right(cr, FONT_MONO, x + width - PAD, 100.0, 10.0, 0, buffer);
		bar_bipolar(cr, x + PAD, 106.0, inner, 7.0, d->a_lat, 4.0, 0.851, 0.643, 0.255, 0, 0.0);

		cairo_set_source_rgba(cr, C_LABEL);
		text_at(cr, FONT_UI, x + PAD, 144.0, 9.0, 0, "escala ±4 m/s²");

		x += width;
	}

	// ---- historico -------------------------------------------------------------------
	if (show_hist)
	{
		double width = W_HIST;

		vrule(cr, x, rule_top, rule_bottom);
		section_label(cr, x + PAD, "HISTÓRICO");

		sprintf(buffer, "%+.2f", d->v);
		sparkline(cr, x + PAD, 50.0, width - 2.0 * PAD, 42.0, g_hist_v, g_hist_count,
				g_hist_index, -5.0, 15.0, 0.290, 0.620, 0.855, "VELOCIDADE  m/s", buffer);

		sprintf(buffer, "%+.3f", d->phi);
		sparkline(cr, x + PAD, 112.0, width - 2.0 * PAD, 42.0, g_hist_phi, g_hist_count,
				g_hist_index, -0.5, 0.5, 0.851, 0.643, 0.255, "ESTERÇAMENTO  rad", buffer);

		x += width;
	}

	// ---- estado ----------------------------------------------------------------------
	if (show_state)
	{
		double width = state_w;
		double inner = width - 2.0 * PAD;
		double y;

		// em janela muito larga nao adianta jogar o valor la' na ponta: a linha fica ilegivel
		if (inner > 400.0)
			inner = 400.0;

		vrule(cr, x, rule_top, rule_bottom);
		section_label(cr, x + PAD, "ESTADO");

		chip(cr, x + PAD, 34.0, 82.0, 17.0, d->autonomous ? "AUTÔNOMO" : "MANUAL",
				d->autonomous, 0.310, 0.706, 0.467);
		chip(cr, x + PAD + 88.0, 34.0, 76.0, 17.0, "FREIO EST.", d->parking_brake,
				0.851, 0.643, 0.255);
		chip(cr, x + PAD + 170.0, 34.0, 60.0, 17.0, "MOTOR", d->engine, 0.290, 0.620, 0.855);

		y = 68.0;

		age_color(d->t_route, &r, &g, &b);
		kv_row(cr, x + PAD, y, inner, "route planner", d->route_planner_state, r, g, b);
		y += 17.0;

		kv_row(cr, x + PAD, y, inner, "behavior selector", d->behavior_state, 0.906, 0.925, 0.945);
		y += 17.0;

		if (d->goal_distance >= 0.0)
			sprintf(buffer, "%s · %.1f m", (d->mission_name[0] != '\0') ? d->mission_name : "—",
					d->goal_distance);
		else
			sprintf(buffer, "%s", (d->mission_name[0] != '\0') ? d->mission_name : "—");
		kv_row(cr, x + PAD, y, inner, "missão · goal", buffer, 0.906, 0.925, 0.945);
		y += 17.0;

		kv_row(cr, x + PAD, y, inner, "estado da missão",
				(d->mission_state[0] != '\0') ? d->mission_state : "—", 0.290, 0.620, 0.855);
		y += 17.0;

		age_color(d->t_globalpos, &r, &g, &b);
		sprintf(buffer, "%.2f %.2f %+.3f", d->x, d->y, d->theta);
		kv_row(cr, x + PAD, y, inner, "pose", buffer, r, g, b);

		hairline(cr, x + PAD, 142.0, inner, 0.06);

		cairo_set_source_rgba(cr, C_LABEL);
		text_tracked(cr, x + PAD, 159.0, 8.5, 0.9, "FRENET");
		frenet_strip(cr, x + PAD + 56.0, 150.0, inner - 56.0 - 48.0, 10.0,
				d->frenet_num_paths, d->frenet_selected);

		x += width;
	}

	x += right_gap;

	// ---- accoes ----------------------------------------------------------------------
	{
		double width = W_ACTIONS;
		double bx = x + PAD;
		double bw = width - 2.0 * PAD;

		vrule(cr, x, rule_top, rule_bottom);
		section_label(cr, x + PAD, "AÇÕES");

		add_button(cr, bx, 32.0, bw, 24.0, "POR O ROBÔ", DASHBOARD_ACTION_SET_POSE,
				g_mode == DASHBOARD_MODE_PICK_POSE, 0.851, 0.643, 0.255);

		add_button(cr, bx, 60.0, bw, 24.0, "DESTINO", DASHBOARD_ACTION_SET_GOAL,
				g_mode == DASHBOARD_MODE_PICK_GOAL, 0.290, 0.620, 0.855);

		add_button(cr, bx, 88.0, bw / 2.0 - 3.0, 24.0, "IR", DASHBOARD_ACTION_GO,
				d->autonomous, 0.310, 0.706, 0.467);
		add_button(cr, bx + bw / 2.0 + 3.0, 88.0, bw / 2.0 - 3.0, 24.0, "PARAR",
				DASHBOARD_ACTION_STOP, 0, 0.851, 0.325, 0.310);

		add_button(cr, bx, 116.0, bw / 2.0 - 3.0, 24.0, "MISSÕES", DASHBOARD_ACTION_MISSIONS,
				g_missions_panel.visible, 0.545, 0.500, 0.800);
		add_button(cr, bx + bw / 2.0 + 3.0, 116.0, bw / 2.0 - 3.0, 24.0, "CÂMERA",
				DASHBOARD_ACTION_CAMERA_PANEL, g_camera_panel.visible, 0.290, 0.620, 0.855);

		if (g_mode == DASHBOARD_MODE_PICK_POSE)
			cairo_set_source_rgba(cr, C_WARN);
		else if (g_mode == DASHBOARD_MODE_PICK_GOAL)
			cairo_set_source_rgba(cr, C_ACCENT);
		else
			cairo_set_source_rgba(cr, C_LABEL);

		if (g_mode == DASHBOARD_MODE_NORMAL)
			text_centered(cr, FONT_UI, x + width / 2.0, 154.0, 9.0, 0, "clique para usar");
		else
			text_centered(cr, FONT_UI, x + width / 2.0, 154.0, 9.0, 0,
					"clique no mapa e arraste");

		x += width;
	}

	// ---- opccoes ---------------------------------------------------------------------
	if (show_keys)
	{
		double width = W_KEYS;
		double y = 44.0;
		struct { const char *key; const char *what; const char *value; } rows[7];
		char scale_text[32];
		char opacity_text[32];

		sprintf(scale_text, "%.0f%%", 100.0 * g_scale);
		sprintf(opacity_text, "%.0f%%", 100.0 * g_opacity);

		rows[0].key = "F";   rows[0].what = "painel";     rows[0].value = NULL;
		rows[1].key = "G";   rows[1].what = "missões";    rows[1].value = NULL;
		rows[2].key = "Z";   rows[2].what = "mirar em";   rows[2].value = NULL;
		rows[3].key = "H";   rows[3].what = "miniatura";  rows[3].value = NULL;
		rows[4].key = "O";   rows[4].what = "posição";    rows[4].value = NULL;
		rows[5].key = "[ ]"; rows[5].what = "escala";     rows[5].value = scale_text;
		rows[6].key = ", ."; rows[6].what = "opacidade";  rows[6].value = opacity_text;

		vrule(cr, x, rule_top, rule_bottom);
		section_label(cr, x + PAD, "OPÇÕES");

		for (int i = 0; i < 7; i++)
		{
			cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.055);
			rounded_rect(cr, x + PAD, y - 10.0, 24.0, 14.0, 2.0);
			cairo_fill(cr);

			cairo_set_source_rgba(cr, C_DIM);
			text_centered(cr, FONT_MONO, x + PAD + 12.0, y, 9.0, 0, rows[i].key);

			cairo_set_source_rgba(cr, C_LABEL);
			text_at(cr, FONT_UI, x + PAD + 32.0, y, 9.5, 0, rows[i].what);

			if (rows[i].value != NULL)
			{
				cairo_set_source_rgba(cr, C_DIM);
				text_right(cr, FONT_MONO, x + width - PAD, y, 9.5, 0, rows[i].value);
			}

			y += 16.0;
		}

		x += width;
	}

	layer_upload(&g_bar);
	g_drawn = g_data;
	g_last_render_time = now_seconds();

	// depuraccao: DASH_DUMP_PNG=<arquivo> grava o painel como PNG a cada 2 s
	{
		const char *dump = getenv("DASH_DUMP_PNG");
		static double last_dump = 0.0;

		if (dump != NULL && (now_seconds() - last_dump) > 2.0)
		{
			cairo_surface_write_to_png(g_bar.surface, dump);
			last_dump = now_seconds();
		}
	}
}


// ---------------------------------------------------------------------------------------------
// Painel lateral de missoes
// ---------------------------------------------------------------------------------------------
// Desenha um painel lateral (missoes ou lugares da camera) na sua propria camada, em
// coordenadas de projeto. Quem escala e' o dashboard_draw, junto com a barra -- era isto que
// faltava: em tela cheia o painel ficava do tamanho de sempre enquanto a barra crescia.
static void
render_list_panel(list_panel_t *panel, int window_height, int bar_height)
{
	cairo_t *cr;
	int height = 88 + panel->num_items * 30 + 50;
	int max_height = (int) ((window_height - bar_height) / g_scale) - 2 * MARGIN;

	if (max_height < 160)
		max_height = 160;
	if (height > max_height)
		height = max_height;

	layer_ensure(&panel->layer, MISSION_PANEL_WIDTH, height);
	cr = panel->layer.cr;
	layer_clear(&panel->layer);

	cairo_set_source_rgba(cr, C_BG);
	rounded_rect(cr, 0.0, 0.0, MISSION_PANEL_WIDTH, height, 3.0);
	cairo_fill(cr);

	cairo_set_source_rgba(cr, C_RULE);
	cairo_set_line_width(cr, 1.0);
	rounded_rect(cr, 0.5, 0.5, MISSION_PANEL_WIDTH - 1.0, height - 1.0, 3.0);
	cairo_stroke(cr);

	cairo_set_source_rgba(cr, C_RULE_TOP);
	cairo_rectangle(cr, 0.0, 0.0, MISSION_PANEL_WIDTH, 1.0);
	cairo_fill(cr);

	cairo_set_source_rgba(cr, C_LABEL);
	text_tracked(cr, PAD, 28.0, 9.5, 1.3, panel->title);

	cairo_set_source_rgba(cr, 0.345, 0.385, 0.430, 1.0);
	text_right(cr, FONT_UI, MISSION_PANEL_WIDTH - PAD, 28.0, 9.0, 0, panel->hint);

	hairline(cr, PAD, 40.0, MISSION_PANEL_WIDTH - 2.0 * PAD, 0.09);

	double y = 54.0;
	for (int i = 0; i < panel->num_items; i++)
	{
		int hot = (i == panel->hover);
		int marked = (i == panel->selected);

		if (y + 26.0 > height - 54.0)
			break;

		if (hot || marked)
		{
			cairo_set_source_rgba(cr, 0.290, 0.620, 0.855, hot ? 0.14 : 0.07);
			rounded_rect(cr, PAD - 6.0, y, MISSION_PANEL_WIDTH - 2.0 * PAD + 12.0, 26.0, 2.0);
			cairo_fill(cr);

			cairo_set_source_rgba(cr, C_ACCENT);
			cairo_rectangle(cr, PAD - 6.0, y, 2.0, 26.0);
			cairo_fill(cr);
		}

		if (hot || marked)
			cairo_set_source_rgba(cr, C_TEXT);
		else
			cairo_set_source_rgba(cr, 0.750, 0.790, 0.835, 1.0);
		text_at(cr, FONT_UI, PAD + 2.0, y + 17.0, 11.0, 0, panel->items[i]);

		y += 30.0;
	}

	if (panel->num_items == 0)
	{
		cairo_set_source_rgba(cr, 0.345, 0.385, 0.430, 1.0);
		text_at(cr, FONT_UI, PAD, 72.0, 10.5, 0, "nada nesta lista");
	}

	// rodape clicavel (ABORTAR MISSAO / SEGUIR O CARRO)
	double footer_y = height - 44.0;

	hairline(cr, PAD, footer_y - 12.0, MISSION_PANEL_WIDTH - 2.0 * PAD, 0.07);

	cairo_set_source_rgba(cr, panel->footer_r, panel->footer_g, panel->footer_b, 0.10);
	rounded_rect(cr, PAD, footer_y, MISSION_PANEL_WIDTH - 2.0 * PAD, 30.0, 2.0);
	cairo_fill(cr);

	cairo_set_source_rgba(cr, panel->footer_r, panel->footer_g, panel->footer_b, 0.55);
	cairo_set_line_width(cr, 1.0);
	rounded_rect(cr, PAD + 0.5, footer_y + 0.5, MISSION_PANEL_WIDTH - 2.0 * PAD - 1.0, 29.0, 2.0);
	cairo_stroke(cr);

	{
		double width;

		cairo_push_group(cr);
		width = text_tracked(cr, 0.0, 0.0, 9.5, 1.1, panel->footer);
		cairo_pattern_destroy(cairo_pop_group(cr));

		cairo_set_source_rgba(cr, panel->footer_r, panel->footer_g, panel->footer_b, 1.0);
		text_tracked(cr, (MISSION_PANEL_WIDTH - width) / 2.0, footer_y + 19.0, 9.5, 1.1,
				panel->footer);
	}

	layer_upload(&panel->layer);
	panel->dirty = 0;
}


// Cola o painel na tela ja' escalado e guarda as areas de clique em coordenadas de tela.
static void
blit_list_panel(list_panel_t *panel, int x, int y)
{
	int w = (int) (panel->layer.width * g_scale);
	int h = (int) (panel->layer.height * g_scale);

	blit_texture(panel->layer.texture, x, y, w, h);

	panel->row_x0 = x + (int) ((PAD - 6) * g_scale);
	panel->row_x1 = x + (int) ((MISSION_PANEL_WIDTH - PAD + 6) * g_scale);

	for (int i = 0; i < panel->num_items; i++)
	{
		panel->row_y0[i] = y + (int) ((54 + i * 30) * g_scale);
		panel->row_y1[i] = panel->row_y0[i] + (int) (26 * g_scale);
	}

	panel->footer_y0 = y + h - (int) (44 * g_scale);
	panel->footer_y1 = panel->footer_y0 + (int) (30 * g_scale);
}


// Trata o mouse sobre um painel lateral. Devolve: 0 nao era aqui, 1 consumiu,
// 2 clicou num item (indice em *item_out), 3 clicou no rodape.
static int
list_panel_mouse(list_panel_t *panel, int type, int x, int y, int *item_out)
{
	if (!panel->visible)
		return (0);

	if (x < panel->row_x0 || x > panel->row_x1)
	{
		if (panel->hover != -1)
		{
			panel->hover = -1;
			panel->dirty = 1;
		}

		return (0);
	}

	if (type == 6)		// MotionNotify
	{
		int hover = -1;

		for (int i = 0; i < panel->num_items; i++)
			if (y >= panel->row_y0[i] && y <= panel->row_y1[i])
				hover = i;

		if (hover != panel->hover)
		{
			panel->hover = hover;
			panel->dirty = 1;
		}

		return ((hover >= 0) ? 1 : 0);
	}

	if (type != 4)
		return (0);

	if (y >= panel->footer_y0 && y <= panel->footer_y1)
		return (3);

	for (int i = 0; i < panel->num_items; i++)
	{
		if (y >= panel->row_y0[i] && y <= panel->row_y1[i])
		{
			*item_out = i;

			return (2);
		}
	}

	return (0);
}


// ---------------------------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------------------------
void
dashboard_init(void)
{
	memset(&g_data, 0, sizeof(g_data));
	memset(&g_drawn, 0, sizeof(g_drawn));
	memset(&g_bar, 0, sizeof(g_bar));

	memset(&g_missions_panel, 0, sizeof(g_missions_panel));
	g_missions_panel.title = "MISSÕES";
	g_missions_panel.hint = "clique para executar";
	g_missions_panel.footer = "ABORTAR MISSÃO";
	g_missions_panel.footer_r = 0.851;
	g_missions_panel.footer_g = 0.325;
	g_missions_panel.footer_b = 0.310;
	g_missions_panel.hover = -1;
	g_missions_panel.selected = -1;
	g_missions_panel.dirty = 1;

	memset(&g_camera_panel, 0, sizeof(g_camera_panel));
	g_camera_panel.title = "CÂMERA";
	g_camera_panel.hint = "clique para mirar";
	g_camera_panel.footer = "SEGUIR O CARRO";
	g_camera_panel.footer_r = 0.290;
	g_camera_panel.footer_g = 0.620;
	g_camera_panel.footer_b = 0.855;
	g_camera_panel.hover = -1;
	g_camera_panel.selected = -1;
	g_camera_panel.dirty = 1;

	g_data.frenet_selected = -1;
	g_data.goal_distance = -1.0;
	g_data.max_speed = 0.0;
	g_data.max_phi = 0.0;
	g_data.steering_ratio = 16.0;		// mesma relaccao do painel do astro

	// enquanto nao chegar um status de CAN com marcha conhecida, ela vem do movimento
	g_data.gear = DASHBOARD_GEAR_UNKNOWN;
	g_data.gear_estimated = 1;
	strcpy(g_data.route_planner_state, "—");
	strcpy(g_data.behavior_state, "—");
	strcpy(g_data.mission_state, "—");
	strcpy(g_data.localize_state, "—");
}


void
dashboard_destroy(void)
{
	if (g_bar.cr != NULL)
		cairo_destroy(g_bar.cr);
	if (g_bar.surface != NULL)
		cairo_surface_destroy(g_bar.surface);
	list_panel_t *panels[2] = { &g_missions_panel, &g_camera_panel };

	for (int i = 0; i < 2; i++)
	{
		if (panels[i]->layer.cr != NULL)
			cairo_destroy(panels[i]->layer.cr);
		if (panels[i]->layer.surface != NULL)
			cairo_surface_destroy(panels[i]->layer.surface);
		memset(&panels[i]->layer, 0, sizeof(layer_t));
	}

	if (g_camera_rgb != NULL)
		free(g_camera_rgb);

	memset(&g_bar, 0, sizeof(g_bar));
	g_camera_rgb = NULL;
}


dashboard_data_t *
dashboard_data(void)
{
	return (&g_data);
}


static void
set_panel_items(list_panel_t *panel, char **names, int num_items)
{
	if (num_items > PANEL_MAX_ITEMS)
		num_items = PANEL_MAX_ITEMS;

	for (int i = 0; i < num_items; i++)
	{
		strncpy(panel->items[i], names[i], DASHBOARD_MISSION_NAME_SIZE - 1);
		panel->items[i][DASHBOARD_MISSION_NAME_SIZE - 1] = '\0';
	}

	panel->num_items = num_items;
	if (panel->selected >= num_items)
		panel->selected = -1;
	panel->dirty = 1;
}


void
dashboard_set_missions(char **names, int num_missions)
{
	set_panel_items(&g_missions_panel, names, num_missions);
}


void
dashboard_set_camera_targets(char **names, int num_targets)
{
	set_panel_items(&g_camera_panel, names, num_targets);
}


void
dashboard_set_camera_following(int following)
{
	if (g_camera_following == following)
		return;

	g_camera_following = following;
	if (following)
		g_camera_panel.selected = -1;
	g_camera_panel.dirty = 1;
}


void
dashboard_set_camera_image(const unsigned char *data, int width, int height, int channels)
{
	int needed = width * height * 3;

	if (data == NULL || width <= 0 || height <= 0)
		return;

	if (g_camera_rgb == NULL || g_camera_rgb_size < needed)
	{
		g_camera_rgb = (unsigned char *) realloc(g_camera_rgb, needed);
		g_camera_rgb_size = needed;
	}

	if (channels == 3)
	{
		// camera_message vem em BGR (ver a nota do camera_drivers)
		for (int i = 0; i < width * height; i++)
		{
			g_camera_rgb[3 * i + 0] = data[3 * i + 2];
			g_camera_rgb[3 * i + 1] = data[3 * i + 1];
			g_camera_rgb[3 * i + 2] = data[3 * i + 0];
		}
	}
	else if (channels == 1)
	{
		for (int i = 0; i < width * height; i++)
		{
			g_camera_rgb[3 * i + 0] = data[i];
			g_camera_rgb[3 * i + 1] = data[i];
			g_camera_rgb[3 * i + 2] = data[i];
		}
	}
	else
		return;

	g_camera_width = width;
	g_camera_height = height;
	g_camera_dirty = 1;
}


void
dashboard_toggle_missions(void)
{
	g_missions_panel.visible = !g_missions_panel.visible;
	g_missions_panel.dirty = 1;
	g_bar.uploaded = 0;
}


void
dashboard_toggle_camera_panel(void)
{
	g_camera_panel.visible = !g_camera_panel.visible;
	g_camera_panel.dirty = 1;
	g_bar.uploaded = 0;
}


int
dashboard_camera_panel_visible(void)
{
	return (g_camera_panel.visible);
}


void
dashboard_toggle_camera(void)
{
	g_camera_visible = !g_camera_visible;
}


void
dashboard_toggle_panel(void)
{
	g_panel_visible = !g_panel_visible;
}


void
dashboard_set_style(double scale, double opacity, int position)
{
	if (scale >= 0.0)
		g_scale_setting = (scale > 0.0 && scale < 0.4) ? 0.4 : ((scale > 4.0) ? 4.0 : scale);
	if (opacity > 0.0)
		g_opacity = (opacity > 1.0) ? 1.0 : ((opacity < 0.15) ? 0.15 : opacity);
	if (position >= 0 && position < DASHBOARD_NUM_POSITIONS)
		g_position = position;

	g_bar.uploaded = 0;
	g_missions_panel.dirty = 1;
	g_camera_panel.dirty = 1;
}


void
dashboard_cycle_position(void)
{
	g_position = (g_position + 1) % DASHBOARD_NUM_POSITIONS;
	g_bar.uploaded = 0;
	g_missions_panel.dirty = 1;
	g_camera_panel.dirty = 1;
}


void
dashboard_add_scale(double delta)
{
	// a primeira mexida parte da escala que estava valendo, nao de 1.0
	if (g_scale_setting <= 0.0)
		g_scale_setting = g_scale;

	g_scale_setting += delta;
	if (g_scale_setting < 0.5)
		g_scale_setting = 0.5;
	if (g_scale_setting > 4.0)
		g_scale_setting = 4.0;

	g_bar.uploaded = 0;
}


void
dashboard_add_opacity(double delta)
{
	g_opacity += delta;
	if (g_opacity < 0.15)
		g_opacity = 0.15;
	if (g_opacity > 1.0)
		g_opacity = 1.0;

	g_bar.uploaded = 0;
}


double
dashboard_get_scale(void)
{
	return (g_scale);
}


double
dashboard_get_opacity(void)
{
	return (g_opacity);
}


int
dashboard_get_position(void)
{
	return (g_position);
}


void
dashboard_set_mode(int mode)
{
	if (g_mode != mode)
	{
		g_mode = mode;
		g_bar.uploaded = 0;
	}
}


int
dashboard_get_mode(void)
{
	return (g_mode);
}


int
dashboard_missions_visible(void)
{
	return (g_missions_panel.visible);
}


static int
data_changed(void)
{
	const dashboard_data_t *a = &g_data;
	const dashboard_data_t *b = &g_drawn;

	if (fabs(a->v - b->v) > 0.005 || fabs(a->phi - b->phi) > 0.0005)
		return (1);
	if (fabs(a->throttle - b->throttle) > 0.5 || fabs(a->brake - b->brake) > 0.5)
		return (1);
	if (fabs(a->a_long - b->a_long) > 0.02 || fabs(a->a_lat - b->a_lat) > 0.02)
		return (1);
	if (a->gear != b->gear || a->autonomous != b->autonomous || a->engine != b->engine)
		return (1);
	if (a->gear_estimated != b->gear_estimated || a->gear_raw != b->gear_raw)
		return (1);
	if (fabs(a->v_command - b->v_command) > 0.01 || fabs(a->phi_command - b->phi_command) > 0.002)
		return (1);
	if (a->parking_brake != b->parking_brake || a->turn_signal != b->turn_signal)
		return (1);
	if (a->frenet_num_paths != b->frenet_num_paths || a->frenet_selected != b->frenet_selected)
		return (1);
	if (fabs(a->goal_distance - b->goal_distance) > 0.05)
		return (1);
	if (strcmp(a->route_planner_state, b->route_planner_state) != 0)
		return (1);
	if (strcmp(a->behavior_state, b->behavior_state) != 0)
		return (1);
	if (strcmp(a->mission_state, b->mission_state) != 0)
		return (1);
	if (strcmp(a->mission_name, b->mission_name) != 0)
		return (1);

	return (0);
}


static void
push_history(void)
{
	double t = now_seconds();

	if (t - g_hist_last_time < 0.1)
		return;

	g_hist_last_time = t;
	g_hist_v[g_hist_index] = (float) g_data.v;
	g_hist_phi[g_hist_index] = (float) g_data.phi;
	g_hist_index = (g_hist_index + 1) % DASHBOARD_HISTORY_SIZE;
	if (g_hist_count < DASHBOARD_HISTORY_SIZE)
		g_hist_count++;
}


static void
begin_2d(int window_width, int window_height)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, window_width, window_height, 0.0, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);	// cairo entrega alfa pre-multiplicado
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}


static void
end_2d(void)
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glPopAttrib();
}


static void
draw_camera(int window_width, int window_height)
{
	int width;
	int height;

	if (!g_camera_visible || g_camera_width <= 0)
		return;

	if (g_camera_texture == 0)
		glGenTextures(1, &g_camera_texture);

	if (g_camera_dirty)
	{
		glBindTexture(GL_TEXTURE_2D, g_camera_texture);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		// alinhamento 1 porque a linha da imagem RGB nao e' multipla de 4; o par
		// push/popClientAttrib impede que isso vaze para os outros uploads do viewer_3D
		glPushClientAttrib(GL_CLIENT_PIXEL_STORE_BIT);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_camera_width, g_camera_height, 0,
				GL_RGB, GL_UNSIGNED_BYTE, g_camera_rgb);
		glPopClientAttrib();
		g_camera_dirty = 0;
	}

	width = (int) (CAMERA_WIDTH * g_scale);
	if (width > window_width / 3)
		width = window_width / 3;
	height = (int) (width * g_camera_height / (double) g_camera_width);

	int x = window_width - width - MARGIN;
	int y = MARGIN;

	// se o painel esta' em cima, a camera desce para o rodape
	if (g_position == DASHBOARD_POSITION_TOP || g_position == DASHBOARD_POSITION_TOP_RIGHT)
		y = window_height - height - MARGIN;
	if (g_position == DASHBOARD_POSITION_BOTTOM_RIGHT || g_position == DASHBOARD_POSITION_TOP_RIGHT)
		x = MARGIN;

	// moldura: mesmo grafite do painel, com um filete claro em volta
	glDisable(GL_TEXTURE_2D);
	glColor4f(0.043f, 0.051f, 0.063f, 0.93f);
	glBegin(GL_QUADS);
		glVertex2i(x - 5,         y - 5);
		glVertex2i(x + width + 5, y - 5);
		glVertex2i(x + width + 5, y + height + 5);
		glVertex2i(x - 5,         y + height + 5);
	glEnd();

	glColor4f(1.0f, 1.0f, 1.0f, 0.09f);
	glBegin(GL_LINE_LOOP);
		glVertex2i(x - 5,         y - 5);
		glVertex2i(x + width + 5, y - 5);
		glVertex2i(x + width + 5, y + height + 5);
		glVertex2i(x - 5,         y + height + 5);
	glEnd();
	glEnable(GL_TEXTURE_2D);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	blit_texture(g_camera_texture, x, y, width, height);
}


static void
compute_geometry(int window_width, int window_height, int *bar_x, int *bar_y,
		int *bar_w, int *bar_h, int *design_w, int *floating)
{
	double scale = (g_scale_setting > 0.0) ? g_scale_setting : (window_width / 1400.0);
	int automatic = (g_scale_setting <= 0.0);

	if (automatic)
	{
		if (scale < 1.0)
			scale = 1.0;
		if (scale > 2.4)
			scale = 2.4;
	}

	// o painel nunca pode comer mais que metade da janela
	if (PANEL_HEIGHT * scale > window_height * 0.5)
		scale = (window_height * 0.5) / PANEL_HEIGHT;
	if (scale < 0.4)
		scale = 0.4;

	g_scale = scale;

	*floating = (g_position >= DASHBOARD_POSITION_BOTTOM_LEFT);
	*bar_w = *floating ? (int) (window_width * 0.62) : window_width;

	*design_w = (int) (*bar_w / scale);
	if (*design_w < 660)					// abaixo disso as secccoes fixas nao cabem
	{
		*design_w = 660;
		*bar_w = (int) (*design_w * scale);
	}
	if (*bar_w > window_width)
	{
		*bar_w = window_width;
		*design_w = (int) (*bar_w / scale);
	}
	*bar_h = (int) (PANEL_HEIGHT * scale);

	switch (g_position)
	{
	case DASHBOARD_POSITION_TOP:
		*bar_x = 0;
		*bar_y = 0;
		break;

	case DASHBOARD_POSITION_BOTTOM_LEFT:
		*bar_x = MARGIN;
		*bar_y = window_height - *bar_h - MARGIN;
		break;

	case DASHBOARD_POSITION_BOTTOM_RIGHT:
		*bar_x = window_width - *bar_w - MARGIN;
		*bar_y = window_height - *bar_h - MARGIN;
		break;

	case DASHBOARD_POSITION_TOP_LEFT:
		*bar_x = MARGIN;
		*bar_y = MARGIN;
		break;

	case DASHBOARD_POSITION_TOP_RIGHT:
		*bar_x = window_width - *bar_w - MARGIN;
		*bar_y = MARGIN;
		break;

	case DASHBOARD_POSITION_BOTTOM:
	default:
		*bar_x = 0;
		*bar_y = window_height - *bar_h;
		break;
	}
}


double
dashboard_menu_bottom_offset(int window_width, int window_height)
{
	int bar_x, bar_y, bar_w, bar_h, design_w, floating;

	if (!g_panel_visible || window_width < 64 || window_height < 64)
		return (0.0);

	if (g_position != DASHBOARD_POSITION_BOTTOM &&
			g_position != DASHBOARD_POSITION_BOTTOM_LEFT &&
			g_position != DASHBOARD_POSITION_BOTTOM_RIGHT)
		return (0.0);

	compute_geometry(window_width, window_height, &bar_x, &bar_y, &bar_w, &bar_h,
			&design_w, &floating);

	return (bar_h + (floating ? MARGIN : 0) + 8);
}


void
dashboard_draw(int window_width, int window_height)
{
	int bar_x, bar_y, bar_w, bar_h, design_w, floating;
	static int last_design_w = 0;
	static int last_bar_h = 0;
	static double last_panel_scale = 0.0;

	if (window_width < 64 || window_height < 64)
		return;

	push_history();

	compute_geometry(window_width, window_height, &bar_x, &bar_y, &bar_w, &bar_h,
			&design_w, &floating);

	g_bar_x = bar_x;
	g_bar_y = bar_y;
	g_bar_w = bar_w;
	g_bar_h = bar_h;

	if (!g_panel_visible)
	{
		g_bar_w = 0;
		g_bar_h = 0;

		begin_2d(window_width, window_height);
		draw_camera(window_width, window_height);
		end_2d();

		return;
	}

	layer_ensure(&g_bar, bar_w, bar_h);

	if (!g_bar.uploaded || design_w != last_design_w || bar_h != last_bar_h ||
			data_changed() || (now_seconds() - g_last_render_time) > 0.2)
	{
		cairo_save(g_bar.cr);
		cairo_scale(g_bar.cr, g_scale, g_scale);
		render_bar(design_w, floating);
		cairo_restore(g_bar.cr);

		last_design_w = design_w;
		last_bar_h = bar_h;
	}

	{
		list_panel_t *panels[2] = { &g_missions_panel, &g_camera_panel };

		for (int i = 0; i < 2; i++)
			if (panels[i]->visible && (panels[i]->dirty || !panels[i]->layer.uploaded ||
					fabs(g_scale - last_panel_scale) > 0.001))
			{
				render_list_panel(panels[i], window_height, bar_h);
			}

		last_panel_scale = g_scale;
	}

	begin_2d(window_width, window_height);

	blit_texture(g_bar.texture, bar_x, bar_y, bar_w, bar_h);

	// Os paineis laterais ficam lado a lado no alto (ou abaixo da barra, quando ela esta no
	// alto) e **acompanham a escala do painel** -- antes eram colados no tamanho de projeto e
	// ficavam minusculos em tela cheia.
	{
		list_panel_t *panels[2] = { &g_missions_panel, &g_camera_panel };
		int x = MARGIN;
		int y = MARGIN;

		if (g_position == DASHBOARD_POSITION_TOP || g_position == DASHBOARD_POSITION_TOP_LEFT ||
				g_position == DASHBOARD_POSITION_TOP_RIGHT)
			y = bar_y + bar_h + MARGIN;

		for (int i = 0; i < 2; i++)
		{
			if (!panels[i]->visible || !panels[i]->layer.uploaded)
				continue;

			blit_list_panel(panels[i], x, y);
			x += (int) (MISSION_PANEL_WIDTH * g_scale) + MARGIN;
		}
	}

	draw_camera(window_width, window_height);

	end_2d();
}


int
dashboard_mouse(int type, int button, int x, int y, int window_width, int window_height,
		char *mission_out, int mission_out_size)
{
	int item;

	(void) button;
	(void) window_width;
	(void) window_height;

	if (!g_panel_visible)
		return (DASHBOARD_ACTION_NONE);

	int inside_bar = (x >= g_bar_x && x <= g_bar_x + g_bar_w &&
			y >= g_bar_y && y <= g_bar_y + g_bar_h);

	if (inside_bar)
	{
		// converte a coordenada da tela para a coordenada de projeto do painel
		double dx = (x - g_bar_x) / g_scale;
		double dy = (y - g_bar_y) / g_scale;
		int hit = -1;

		for (int i = 0; i < g_num_buttons; i++)
		{
			if (dx >= g_buttons[i].x && dx <= g_buttons[i].x + g_buttons[i].w &&
					dy >= g_buttons[i].y && dy <= g_buttons[i].y + g_buttons[i].h)
				hit = i;
		}

		if (type == 6)		// MotionNotify: so' destaca
		{
			if (hit != g_button_hover)
			{
				g_button_hover = hit;
				g_bar.uploaded = 0;
			}

			return (DASHBOARD_ACTION_CONSUMED);
		}

		if (type == 4 && hit >= 0)		// ButtonPress
		{
			int action = g_buttons[hit].action;

			// MISSOES e CAMERA_PANEL sobem para o hospedeiro em vez de serem tratados aqui:
			// ele releh a pasta de missoes / a lista de anotaccoes antes de abrir o painel.
			return (action);
		}

		// clique no painel, fora de botao: engole para nao girar a camera
		return (DASHBOARD_ACTION_CONSUMED);
	}

	if (g_button_hover != -1)
	{
		g_button_hover = -1;
		g_bar.uploaded = 0;
	}

	// ---- lista de missoes ------------------------------------------------------------
	switch (list_panel_mouse(&g_missions_panel, type, x, y, &item))
	{
	case 1:
		return (DASHBOARD_ACTION_CONSUMED);

	case 2:
		strncpy(mission_out, g_missions_panel.items[item], mission_out_size - 1);
		mission_out[mission_out_size - 1] = '\0';

		return (DASHBOARD_ACTION_MISSION);

	case 3:
		return (DASHBOARD_ACTION_ABORT);
	}

	// ---- lugares para mirar a camera -------------------------------------------------
	switch (list_panel_mouse(&g_camera_panel, type, x, y, &item))
	{
	case 1:
		return (DASHBOARD_ACTION_CONSUMED);

	case 2:
		strncpy(mission_out, g_camera_panel.items[item], mission_out_size - 1);
		mission_out[mission_out_size - 1] = '\0';
		g_camera_panel.selected = item;
		g_camera_panel.dirty = 1;

		return (DASHBOARD_ACTION_CAMERA_TARGET);

	case 3:
		g_camera_panel.selected = -1;
		g_camera_panel.dirty = 1;

		return (DASHBOARD_ACTION_CAMERA_FOLLOW);
	}

	return (DASHBOARD_ACTION_NONE);
}
