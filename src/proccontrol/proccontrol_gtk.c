 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

/*
 * proccontrol_gtk.c — GUI do proccontrol em GTK3.
 *
 * Migração Ubuntu 26.04: substitui o antigo proccontrol_gui.cpp, que era Qt4 + Qt3Support
 * (módulo de compatibilidade extinto) e precisava de um passo de build à parte, por qmake.
 * A estrutura segue a mesma solução já adotada num fork deste código (GHashTable de grupos
 * e módulos, menu por botão, cor via CSS, GMainLoop + dispatch não-bloqueante do IPC), mas
 * o layout e o comportamento reproduzem o GUI original do CARMEN:
 *
 *   - uma janela com os grupos empilhados verticalmente;
 *   - cada grupo é um GtkFrame com o nome do grupo, contendo uma LINHA de botões:
 *     primeiro o botão "All" (estreito), depois um botão por módulo;
 *   - cada botão mostra "nome_do_modulo\npid: N" e abre um menu com
 *     Start Program / Stop Program / Show Output / No Output;
 *   - o botão "All" abre um menu com Start / Stop, que agem sobre o grupo inteiro;
 *   - abaixo de tudo, a área de texto com a saída dos módulos ("modulo (pid): linha");
 *   - verde = ativo, vermelho = parado, amarelo = reiniciando demais.
 *
 * Este arquivo é C puro e é construído pelo Makefile normal do CARMEN — não há mais qmake,
 * moc, nem .pro.
 */

#include <carmen/carmen.h>
#include <carmen/proccontrol_interface.h>
#include <carmen/user_preferences.h>
#include <gtk/gtk.h>
#include <libgen.h>

/* Geometria herdada do GUI Qt original */
#define ALL_BUTTON_WIDTH     60		/* o "All" era setMinimumWidth(60)/setMaximumWidth(60) */
#define MODULE_BUTTON_WIDTH  100	/* os botões de módulo eram setMaximumWidth(100)       */
#define MAX_LOG_LINES        500	/* era setMaxLogLines(500) no Q3TextView               */
#define STABLE_TIMER         20		/* segundos sem reinício para zerar o contador         */
#define MAX_REINITIALIZATIONS 3		/* acima disso o módulo é considerado instável         */
#define IPC_DISPATCH_PERIOD_MS 20	/* equivalente ao carmen_ipc_sleep(0.02) do laço Qt    */

/* Cores exatamente como no GUI Qt original */
#define CSS_ACTIVE    "button { color: rgb(0,0,0); background-image: none; background-color: rgb(60,192,34); }"
#define CSS_INACTIVE  "button { color: rgb(0,0,0); background-image: none; background-color: rgb(221,0,3); }"
#define CSS_UNSTABLE  "button { color: rgb(0,0,0); background-image: none; background-color: rgb(255,255,0); }"

typedef struct
{
	char *name;
	int pid;
	int active;
	int status;			/* 0 parado, 1 ativo, 2 instável */
	int requested_state;
	gboolean show_output;
	unsigned int reinitializations;
	double previous_change;
	GtkWidget *button;
} Module;

typedef struct
{
	char *name;
	GList *modules;
	GtkWidget *frame;
	GtkWidget *box;
	GtkWidget *all_button;
} ModuleGroup;

typedef enum
{
	ACTION_START,
	ACTION_STOP,
	ACTION_SHOW_OUTPUT,
	ACTION_NO_OUTPUT
} ModuleAction;

static GtkWidget *window, *main_box, *scrolled_window, *text_view;
static GtkTextBuffer *text_buffer;
static GMainLoop *main_loop;

static GHashTable *groups_hash, *modules_hash;
static GHashTable *user_show_output_hash = NULL;	/* -show modulo1 modulo2 ... */

static int output_subscribed = FALSE;

/* Preferências de usuário — mesmos nomes de chave do GUI Qt, para reaproveitar o arquivo */
static char *user_pref_filename = NULL;
static const char *user_pref_module;
static user_param_t *user_pref_param_list;
static int user_pref_num_items;
static int user_pref_window_width  = 600;
static int user_pref_window_height = 400;
static int user_pref_window_x = -1;
static int user_pref_window_y = -1;
static int user_pref_output_lines = 10;

static void carmen_output_handler(carmen_proccontrol_output_message *msg);


/**********************************************************************
 *
 *	SAÍDA DOS MÓDULOS
 *
 **********************************************************************/

/*
 * O GUI original só assinava as mensagens de saída quando havia pelo menos um módulo com
 * "Show Output" ligado, e cancelava a assinatura quando o último era desligado. Isso é
 * mantido: sem ninguém olhando, não faz sentido trafegar a saída de todos os módulos.
 */
static void
update_output_subscription(void)
{
	GHashTableIter iter;
	gpointer key, value;
	int anyone_wants_output = FALSE;

	g_hash_table_iter_init(&iter, modules_hash);
	while (g_hash_table_iter_next(&iter, &key, &value))
	{
		if (((Module *) value)->show_output)
		{
			anyone_wants_output = TRUE;
			break;
		}
	}

	if (anyone_wants_output && !output_subscribed)
	{
		fprintf(stderr, "INFO: subscribe output messages\n");
		carmen_proccontrol_subscribe_output_message(NULL, (carmen_handler_t) carmen_output_handler,
				CARMEN_SUBSCRIBE_ALL);
		output_subscribed = TRUE;
	}
	else if (!anyone_wants_output && output_subscribed)
	{
		fprintf(stderr, "INFO: unsubscribe output\n");
		carmen_proccontrol_unsubscribe_output_message((carmen_handler_t) carmen_output_handler);
		output_subscribed = FALSE;
	}
}


/**********************************************************************
 *
 *	MENUS
 *
 **********************************************************************/

static void
apply_module_action(Module *module, ModuleAction action)
{
	switch (action)
	{
		case ACTION_START:
			fprintf(stderr, "INFO: start module %s\n", module->name);
			carmen_proccontrol_set_module_state(module->name, 1);
			module->reinitializations = 0;
			break;
		case ACTION_STOP:
			fprintf(stderr, "INFO: stop module %s\n", module->name);
			carmen_proccontrol_set_module_state(module->name, 0);
			break;
		case ACTION_SHOW_OUTPUT:
			module->show_output = TRUE;
			update_output_subscription();
			break;
		case ACTION_NO_OUTPUT:
			module->show_output = FALSE;
			update_output_subscription();
			break;
	}
}


static void
on_module_menu_item_clicked(GtkMenuItem *item, gpointer data __attribute__ ((unused)))
{
	Module *module = g_object_get_data(G_OBJECT(item), "module");

	if (module == NULL)
	{
		g_warning("Invalid module in menu\n");
		return;
	}

	apply_module_action(module, GPOINTER_TO_INT(g_object_get_data(G_OBJECT(item), "action")));
}


static void
add_menu_item(GtkWidget *menu, const char *label, const char *data_key, gpointer data_value,
		ModuleAction action, GCallback callback)
{
	GtkWidget *item = gtk_menu_item_new_with_label(label);

	g_object_set_data(G_OBJECT(item), data_key, data_value);
	g_object_set_data(G_OBJECT(item), "action", GINT_TO_POINTER(action));
	g_signal_connect(item, "activate", callback, NULL);
	gtk_menu_shell_append(GTK_MENU_SHELL(menu), item);
}


/* Mesmos itens (e mesmos nomes) do menu do GUI Qt original */
static GtkWidget *
create_module_menu(Module *module)
{
	GtkWidget *menu = gtk_menu_new();

	add_menu_item(menu, "Start Program", "module", module, ACTION_START,
			G_CALLBACK(on_module_menu_item_clicked));
	add_menu_item(menu, "Stop Program",  "module", module, ACTION_STOP,
			G_CALLBACK(on_module_menu_item_clicked));
	add_menu_item(menu, "Show Output",   "module", module, ACTION_SHOW_OUTPUT,
			G_CALLBACK(on_module_menu_item_clicked));
	add_menu_item(menu, "No Output",     "module", module, ACTION_NO_OUTPUT,
			G_CALLBACK(on_module_menu_item_clicked));

	gtk_widget_show_all(menu);

	return menu;
}


static void
on_group_menu_item_clicked(GtkMenuItem *item, gpointer data __attribute__ ((unused)))
{
	ModuleGroup *group = g_object_get_data(G_OBJECT(item), "group");

	if (group == NULL)
	{
		g_warning("Invalid group in menu\n");
		return;
	}

	ModuleAction action = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(item), "action"));

	/*
	 * O GUI Qt mandava uma única mensagem de grupo (carmen_proccontrol_set_group_state) em vez
	 * de iterar módulo a módulo — o proccontrol já sabe expandir isso. Mantido.
	 */
	if (action == ACTION_START)
	{
		fprintf(stderr, "INFO: start group %s\n", group->name);
		carmen_proccontrol_set_group_state(group->name, 1);
	}
	else if (action == ACTION_STOP)
	{
		fprintf(stderr, "INFO: stop group %s\n", group->name);
		carmen_proccontrol_set_group_state(group->name, 0);
	}
}


/* O botão "All" do CARMEN só tinha Start e Stop */
static GtkWidget *
create_group_menu(ModuleGroup *group)
{
	GtkWidget *menu = gtk_menu_new();

	add_menu_item(menu, "Start", "group", group, ACTION_START,
			G_CALLBACK(on_group_menu_item_clicked));
	add_menu_item(menu, "Stop",  "group", group, ACTION_STOP,
			G_CALLBACK(on_group_menu_item_clicked));

	gtk_widget_show_all(menu);

	return menu;
}


static void
on_button_click(GtkWidget *widget __attribute__ ((unused)), GdkEventButton *event, gpointer menu)
{
	if (event->button == 1)
		gtk_menu_popup_at_pointer(GTK_MENU(menu), (GdkEvent *) event);
}


/**********************************************************************
 *
 *	WIDGETS DOS GRUPOS E MÓDULOS
 *
 **********************************************************************/

static void
apply_css(GtkWidget *widget, const char *css)
{
	GtkCssProvider *provider = gtk_css_provider_new();

	gtk_css_provider_load_from_data(provider, css, -1, NULL);
	gtk_style_context_add_provider(gtk_widget_get_style_context(widget),
			GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);
	g_object_unref(provider);
}


static void
update_module_status(Module *module, carmen_proccontrol_process_p msg, double timestamp)
{
	if (msg != NULL)
	{
		if (module->requested_state && !msg->requested_state)
		{
			module->reinitializations = 0;
			module->previous_change = timestamp;
		}
		module->requested_state = msg->requested_state;

		if (module->requested_state && msg->active && !module->active)
		{
			module->reinitializations++;
			module->previous_change = timestamp;
		}
		module->active = msg->active;

		if (module->requested_state && module->active && module->pid != msg->pid)
		{
			module->reinitializations++;
			module->previous_change = timestamp;
		}
		module->pid = msg->pid;
	}
	else
		timestamp = carmen_get_time();

	if (timestamp - module->previous_change > STABLE_TIMER)
	{
		module->reinitializations = 0;
		module->previous_change = timestamp;
	}

	if (module->active && module->reinitializations > MAX_REINITIALIZATIONS)
		module->status = 2;
	else
		module->status = module->active;

	if (module->status == 2)
		apply_css(module->button, CSS_UNSTABLE);
	else if (module->status == 1)
		apply_css(module->button, CSS_ACTIVE);
	else
		apply_css(module->button, CSS_INACTIVE);

	char label[256];
	snprintf(label, sizeof(label), "%s\npid: %d", module->name, module->pid);
	gtk_button_set_label(GTK_BUTTON(module->button), label);

	gtk_widget_show(module->button);
}


static ModuleGroup *
create_group(const char *group_name)
{
	ModuleGroup *group = g_new(ModuleGroup, 1);

	group->name = g_strdup(group_name);
	group->modules = NULL;

	/* O grupo é um quadro com título — o equivalente do Q3ButtonGroup do GUI Qt */
	group->frame = gtk_frame_new(group_name);

	/* HORIZONTAL: os botões do grupo ficam numa linha só, como no layout original */
	group->box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
	gtk_container_set_border_width(GTK_CONTAINER(group->box), 2);
	gtk_container_add(GTK_CONTAINER(group->frame), group->box);

	group->all_button = gtk_button_new_with_label("All");
	gtk_widget_set_size_request(group->all_button, ALL_BUTTON_WIDTH, -1);
	g_signal_connect(group->all_button, "button-release-event", G_CALLBACK(on_button_click),
			create_group_menu(group));
	gtk_box_pack_start(GTK_BOX(group->box), group->all_button, FALSE, FALSE, 0);

	/* pack_start com expand=FALSE: os grupos ficam no topo e a saída ocupa o resto */
	gtk_box_pack_start(GTK_BOX(main_box), group->frame, FALSE, FALSE, 0);
	gtk_widget_show_all(group->frame);

	g_hash_table_insert(groups_hash, group->name, group);

	return group;
}


static Module *
create_module(carmen_proccontrol_process_p msg)
{
	Module *module = g_new(Module, 1);

	module->name = g_strdup(msg->module_name);
	module->pid = msg->pid;
	module->active = msg->active;
	module->requested_state = msg->requested_state;
	module->reinitializations = 0;
	module->previous_change = 0.0;
	module->status = 0;
	module->show_output = (user_show_output_hash != NULL &&
			g_hash_table_contains(user_show_output_hash, msg->module_name));

	ModuleGroup *group = g_hash_table_lookup(groups_hash, msg->group_name);
	if (group == NULL)
		group = create_group(msg->group_name);

	module->button = gtk_button_new_with_label("");
	gtk_widget_set_size_request(module->button, MODULE_BUTTON_WIDTH, -1);
	g_signal_connect(module->button, "button-release-event", G_CALLBACK(on_button_click),
			create_module_menu(module));

	group->modules = g_list_append(group->modules, module);
	gtk_box_pack_start(GTK_BOX(group->box), module->button, FALSE, FALSE, 0);

	update_module_status(module, NULL, 0.0);

	g_hash_table_insert(modules_hash, module->name, module);

	return module;
}


/**********************************************************************
 *
 *	HANDLERS
 *
 **********************************************************************/

static void
carmen_proccontrol_pidtable_handler(carmen_proccontrol_pidtable_message *msg)
{
	for (int i = 0; i < msg->num_processes; i++)
	{
		Module *module = g_hash_table_lookup(modules_hash, msg->process[i].module_name);

		if (module == NULL)
			module = create_module(msg->process + i);

		update_module_status(module, msg->process + i, msg->timestamp);
	}

	if (user_show_output_hash != NULL)
	{
		/* -show só pode ser aplicado depois que os módulos existem */
		update_output_subscription();
		g_hash_table_destroy(user_show_output_hash);
		user_show_output_hash = NULL;
	}
}


static void
carmen_output_handler(carmen_proccontrol_output_message *msg)
{
	Module *module = g_hash_table_lookup(modules_hash, msg->module_name);

	if (module == NULL || !module->show_output)
		return;

	/* Troca as sequências de escape ANSI por espaços — senão elas sujam o GtkTextView */
	for (int i = 0; msg->output[i] != '\0'; i++)
	{
		if (msg->output[i] == '\x1b' && msg->output[i + 1] == '[')
		{
			msg->output[i++] = ' ';
			msg->output[i++] = ' ';
			while (msg->output[i] && !isalpha(msg->output[i]))
				msg->output[i++] = ' ';
			msg->output[i] = ' ';
		}
	}

	gchar *line = g_strdup_printf("%s (%d): %s\n", module->name, msg->pid, msg->output);
	GtkTextIter end_iter;

	gtk_text_buffer_get_end_iter(text_buffer, &end_iter);
	gtk_text_buffer_insert(text_buffer, &end_iter, line, -1);
	g_free(line);

	gtk_text_buffer_get_end_iter(text_buffer, &end_iter);
	gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(text_view), &end_iter, 0.0, TRUE, 0.0, 0.0);

	gint excess = gtk_text_buffer_get_line_count(text_buffer) - MAX_LOG_LINES;
	if (excess > 0)
	{
		GtkTextIter start_iter;
		gtk_text_buffer_get_start_iter(text_buffer, &start_iter);
		gtk_text_buffer_get_iter_at_line(text_buffer, &end_iter, excess);
		gtk_text_buffer_delete(text_buffer, &start_iter, &end_iter);
	}
}


/*
 * O laço do GUI Qt era `app.processEvents(); carmen_ipc_sleep(0.02)`. Em GTK quem manda no
 * laço é o GMainLoop, então o dispatch do IPC entra como um timeout periódico de 20 ms.
 */
static gboolean
ipc_dispatch_timeout(gpointer data __attribute__ ((unused)))
{
	carmen_ipc_dispatch_nonblocking();

	return TRUE;
}


/**********************************************************************
 *
 *	PREFERÊNCIAS DE USUÁRIO
 *
 **********************************************************************/

static void
read_user_preferences(int argc, char **argv)
{
	static user_param_t param_list[] =
	{
		{"window_width",  USER_PARAM_TYPE_INT, &user_pref_window_width},
		{"window_height", USER_PARAM_TYPE_INT, &user_pref_window_height},
		{"window_x",      USER_PARAM_TYPE_INT, &user_pref_window_x},
		{"window_y",      USER_PARAM_TYPE_INT, &user_pref_window_y},
		{"output_lines",  USER_PARAM_TYPE_INT, &user_pref_output_lines},
	};

	user_pref_module = basename(argv[0]);
	user_pref_param_list = param_list;
	user_pref_num_items = sizeof(param_list) / sizeof(param_list[0]);
	user_preferences_read(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
	user_preferences_read_commandline(argc, argv, user_pref_param_list, user_pref_num_items);
}


static void
set_user_preferences(void)
{
	if (user_pref_window_width > 0 && user_pref_window_height > 0)
		gtk_window_resize(GTK_WINDOW(window), user_pref_window_width, user_pref_window_height);
	if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
		gtk_window_move(GTK_WINDOW(window), user_pref_window_x, user_pref_window_y);
}


static void
save_user_preferences(void)
{
	gtk_window_get_size(GTK_WINDOW(window), &user_pref_window_width, &user_pref_window_height);
	gtk_window_get_position(GTK_WINDOW(window), &user_pref_window_x, &user_pref_window_y);
	user_pref_window_x += 10;
	user_pref_window_y += 10;
	user_preferences_save(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
}


/**********************************************************************
 *
 *	SHUTDOWN
 *
 **********************************************************************/

static void
shutdown_procedure(int sig)
{
	save_user_preferences();

	if (main_loop != NULL && g_main_loop_is_running(main_loop))
		g_main_loop_quit(main_loop);

	exit(sig);
}


static void
gtk_shutdown_wrapper(GtkWidget *widget __attribute__ ((unused)), gpointer data __attribute__ ((unused)))
{
	shutdown_procedure(0);
}


/**********************************************************************
 *
 *	MAIN
 *
 **********************************************************************/

static void
set_window_icon(void)
{
	gchar *icon_path = g_strdup_printf("%s/data/gui/Control-Panel-icon.png", getenv("CARMEN_HOME"));
	GdkPixbuf *icon = gdk_pixbuf_new_from_file(icon_path, NULL);

	if (icon != NULL)
	{
		gtk_window_set_icon(GTK_WINDOW(window), icon);
		g_object_unref(icon);
	}
	g_free(icon_path);
}


static void
parse_show_option(int argc, char **argv)
{
	/* Mesma opção do GUI Qt: -show modulo1 modulo2 ... liga a saída desses módulos */
	for (int i = 1; i < argc; i++)
	{
		if (strcmp("-show", argv[i]) == 0)
		{
			user_show_output_hash = g_hash_table_new(g_str_hash, g_str_equal);
			for (++i; i < argc && argv[i][0] != '-'; i++)
				g_hash_table_insert(user_show_output_hash, argv[i], GINT_TO_POINTER(1));
			i--;
		}
	}
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	gtk_init(&argc, &argv);

	parse_show_option(argc, argv);

	groups_hash = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, NULL);
	modules_hash = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, NULL);

	window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW(window), "PROCCONTROL GUI");
	gtk_container_set_border_width(GTK_CONTAINER(window), 2);
	set_window_icon();

	main_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
	gtk_container_add(GTK_CONTAINER(window), main_box);

	/* Área de saída, no rodapé — o Q3TextView do GUI original, em Courier 10 */
	text_buffer = gtk_text_buffer_new(NULL);
	text_view = gtk_text_view_new_with_buffer(text_buffer);
	gtk_text_view_set_editable(GTK_TEXT_VIEW(text_view), FALSE);
	gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(text_view), FALSE);
	gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(text_view), GTK_WRAP_WORD);
	apply_css(text_view, "textview { font-family: Courier; font-size: 10pt; }");

	scrolled_window = gtk_scrolled_window_new(NULL, NULL);
	gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
			GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
	gtk_container_add(GTK_CONTAINER(scrolled_window), text_view);
	gtk_box_pack_end(GTK_BOX(main_box), scrolled_window, TRUE, TRUE, 0);

	read_user_preferences(argc, argv);
	gtk_widget_show_all(window);
	set_user_preferences();

	g_signal_connect(window, "destroy", G_CALLBACK(gtk_shutdown_wrapper), NULL);
	signal(SIGINT, shutdown_procedure);

	carmen_proccontrol_subscribe_pidtable_message(NULL,
			(carmen_handler_t) carmen_proccontrol_pidtable_handler, CARMEN_SUBSCRIBE_ALL);

	g_timeout_add(IPC_DISPATCH_PERIOD_MS, ipc_dispatch_timeout, NULL);

	main_loop = g_main_loop_new(NULL, FALSE);
	g_main_loop_run(main_loop);

	return 0;
}
