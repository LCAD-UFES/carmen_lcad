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

#include <carmen/carmen_graphics.h>
#include <carmen/user_preferences.h>
#include <locale.h>
#include <libgen.h>

#include "playback_messages.h"
#include "playback_interface.h"


GtkWidget *window;
GdkGC *rrwd_gc, *rewind_gc, *stop_gc, *play_gc, *fwd_gc, *ffwd_gc;
GtkWidget *playback_speed_widget_label, *playback_speed_widget_status, *playback_speed_widget;
GtkWidget *playback_initial_time_widget_label, *playback_initial_time_widget_status, *playback_initial_time_widget;
GtkWidget *gtk_label_info_speed_value, *gtk_label_info_current_message_value, *gtk_label_info_timestamp_value, *gtk_label_info_timestamp_difference_value;
int speed_pending_update = 0;
int initial_time_pending_update = 0;
double playback_speed = 1.0;
char *playback_message = NULL;

#define FILE_FILTER_MSG_DEFAULT "bin/playback_filter_msg_default.txt"
#define MAX_PLAYBACK_MESSAGE_NUMBER 300

GtkWidget *filter_message_checkboxes[MAX_PLAYBACK_MESSAGE_NUMBER];
char *message_string_from_playback[MAX_PLAYBACK_MESSAGE_NUMBER];
gboolean checkbox_states[MAX_PLAYBACK_MESSAGE_NUMBER];
int quant_of_messages_from_playback = 0;

char *filter_msg[MAX_PLAYBACK_MESSAGE_NUMBER];	/* mensagens que comecam desligadas */
int filter_msg_on = -1;

char *user_pref_filename = NULL;
const char *user_pref_module;
user_param_t *user_pref_param_list;
int user_pref_num_items;
int user_pref_window_width  = -1;
int user_pref_window_height = -1;
int user_pref_window_x = -1;
int user_pref_window_y = -1;


void Redraw(GtkWidget *widget, GdkEventExpose *event, char *data);
void Send_Command(GtkWidget *widget, char *data);

// HiDPI: os tamanhos passados ao gtk_widget_set_usize() estao em pixels dimensionados para
// 96 DPI. O GTK2 nao tem suporte a HiDPI: ele escala a fonte pelo gtk-xft-dpi, mas nao esses
// valores. Numa tela 4K com a sessao em 200% (gtk-xft-dpi = 192) a fonte dobra e nao cabe mais
// nas caixas, deixando os rotulos truncados e sobrepostos ("Current M< Spe Tim Timesta").
// Escalamos pelo mesmo fator que o GTK ja' aplica na fonte; a 96 DPI o fator e' 1.0 e nada
// muda (caso do PC do veiculo). Mesma solucao que o astro usa.
static gint
scale_dpi(gint value)
{
    static double factor = -1.0;

    if (factor < 0.0)
    {
        gint xft_dpi = 0;

        g_object_get(gtk_settings_get_default(), "gtk-xft-dpi", &xft_dpi, NULL);
        factor = (xft_dpi > 0) ? ((xft_dpi / 1024.0) / 96.0) : 1.0;

        if (factor < 1.0)
            factor = 1.0;
    }

    return ((gint) (value * factor + 0.5));
}


static void
set_usize_dpi(GtkWidget *widget, gint width, gint height)
{
    gtk_widget_set_usize(widget, scale_dpi(width), scale_dpi(height));
}


static void
delete_event(GtkWidget *widget, GdkEvent *event, gpointer data)
{
    widget = widget;
    event = event;
    data = data;

    gtk_main_quit();
}

static void
speed_changed(GtkWidget *w, gpointer data __attribute__((unused)))
{
    //char *value;

    /*value =*/ gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);

    speed_pending_update++;
    if (speed_pending_update > 0)
        gtk_label_set_pattern(GTK_LABEL(playback_speed_widget_label),
                              "___________________________________________");
    else
        gtk_label_set_pattern(GTK_LABEL(playback_speed_widget_label), "");
}

static void
initial_time_changed(GtkWidget *w, gpointer data __attribute__((unused)))
{
    //char *value;

    /*value = */gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);

    initial_time_pending_update++;
    if (initial_time_pending_update > 0)
        gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label),
                              "___________________________________________");
    else
        gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
}

static gboolean
speed_params_save(GtkWidget *w, // __attribute__ ((unused)),
                  GdkEvent *event, gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);

        double speed_val;

        if (carmen_playback_is_valid_speed(value, &speed_val))
        {
        	playback_speed = speed_val;
			speed_pending_update = 0;
			gtk_label_set_pattern(GTK_LABEL(playback_speed_widget_label), "");
			gtk_label_set_text(GTK_LABEL(playback_speed_widget_status), "");
			carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_SPEED, NULL, 0, playback_speed);
			return TRUE;
        }
        gtk_label_set_text(GTK_LABEL(playback_speed_widget_status), " (ERROR)");
    }
    return FALSE;
}

static gboolean
initial_time_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
    	playback_message = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);

    	int start_msg, stop_msg;
    	double start_ts, stop_ts, recur_start_ts, recur_stop_ts, start_x, start_y, stop_x, stop_y, radius;

        if (carmen_playback_is_valid_message(playback_message, &start_msg, &stop_msg, &start_ts, &stop_ts, &recur_start_ts, &recur_stop_ts, &start_x, &start_y, &stop_x, &stop_y, &radius))
        {
			initial_time_pending_update = 0;
			gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
			gtk_label_set_text(GTK_LABEL(playback_initial_time_widget_status), "");
			carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_MESSAGE, playback_message, 0, playback_speed);
			return TRUE;
        }
        gtk_label_set_text(GTK_LABEL(playback_initial_time_widget_status), " (ERROR)");
    }
    return FALSE;
}

static void
updateIPC(gpointer data __attribute__ ((unused)), gint source __attribute__ ((unused)), GdkInputCondition condition __attribute__ ((unused)))
{
    carmen_ipc_sleep(0.01);
    carmen_graphics_update_ipc_callbacks(updateIPC);
}

/*
 * Filtro de mensagens (equivalente ao do astro). O playback publica na info message a lista
 * das tags que existem no log; aqui elas viram caixas de selecao, e o que ficar marcado
 * volta para o playback no comando SET_ACTIVATE_MESSAGE.
 */

static void
split_and_store_words(const char *input_string)
{
    char *copy = strdup(input_string);
    char *token = strtok(copy, " ");

    while (token != NULL && quant_of_messages_from_playback < MAX_PLAYBACK_MESSAGE_NUMBER)
    {
        message_string_from_playback[quant_of_messages_from_playback] = strdup(token);
        checkbox_states[quant_of_messages_from_playback] = TRUE;
        quant_of_messages_from_playback++;
        token = strtok(NULL, " ");
    }

    free(copy);
}


static int
set_active_message_state(const char *message, gboolean state)
{
    for (int i = 0; i < quant_of_messages_from_playback; i++)
    {
        if (strcmp(message, message_string_from_playback[i]) == 0)
        {
            checkbox_states[i] = state;
            return (1);
        }
    }

    return (0);
}


static void
filter_messages_by_user_settings()
{
    for (int i = 0; i < filter_msg_on; i++)
        set_active_message_state(filter_msg[i], FALSE);
}


static void
send_active_message_command()
{
    char *selected_words_string = NULL;
    int num_selected_words = 0;

    for (int i = 0; i < quant_of_messages_from_playback; i++)
    {
        if (!checkbox_states[i])
            continue;

        char *previous = selected_words_string;
        selected_words_string = g_strdup_printf("%s %s", previous ? previous : "", message_string_from_playback[i]);
        g_free(previous);
        num_selected_words++;
    }

    /* Uma lista vazia desligaria tudo; nesse caso o playback fica como esta'. */
    if (num_selected_words > 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_ACTIVATE_MESSAGE, selected_words_string, 0, playback_speed);

    g_free(selected_words_string);
}


static int
save_current_filter_msg()
{
    char *carmen_home = getenv("CARMEN_HOME");
    char filter_file_name[512];
    FILE *file_w;
    int lines = 0;

    if (quant_of_messages_from_playback == 0)
        return (0);	/* o playback nem chegou a mandar a lista: nao apaga o arquivo que existe */

    if (carmen_home == NULL)
    {
        fprintf(stderr, "Error environment variable \"CARMEN_HOME\" not defined.\n");
        return (-1);
    }

    snprintf(filter_file_name, sizeof(filter_file_name), "%s/%s", carmen_home, FILE_FILTER_MSG_DEFAULT);
    file_w = fopen(filter_file_name, "w");

    if (file_w == NULL)
    {
        fprintf(stderr, "Error trying to open file \"%s\" for writing.\n", filter_file_name);
        return (-1);
    }

    fprintf(file_w, "# Mensagens que o playback_control desliga ao subir. Uma por linha, seguida de \"off\".\n");

    for (int i = 0; i < quant_of_messages_from_playback; i++)
    {
        if (!checkbox_states[i])
        {
            fprintf(file_w, "%s\t\toff\n", message_string_from_playback[i]);
            lines++;
        }
    }

    fclose(file_w);

    return (lines);
}


static int
load_filter_msg(const char *file_name, char **buffer)
{
    char *carmen_home = getenv("CARMEN_HOME");
    char filter_file_name[512];
    const char *filter_file = file_name;
    char line[512];
    FILE *file_r;
    int index_filter_msg = 0;

    if (filter_file == NULL)
    {
        if (carmen_home == NULL)
        {
            fprintf(stderr, "Error environment variable \"CARMEN_HOME\" not defined.\n");
            return (-1);
        }

        snprintf(filter_file_name, sizeof(filter_file_name), "%s/%s", carmen_home, FILE_FILTER_MSG_DEFAULT);
        filter_file = (const char *) filter_file_name;
    }

    file_r = fopen(filter_file, "r");

    if (file_r == NULL)
    {
        fprintf(stderr, "Error trying to open file \"%s\" for reading.\n", filter_file);
        return (-1);
    }

    while (fgets(line, sizeof(line), file_r) != NULL)
    {
        char *token;

        line[strcspn(line, "\n")] = '\0';
        token = strtok(line, " \t\r");

        if (token == NULL || token[0] == '#')
            continue;

        char *message = strdup(token);
        carmen_test_alloc(message);

        token = strtok(NULL, " \t\r");

        if (token != NULL && strcmp(token, "off") == 0 && index_filter_msg < MAX_PLAYBACK_MESSAGE_NUMBER)
            buffer[index_filter_msg++] = message;
        else
            free(message);
    }

    fclose(file_r);

    return (index_filter_msg);
}


static void
on_filter_message_checkbox_toggled(GtkToggleButton *button, gpointer user_data)
{
    checkbox_states[GPOINTER_TO_INT(user_data)] = gtk_toggle_button_get_active(button);
}


static void
on_filter_message_apply_clicked(GtkButton *button __attribute__ ((unused)), gpointer user_data)
{
    send_active_message_command();
    gtk_widget_destroy(GTK_WIDGET(user_data));
}


static void
on_filter_message_all_clicked(GtkButton *button __attribute__ ((unused)), gpointer user_data)
{
    gboolean state = GPOINTER_TO_INT(user_data);

    for (int i = 0; i < quant_of_messages_from_playback; i++)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(filter_message_checkboxes[i]), state);
}


static void
create_filter_message_window()
{
    GtkWidget *filter_window, *vbox, *scrolled, *checkbox_vbox, *button_hbox, *apply, *all, *none;

    if (quant_of_messages_from_playback == 0)
    {
        fprintf(stderr, "playback_control: o playback ainda nao informou as mensagens do log.\n");
        return;
    }

    filter_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(filter_window), "Filter Message");
    gtk_window_set_position(GTK_WINDOW(filter_window), GTK_WIN_POS_CENTER);
    gtk_window_set_transient_for(GTK_WINDOW(filter_window), GTK_WINDOW(window));
    gtk_window_set_default_size(GTK_WINDOW(filter_window), scale_dpi(420), scale_dpi(600));

    vbox = gtk_vbox_new(FALSE, 5);
    gtk_container_add(GTK_CONTAINER(filter_window), vbox);

    /* Um log de audit passa de 40 tipos de mensagem: sem rolagem a janela nao cabe na tela. */
    scrolled = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled), GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_box_pack_start(GTK_BOX(vbox), scrolled, TRUE, TRUE, 0);

    checkbox_vbox = gtk_vbox_new(FALSE, 0);
    gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(scrolled), checkbox_vbox);

    for (int i = 0; i < quant_of_messages_from_playback; i++)
    {
        filter_message_checkboxes[i] = gtk_check_button_new_with_label(message_string_from_playback[i]);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(filter_message_checkboxes[i]), checkbox_states[i]);
        gtk_signal_connect(GTK_OBJECT(filter_message_checkboxes[i]), "toggled",
                           GTK_SIGNAL_FUNC(on_filter_message_checkbox_toggled), GINT_TO_POINTER(i));
        gtk_box_pack_start(GTK_BOX(checkbox_vbox), filter_message_checkboxes[i], FALSE, FALSE, 0);
    }

    button_hbox = gtk_hbox_new(FALSE, 5);
    gtk_box_pack_start(GTK_BOX(vbox), button_hbox, FALSE, FALSE, 5);

    all = gtk_button_new_with_label("Todas");
    gtk_signal_connect(GTK_OBJECT(all), "clicked", GTK_SIGNAL_FUNC(on_filter_message_all_clicked), GINT_TO_POINTER(TRUE));
    gtk_box_pack_start(GTK_BOX(button_hbox), all, TRUE, TRUE, 5);

    none = gtk_button_new_with_label("Nenhuma");
    gtk_signal_connect(GTK_OBJECT(none), "clicked", GTK_SIGNAL_FUNC(on_filter_message_all_clicked), GINT_TO_POINTER(FALSE));
    gtk_box_pack_start(GTK_BOX(button_hbox), none, TRUE, TRUE, 5);

    apply = gtk_button_new_with_label("Filtrar");
    gtk_signal_connect(GTK_OBJECT(apply), "clicked", GTK_SIGNAL_FUNC(on_filter_message_apply_clicked), filter_window);
    gtk_box_pack_start(GTK_BOX(button_hbox), apply, TRUE, TRUE, 5);

    gtk_widget_show_all(filter_window);
}


static void
on_filter_message_button_clicked(GtkButton *button __attribute__ ((unused)), gpointer user_data __attribute__ ((unused)))
{
    create_filter_message_window();
}


char playback_info_message_number_string[256];
char playback_info_message_timestamp_string[256];
char playback_info_message_timestamp_difference_string[256];
char playback_info_message_playback_speed_string[256];

void
carmen_playback_info_message_handler(carmen_playback_info_message *message)
{
    //	static int flag = 0;

    //	printf ("MSG: %d %lf %lf %lf\n",
    //		message->message_number,
    //		message->message_timestamp,
    //		message->message_timestamp_difference,
    //		message->playback_speed
    //	);

    sprintf(playback_info_message_number_string, "%07d", message->message_number);
    sprintf(playback_info_message_playback_speed_string, "%.2lf", message->playback_speed);
    sprintf(playback_info_message_timestamp_string, "%05.2lf", message->message_timestamp);
    sprintf(playback_info_message_timestamp_difference_string, "%lf", message->message_timestamp_difference);

    if (quant_of_messages_from_playback == 0 && message->all_messages_tag != NULL && message->all_messages_tag[0] != '\0')
    {
        split_and_store_words(message->all_messages_tag);
        filter_messages_by_user_settings();
        send_active_message_command();
    }

    gtk_label_set_text((GtkLabel *) gtk_label_info_current_message_value, playback_info_message_number_string);
    gtk_label_set_text((GtkLabel *) gtk_label_info_speed_value, playback_info_message_playback_speed_string);
    gtk_label_set_text((GtkLabel *) gtk_label_info_timestamp_value, playback_info_message_timestamp_string);
    gtk_label_set_text((GtkLabel *) gtk_label_info_timestamp_difference_value, playback_info_message_timestamp_difference_string);

    //	if ((message->message_number > 60000) && (flag == 0))
    //	{
    //		carmen_playback_command(CARMEN_PLAYBACK_COMMAND_STOP, 0, playback_speed);
    //		flag = 1;
    //	}
}

GdkPixbuf *
create_pixbuf(const gchar * filename)
{
    GdkPixbuf *pixbuf;
    GError *error = NULL;
    pixbuf = gdk_pixbuf_new_from_file(filename, &error);
    if (!pixbuf)
    {
        fprintf(stderr, "%s\n", error->message);
        g_error_free(error);
    }

    return pixbuf;
}


void
read_parameters(int argc, char *argv[])
{
	char *speed = NULL;
	char *message = NULL;
	char *file_filter_msg = NULL;
	int autostart = 0;

	carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline",	(char *) "speed",		CARMEN_PARAM_STRING,	&(speed),		0, NULL},
		{(char *) "commandline",	(char *) "message",		CARMEN_PARAM_STRING, 	&(message),		0, NULL},
		{(char *) "commandline",	(char *) "autostart",	CARMEN_PARAM_ONOFF, 	&(autostart),	0, NULL},
		{(char *) "commandline",	(char *) "filter_msg",	CARMEN_PARAM_STRING, 	&(file_filter_msg), 0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	// Before start publishing playback commands, wait for a while until playback module is up and running
	for (int i = 0; i < 20; i++)
	{
		if (IPC_numHandlers(CARMEN_PLAYBACK_COMMAND_NAME) > 0)
			break;
		usleep(0.2 * 1e6);
	}

	if (speed)
	{
	    gtk_entry_set_text(GTK_ENTRY(playback_speed_widget), speed);

	    double speed_val;

        if (carmen_playback_is_valid_speed(speed, &speed_val))
        {
        	playback_speed = speed_val;
			gtk_label_set_pattern(GTK_LABEL(playback_speed_widget_label), "");
			carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_SPEED, NULL, 0, playback_speed);
        }
        else
        	gtk_label_set_text(GTK_LABEL(playback_speed_widget_status), " (ERROR)");
	}

    if (message)
    {
        gtk_entry_set_text(GTK_ENTRY(playback_initial_time_widget), message);

    	int start_msg, stop_msg;
    	double start_ts, stop_ts, recur_start_ts, recur_stop_ts, start_x, start_y, stop_x, stop_y, radius;

        if (carmen_playback_is_valid_message(message, &start_msg, &stop_msg, &start_ts, &stop_ts, &recur_start_ts, &recur_stop_ts, &start_x, &start_y, &stop_x, &stop_y, &radius))
        {
        	playback_message = message;
			gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
			carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_MESSAGE, playback_message, 0, playback_speed);
        }
        else
        	gtk_label_set_text(GTK_LABEL(playback_initial_time_widget_status), " (ERROR)");
    }

    if (autostart)
    	carmen_playback_command(CARMEN_PLAYBACK_COMMAND_PLAY, NULL, 0, playback_speed);

    // -filter_msg default usa bin/playback_filter_msg_default.txt, que e' o mesmo arquivo
    // que o playback_control regrava ao sair. Sem a opcao, nada comeca filtrado.
    if (file_filter_msg)
    {
        if (strcmp(file_filter_msg, "default") == 0)
            filter_msg_on = load_filter_msg(NULL, filter_msg);
        else if (access(file_filter_msg, F_OK) != -1)
            filter_msg_on = load_filter_msg(file_filter_msg, filter_msg);
        else
        {
            fprintf(stderr, "error trying to open filter_msg file \"%s\", loading default file.\n", file_filter_msg);
            filter_msg_on = load_filter_msg(NULL, filter_msg);
        }
    }
}


void
read_user_preferences(int argc, char** argv)
{
	static user_param_t param_list[] =
	{
		{"window_width",     USER_PARAM_TYPE_INT,    &user_pref_window_width},
		{"window_height",    USER_PARAM_TYPE_INT,    &user_pref_window_height},
		{"window_x",         USER_PARAM_TYPE_INT,    &user_pref_window_x},
		{"window_y",         USER_PARAM_TYPE_INT,    &user_pref_window_y},
	};
	user_pref_module = basename(argv[0]);
	user_pref_param_list = param_list;
	user_pref_num_items = sizeof(param_list) / sizeof(param_list[0]);
	user_preferences_read(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
	user_preferences_read_commandline(argc, argv, user_pref_param_list, user_pref_num_items);
}


void
set_user_preferences()
{
	if (user_pref_window_width > 0 && user_pref_window_height > 0)
		gtk_window_resize(GTK_WINDOW(window), user_pref_window_width, user_pref_window_height);
	if (user_pref_window_x >= 0 && user_pref_window_y >= 0)
		gtk_window_move(GTK_WINDOW(window), user_pref_window_x, user_pref_window_y);
}


void
save_user_preferences()
{
	gtk_window_get_size(GTK_WINDOW(window), &user_pref_window_width, &user_pref_window_height);
	gtk_window_get_position(GTK_WINDOW(window), &user_pref_window_x, &user_pref_window_y);
	user_preferences_save(user_pref_filename, user_pref_module, user_pref_param_list, user_pref_num_items);
}


static void
shutdown(int sig __attribute__ ((unused)))
{
	save_current_filter_msg();
	save_user_preferences();
	carmen_ipc_disconnect();
	exit(1);
}


void usage(char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	fprintf(stderr, " [args]:\n"
			        "\t-speed <value>               speed option (default: 1.0)\n"
			        "\t-autostart on|off            auto start option (default: off)\n"
					"\t-message <option>            message play:stop option (default: 0) Can be used with any runtime option. Ex: -message \"t <init_time>\"\n"
					"\t-filter_msg <file>|default   messages to start filtered out (default file: $CARMEN_HOME/" FILE_FILTER_MSG_DEFAULT ")\n");

	fprintf(stderr, "\n Message play:stop runtime options:\n");
	fprintf(stderr, "\tplay from message number:    <num>\n");
	fprintf(stderr, "\tstop at message number:      :<num>\n");
	fprintf(stderr, "\tplay:stop message numbers:   <num>:<num>\n");
	fprintf(stderr, "\tplay from time (s):          t <num>\n");
	fprintf(stderr, "\tstop at time (s):            t :<num>\n");
	fprintf(stderr, "\tplay:stop times (s):         t <num>:<num>\n");
	fprintf(stderr, "\trecur play from time (s):    r <num>\n");
	fprintf(stderr, "\trecur stop at time (s):      r :<num>\n");
	fprintf(stderr, "\trecur play:stop times (s):   r <num>:<num>\n");
	fprintf(stderr, "\tplay from pose:              p <x> <y>\n");
	fprintf(stderr, "\tstop at pose:                p :<x> <y>\n");
	fprintf(stderr, "\tplay:stop poses:             p <x> <y>:<x> <y>\n");
	fprintf(stderr, "\tpose search radius (m):      p ::<num>\n");
	fprintf(stderr, "\t                             p <x> <y>::<num>\n");
	fprintf(stderr, "\t                             p :<x> <y>:<num>\n");
	fprintf(stderr, "\t                             p <x> <y>:<x> <y>:<num>\n");
	exit(-1);
}


int
main(int argc, char *argv[])
{
    GdkColor Red, Green, Blue;
    GdkColormap *cmap;
    GtkWidget *hbox, *rrwd, *rwd, *play, *stop, *ffwd, *fwd, *reset_button, *filter_button, *vbox, *hbox2;
    GtkWidget *rrwd_darea, *rwd_darea, *stop_darea, *play_darea,
            *ffwd_darea, *fwd_darea, *reset_darea, *filter_darea;

	if (argc > 1 && strcmp(argv[1], "-h") == 0)
		usage("%s [args]\n", argv[0]);

    gtk_init(&argc, &argv);

    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown);

    carmen_subscribe_playback_info_message(NULL,
                                           (carmen_handler_t) carmen_playback_info_message_handler,
                                           CARMEN_SUBSCRIBE_LATEST);

    cmap = gdk_colormap_get_system();

    gdk_color_parse("red", &Red);
    if (!gdk_color_alloc(cmap, &Red))
    {
        g_error("couldn't allocate color");
    }

    gdk_color_parse("blue", &Blue);
    if (!gdk_color_alloc(cmap, &Blue))
    {
        g_error("couldn't allocate color");
    }

    gdk_color_parse("green", &Green);
    if (!gdk_color_alloc(cmap, &Green))
    {
        g_error("couldn't allocate color");
    }

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    // Sem tamanho forcado: o gtk_widget_set_usize() daqui era tamanho MINIMO, nao inicial --
    // impedia encolher a janela, e em HiDPI (890 x 2) sobrava um palmo de espaco vazio a'
    // direita. Deixando a janela se ajustar ao conteudo, ela nasce do tamanho exato dos
    // widgets, em qualquer DPI, e pode ser redimensionada a' vontade.

    gtk_signal_connect(GTK_OBJECT(window), "destroy",
                       GTK_SIGNAL_FUNC(gtk_main_quit),
                       "WM destroy");

    gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                       GTK_SIGNAL_FUNC(delete_event), NULL);

    gtk_window_set_title(GTK_WINDOW(window), "playback control");
    char *aux = getenv("CARMEN_HOME");
    strcat(aux, "/data/gui/playback.png");
    gtk_window_set_icon(GTK_WINDOW(window), create_pixbuf(aux));
    gtk_widget_realize(window);

    vbox = gtk_vbox_new(0, 0);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 2);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    hbox = gtk_hbox_new(0, 0);
    gtk_container_set_border_width(GTK_CONTAINER(hbox), 2);
    gtk_container_add(GTK_CONTAINER(vbox), hbox);

    hbox2 = gtk_hbox_new(0, 0);
    gtk_container_set_border_width(GTK_CONTAINER(hbox2), 2);
    gtk_container_add(GTK_CONTAINER(vbox), hbox2);

    playback_speed_widget_label = gtk_label_new("Speed");
    playback_speed_widget_status = gtk_label_new("");
    playback_speed_widget = gtk_entry_new_with_max_length(5);
    gtk_entry_set_text(GTK_ENTRY(playback_speed_widget), "1.0");
    gtk_editable_select_region(GTK_EDITABLE(playback_speed_widget), 0, GTK_ENTRY(playback_speed_widget)->text_length);
    gtk_signal_connect(GTK_OBJECT(playback_speed_widget), "changed",
                       GTK_SIGNAL_FUNC(speed_changed), NULL);
    gtk_signal_connect(GTK_OBJECT(playback_speed_widget), "key_press_event",
                       GTK_SIGNAL_FUNC(speed_params_save), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), playback_speed_widget_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), playback_speed_widget_status, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), playback_speed_widget, FALSE, FALSE, 5);
    set_usize_dpi(playback_speed_widget, 58, 26);
    gtk_widget_show(playback_speed_widget);
    gtk_widget_show(playback_speed_widget_label);
    gtk_widget_show(playback_speed_widget_status);

    playback_initial_time_widget_label = gtk_label_new("Message play:stop");
    playback_initial_time_widget_status = gtk_label_new("");
    playback_initial_time_widget = gtk_entry_new_with_max_length(60);
    gtk_entry_set_text(GTK_ENTRY(playback_initial_time_widget), "0");
    gtk_editable_select_region(GTK_EDITABLE(playback_initial_time_widget), 0, GTK_ENTRY(playback_initial_time_widget)->text_length);
    gtk_signal_connect(GTK_OBJECT(playback_initial_time_widget), "changed",
                       GTK_SIGNAL_FUNC(initial_time_changed), NULL);
    gtk_signal_connect(GTK_OBJECT(playback_initial_time_widget), "key_press_event",
                       GTK_SIGNAL_FUNC(initial_time_params_save), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), playback_initial_time_widget_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), playback_initial_time_widget_status, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), playback_initial_time_widget, FALSE, FALSE, 5);
    set_usize_dpi(playback_initial_time_widget, 110, 26);
    gtk_widget_show(playback_initial_time_widget);
    gtk_widget_show(playback_initial_time_widget_label);
    gtk_widget_show(playback_initial_time_widget_status);

    rrwd = gtk_button_new();
    rrwd_darea = gtk_drawing_area_new();
    set_usize_dpi(rrwd_darea, 26, 26);
    rrwd_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(rrwd_gc, &Blue);
    gdk_gc_set_line_attributes(rrwd_gc, 2, GDK_LINE_SOLID,
                               GDK_CAP_BUTT, GDK_JOIN_MITER);
    gtk_signal_connect(GTK_OBJECT(rrwd_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "RRW");
    gtk_container_add(GTK_CONTAINER(rrwd), rrwd_darea);
    gtk_box_pack_start(GTK_BOX(hbox), rrwd, FALSE, FALSE, 2);
    gtk_signal_connect(GTK_OBJECT(rrwd), "clicked",
                       (GtkSignalFunc) Send_Command, "RRW");

    rwd = gtk_button_new();
    rwd_darea = gtk_drawing_area_new();
    set_usize_dpi(rwd_darea, 26, 26);
    rewind_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(rewind_gc, &Blue);
    gdk_gc_set_line_attributes(rewind_gc, 2, GDK_LINE_SOLID,
                               GDK_CAP_BUTT, GDK_JOIN_MITER);
    gtk_signal_connect(GTK_OBJECT(rwd_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "RW");
    gtk_container_add(GTK_CONTAINER(rwd), rwd_darea);
    gtk_box_pack_start(GTK_BOX(hbox), rwd, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(rwd), "clicked",
                       (GtkSignalFunc) Send_Command, "RW");

    stop = gtk_button_new();
    stop_darea = gtk_drawing_area_new();
    set_usize_dpi(stop_darea, 26, 26);
    stop_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(stop_gc, &Red);
    gtk_signal_connect(GTK_OBJECT(stop_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "Stop");
    gtk_container_add(GTK_CONTAINER(stop), stop_darea);
    gtk_box_pack_start(GTK_BOX(hbox), stop, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(stop), "clicked",
                       (GtkSignalFunc) Send_Command, "Stop");

    play = gtk_button_new();
    play_darea = gtk_drawing_area_new();
    set_usize_dpi(play_darea, 26, 26);
    play_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(play_gc, &Green);
    gtk_signal_connect(GTK_OBJECT(play_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "Play");
    gtk_container_add(GTK_CONTAINER(play), play_darea);
    gtk_box_pack_start(GTK_BOX(hbox), play, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(play), "clicked",
                       (GtkSignalFunc) Send_Command, "Play");

    fwd = gtk_button_new();
    fwd_darea = gtk_drawing_area_new();
    set_usize_dpi(fwd_darea, 26, 26);
    fwd_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(fwd_gc, &Blue);
    gtk_signal_connect(GTK_OBJECT(fwd_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "FW");
    gdk_gc_set_line_attributes(fwd_gc, 2, GDK_LINE_SOLID,
                               GDK_CAP_BUTT, GDK_JOIN_MITER);
    gtk_container_add(GTK_CONTAINER(fwd), fwd_darea);
    gtk_box_pack_start(GTK_BOX(hbox), fwd, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(fwd), "clicked",
                       (GtkSignalFunc) Send_Command, "FWD");

    ffwd = gtk_button_new();
    ffwd_darea = gtk_drawing_area_new();
    set_usize_dpi(ffwd_darea, 26, 26);
    ffwd_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(ffwd_gc, &Blue);
    gdk_gc_set_line_attributes(ffwd_gc, 2, GDK_LINE_SOLID,
                               GDK_CAP_BUTT, GDK_JOIN_MITER);
    gtk_signal_connect(GTK_OBJECT(ffwd_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "FFW");
    gtk_container_add(GTK_CONTAINER(ffwd), ffwd_darea);
    gtk_box_pack_start(GTK_BOX(hbox), ffwd, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(ffwd), "clicked",
                       (GtkSignalFunc) Send_Command, "FFWD");

    reset_button = gtk_button_new();
    reset_darea = gtk_drawing_area_new();
    set_usize_dpi(reset_darea, 26, 26);
    gtk_signal_connect(GTK_OBJECT(reset_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "RESET");
    gtk_container_add(GTK_CONTAINER(reset_button), reset_darea);
    gtk_box_pack_start(GTK_BOX(hbox), reset_button, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(reset_button), "clicked",
                       (GtkSignalFunc) Send_Command, "RESET");

    filter_button = gtk_button_new();
    filter_darea = gtk_drawing_area_new();
    set_usize_dpi(filter_darea, 26, 26);
    gtk_signal_connect(GTK_OBJECT(filter_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "FILTER");
    gtk_container_add(GTK_CONTAINER(filter_button), filter_darea);
    gtk_box_pack_start(GTK_BOX(hbox), filter_button, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(filter_button), "clicked",
                       (GtkSignalFunc) on_filter_message_button_clicked, NULL);

    GtkWidget *gtk_label_info_speed, *gtk_label_info_current_message, *gtk_label_info_timestamp, *gtk_label_info_timestamp_difference;

    gtk_label_info_current_message = gtk_label_new("Current Message:");
    gtk_label_info_current_message_value = gtk_label_new("");

    gtk_label_info_speed = gtk_label_new("Speed:");
    gtk_label_info_speed_value = gtk_label_new("");

    gtk_label_info_timestamp = gtk_label_new("Time:");
    gtk_label_info_timestamp_value = gtk_label_new("");

    gtk_label_info_timestamp_difference = gtk_label_new("Timestamp:");
    gtk_label_info_timestamp_difference_value = gtk_label_new("");

    // Os rotulos ficam com a largura natural (assim nunca truncam, em qualquer DPI); quem tem
    // largura fixa e' o VALOR, para que o numero mudando nao empurre o resto da linha.
    set_usize_dpi(gtk_label_info_current_message_value, 80, 22);
    set_usize_dpi(gtk_label_info_speed_value, 45, 22);
    set_usize_dpi(gtk_label_info_timestamp_value, 60, 22);
    set_usize_dpi(gtk_label_info_timestamp_difference_value, 145, 22);

    gtk_misc_set_alignment(GTK_MISC(gtk_label_info_current_message_value), 0.0, 0.5);
    gtk_misc_set_alignment(GTK_MISC(gtk_label_info_speed_value), 0.0, 0.5);
    gtk_misc_set_alignment(GTK_MISC(gtk_label_info_timestamp_value), 0.0, 0.5);
    gtk_misc_set_alignment(GTK_MISC(gtk_label_info_timestamp_difference_value), 0.0, 0.5);

    gtk_box_pack_start(GTK_BOX(hbox2), gtk_label_info_current_message, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox2), gtk_label_info_current_message_value, FALSE, FALSE, 5);

    gtk_box_pack_start(GTK_BOX(hbox2), gtk_label_info_speed, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox2), gtk_label_info_speed_value, FALSE, FALSE, 5);

    gtk_box_pack_start(GTK_BOX(hbox2), gtk_label_info_timestamp, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox2), gtk_label_info_timestamp_value, FALSE, FALSE, 5);

    gtk_box_pack_start(GTK_BOX(hbox2), gtk_label_info_timestamp_difference, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox2), gtk_label_info_timestamp_difference_value, FALSE, FALSE, 5);

    gtk_widget_show(gtk_label_info_current_message);
    gtk_widget_show(gtk_label_info_current_message_value);

    gtk_widget_show(gtk_label_info_speed);
    gtk_widget_show(gtk_label_info_speed_value);

    gtk_widget_show(gtk_label_info_timestamp);
    gtk_widget_show(gtk_label_info_timestamp_value);

    gtk_widget_show(gtk_label_info_timestamp_difference);
    gtk_widget_show(gtk_label_info_timestamp_difference_value);

	read_parameters(argc, argv);

	read_user_preferences(argc, argv);

    gtk_widget_show_all(window);

 	set_user_preferences();

    setlocale(LC_ALL, "C");

    carmen_graphics_update_ipc_callbacks(updateIPC);

    gtk_main();
    return 0;
}

void
Redraw(GtkWidget *widget __attribute__((unused)),
       GdkEventExpose *event __attribute__((unused)), char *data)
{
    int width, height;
    int mid_h, mid_v;
    int left, right, top, bottom;
    GdkPoint triangle[3];
    GdkPoint square[4];

    width = widget->allocation.width;
    height = widget->allocation.height;
    mid_h = width / 2;
    mid_v = height / 2;
    left = mid_h - 10;
    right = mid_h + 10;
    top = mid_v - 10;
    bottom = mid_v + 10;

    if (strcmp(data, "Play") == 0)
    {
        triangle[0].x = left;
        triangle[0].y = top;
        triangle[1].x = right;
        triangle[1].y = mid_v;
        triangle[2].x = left;
        triangle[2].y = bottom;
        gdk_draw_polygon(widget->window, play_gc, 1, triangle, 3);
    }
    else if (strcmp(data, "Stop") == 0)
    {
        square[0].x = left;
        square[0].y = top;
        square[1].x = right;
        square[1].y = top;
        square[2].x = right;
        square[2].y = bottom;
        square[3].x = left;
        square[3].y = bottom;
        gdk_draw_polygon(widget->window, stop_gc, 1, square, 4);
    }
    else if (strcmp(data, "RRW") == 0)
    {
        triangle[0].x = mid_h;
        triangle[0].y = top;
        triangle[1].x = left;
        triangle[1].y = mid_v;
        triangle[2].x = mid_h;
        triangle[2].y = bottom;
        gdk_draw_polygon(widget->window, rrwd_gc, 1, triangle, 3);
        triangle[0].x = right;
        triangle[0].y = top;
        triangle[1].x = mid_h;
        triangle[1].y = mid_v;
        triangle[2].x = right;
        triangle[2].y = bottom;
        gdk_draw_polygon(widget->window, rrwd_gc, 1, triangle, 3);
        gdk_draw_line(widget->window, rrwd_gc, left, top, left, bottom);
    }
    else if (strcmp(data, "RW") == 0)
    {
        triangle[0].x = mid_h;
        triangle[0].y = top;
        triangle[1].x = left;
        triangle[1].y = mid_v;
        triangle[2].x = mid_h;
        triangle[2].y = bottom;
        gdk_draw_polygon(widget->window, rewind_gc, 1, triangle, 3);
        gdk_draw_line(widget->window, rewind_gc, left, top, left, bottom);
    }
    else if (strcmp(data, "FFW") == 0)
    {
        triangle[0].x = left;
        triangle[0].y = top;
        triangle[1].x = mid_h;
        triangle[1].y = mid_v;
        triangle[2].x = left;
        triangle[2].y = bottom;
        gdk_draw_polygon(widget->window, ffwd_gc, 1, triangle, 3);
        triangle[0].x = mid_h;
        triangle[0].y = top;
        triangle[1].x = right;
        triangle[1].y = mid_v;
        triangle[2].x = mid_h;
        triangle[2].y = bottom;
        gdk_draw_polygon(widget->window, ffwd_gc, 1, triangle, 3);
        gdk_draw_line(widget->window, ffwd_gc, right, top, right, bottom);
    }
    else if (strcmp(data, "FW") == 0)
    {
        triangle[0].x = mid_h;
        triangle[0].y = top;
        triangle[1].x = right;
        triangle[1].y = mid_v;
        triangle[2].x = mid_h;
        triangle[2].y = bottom;
        gdk_draw_polygon(widget->window, fwd_gc, 1, triangle, 3);
        gdk_draw_line(widget->window, fwd_gc, right, top, right, bottom);
    }
    else if (strcmp(data, "RESET") == 0)
    {
        gdk_draw_line(widget->window, stop_gc, left, top, left, bottom);
        gdk_draw_line(widget->window, stop_gc, left, top, right, top);
        gdk_draw_line(widget->window, stop_gc, left, mid_v, right, mid_v);
        gdk_draw_line(widget->window, stop_gc, right, top, right, mid_v);
        gdk_draw_line(widget->window, stop_gc, left, mid_v, right, bottom);
    }
    else if (strcmp(data, "FILTER") == 0)
    {
        // Um funil: as duas diagonais de cima e a haste descendo do bico.
        gdk_draw_line(widget->window, play_gc, left, top, right, top);
        gdk_draw_line(widget->window, play_gc, left, top, mid_h - 2, mid_v);
        gdk_draw_line(widget->window, play_gc, right, top, mid_h + 2, mid_v);
        gdk_draw_line(widget->window, play_gc, mid_h - 2, mid_v, mid_h - 2, bottom);
        gdk_draw_line(widget->window, play_gc, mid_h + 2, mid_v, mid_h + 2, bottom);
        gdk_draw_line(widget->window, play_gc, mid_h - 2, bottom, mid_h + 2, bottom);
    }
}

void
Send_Command(GtkWidget *widget __attribute__((unused)), char *data)
{
    if (strcmp(data, "Stop") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_STOP, NULL, 0, playback_speed);
    else if (strcmp(data, "Play") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_PLAY, NULL, 0, playback_speed);
    else if (strcmp(data, "RRW") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_REWIND, NULL, 100, playback_speed);
    else if (strcmp(data, "RW") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_RWD_SINGLE, NULL, 1, playback_speed);
    else if (strcmp(data, "FFWD") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_FORWARD, NULL, 100, playback_speed);
    else if (strcmp(data, "FWD") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_FWD_SINGLE, NULL, 1, playback_speed);
    else if (strcmp(data, "RESET") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_RESET, NULL, 0, playback_speed);
}
