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
#include <locale.h>

#include "playback_messages.h"
#include "playback_interface.h"


GdkGC *rrwd_gc, *rewind_gc, *stop_gc, *play_gc, *fwd_gc, *ffwd_gc;
GtkWidget *playback_speed_widget_label, *playback_speed_widget;
GtkWidget *playback_initial_time_widget_label, *playback_initial_time_widget;
GtkWidget *gtk_label_info_speed_value, *gtk_label_info_current_message_value, *gtk_label_info_timestamp_value, *gtk_label_info_timestamp_difference_value;
int speed_pending_update = 0;
int initial_time_pending_update = 0;
double playback_speed = 1.0;
int playback_initial_time = 0.0;


void Redraw(GtkWidget *widget, GdkEventExpose *event, char *data);
void Send_Command(GtkWidget *widget, char *data);

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
    char *possible_comma;

    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);

        possible_comma = strchr(value, ','); // para o caso de alguem digitar virgula ao inves de ponto na velocidade
        if (possible_comma != NULL)
            possible_comma[0] = '.';

        playback_speed = atof(value);

        speed_pending_update = 0;
        gtk_label_set_pattern(GTK_LABEL(playback_speed_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_SPEED,
                                0, playback_speed);
        return TRUE;
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
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        playback_initial_time = atoi(value);

        initial_time_pending_update = 0;
        gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_INITIAL_TIME,
                                playback_initial_time, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gint
updateIPC(gpointer *data __attribute__((unused)))
{
    carmen_ipc_sleep(0.01);
    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);
    return 1;
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

int
main(int argc, char *argv[])
{
    GdkColor Red, Green, Blue;
    GdkColormap *cmap;
    GtkWidget *window;
    GtkWidget *hbox, *rrwd, *rwd, *play, *stop, *ffwd, *fwd, *reset_button, *vbox, *hbox2;
    GtkWidget *rrwd_darea, *rwd_darea, *stop_darea, *play_darea,
            *ffwd_darea, *fwd_darea, *reset_darea;

    gtk_init(&argc, &argv);

    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

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
    gtk_widget_set_usize(window, 720, 100);

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
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 5);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    hbox = gtk_hbox_new(0, 0);
    gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
    gtk_container_add(GTK_CONTAINER(vbox), hbox);

    hbox2 = gtk_hbox_new(0, 0);
    gtk_container_set_border_width(GTK_CONTAINER(hbox2), 5);
    gtk_container_add(GTK_CONTAINER(vbox), hbox2);

    playback_speed_widget_label = gtk_label_new("Speed");
    playback_speed_widget = gtk_entry_new_with_max_length(5);
    gtk_entry_set_text(GTK_ENTRY(playback_speed_widget), "1.0");
    gtk_editable_select_region(GTK_EDITABLE(playback_speed_widget), 0, GTK_ENTRY(playback_speed_widget)->text_length);
    gtk_signal_connect(GTK_OBJECT(playback_speed_widget), "changed",
                       GTK_SIGNAL_FUNC(speed_changed), NULL);
    gtk_signal_connect(GTK_OBJECT(playback_speed_widget), "key_press_event",
                       GTK_SIGNAL_FUNC(speed_params_save), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), playback_speed_widget_label,
                       FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), playback_speed_widget, FALSE, FALSE, 5);
    gtk_widget_set_usize(playback_speed_widget, 50, 30);
    gtk_widget_show(playback_speed_widget);
    gtk_widget_show(playback_speed_widget_label);

    playback_initial_time_widget_label = gtk_label_new("Initial Message");
    playback_initial_time_widget = gtk_entry_new_with_max_length(9);
    gtk_entry_set_text(GTK_ENTRY(playback_initial_time_widget), "0");
    gtk_editable_select_region(GTK_EDITABLE(playback_initial_time_widget), 0, GTK_ENTRY(playback_initial_time_widget)->text_length);
    gtk_signal_connect(GTK_OBJECT(playback_initial_time_widget), "changed",
                       GTK_SIGNAL_FUNC(initial_time_changed), NULL);
    gtk_signal_connect(GTK_OBJECT(playback_initial_time_widget), "key_press_event",
                       GTK_SIGNAL_FUNC(initial_time_params_save), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), playback_initial_time_widget_label,
                       FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), playback_initial_time_widget, FALSE, FALSE, 5);
    gtk_widget_set_usize(playback_initial_time_widget, 100, 30);
    gtk_widget_show(playback_initial_time_widget);
    gtk_widget_show(playback_initial_time_widget_label);

    rrwd = gtk_button_new();
    rrwd_darea = gtk_drawing_area_new();
    gtk_widget_set_usize(rrwd_darea, 30, 40);
    rrwd_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(rrwd_gc, &Blue);
    gdk_gc_set_line_attributes(rrwd_gc, 2, GDK_LINE_SOLID,
                               GDK_CAP_BUTT, GDK_JOIN_MITER);
    gtk_signal_connect(GTK_OBJECT(rrwd_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "RRW");
    gtk_container_add(GTK_CONTAINER(rrwd), rrwd_darea);
    gtk_box_pack_start(GTK_BOX(hbox), rrwd, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(rrwd), "clicked",
                       (GtkSignalFunc) Send_Command, "RRW");

    rwd = gtk_button_new();
    rwd_darea = gtk_drawing_area_new();
    gtk_widget_set_usize(rwd_darea, 30, 40);
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
    gtk_widget_set_usize(stop_darea, 30, 40);
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
    gtk_widget_set_usize(play_darea, 30, 40);
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
    gtk_widget_set_usize(fwd_darea, 30, 40);
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
    gtk_widget_set_usize(ffwd_darea, 30, 40);
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
    gtk_widget_set_usize(reset_darea, 30, 40);
    gtk_signal_connect(GTK_OBJECT(reset_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "RESET");
    gtk_container_add(GTK_CONTAINER(reset_button), reset_darea);
    gtk_box_pack_start(GTK_BOX(hbox), reset_button, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(reset_button), "clicked",
                       (GtkSignalFunc) Send_Command, "RESET");

    GtkWidget *gtk_label_info_speed, *gtk_label_info_current_message, *gtk_label_info_timestamp, *gtk_label_info_timestamp_difference;

    gtk_label_info_current_message = gtk_label_new("Current Message:");
    gtk_label_info_current_message_value = gtk_label_new("");

    gtk_label_info_speed = gtk_label_new("Speed:");
    gtk_label_info_speed_value = gtk_label_new("");

    gtk_label_info_timestamp = gtk_label_new("Time:");
    gtk_label_info_timestamp_value = gtk_label_new("");

    gtk_label_info_timestamp_difference = gtk_label_new("Timestamp:");
    gtk_label_info_timestamp_difference_value = gtk_label_new("");

    gtk_widget_set_usize(gtk_label_info_current_message, 140, 30);
    gtk_widget_set_usize(gtk_label_info_speed, 50, 30);
    gtk_widget_set_usize(gtk_label_info_timestamp, 40, 30);
    gtk_widget_set_usize(gtk_label_info_timestamp_difference, 100, 30);

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

    gtk_widget_show_all(window);

    setlocale(LC_ALL, "C");

    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);

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
}

void
Send_Command(GtkWidget *widget __attribute__((unused)), char *data)
{
    if (strcmp(data, "Stop") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_STOP, 0, playback_speed);
    else if (strcmp(data, "Play") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_PLAY, 0, playback_speed);
    else if (strcmp(data, "RRW") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_REWIND, 100,
                                playback_speed);
    else if (strcmp(data, "RW") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_RWD_SINGLE, 1,
                                playback_speed);
    else if (strcmp(data, "FFWD") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_FORWARD, 100,
                                playback_speed);
    else if (strcmp(data, "FWD") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_FWD_SINGLE, 1,
                                playback_speed);
    else if (strcmp(data, "RESET") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_RESET, 0, playback_speed);
}
