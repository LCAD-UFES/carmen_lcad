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

GdkGC *rrwd_gc, *rewind_gc, *stop_gc, *play_gc, *fwd_gc, *ffwd_gc, *showPoints_gc, *calibration_gc;
GtkWidget *playback_speed_widget_label, *playback_speed_widget;
GtkWidget *playback_initial_time_widget_label, *playback_initial_time_widget;
GtkWidget  *playback_final_time_widget_label, *playback_final_time_widget;
GtkWidget *gtk_label_info_speed_value, *gtk_label_info_current_message_value, *gtk_label_info_timestamp_value, *gtk_label_info_timestamp_difference_value;

int speed_pending_update = 0;
int initial_time_pending_update = 0;
double playback_speed = 1.0;
int playback_initial_time = 0.0;
int playback_final_time = 0.0;

int number_of_messages_v1 = 1;
int number_of_messages_v2 = 1;
float cx1 = 0, cy1 = 0, cz1 = 0;
float rx1 = 0, ry1 = 0, rz1 = 0;
float tx1 = 0, ty1 = 0, tz1 = 0;
float cx2 = 0, cy2 = 0, cz2 = 0;
float rx2 = 0, ry2 = 0, rz2 = 0;
float tx2 = 0, ty2 = 0, tz2 = 0;
float c_rx = 0, c_ry = 0, c_rz = 0;
float c_tx = 0, c_ty = 0, c_tz = 0;

GtkWidget  *number_of_messages_v1_widget_label, *number_of_messages_v1_widget;
GtkWidget  *cx1_widget_label, *cx1_widget;
GtkWidget  *cy1_widget_label, *cy1_widget;
GtkWidget  *cz1_widget_label, *cz1_widget;
GtkWidget  *rx1_widget_label, *rx1_widget;
GtkWidget  *ry1_widget_label, *ry1_widget;
GtkWidget  *rz1_widget_label, *rz1_widget;
GtkWidget  *tx1_widget_label, *tx1_widget;
GtkWidget  *ty1_widget_label, *ty1_widget;
GtkWidget  *tz1_widget_label, *tz1_widget;

GtkWidget  *number_of_messages_v2_widget_label, *number_of_messages_v2_widget;
GtkWidget  *cx2_widget_label, *cx2_widget;
GtkWidget  *cy2_widget_label, *cy2_widget;
GtkWidget  *cz2_widget_label, *cz2_widget;
GtkWidget  *rx2_widget_label, *rx2_widget;
GtkWidget  *ry2_widget_label, *ry2_widget;
GtkWidget  *rz2_widget_label, *rz2_widget;
GtkWidget  *tx2_widget_label, *tx2_widget;
GtkWidget  *ty2_widget_label, *ty2_widget;
GtkWidget  *tz2_widget_label, *tz2_widget;

GtkWidget  *calibrationRX_widget_label, *calibrationRX_widget;
GtkWidget  *calibrationRY_widget_label, *calibrationRY_widget;
GtkWidget  *calibrationRZ_widget_label, *calibrationRZ_widget;
GtkWidget  *calibrationTX_widget_label, *calibrationTX_widget;
GtkWidget  *calibrationTY_widget_label, *calibrationTY_widget;
GtkWidget  *calibrationTZ_widget_label, *calibrationTZ_widget;

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

static gboolean
final_time_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        playback_final_time = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_FINAL_TIME,
                                playback_final_time, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
number_of_messages_v1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        number_of_messages_v1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_NUMBER_OF_MESSAGES_V1,
        		number_of_messages_v1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
number_of_messages_v2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        number_of_messages_v2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SET_NUMBER_OF_MESSAGES_V2,
        		number_of_messages_v2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
cx1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        cx1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_CX1,
        		cx1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
cy1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        cy1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_CY1,
        		cy1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
cz1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        cz1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_CZ1,
        		cz1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
rx1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        rx1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_RX1,
        		rx1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
ry1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        ry1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_RY1,
        		ry1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
rz1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        rz1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_RZ1,
        		rz1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
tx1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        tx1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_TX1,
        		tx1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
ty1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        ty1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_TY1,
        		ty1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
tz1_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        tz1 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_TZ1,
        		tz1, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
cx2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        cx2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_CX2,
        		cx2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
cy2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        cy2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_CY2,
        		cy2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
cz2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        cz2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_CZ2,
        		cz2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
rx2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        rx2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_RX2,
        		rx2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
ry2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        ry2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_RY2,
        		ry2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
rz2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        rz2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_RZ2,
        		rz2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
tx2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        tx2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_TX2,
        		tx2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
ty2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        ty2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_TY2,
        		ty2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
tz2_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        tz2 = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_TZ2,
        		tz2, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
c_rx_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        c_rx = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_C_RX,
        		c_rx, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
c_ry_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        c_ry = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_C_RY,
        		c_ry, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
c_rz_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        c_rz = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_C_RZ,
        		c_rz, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
c_tx_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        c_tx = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_C_TX,
        		c_tx, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
c_ty_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        c_ty = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_C_TY,
        		c_ty, playback_speed);
        return TRUE;
    }
    return FALSE;
}

static gboolean
c_tz_params_save(GtkWidget *w, // __attribute__ ((unused)),
                         GdkEvent *event,
                         gpointer pntr __attribute__((unused)))
{
    if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
        (event->key.keyval == gdk_keyval_from_name("Return")))
    {
        gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
        c_tz = atoi(value);

        //initial_time_pending_update = 0;
        //gtk_label_set_pattern(GTK_LABEL(playback_initial_time_widget_label), "");
        carmen_playback_command(CARMEN_PLAYBACK_C_TZ,
        		c_tz, playback_speed);
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
    GtkWidget *hbox, *rrwd, *rwd, *play, *stop, *ffwd, *fwd, *reset_button, *vbox, *hbox2, *hbox3, *hbox4, *hbox5, *showPoints, *calibration;
    GtkWidget *rrwd_darea, *rwd_darea, *stop_darea, *play_darea, *showPoints_darea, *calibration_darea,
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
    gtk_widget_set_usize(window, 1050, 420);

    gtk_signal_connect(GTK_OBJECT(window), "destroy",
                       GTK_SIGNAL_FUNC(gtk_main_quit),
                       "WM destroy");

    gtk_signal_connect(GTK_OBJECT(window), "delete_event",
                       GTK_SIGNAL_FUNC(delete_event), NULL);

    gtk_window_set_title(GTK_WINDOW(window), "playback control");
    char *aux = getenv("CARMEN_HOME");
    //strcat(aux, "/data/gui/playback.png");
    //gtk_window_set_icon(GTK_WINDOW(window), create_pixbuf(aux));
    gtk_widget_realize(window);

    vbox = gtk_vbox_new(0, 0);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 5);
    gtk_container_add(GTK_CONTAINER(window), vbox);

	GtkWidget *frame_horz;
	frame_horz = gtk_frame_new ("Playback Control");
	gtk_box_pack_start (GTK_BOX (vbox), frame_horz, TRUE, TRUE, 10);

    hbox = gtk_hbox_new(0, 0);
    gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
    gtk_container_add(GTK_CONTAINER(frame_horz), hbox);

    GtkWidget *frame_horz_2;
    frame_horz_2 = gtk_frame_new ("Messages Timeline");
	gtk_box_pack_start (GTK_BOX (vbox), frame_horz_2, TRUE, TRUE, 10);

    hbox2 = gtk_hbox_new(0, 0);
    gtk_container_set_border_width(GTK_CONTAINER(hbox2), 5);
    gtk_container_add(GTK_CONTAINER(frame_horz_2), hbox2);

	//-----------------------------------------------------------------------

    GtkWidget *frame_horz_3;
	frame_horz_3 = gtk_frame_new ("Velodyne 1");
	gtk_box_pack_start (GTK_BOX (vbox), frame_horz_3, TRUE, TRUE, 10);

	hbox3 = gtk_hbox_new(0, 0);
	gtk_container_set_border_width(GTK_CONTAINER(hbox3), 5);
	gtk_container_add(GTK_CONTAINER(frame_horz_3), hbox3);

	//-----------------------------------------------------------------------

    GtkWidget *frame_horz_4;
	frame_horz_4 = gtk_frame_new ("Velodyne 2");
	gtk_box_pack_start (GTK_BOX (vbox), frame_horz_4, TRUE, TRUE, 10);

	hbox4 = gtk_hbox_new(0, 0);
	gtk_container_set_border_width(GTK_CONTAINER(hbox4), 5);
	gtk_container_add(GTK_CONTAINER(frame_horz_4), hbox4);

	//-----------------------------------------------------------------------

    GtkWidget *frame_horz_5;
	frame_horz_5 = gtk_frame_new ("Calibration");
	gtk_box_pack_start (GTK_BOX (vbox), frame_horz_5, TRUE, TRUE, 10);

	hbox5 = gtk_hbox_new(0, 0);
	gtk_container_set_border_width(GTK_CONTAINER(hbox5), 5);
	gtk_container_add(GTK_CONTAINER(frame_horz_5), hbox5);

	//-----------------------------------------------------------------------

	playback_speed_widget_label = gtk_label_new("Speed: ");
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

    playback_initial_time_widget_label = gtk_label_new("Initial Message: ");
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

    playback_final_time_widget_label = gtk_label_new("Final Message: ");
    playback_final_time_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(playback_final_time_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(playback_final_time_widget), 0, GTK_ENTRY(playback_final_time_widget)->text_length);
	/*gtk_signal_connect(GTK_OBJECT(playback_final_time_widget), "changed",
					   GTK_SIGNAL_FUNC(initial_time_changed), NULL);*/
	gtk_signal_connect(GTK_OBJECT(playback_final_time_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(final_time_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox), playback_final_time_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox), playback_final_time_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(playback_final_time_widget, 100, 30);
	gtk_widget_show(playback_final_time_widget);
	gtk_widget_show(playback_final_time_widget_label);

	//velodyne 1 block

    number_of_messages_v1_widget_label = gtk_label_new("KD-Tree Messages:"); //number of messages for calibration of the KD-Tree
    number_of_messages_v1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(number_of_messages_v1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(number_of_messages_v1_widget), 0, GTK_ENTRY(number_of_messages_v1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(number_of_messages_v1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(number_of_messages_v1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), number_of_messages_v1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), number_of_messages_v1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(number_of_messages_v1_widget, 60, 30);
	gtk_widget_show(number_of_messages_v1_widget);
	gtk_widget_show(number_of_messages_v1_widget_label);

	cx1_widget_label = gtk_label_new("CX:");
	cx1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(cx1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(cx1_widget), 0, GTK_ENTRY(cx1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(cx1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(cx1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), cx1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), cx1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(cx1_widget, 60, 30);
	gtk_widget_show(cx1_widget);
	gtk_widget_show(cx1_widget_label);

    cy1_widget_label = gtk_label_new("CY:"); //number of messages for calibration of the KD-Tree
    cy1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(cy1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(cy1_widget), 0, GTK_ENTRY(cy1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(cy1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(cy1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), cy1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), cy1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(cy1_widget, 60, 30);
	gtk_widget_show(cy1_widget);
	gtk_widget_show(cy1_widget_label);

    cz1_widget_label = gtk_label_new("CZ:"); //number of messages for calibration of the KD-Tree
    cz1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(cz1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(cz1_widget), 0, GTK_ENTRY(cz1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(cz1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(cz1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), cz1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), cz1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(cz1_widget, 60, 30);
	gtk_widget_show(cz1_widget);
	gtk_widget_show(cz1_widget_label);

	rx1_widget_label = gtk_label_new("RX:");
	rx1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(rx1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(rx1_widget), 0, GTK_ENTRY(rx1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(rx1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(rx1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), rx1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), rx1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(rx1_widget, 60, 30);
	gtk_widget_show(rx1_widget);
	gtk_widget_show(rx1_widget_label);

	ry1_widget_label = gtk_label_new("RY:");
	ry1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(ry1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(ry1_widget), 0, GTK_ENTRY(ry1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(ry1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(ry1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), ry1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), ry1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(ry1_widget, 60, 30);
	gtk_widget_show(ry1_widget);
	gtk_widget_show(ry1_widget_label);

	rz1_widget_label = gtk_label_new("RZ:");
	rz1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(rz1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(rz1_widget), 0, GTK_ENTRY(rz1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(rz1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(rz1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), rz1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), rz1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(rz1_widget, 60, 30);
	gtk_widget_show(rz1_widget);
	gtk_widget_show(rz1_widget_label);

	tx1_widget_label = gtk_label_new("TX:");
	tx1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(tx1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(tx1_widget), 0, GTK_ENTRY(tx1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(tx1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(tx1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), tx1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), tx1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(tx1_widget, 60, 30);
	gtk_widget_show(tx1_widget);
	gtk_widget_show(tx1_widget_label);

	ty1_widget_label = gtk_label_new("TY:");
	ty1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(ty1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(ty1_widget), 0, GTK_ENTRY(ty1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(ty1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(ty1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), ty1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), ty1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(ty1_widget, 60, 30);
	gtk_widget_show(ty1_widget);
	gtk_widget_show(ty1_widget_label);

	tz1_widget_label = gtk_label_new("TZ:");
	tz1_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(tz1_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(tz1_widget), 0, GTK_ENTRY(tz1_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(tz1_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(tz1_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox3), tz1_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox3), tz1_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(tz1_widget, 60, 30);
	gtk_widget_show(tz1_widget);
	gtk_widget_show(tz1_widget_label);

	//velodyne 2 block

    number_of_messages_v2_widget_label = gtk_label_new("KD-Tree Messages:"); //number of messages for calibration of the KD-Tree
    number_of_messages_v2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(number_of_messages_v2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(number_of_messages_v2_widget), 0, GTK_ENTRY(number_of_messages_v2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(number_of_messages_v2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(number_of_messages_v2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), number_of_messages_v2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), number_of_messages_v2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(number_of_messages_v2_widget, 60, 30);
	gtk_widget_show(number_of_messages_v2_widget);
	gtk_widget_show(number_of_messages_v2_widget_label);

	cx2_widget_label = gtk_label_new("CX:");
	cx2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(cx2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(cx2_widget), 0, GTK_ENTRY(cx2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(cx2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(cx2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), cx2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), cx2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(cx2_widget, 60, 30);
	gtk_widget_show(cx2_widget);
	gtk_widget_show(cx2_widget_label);

    cy2_widget_label = gtk_label_new("CY:"); //number of messages for calibration of the KD-Tree
    cy2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(cy2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(cy2_widget), 0, GTK_ENTRY(cy2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(cy2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(cy2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), cy2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), cy2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(cy2_widget, 60, 30);
	gtk_widget_show(cy2_widget);
	gtk_widget_show(cy2_widget_label);

    cz2_widget_label = gtk_label_new("CZ:"); //number of messages for calibration of the KD-Tree
    cz2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(cz2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(cz2_widget), 0, GTK_ENTRY(cz2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(cz2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(cz2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), cz2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), cz2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(cz2_widget, 60, 30);
	gtk_widget_show(cz2_widget);
	gtk_widget_show(cz2_widget_label);

	rx2_widget_label = gtk_label_new("RX:");
	rx2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(rx2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(rx2_widget), 0, GTK_ENTRY(rx2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(rx2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(rx2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), rx2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), rx2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(rx2_widget, 60, 30);
	gtk_widget_show(rx2_widget);
	gtk_widget_show(rx2_widget_label);

	ry2_widget_label = gtk_label_new("RY:");
	ry2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(ry2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(ry2_widget), 0, GTK_ENTRY(ry2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(ry2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(ry2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), ry2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), ry2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(ry2_widget, 60, 30);
	gtk_widget_show(ry2_widget);
	gtk_widget_show(ry2_widget_label);

	rz2_widget_label = gtk_label_new("RZ:");
	rz2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(rz2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(rz2_widget), 0, GTK_ENTRY(rz2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(rz2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(rz2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), rz2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), rz2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(rz2_widget, 60, 30);
	gtk_widget_show(rz2_widget);
	gtk_widget_show(rz2_widget_label);

	tx2_widget_label = gtk_label_new("TX:");
	tx2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(tx2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(tx2_widget), 0, GTK_ENTRY(tx2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(tx2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(tx2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), tx2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), tx2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(tx2_widget, 60, 30);
	gtk_widget_show(tx2_widget);
	gtk_widget_show(tx2_widget_label);

	ty2_widget_label = gtk_label_new("TY:");
	ty2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(ty2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(ty2_widget), 0, GTK_ENTRY(ty2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(ty2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(ty2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), ty2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), ty2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(ty2_widget, 60, 30);
	gtk_widget_show(ty2_widget);
	gtk_widget_show(ty2_widget_label);

	tz2_widget_label = gtk_label_new("TZ:");
	tz2_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(tz2_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(tz2_widget), 0, GTK_ENTRY(tz2_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(tz2_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(tz2_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox4), tz2_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox4), tz2_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(tz2_widget, 60, 30);
	gtk_widget_show(tz2_widget);
	gtk_widget_show(tz2_widget_label);

	//Calibration block

	calibrationRX_widget_label = gtk_label_new("RX:");
	calibrationRX_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(calibrationRX_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(calibrationRX_widget), 0, GTK_ENTRY(calibrationRX_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(calibrationRX_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(c_rx_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationRX_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationRX_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(calibrationRX_widget, 60, 30);
	gtk_widget_show(calibrationRX_widget);
	gtk_widget_show(calibrationRX_widget_label);

	calibrationRY_widget_label = gtk_label_new("RY:");
	calibrationRY_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(calibrationRY_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(calibrationRY_widget), 0, GTK_ENTRY(calibrationRY_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(calibrationRY_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(c_ry_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationRY_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationRY_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(calibrationRY_widget, 60, 30);
	gtk_widget_show(calibrationRY_widget);
	gtk_widget_show(calibrationRY_widget_label);

	calibrationRZ_widget_label = gtk_label_new("RZ:");
	calibrationRZ_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(calibrationRZ_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(calibrationRZ_widget), 0, GTK_ENTRY(calibrationRZ_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(calibrationRZ_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(c_rz_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationRZ_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationRZ_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(calibrationRZ_widget, 60, 30);
	gtk_widget_show(calibrationRZ_widget);
	gtk_widget_show(calibrationRZ_widget_label);

	calibrationTX_widget_label = gtk_label_new("TX:");
	calibrationTX_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(calibrationTX_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(calibrationTX_widget), 0, GTK_ENTRY(calibrationTX_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(calibrationTX_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(c_tx_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationTX_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationTX_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(calibrationTX_widget, 60, 30);
	gtk_widget_show(calibrationTX_widget);
	gtk_widget_show(calibrationTX_widget_label);

	calibrationTY_widget_label = gtk_label_new("TY:");
	calibrationTY_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(calibrationTY_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(calibrationTY_widget), 0, GTK_ENTRY(calibrationTY_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(calibrationTY_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(c_ty_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationTY_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationTY_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(calibrationTY_widget, 60, 30);
	gtk_widget_show(calibrationTY_widget);
	gtk_widget_show(calibrationTY_widget_label);

	calibrationTZ_widget_label = gtk_label_new("TZ:");
	calibrationTZ_widget = gtk_entry_new_with_max_length(9);
	gtk_entry_set_text(GTK_ENTRY(calibrationTZ_widget), "");
	gtk_editable_select_region(GTK_EDITABLE(calibrationTZ_widget), 0, GTK_ENTRY(calibrationTZ_widget)->text_length);
	gtk_signal_connect(GTK_OBJECT(calibrationTZ_widget), "key_press_event",
					   GTK_SIGNAL_FUNC(c_tz_params_save), NULL);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationTZ_widget_label,
					   FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(hbox5), calibrationTZ_widget, FALSE, FALSE, 5);
	gtk_widget_set_usize(calibrationTZ_widget, 60, 30);
	gtk_widget_show(calibrationTZ_widget);
	gtk_widget_show(calibrationTZ_widget_label);

	////////////////////////////////////////

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

    showPoints = gtk_button_new();
    showPoints_darea = gtk_drawing_area_new();
    gtk_widget_set_usize(showPoints_darea, 30, 40);
    showPoints_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(showPoints_gc, &Red);
    gtk_signal_connect(GTK_OBJECT(showPoints_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "ShowPoints");
    gtk_container_add(GTK_CONTAINER(showPoints), showPoints_darea);
    gtk_box_pack_start(GTK_BOX(hbox), showPoints, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(showPoints), "clicked",
                       (GtkSignalFunc) Send_Command, "ShowPoints");

    calibration = gtk_button_new();
    calibration_darea = gtk_drawing_area_new();
    gtk_widget_set_usize(calibration_darea, 30, 40);
    calibration_gc = gdk_gc_new(window->window);
    gdk_gc_set_foreground(calibration_gc, &Blue);
    gtk_signal_connect(GTK_OBJECT(calibration_darea), "expose_event",
                       (GtkSignalFunc) Redraw, "Calibration");
    gtk_container_add(GTK_CONTAINER(calibration), calibration_darea);
    gtk_box_pack_start(GTK_BOX(hbox), calibration, FALSE, FALSE, 5);
    gtk_signal_connect(GTK_OBJECT(calibration), "clicked",
                       (GtkSignalFunc) Send_Command, "Calibration");


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
    else if (strcmp(data, "ShowPoints") == 0)
	{
		square[0].x = left;
		square[0].y = top;
		square[1].x = right;
		square[1].y = top;
		square[2].x = right-10;
		square[2].y = bottom;
		square[3].x = left+10;
		square[3].y = bottom;
		gdk_draw_polygon(widget->window, showPoints_gc, 1, square, 4);
	}
    else if (strcmp(data, "Calibration") == 0)
	{
		square[0].x = left;
		square[0].y = top;
		square[1].x = right;
		square[1].y = top;
		square[2].x = left;
		square[2].y = bottom;
		square[3].x = right;
		square[3].y = bottom;
		gdk_draw_polygon(widget->window, calibration_gc, 1, square, 4);
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
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_REWIND, 100, playback_speed);
    else if (strcmp(data, "RW") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_RWD_SINGLE, 1, playback_speed);
    else if (strcmp(data, "FFWD") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_FORWARD, 100, playback_speed);
    else if (strcmp(data, "FWD") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_FWD_SINGLE, 1, playback_speed);
    else if (strcmp(data, "RESET") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_RESET, 0, playback_speed);
    else if (strcmp(data, "ShowPoints") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_SHOW_POINTS, 0, playback_speed);
    else if (strcmp(data, "Calibration") == 0)
        carmen_playback_command(CARMEN_PLAYBACK_COMMAND_CALIBRATION, 0, playback_speed);
}
