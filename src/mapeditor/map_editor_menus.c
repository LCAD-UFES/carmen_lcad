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

/***********************************
 * map_editor_menus implements the *
 * menu items in the map_editor    *
 ***********************************/
#include <gtk/gtk.h>
#include <carmen/carmen.h>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR == 3
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

extern GdkColor carmen_red, carmen_blue, carmen_white, carmen_yellow, 
carmen_green, carmen_light_blue, carmen_black, carmen_orange,
carmen_grey, carmen_light_grey, carmen_purple;

#include "map_editor.h"
#include "map_editor_menus.h"
#include "map_editor_graphics.h"

static GtkWidget *height_entry, *width_entry, *resolution_entry;
static GtkWidget *placename_dialog;
static GtkWidget *name_label, *x_label, *y_label, *theta_label;
static GtkWidget *name_entry, *x_entry, *y_entry, *theta_entry, *x_std_entry, 
*y_std_entry, *theta_std_entry;

/********************************************************
 *  Menu functions                                      *
 ********************************************************/

void toggle_view(GtkAction *action, gpointer user_data __attribute__ ((unused)))
{
	char *name;
	GtkToggleAction *toggle;

	name = (char *)gtk_action_get_name(action);

	if (strcmp(name, "ShowPlacenames") == 0) {
		toggle = GTK_TOGGLE_ACTION(action);
		show_place_names = gtk_toggle_action_get_active(toggle);
	} else if (strcmp(name, "ShowOfflimits") == 0) {
		toggle = GTK_TOGGLE_ACTION(action);
		show_offlimits = gtk_toggle_action_get_active(toggle);
	}
	if (tmp_pixmap) {
		gdk_pixmap_unref(tmp_pixmap);
		tmp_pixmap = NULL;
	}
	redraw();
}

void add_placename(GtkAction *action  __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	adding_placename = 1;
}

void delete_placename(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	deleting_placename = 1;
}

void add_door(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	adding_door = 2;
}

void do_delete_placename(int i)
{
	int num_to_move = place_list->num_places-i;

	memmove(place_list->places+i, place_list->places+i+1,
			sizeof(carmen_place_t)*num_to_move);

	place_list->num_places--;

	gdk_pixmap_unref(tmp_pixmap);
	tmp_pixmap = NULL;
	redraw();

	modified++;
	deleting_placename = 0;
}

static void highlight(GtkWidget *label, GtkWidget *entry)
{
	GtkStyle *style;

	style = gtk_widget_get_style(entry);
	style = gtk_style_copy(style);
	style->fg[0] = carmen_red;
	style->fg[1] = carmen_red;
	style->fg[2] = carmen_red;
	style->fg[3] = carmen_red;
	style->fg[4] = carmen_red;
	gtk_widget_set_style(entry, style);
	gtk_widget_set_style(label, style);
}

static void restore(GtkWidget *label, GtkWidget *entry)
{
	gtk_widget_restore_default_style (label);
	gtk_widget_restore_default_style (entry);
}

static int
is_not_blanks(char *errs)
{
	if (errs == NULL || strlen(errs) == 0)
		return 0;
	if (strcspn(gtk_entry_get_text(GTK_ENTRY(name_entry)), " \t\r\n") == 0)
		return 0;
	return 1;
}

static
void add_place_button(GtkWidget *button __attribute__ ((unused)),
		gpointer user_data __attribute__ ((unused)))
{
	int num_places;
	int errors = 0;
	double x, y, theta;
	char *errs;

	if (place_list == NULL) {
		place_list = (carmen_map_placelist_p) calloc(1, sizeof(carmen_map_placelist_t));
		carmen_test_alloc(place_list);
	}

	num_places = place_list->num_places;
	place_list->places = (carmen_place_p) realloc(place_list->places,
			sizeof(carmen_place_t)*(num_places+1));
	carmen_test_alloc(place_list->places);
	place_list->places[num_places].type = CARMEN_NAMED_POSITION_TYPE;

	if (strlen(gtk_entry_get_text(GTK_ENTRY(name_entry))) == 0 ||
			strcspn(gtk_entry_get_text(GTK_ENTRY(name_entry)), " \t\r\n") == 0)
	{
		errors = 1;
		highlight(name_label, name_entry);
	}
	else {
		restore(name_label, name_entry);
		strcpy(place_list->places[num_places].name,
				gtk_entry_get_text(GTK_ENTRY(name_entry)));
	}

	x = strtod(gtk_entry_get_text(GTK_ENTRY(x_entry)), &errs);
	if (strlen(gtk_entry_get_text(GTK_ENTRY(x_entry))) == 0 ||
			is_not_blanks(errs) || x < 0 || x >= map->config.x_size)
	{
		errors = 1;
		highlight(x_label, x_entry);
	}
	else
	{
		restore(x_label, x_entry);
		place_list->places[num_places].x = x;
	}

	y = strtod(gtk_entry_get_text(GTK_ENTRY(y_entry)), &errs);
	if (strlen(gtk_entry_get_text(GTK_ENTRY(y_entry))) == 0 ||
			is_not_blanks(errs) || y < 0 || y >= map->config.y_size)
	{
		errors = 1;
		highlight(y_label, y_entry);
	}
	else
	{
		restore(y_label, y_entry);
		place_list->places[num_places].y = y;
	}

	if (strlen(gtk_entry_get_text(GTK_ENTRY(theta_entry))) > 0)
	{
		theta = strtod(gtk_entry_get_text(GTK_ENTRY(theta_entry)), &errs);
		if (is_not_blanks(errs))
		{
			errors = 1;
			highlight(theta_label, theta_entry);
		}
		else
		{
			restore(theta_label, theta_entry);
			place_list->places[num_places].theta =
					carmen_degrees_to_radians(theta);
		}
	}

	if (strlen(gtk_entry_get_text(GTK_ENTRY(x_std_entry))) > 0 &&
			strlen(gtk_entry_get_text(GTK_ENTRY(y_std_entry))) > 0 &&
			strlen(gtk_entry_get_text(GTK_ENTRY(theta_std_entry))) > 0)
	{
		place_list->places[num_places].x_std =
				strtod(gtk_entry_get_text(GTK_ENTRY(x_std_entry)), NULL);
		place_list->places[num_places].y_std =
				strtod(gtk_entry_get_text(GTK_ENTRY(y_std_entry)), NULL);
		place_list->places[num_places].theta_std =
				strtod(gtk_entry_get_text(GTK_ENTRY(theta_std_entry)), NULL);
		place_list->places[num_places].theta_std =
				carmen_degrees_to_radians(place_list->places[num_places].theta_std);
		place_list->places[num_places].type = CARMEN_LOCALIZATION_INIT_TYPE;
	}

	if (errors)
		return;

	place_list->num_places++;

	gdk_pixmap_unref(tmp_pixmap);
	tmp_pixmap = NULL;
	redraw();

	modified++;
	adding_placename = 0;
	gtk_widget_destroy(placename_dialog);
}

void start_add_placename(double x, double y)
{
	GtkWidget *hbox, *label, *button;
	char buffer[10];

	placename_dialog = gtk_dialog_new();
	hbox = gtk_hbox_new(FALSE, 0);
	gtk_box_pack_start (GTK_BOX (GTK_DIALOG (placename_dialog)->vbox),
			hbox, TRUE, TRUE, 0);
	name_label = gtk_label_new("Place name: ");
	gtk_box_pack_start (GTK_BOX (hbox), name_label, TRUE, TRUE, 0);
	name_entry = gtk_entry_new_with_max_length(21);
	gtk_widget_set_usize(name_entry, 90, 20);
	gtk_box_pack_start (GTK_BOX(hbox), name_entry, TRUE, TRUE, 0);

	hbox = gtk_hbox_new(FALSE, 3);
	gtk_box_pack_start (GTK_BOX (GTK_DIALOG (placename_dialog)->vbox),
			hbox, TRUE, TRUE, 0);

	x_label = gtk_label_new("X: ");
	gtk_box_pack_start (GTK_BOX (hbox), x_label, TRUE, TRUE, 0);
	x_entry = gtk_entry_new_with_max_length(5);
	gtk_widget_set_usize(x_entry, 45, 20);
	gtk_box_pack_start (GTK_BOX (hbox), x_entry, TRUE, TRUE, 0);
	sprintf(buffer, "%.2f", x);
	gtk_entry_set_text(GTK_ENTRY(x_entry), buffer);

	y_label = gtk_label_new("Y: ");
	gtk_box_pack_start (GTK_BOX (hbox), y_label, TRUE, TRUE, 0);
	y_entry = gtk_entry_new_with_max_length(5);
	gtk_widget_set_usize(y_entry, 45, 20);
	gtk_box_pack_start (GTK_BOX (hbox), y_entry, TRUE, TRUE, 0);
	sprintf(buffer, "%.2f", y);
	gtk_entry_set_text(GTK_ENTRY(y_entry), buffer);

	theta_label = gtk_label_new("Theta: ");
	gtk_box_pack_start (GTK_BOX (hbox), theta_label, TRUE, TRUE, 0);
	theta_entry = gtk_entry_new_with_max_length(5);
	gtk_widget_set_usize(theta_entry, 45, 20);
	gtk_box_pack_start (GTK_BOX (hbox), theta_entry, TRUE, TRUE, 0);

	hbox = gtk_hbox_new(FALSE, 3);
	gtk_box_pack_start (GTK_BOX (GTK_DIALOG (placename_dialog)->vbox),
			hbox, TRUE, TRUE, 0);

	label = gtk_label_new("Std X: ");
	gtk_box_pack_start (GTK_BOX (hbox), label, TRUE, TRUE, 0);
	x_std_entry = gtk_entry_new_with_max_length(5);
	gtk_widget_set_usize(x_std_entry, 45, 20);
	gtk_box_pack_start (GTK_BOX (hbox), x_std_entry, TRUE, TRUE, 0);

	label = gtk_label_new("Std Y: ");
	gtk_box_pack_start (GTK_BOX (hbox), label, TRUE, TRUE, 0);
	y_std_entry = gtk_entry_new_with_max_length(5);
	gtk_widget_set_usize(y_std_entry, 45, 20);
	gtk_box_pack_start (GTK_BOX (hbox), y_std_entry, TRUE, TRUE, 0);

	label = gtk_label_new("Std Theta: ");
	gtk_box_pack_start (GTK_BOX (hbox), label, TRUE, TRUE, 0);
	theta_std_entry = gtk_entry_new_with_max_length(5);
	gtk_widget_set_usize(theta_std_entry, 45, 20);
	gtk_box_pack_start (GTK_BOX (hbox), theta_std_entry, TRUE, TRUE, 0);

	hbox = GTK_DIALOG(placename_dialog)->action_area;

	button = gtk_button_new_with_label("OK");
	gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);

	gtk_signal_connect(GTK_OBJECT(button), "clicked",
			(GtkSignalFunc)add_place_button, NULL);

	button = gtk_button_new_with_label("Cancel");
	gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);

	gtk_signal_connect_object(GTK_OBJECT(button),
			"clicked", (GtkSignalFunc)gtk_widget_destroy,
			(gpointer)placename_dialog);

	gtk_widget_show_all(placename_dialog);
}

int next_door_num()
{
	int i, n, k, xxx, loop;

	if (place_list == NULL)
		return 0;

	n = 0;
	loop = 1;
	while (loop) {
		loop = 0;
		for (i = 0; i < place_list->num_places; i++) {
			if (sscanf(place_list->places[i].name, "b%d.%d", &k, &xxx) == 2) {
				if (k == n) {
					n++;
					loop = 1;
				}
			}
		}
	}

	return n;
}

void start_add_door(double x, double y)
{
	int num_places;

	if (place_list == NULL) {
		place_list = (carmen_map_placelist_p) calloc(1, sizeof(carmen_map_placelist_t));
		carmen_test_alloc(place_list);
	}

	num_places = place_list->num_places;
	place_list->places = (carmen_place_p) realloc(place_list->places,
			sizeof(carmen_place_t)*(num_places+1));
	carmen_test_alloc(place_list->places);
	place_list->places[num_places].type = CARMEN_NAMED_POSITION_TYPE;

	current_door_num = next_door_num();
	sprintf(place_list->places[num_places].name, "b%d.1", current_door_num);

	place_list->places[num_places].x = x;
	place_list->places[num_places].y = y;
	place_list->places[num_places].theta = 0;
	place_list->places[num_places].x_std = 0;
	place_list->places[num_places].y_std = 0;
	place_list->places[num_places].theta_std = 0;

	place_list->num_places++;

	gdk_pixmap_unref(tmp_pixmap);
	tmp_pixmap = NULL;
	redraw();

	modified++;
	adding_door--;
}

void finish_add_door(double x, double y)
{
	int num_places;

	if (place_list == NULL) {
		place_list = (carmen_map_placelist_p) calloc(1, sizeof(carmen_map_placelist_t));
		carmen_test_alloc(place_list);
	}

	num_places = place_list->num_places;
	place_list->places = (carmen_place_p) realloc(place_list->places,
			sizeof(carmen_place_t)*(num_places+1));
	carmen_test_alloc(place_list->places);
	place_list->places[num_places].type = CARMEN_NAMED_POSITION_TYPE;

	sprintf(place_list->places[num_places].name, "b%d.2", current_door_num);

	place_list->places[num_places].x = x;
	place_list->places[num_places].y = y;
	place_list->places[num_places].theta = 0;
	place_list->places[num_places].x_std = 0;
	place_list->places[num_places].y_std = 0;
	place_list->places[num_places].theta_std = 0;

	place_list->num_places++;

	gdk_pixmap_unref(tmp_pixmap);
	tmp_pixmap = NULL;
	redraw();

	modified++;
	adding_door--;
}

gint map_save(char *filename)
{
//	carmen_FILE *fp_in;
	carmen_FILE *fp_out;
//	char cmd[100];


	fp_out = carmen_fopen(filename, "w+");

	if (fp_out == NULL)
	{
		fprintf(stderr, "Could not write!\n");
		return 0;
	}

	if (carmen_map_write_all(fp_out, map->map,
				map->config.x_size,
				map->config.y_size,
				map->config.resolution,
				(char *) "", (char *) "", (char *) "", (char *) "Generated by big_map",
				NULL, 0, NULL, 0, NULL, 0) == -1)
		{
			return 0;
		}

		carmen_fclose(fp_out);

		strcpy(map_filename, filename);
		modified = 0;

		return  1;


//
//
//
//
//
//
//
//
//	if(carmen_map_file(map_filename)) {
//		fp_in = carmen_fopen(map_filename, "r");
//#ifdef NO_ZLIB
//		fp_out = carmen_fopen("/tmp/tmp.cmf", "w");
//#else
//		fp_out = carmen_fopen("/tmp/tmp.cmf.gz", "w");
//#endif
//		if(carmen_map_vstrip(fp_in, fp_out, 3, CARMEN_MAP_GRIDMAP_CHUNK,
//				CARMEN_MAP_OFFLIMITS_CHUNK,
//				CARMEN_MAP_PLACES_CHUNK) < 0)
//			carmen_warn("Could not strip gridmap from original map.\n");
//		if(carmen_map_write_gridmap_chunk(fp_out, map->map, map->config.x_size,
//				map->config.y_size,
//				map->config.resolution) < 0)
//			carmen_warn("Could not write gridmap.\n");
//
//		if (num_offlimits_segments > 0)
//			if(carmen_map_write_offlimits_chunk(fp_out, offlimits_array,
//					num_offlimits_segments) < 0)
//				carmen_warn("Could not write offlimits chunk.\n");
//
//		if (place_list != NULL && place_list->num_places > 0)
//			if(carmen_map_write_places_chunk(fp_out, place_list->places,
//					place_list->num_places) < 0)
//				carmen_warn("Could not write offlimits chunk.\n");
//
//		carmen_fclose(fp_in);
//		carmen_fclose(fp_out);
//#ifdef NO_ZLIB
//		sprintf(cmd, "mv /tmp/tmp.cmf %s", filename);
//#else
//		sprintf(cmd, "mv /tmp/tmp.cmf.gz %s", filename);
//#endif
//		system(cmd);
//	}
//	else {
//		fp_out = carmen_fopen(filename, "w");
//		if(carmen_map_write_comment_chunk(fp_out, map->config.x_size,
//				map->config.y_size,
//				map->config.resolution,
//				"map_editor",
//				"unknown") < 0)
//			carmen_warn("Could not write comment chunk to map file.\n");
//		if(carmen_map_write_id(fp_out) < 0)
//			carmen_warn("Could not write id to map file.\n");
//		if(carmen_map_write_creator_chunk(fp_out, "map_editor", "unknown") < 0)
//			carmen_warn("Could not write creator chunk to map file.\n");
//		if(carmen_map_write_gridmap_chunk(fp_out, map->map, map->config.x_size,
//				map->config.y_size,
//				map->config.resolution) < 0)
//			carmen_warn("Could not write gridmap.\n");
//		if (num_offlimits_segments > 0)
//			if(carmen_map_write_offlimits_chunk(fp_out, offlimits_array,
//					num_offlimits_segments) < 0)
//				carmen_warn("Could not write offlimits chunk.\n");
//		if (place_list && place_list->num_places > 0)
//			if(carmen_map_write_places_chunk(fp_out, place_list->places,
//					place_list->num_places) < 0)
//				carmen_warn("Could not write offlimits chunk.\n");
//
//		carmen_fclose(fp_out);
//	}
//	strcpy(map_filename, filename);
//	modified = 0;
//	return 1;
}

int map_open(char *filename, int have_graphics __attribute__ ((unused)))
{
	char error_str[1000];
	carmen_map_p new_map;
	int err;

	if(!carmen_map_file(filename)) {
		sprintf(error_str,
				"Error: %s does not appear to be a valid carmen map file;\n"
				"if it is gzipped, make sure it has a \".gz\" extension\n",
				filename);
		carmen_warn("%s\n", error_str);

		return -1;
	}

	if (strlen(filename) >= 254)
		return -1;

	strcpy(map_filename, filename);

	new_map = (carmen_map_p)calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(new_map);

	if (!carmen_map_chunk_exists(filename, CARMEN_MAP_GRIDMAP_CHUNK)) {
		sprintf(error_str, "The map file %s contains no gridmap.", filename);
		/* dbug
      if (have_graphics) 
        gnome_error_dialog_parented(error_str, GTK_WINDOW(window));
	else 
		 */
		carmen_warn("%s\n", error_str);
		return -1;
	}

	err = carmen_map_read_gridmap_chunk(map_filename, new_map);
	if (err < 0) {
		sprintf(error_str, "Error reading file %s : %s.", filename,
				strerror(errno));
		/* dbug
      if (have_graphics) 
        gnome_error_dialog_parented(error_str, GTK_WINDOW(window));
	else 
		 */
		carmen_warn("%s\n", error_str);

		if (new_map->map)
			free(new_map->map);
		if (new_map->complete_map)
			free(new_map->complete_map);
		free(new_map);

		return -1;
	}

	if(new_map->config.x_size == 0 || new_map->config.y_size == 0 ||
			new_map->config.resolution == 0.0)
	{
		sprintf(error_str, "File %s contains no gridmap.", filename);
		/* dbug
	if (have_graphics) 
	  gnome_error_dialog_parented(error_str, GTK_WINDOW(window));
	  else 
		 */
		carmen_warn("%s\n", error_str);

		free(new_map);

		return -1;
	}

	if (offlimits_array != NULL) {
		free(offlimits_array);
		offlimits_array = NULL;
		num_offlimits_segments = 0;
		offlimits_capacity = 0;
	}

	if (carmen_map_chunk_exists(filename, CARMEN_MAP_OFFLIMITS_CHUNK)) {
		carmen_map_read_offlimits_chunk(map_filename, &offlimits_array,
				&num_offlimits_segments);
		if (num_offlimits_segments > 0)
			offlimits_capacity = num_offlimits_segments;
	}

	if (place_list != NULL)
	{
		free(place_list->places);
		free(place_list);
		place_list = NULL;
		places_capacity = 0;
	}

	if (carmen_map_chunk_exists(filename, CARMEN_MAP_PLACES_CHUNK)) {
		place_list = (carmen_map_placelist_p)calloc(1, sizeof(carmen_map_placelist_t));
		carmen_test_alloc(place_list);
		carmen_map_read_places_chunk(map_filename, place_list);
		if (place_list->num_places > 0)
			places_capacity = place_list->num_places;
	}

	if(map) {
		free(map->map);
		free(map->complete_map);
		free(map);
	}

	map = new_map;

	if(tmp_pixmap)
		gdk_pixmap_unref(tmp_pixmap);
	if (map_pixmap)
		gdk_pixmap_unref(map_pixmap);


	map_pixmap = NULL;
	map_pixmap = NULL;

	if(backup)
		free(backup);
	backup = (double *)calloc(map->config.x_size * map->config.y_size,
			sizeof(double));
	carmen_test_alloc(backup);
	memcpy(backup, map->complete_map, sizeof(double) * map->config.x_size *
			map->config.y_size);

	modified = 0;
	return 1;
}

void create_open_map_selector(void)
{
	GtkWidget *dialog;
	char parent_dir[1024];
	char *tail;
	int length;

	dialog = gtk_file_chooser_dialog_new("Please select a map for editing.",
			GTK_WINDOW(window),
			GTK_FILE_CHOOSER_ACTION_OPEN,
			GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
			GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT,
			NULL);
	if(strlen(map_filename)) {
		if (map_filename[0] == '/') {
			tail = strrchr(map_filename, '/');
			if (tail != map_filename) {
				length = tail-map_filename;
				if (length < 1023) {
					strncpy(parent_dir, map_filename, length);
					parent_dir[length] = '\0';
				}
			}
		} else if (strrchr(map_filename, '/') != NULL && getenv("PWD") != NULL) {
			if (strlen(getenv("PWD")) < 1023)
				sprintf(parent_dir, "%s/", getenv("PWD"));
			tail = strrchr(map_filename, '/');
			if (tail != map_filename) {
				length = tail-map_filename;
				if (strlen(parent_dir) + length < 1023) {
					strncat(parent_dir, map_filename, length);
					parent_dir[length+strlen(getenv("PWD"))+1] = '\0';
				}
			}
		} else if (getenv("PWD") != NULL) {
			if (strlen(getenv("PWD")) < 1023)
				sprintf(parent_dir, "%s/", getenv("PWD"));
		}
		gtk_file_chooser_set_current_folder (GTK_FILE_CHOOSER (dialog), parent_dir);
	}

	if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT) {
		char *filename;

		filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
		map_open(filename, 1);
		set_up_map_widgets();
		g_free (filename);
	}

	gtk_widget_destroy (dialog);
}

int bmp_open(char *filename, int have_graphics __attribute__ ((unused)))
{
	int x,y;
	CvScalar pixel;
	IplImage* bmpfile = (IplImage*)malloc(sizeof(IplImage));
	bmpfile = cvLoadImage(filename,CV_LOAD_IMAGE_GRAYSCALE);
	int width = bmpfile->width;
	int height = bmpfile->height;
	double resolution = 0.1;
	carmen_map_p new_map;

	if (width <= 0)
	{
		/* dbug
	      gnome_error_dialog_parented("Invalid width : should be greater than 0",
					  GTK_WINDOW(window));
		 */
		carmen_warn("Invalid width : should be greater than 0");
		return -1;
	}

	if (height <= 0)
	{
		/* dbug
	      gnome_error_dialog_parented("Invalid height : should be greater than 0",
					  GTK_WINDOW(window));
		 */
		carmen_warn("Invalid height : should be greater than 0");
		return -1;
	}

	if (resolution <= 0)
	{

		carmen_warn("Invalid resolution : should be greater ");
		return -1;
	}

	new_map = (carmen_map_p)calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(new_map);

	new_map->config.x_size = width;
	new_map->config.y_size = height;
	new_map->config.resolution = resolution;

	new_map->complete_map = (double *)calloc(width*height, sizeof(double));
	carmen_test_alloc(new_map->complete_map);

	new_map->map = (double **)calloc(width*height, sizeof(double *));
	carmen_test_alloc(new_map->map);

	new_map->map = (double **)calloc(width*height, sizeof(double *));
	carmen_test_alloc(new_map->map);
	for (x = 0; x < width; x++)
		new_map->map[x] = new_map->complete_map+x*height;


	for(x=0;x<new_map->config.x_size;x++)
	{
		for(y=0;y<new_map->config.y_size;y++)
		{
		#if CV_MAJOR_VERSION == 2
			pixel = cvGetAt(bmpfile,height-y-1,x);
		#elif CV_MAJOR_VERSION == 3
			pixel = CV_IMAGE_ELEM(bmpfile, CvScalar,height-y-1,x);
		#endif
			new_map->map[x][y] = 1-pixel.val[0]/255;
		}
	}

	if (offlimits_array != NULL) {
		free(offlimits_array);
		offlimits_array = NULL;
		num_offlimits_segments = 0;
		offlimits_capacity = 0;
	}

	if (place_list != NULL)
	{
		free(place_list->places);
		free(place_list);
		place_list = NULL;
		places_capacity = 0;
	}

	if(map) {
		free(map->map);
		free(map->complete_map);
		free(map);
	}

	map = new_map;

	if(tmp_pixmap)
		gdk_pixmap_unref(tmp_pixmap);
	if (map_pixmap)
		gdk_pixmap_unref(map_pixmap);

	map_pixmap = NULL;
	tmp_pixmap = NULL;

	if(backup)
		free(backup);
	backup = (double *)calloc(map->config.x_size * map->config.y_size,
			sizeof(double));
	carmen_test_alloc(backup);
	memcpy(backup, map->complete_map, sizeof(double) * map->config.x_size *
			map->config.y_size);
	modified = 0;
	set_up_map_widgets();
	return 0;
}

void create_import_map_selector(void)
{
	GtkWidget *dialog;
	char parent_dir[1024];
	char *tail;
	int length;
	char bmp_filename[255];

	bmp_filename[0] = '\0';

	dialog = gtk_file_chooser_dialog_new("Please select a bmp file",
			GTK_WINDOW(window),
			GTK_FILE_CHOOSER_ACTION_OPEN,
			GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
			GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT,
			NULL);
	if(strlen(bmp_filename)) {
		if (bmp_filename[0] == '/') {
			tail = strrchr(bmp_filename, '/');
			if (tail != bmp_filename) {
				length = tail-bmp_filename;
				if (length < 1023) {
					strncpy(parent_dir, bmp_filename, length);
					parent_dir[length] = '\0';
				}
			}
		} else if (strrchr(bmp_filename, '/') != NULL && getenv("PWD") != NULL) {
			if (strlen(getenv("PWD")) < 1023)
				sprintf(parent_dir, "%s/", getenv("PWD"));
			tail = strrchr(bmp_filename, '/');
			if (tail != bmp_filename) {
				length = tail-bmp_filename;
				if (strlen(parent_dir) + length < 1023) {
					strncat(parent_dir, bmp_filename, length);
					parent_dir[length+strlen(getenv("PWD"))+1] = '\0';
				}
			}
		} else if (getenv("PWD") != NULL) {
			if (strlen(getenv("PWD")) < 1023)
				sprintf(parent_dir, "%s/", getenv("PWD"));
		}
		gtk_file_chooser_set_current_folder (GTK_FILE_CHOOSER (dialog), parent_dir);
	}

	if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT) {
		char *filename;

		filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
		bmp_open(filename, 1);
		set_up_map_widgets();
		g_free (filename);
	}

	gtk_widget_destroy (dialog);
}

void new_map_button(GtkWidget *selector_button __attribute__ ((unused)),
		gpointer user_data __attribute__ ((unused)))
{
	int width = atoi(gtk_entry_get_text(GTK_ENTRY(width_entry)));
	int height = atoi(gtk_entry_get_text(GTK_ENTRY(height_entry)));
	double resolution = atof(gtk_entry_get_text(GTK_ENTRY(resolution_entry)));
	int i;
	carmen_map_p new_map;

	if (width <= 0)
	{
		/* dbug
      gnome_error_dialog_parented("Invalid width : should be greater than 0",
				  GTK_WINDOW(window));
		 */
		carmen_warn("Invalid width : should be greater than 0");
		return;
	}

	if (height <= 0)
	{
		/* dbug
      gnome_error_dialog_parented("Invalid height : should be greater than 0",
				  GTK_WINDOW(window));
		 */
		carmen_warn("Invalid height : should be greater than 0");
		return;
	}

	if (resolution <= 0)
	{

		carmen_warn("Invalid resolution : should be greater ");
		return;
	}

	new_map = (carmen_map_p)calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(new_map);

	new_map->config.x_size = width;
	new_map->config.y_size = height;
	new_map->config.resolution = resolution;

	new_map->complete_map = (double *)calloc(width*height, sizeof(double));
	carmen_test_alloc(new_map->complete_map);

	new_map->map = (double **)calloc(width*height, sizeof(double *));
	carmen_test_alloc(new_map->map);
	for (i = 0; i < width; i++)
		new_map->map[i] = new_map->complete_map+i*height;

	if (offlimits_array != NULL) {
		free(offlimits_array);
		offlimits_array = NULL;
		num_offlimits_segments = 0;
		offlimits_capacity = 0;
	}

	if (place_list != NULL)
	{
		free(place_list->places);
		free(place_list);
		place_list = NULL;
		places_capacity = 0;
	}

	if(map) {
		free(map->map);
		free(map->complete_map);
		free(map);
	}

	map = new_map;

	if(tmp_pixmap)
		gdk_pixmap_unref(tmp_pixmap);
	if (map_pixmap)
		gdk_pixmap_unref(map_pixmap);

	map_pixmap = NULL;
	tmp_pixmap = NULL;

	if(backup)
		free(backup);
	backup = (double *)calloc(map->config.x_size * map->config.y_size,
			sizeof(double));
	carmen_test_alloc(backup);
	memcpy(backup, map->complete_map, sizeof(double) * map->config.x_size *
			map->config.y_size);

	modified = 0;

	set_up_map_widgets();
	return;
}

void create_new_map(void)
{
	GtkWidget *dialog = NULL;
	GtkWidget *vbox, *hbox, *hbox2, *label, *button;

	dialog = gtk_dialog_new();
	vbox = GTK_DIALOG(dialog)->vbox;
	gtk_window_set_modal(&(GTK_DIALOG(dialog)->window), 1);
	gtk_widget_grab_focus(dialog);

	label = gtk_label_new("New map size: ");
	gtk_box_pack_start(GTK_BOX(vbox), label, TRUE, TRUE, 5);

	hbox = gtk_hbox_new(FALSE, 5);
	gtk_container_add(GTK_CONTAINER(vbox), hbox);

	label = gtk_label_new("Width: ");
	gtk_box_pack_start(GTK_BOX(hbox), label, TRUE, TRUE, 5);

	width_entry = gtk_entry_new_with_max_length(4);
	gtk_widget_set_usize(width_entry, 50, 20);
	gtk_entry_prepend_text(GTK_ENTRY(width_entry), "100");
	gtk_box_pack_start(GTK_BOX(hbox), width_entry, TRUE, TRUE, 5);

	label = gtk_label_new("Height: ");
	gtk_box_pack_start(GTK_BOX(hbox), label, TRUE, TRUE, 5);

	height_entry = gtk_entry_new_with_max_length(4);
	gtk_widget_set_usize(height_entry, 50, 20);
	gtk_entry_prepend_text(GTK_ENTRY(height_entry), "100");
	gtk_box_pack_start(GTK_BOX(hbox), height_entry, TRUE, TRUE, 5);

	hbox2 = gtk_hbox_new(FALSE, 5);
	gtk_container_add(GTK_CONTAINER(vbox), hbox2);

	label = gtk_label_new("Resolution (# of meters per grid cell): ");
	gtk_box_pack_start(GTK_BOX(hbox2), label, TRUE, TRUE, 5);

	resolution_entry = gtk_entry_new_with_max_length(4);
	gtk_widget_set_usize(resolution_entry, 50, 20);
	gtk_entry_prepend_text(GTK_ENTRY(resolution_entry), "0,1");
	gtk_box_pack_start(GTK_BOX(hbox2), resolution_entry, TRUE, TRUE, 5);

	hbox = GTK_DIALOG(dialog)->action_area;

	button = gtk_button_new_with_label("OK");
	gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);

	gtk_signal_connect(GTK_OBJECT(button), "clicked",
			(GtkSignalFunc)new_map_button, NULL);

	gtk_signal_connect_object(GTK_OBJECT(button),
			"clicked", (GtkSignalFunc)gtk_widget_destroy,
			(gpointer)dialog);

	button = gtk_button_new_with_label("Cancel");
	gtk_box_pack_start(GTK_BOX(hbox), button, TRUE, TRUE, 5);

	gtk_signal_connect_object(GTK_OBJECT(button),
			"clicked", (GtkSignalFunc)gtk_widget_destroy,
			(gpointer)dialog);


	gtk_widget_show_all(dialog);

}

void create_save_map_selector(void)
{
	GtkWidget *dialog;
	char parent_dir[1024];
	char *tail;
	int length;

	dialog = gtk_file_chooser_dialog_new("Choose a name for the new map.",
			GTK_WINDOW(window),
			GTK_FILE_CHOOSER_ACTION_SAVE,
			GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
			GTK_STOCK_SAVE, GTK_RESPONSE_ACCEPT,
			NULL);
	if(strlen(map_filename)) {
		if (map_filename[0] == '/') {
			tail = strrchr(map_filename, '/');
			if (tail != map_filename) {
				length = tail-map_filename;
				if (length < 1023) {
					strncpy(parent_dir, map_filename, length);
					parent_dir[length] = '\0';
				}
			}
		} else if (strrchr(map_filename, '/') != NULL && getenv("PWD") != NULL) {
			if (strlen(getenv("PWD")) < 1023)
				sprintf(parent_dir, "%s/", getenv("PWD"));
			tail = strrchr(map_filename, '/');
			if (tail != map_filename) {
				length = tail-map_filename;
				if (strlen(parent_dir) + length < 1023) {
					strncat(parent_dir, map_filename, length);
					parent_dir[length+strlen(getenv("PWD"))+1] = '\0';
				}
			}
		} else if (getenv("PWD") != NULL) {
			if (strlen(getenv("PWD")) < 1023)
				sprintf(parent_dir, "%s/", getenv("PWD"));
		}
		gtk_file_chooser_set_current_folder (GTK_FILE_CHOOSER (dialog), parent_dir);
	}

	if (gtk_dialog_run (GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT) {
		char *filename;

		filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
		map_save(filename);
		g_free (filename);
	}

	gtk_widget_destroy (dialog);
}

/* dbug
void dont_save_question(gint reply, gpointer data)
{
  if(reply == GNOME_YES) 
    {
      if ((int)data == 1)
	create_open_map_selector();
      else if ((int)data == 2)
	create_new_map();
    }
}
 */

void new_map_menu(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	/* dbug
  if(modified)
    gnome_ok_cancel_dialog_modal_parented
      ("The map has been modified.\n"
       "Are you sure you want to\n"
       "throw these changes away?",
       (GnomeReplyCallback)dont_save_question, (gpointer)2, GTK_WINDOW(window));
  else
	 */

	create_new_map();
}

void open_map_menu(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	/* dbug
  if(modified)
    gnome_ok_cancel_dialog_modal_parented
      ("The map has been modified.\n"
       "Are you sure you want to\n"
       "throw these changes away?",
       (GnomeReplyCallback)dont_save_question,(gpointer)1, GTK_WINDOW(window));
  else
	 */

	create_open_map_selector();
}

void save_map_menu(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	if(modified)
		map_save(map_filename);
}

void save_map_as_menu(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	create_save_map_selector();
}

/* dbug
void quit_question(gint reply __attribute__ ((unused)),
		   gpointer data __attribute__ ((unused)))
{
  if(reply == GNOME_YES)
    gtk_exit(1);
}
 */

void import_from_bmp_menu(GtkAction *action __attribute__ ((unused)),
		gpointer user_data __attribute__ ((unused)))
{
	create_import_map_selector();
}

void quit_menu(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	/* dbug
  if(modified)
    gnome_ok_cancel_dialog_modal_parented("The map has been modified.\n"
					  "Are you sure you want to\n"
					  "quit and throw these changes away?",
					  (GnomeReplyCallback)
					  quit_question,
					  NULL, GTK_WINDOW(window));
  else
	 */

	gtk_exit(1);
}

/* undoes the immediately previous action (only one)*/
void undo_menu(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	if(!modified || !backup)
		return;
	memcpy(map->complete_map, backup,
			sizeof(double) * map->config.x_size * map->config.y_size);
	modified--;

	if(tmp_pixmap)
		gdk_pixmap_unref(tmp_pixmap);
	if (map_pixmap)
		gdk_pixmap_unref(map_pixmap);

	map_pixmap = NULL;
	tmp_pixmap = NULL;

	redraw();
}

/* Opens a new window containing helpful info */
void help_menu(GtkAction *action __attribute__ ((unused)), 
		gpointer user_data __attribute__ ((unused)))
{
	GtkWidget *tree;
	static GtkWidget *window1 = NULL;
	GtkWidget *scrolled_win;
	GtkWidget *box;

	if(window1)
		return;

	window1 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_signal_connect (GTK_OBJECT(window1), "delete_event",
			GTK_SIGNAL_FUNC (gtk_widget_destroy), window1);

	box = gtk_vbox_new(FALSE, 0);
	gtk_container_add(GTK_CONTAINER(window1), box);
	gtk_widget_show(box);

	GtkTreeStore *tree_store = gtk_tree_store_new (1, G_TYPE_STRING);
	GtkTreeIter iter1, iter2, iter3, iter4;

	gtk_tree_store_append (tree_store, &iter1, NULL);
	gtk_tree_store_set(tree_store, &iter1, 0, "Menus", -1);
	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "File", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set(tree_store, &iter3, 0, "Open Map", -1);
	gtk_tree_store_append (tree_store, &iter4, &iter3);
	gtk_tree_store_set
	(tree_store, &iter4, 0, "Opens a previously created map file.", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set(tree_store, &iter3, 0, "Save Map", -1);
	gtk_tree_store_append (tree_store, &iter4, &iter3);
	gtk_tree_store_set
	(tree_store, &iter4, 0,
			"Saves the current map to its original file name.", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set(tree_store, &iter3, 0, "Save Map As", -1);
	gtk_tree_store_append (tree_store, &iter4, &iter3);
	gtk_tree_store_set
	(tree_store, &iter4, 0, "Saves the current map to a new file name.", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set(tree_store, &iter3, 0, "Quit", -1);
	gtk_tree_store_append (tree_store, &iter4, &iter3);
	gtk_tree_store_set(tree_store, &iter4, 0, "Exits the program.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Edit", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set(tree_store, &iter3, 0, "Undo", -1);
	gtk_tree_store_append (tree_store, &iter4, &iter3);
	gtk_tree_store_set(tree_store, &iter4, 0, "Undoes the last edit action.", -1);

	gtk_tree_store_append (tree_store, &iter1, NULL);
	gtk_tree_store_set(tree_store, &iter1, 0, "Tools", -1);
	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Brush", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"The brush draws squares ('brush size' on a side) of \n"
			"probability given by 'ink' onto the map. The brush is activated \n"
			"by clicking the left mouse button and dragging the cursor \n"
			"across the screen.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Rectangle", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"The rectangle draws a rectangle of probability 'ink'\n"
			"(either filled or not) with edges of width 'line size' onto the map. \n"
			"The rectangle is activated by clicking the left mouse button, \n"
			"dragging the cursor across the screen, and releasing.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Line", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"The line draws a line of probablilty 'ink' onto the map.\n"
			"The width of the line is (approximately) 'line size'. The line is \n"
			"activated by clicking the left mouse button at the starting location, \n"
			"dragging the cursor across the screen, and releasing.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Fill", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"The fill tool fills in a contiguous area of the same probability\n"
			"with probability 'ink'. The fill tool is activated by clicking the\n"
			"left mouse button on a location in the area you desire to fill.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Fuzzy Fill", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"The fuzzy fill tool fills probability 'ink' into a contiguous area\n"
			"of probability within 'fuzziness' of the probability at the point\n"
			"you clicked. The fill tool is activated by clicking the left mouse \n"
			"button on a location in the area you desire to fill.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Eye Dropper", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"The eye dropper changes the current 'ink' probability \n"
			"to the probability of the grid clicked upon. The eye dropper will get\n"
			"both probabilities and unknown. The eye dropper is actived by \n"
			"clicking the left mouse button on a grid.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Zoom", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"The zoom tool allows you to zoom in or out of the map. \n"
			"Clicking the left mouse button zooms in to the point you clicked.\n"
			"Clicking the right mouse button zooms out from the point you \n"
			"clicked. (currently gtk complains if you zoom in more than twice)", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Mover", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"The mover tool allows you to move the contents of the window.\n"
			"Clicking the left mouse button at the point you want to move, \n"
			"dragging the mouse to where you want that point to be and\n"
			"releasing the button here will move the map in the window as \n"
			"desired. Note that the mover utility won't change your map \n"
			"(as does the zoom).", -1);

	gtk_tree_store_append (tree_store, &iter1, NULL);
	gtk_tree_store_set(tree_store, &iter1, 0, "Settings", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Ink (probability)", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"Ink sets the probability that the tools write onto the map.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Fuzziness", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"Fuzziness sets the range accepted for fuzzy fill.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Brush Size", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"Brush size sets the length of a side of the brush. \n"
			"(the brush is a square)", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Line Size", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"Line size sets the width of lines for the edges of\n"
			"any shapes (rectangles) and the width of lines.", -1);

	gtk_tree_store_append (tree_store, &iter2, &iter1);
	gtk_tree_store_set(tree_store, &iter2, 0, "Shape Fill", -1);
	gtk_tree_store_append (tree_store, &iter3, &iter2);
	gtk_tree_store_set
	(tree_store, &iter3, 0,
			"Shape fill sets whether or not the shapes are filled\n"
			"or not. (Currently the only shape is a rectangle)", -1);

	tree = gtk_tree_view_new_with_model (GTK_TREE_MODEL (tree_store));

	/* The view now holds a reference.  We can get rid of our own
	 * reference */
	g_object_unref (G_OBJECT (tree_store));

	/* Create a cell render and arbitrarily make it red for demonstration
	 * purposes */
	GtkCellRenderer *renderer = gtk_cell_renderer_text_new ();
	g_object_set (G_OBJECT (renderer), "foreground", "black", NULL);

	scrolled_win = gtk_scrolled_window_new (NULL, NULL);
	gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (scrolled_win),
			GTK_POLICY_AUTOMATIC,
			GTK_POLICY_AUTOMATIC);
	gtk_widget_set_usize (scrolled_win, 420, 500);
	gtk_box_pack_start(GTK_BOX(box), scrolled_win, TRUE, TRUE, 0);
	gtk_widget_show (scrolled_win);

	gtk_scrolled_window_add_with_viewport
	(GTK_SCROLLED_WINDOW(scrolled_win), tree);

	GtkTreeViewColumn *column = gtk_tree_view_column_new_with_attributes
			("Help", renderer, "text", 0, NULL);

	gtk_tree_view_append_column (GTK_TREE_VIEW (tree), column);

	gtk_widget_show(tree);
	gtk_widget_show(window1);
	return;
}
