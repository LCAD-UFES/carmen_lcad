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

#include <carmen/global_graphics.h>
#include "map_graphics.h"
#include <carmen/ipc_wrapper.h>
#include "map_interface.h"

#define ELLIPSE_PLOTPOINTS 30

static void redraw (GtkMapViewer *map_view, int map_changed, 
		int viewport_changed);

static carmen_inline int world_to_screen(carmen_world_point_t *wp, carmen_point_t *p,
		GtkMapViewer *map_view)
{
	carmen_map_point_t mp;

	if (carmen_world_to_map(wp, &mp) == -1)
		return (-1);

	mp.y = map_view->internal_map->config.y_size - mp.y;

	p->x = mp.x * map_view->rescale_size;
	p->y = mp.y * map_view->rescale_size;

	p->x -= map_view->x_scroll_adj->value;
	p->y -= map_view->y_scroll_adj->value;

	return (1);
}

static carmen_inline void screen_to_map(carmen_point_t *p, carmen_map_point_t *mp,
		GtkMapViewer *map_view)
{
	mp->x = p->x;
	mp->y = p->y;
	mp->map = map_view->internal_map;

	mp->x /= map_view->rescale_size;
	mp->y /= map_view->rescale_size;

	mp->y = map_view->internal_map->config.y_size - mp->y;
}

static carmen_inline void screen_to_world(carmen_point_t *p, carmen_world_point_t *wp,
		GtkMapViewer *map_view)
{
	carmen_map_point_t mp;

	screen_to_map(p, &mp, map_view);
	carmen_map_to_world(&mp, wp);
}

void carmen_map_graphics_adjust_scrollbars(GtkMapViewer *map_view, 
		carmen_world_point_p new_centre)
{
	double x_value, y_value;
	carmen_point_t screen;
	GtkAdjustment *x_scroll_adj, *y_scroll_adj;

	if (world_to_screen(new_centre, &screen, map_view) == -1)
		return;

	x_scroll_adj = map_view->x_scroll_adj;
	y_scroll_adj = map_view->y_scroll_adj;

	screen.x += map_view->x_scroll_adj->value;
	screen.y += map_view->y_scroll_adj->value;

	x_value = screen.x - (x_scroll_adj->page_size/2);
	y_value = screen.y - (y_scroll_adj->page_size/2);

	if (x_value + x_scroll_adj->page_size > x_scroll_adj->upper)
		x_value = x_scroll_adj->upper - x_scroll_adj->page_size;
	if (x_value < 0)
		x_value = 0;

	if (y_value + y_scroll_adj->page_size > y_scroll_adj->upper)
		y_value = y_scroll_adj->upper - y_scroll_adj->page_size;
	if (y_value < 0)
		y_value = 0;

	if (fabs(x_scroll_adj->value - x_value) < 10.0 &&
			fabs(y_scroll_adj->value - y_value) < 10.0)
		return;

	gtk_adjustment_set_value(x_scroll_adj, x_value);
	gtk_adjustment_set_value(y_scroll_adj, y_value);

	map_view->centre = *new_centre;
}

static int 
move_event(GtkWidget *widget __attribute__ ((unused)), GtkMapViewer *map_view, 
		GdkEventExpose *event __attribute__ ((unused)))
{

	redraw(map_view, 1, 0);

	return 0;
}

static void 
recover_centre(GtkMapViewer *map_view, carmen_world_point_p new_centre) 
{
	carmen_point_t screen;
	double x_value, y_value;

	if (map_view->internal_map == NULL)
		return;

	x_value = map_view->x_scroll_adj->value;
	y_value = map_view->y_scroll_adj->value;

	screen.x = x_value+(map_view->x_scroll_adj->page_size/2);
	screen.y = y_value+(map_view->y_scroll_adj->page_size/2);

	screen_to_world(&screen, new_centre, map_view);
}

static void pixbuf_destroyed(guchar *pixels, 
		gpointer data __attribute__ ((unused)))
{
	free(pixels);
}

static void 
regenerate_map_pixmap(GtkMapViewer *map_view) 
{
	int x_render_size, y_render_size;
	unsigned char *image_data;
	GdkPixbuf *image = NULL;
	GdkPixbuf *image2 = NULL;
	GdkPixbuf *rotated_image = NULL;
	GdkPixbuf *final_image = NULL;
	double x_ratio, y_ratio;
	double scale_to_fit_window;
	carmen_map_config_t config;
	double zoom;
	carmen_map_p superimposed_map;

	if (map_view == NULL || map_view->internal_map == NULL)
		return;

	if (map_view->current_pixbuf != NULL) {
		g_object_unref(map_view->current_pixbuf);
		map_view->current_pixbuf = NULL;
	}

	image_data = carmen_graphics_convert_to_image(map_view->internal_map,
			map_view->draw_flags);

	config = (map_view->internal_map)->config;

	image = gdk_pixbuf_new_from_data((guchar *)image_data, GDK_COLORSPACE_RGB,
			FALSE, 8,  config.y_size, config.x_size,
			config.y_size*3, pixbuf_destroyed, NULL);

	superimposed_map = carmen_map_interface_get_superimposed_map();

	if (superimposed_map)
	{
		image_data = carmen_graphics_convert_to_image(superimposed_map, map_view->draw_flags);

		image2 = gdk_pixbuf_new_from_data((guchar *)image_data, GDK_COLORSPACE_RGB,
				FALSE, 8,  superimposed_map->config.y_size, superimposed_map->config.x_size,
				superimposed_map->config.y_size*3, pixbuf_destroyed, NULL);

		gdk_pixbuf_composite(image2,
				image,
				0, 0,
				config.y_size, config.x_size,
				0, 0,
				1.0, 1.0,
				GDK_INTERP_BILINEAR,
				150);
	}

	rotated_image = gdk_pixbuf_rotate_simple
			(image, GDK_PIXBUF_ROTATE_COUNTERCLOCKWISE);

	if(image)
		g_object_unref(image);
	if(image2)
		g_object_unref(image2);

	x_ratio = map_view->port_size_x / (double)config.x_size;
	y_ratio = map_view->port_size_y / (double)config.y_size;

	scale_to_fit_window = carmen_fmin(x_ratio, y_ratio);
	zoom = 100.0/map_view->zoom;
	map_view->rescale_size = scale_to_fit_window*zoom;

	x_render_size = map_view->rescale_size*config.x_size;
	y_render_size = map_view->rescale_size*config.y_size;

	final_image = gdk_pixbuf_scale_simple(rotated_image, x_render_size,
			y_render_size, GDK_INTERP_TILES);
	map_view->current_pixbuf = final_image;

	g_object_unref(rotated_image);
}

static void 
redraw(GtkMapViewer *map_view, int map_changed, int viewport_changed)
{
	static double last_timestamp = 0.0;
	double now = carmen_get_time();
	if ((now - last_timestamp) < 0.05)
		return;
	last_timestamp = now;

	GdkPixmap *drawing_pixmap;

	GtkWidget *widget;
	int x_render_size, y_render_size;
	carmen_point_t top_left;

	widget = map_view->image_widget;
	if (widget == NULL || widget->window == NULL)
		return;

	if (map_view->internal_map == NULL)
		return;

	if (!map_view->window)
		return;

	if (map_changed)
		regenerate_map_pixmap(map_view);

	if (!map_view->drawing_pixmap || viewport_changed) {
		if (map_view->drawing_pixmap) {
			gdk_pixmap_unref(map_view->drawing_pixmap);
			map_view->drawing_pixmap = NULL;
		}

		drawing_pixmap = gdk_pixmap_new
				(widget->window, map_view->port_size_x, map_view->port_size_y, -1);

		if (drawing_pixmap == NULL)
			carmen_die("Requested pixmap of size %d x %d failed. X must be out "
					"of memory.\n", map_view->port_size_x,
					map_view->port_size_y);

		map_view->drawing_pixmap = drawing_pixmap;
	}

	drawing_pixmap = map_view->drawing_pixmap;

	if (map_view->drawing_gc == NULL)
	{
		map_view->drawing_gc = gdk_gc_new (widget->window);
		gdk_gc_copy (map_view->drawing_gc, widget->style->fg_gc[GTK_WIDGET_STATE (widget)]);
	}

	if (map_view->draw_flags & CARMEN_GRAPHICS_BLACK_AND_WHITE)
		gdk_gc_set_foreground(map_view->drawing_gc, &carmen_white);
	else
		gdk_gc_set_foreground(map_view->drawing_gc, &carmen_light_blue);

	gdk_draw_rectangle (drawing_pixmap, map_view->drawing_gc, TRUE, 0, 0,
			map_view->port_size_x, map_view->port_size_y);

	if (map_view->current_pixbuf) {
		top_left.x = map_view->x_scroll_adj->value;
		top_left.y = map_view->y_scroll_adj->value;

		if (top_left.x + map_view->port_size_x > gdk_pixbuf_get_width(map_view->current_pixbuf))
			x_render_size = gdk_pixbuf_get_width(map_view->current_pixbuf) - top_left.x;
		else
			x_render_size = map_view->port_size_x;

		if (top_left.y + map_view->port_size_y > gdk_pixbuf_get_height(map_view->current_pixbuf))
			y_render_size = gdk_pixbuf_get_height(map_view->current_pixbuf) - top_left.y;
		else
			y_render_size = map_view->port_size_y;

		gdk_draw_pixbuf(map_view->drawing_pixmap,
				widget->style->fg_gc[GTK_WIDGET_STATE (widget)],
				map_view->current_pixbuf, top_left.x, top_left.y,
				0, 0, x_render_size, y_render_size,
				GDK_RGB_DITHER_NONE, 0, 0);
	}

	if (map_view->user_draw_routine != NULL)
		(map_view->user_draw_routine)(map_view);

	gdk_draw_pixmap(widget->window,
			widget->style->fg_gc[GTK_WIDGET_STATE (widget)],
			drawing_pixmap, 0, 0,
			map_view->x_scroll_adj->value,
			map_view->y_scroll_adj->value, -1, -1);
}

static void set_drawing_area_size(GtkMapViewer *map_view)
{
	double zoom = 100/map_view->zoom;

	int port_x = map_view->window->allocation.width - 24;
	int port_y = map_view->window->allocation.height - 24;

	int new_canvas_x = port_x * zoom;
	int new_canvas_y = port_y * zoom;

	map_view->port_size_x = port_x;
	map_view->port_size_y = port_y;

	gtk_drawing_area_size(GTK_DRAWING_AREA(map_view->image_widget),
			new_canvas_x, new_canvas_y);
}

/* redraw the screen from the backing pixmap */
static int expose_event (GtkWidget *widget, GdkEventExpose *event, 
		GtkMapViewer *map_view)
{
	if (event->count > 1)
		return 1;

	if (map_view->window->allocation.width-24 != map_view->port_size_x ||
			map_view->window->allocation.height-24 != map_view->port_size_y) {
		set_drawing_area_size(map_view);
		redraw(map_view, 1, 1);
	} else
		gdk_draw_pixmap(widget->window,
				widget->style->fg_gc[GTK_WIDGET_STATE (widget)],
				map_view->drawing_pixmap,
				event->area.x-map_view->x_scroll_adj->value,
				event->area.y-map_view->y_scroll_adj->value,
				event->area.x,
				event->area.y,
				event->area.width, event->area.height);
	return 1;
}

static int configure_event (GtkWidget *widget __attribute__ ((unused)), 
		GdkEventConfigure *event __attribute__ ((unused)),
		GtkMapViewer *map_view)
{
	if (!map_view || !map_view->window || !map_view->image_widget)
		return 1;

	if (map_view->window->allocation.width-24 != map_view->port_size_x ||
			map_view->window->allocation.height-24 != map_view->port_size_y) {
		set_drawing_area_size(map_view);
		redraw(map_view, 1, 1);
	}

	return 1;
}

static gint rescale_event(GtkWidget *image_widget __attribute__ ((unused)), 
		GtkMapViewer *map_view)
{
	carmen_world_point_t new_centre;

	recover_centre(map_view, &new_centre);
	map_view->centre = new_centre;

	map_view->zoom = GTK_ADJUSTMENT(map_view->zoom_adjustment)->value;
	set_drawing_area_size(map_view);

	carmen_map_graphics_adjust_scrollbars(map_view, &new_centre);

	redraw(map_view, 1, 0);

	return 1;
}

static gint motion_event (GtkWidget *widget __attribute__ ((unused)), 
		GdkEventMotion *event,GtkMapViewer *map_view)
{
	GdkModifierType state;
	carmen_point_t screen_point;
	carmen_map_point_t point;
	carmen_world_point_t world_point;
	gint x, y;

	if (event->is_hint) {
		gdk_window_get_pointer (event->window, &x, &y, &state);
		screen_point.x = x;
		screen_point.y = y;
	} else {
		screen_point.x = event->x;
		screen_point.y = event->y;
		state = event->state;
	}

	if (!(map_view->internal_map))
		return TRUE;

	screen_to_map(&screen_point, &point, map_view);

	if (point.x < 0 || point.x >= (map_view->internal_map)->config.x_size ||
			point.y < 0 || point.y >= (map_view->internal_map)->config.y_size)
		return TRUE;

	if (map_view->button_two_down && !(event->state & ~GDK_BUTTON2_MASK)) {
		screen_to_world(&screen_point, &(map_view->centre), map_view);
		carmen_map_graphics_adjust_scrollbars(map_view, &(map_view->centre));
	} else if (map_view->motion_handler) {
		screen_to_world(&screen_point, &world_point, map_view);
		(map_view->motion_handler)(map_view, &world_point, event);
	}

	return TRUE;
}

static int keyboard_press_event(GtkWidget *widget __attribute__ ((unused)),
		GdkEventKey *event, GtkMapViewer *map_view)
{

	if (!(map_view->internal_map))
		return TRUE;

	if (map_view->keyboard_press_handler) {
		(map_view->keyboard_press_handler)(map_view, event);
	}

	return TRUE;
}

static int button_release_event(GtkWidget *widget __attribute__ ((unused)), 
		GdkEventButton *event, GtkMapViewer *map_view)
{
	carmen_point_t screen_point;
	carmen_world_point_t point;

	screen_point.x = event->x;
	screen_point.y = event->y;

	if (!(map_view->internal_map))
		return TRUE;

	screen_to_world(&screen_point, &point, map_view);

	if (event->button == 2 && !(event->state & ~GDK_BUTTON2_MASK)) {
		map_view->button_two_down = 0;
		map_view->centre = point;
	} else if (map_view->button_release_handler) {
		(map_view->button_release_handler)(map_view, &point, event);
	}

	return TRUE;
}

static int button_press_event(GtkWidget *widget __attribute__ ((unused)), 
		GdkEventButton *event, GtkMapViewer *map_view)
{
	carmen_point_t screen_point;
	carmen_world_point_t point;

	screen_point.x = event->x;
	screen_point.y = event->y;

	if (!(map_view->internal_map))
		return TRUE;

	screen_to_world(&screen_point, &point, map_view);

	if (event->button == 2 && !event->state) {
		map_view->button_two_down = 1;
		carmen_map_graphics_adjust_scrollbars(map_view, &point);
	} else if (map_view->button_press_handler) {
		(map_view->button_press_handler)(map_view, &point, event);
	}

	return TRUE;
}

static void construct_image(int x_size, int y_size, GtkWidget *Parent, 
		GtkMapViewer *map_view)
{
	GtkWidget *scrolled_window;
	GtkWidget *image_widget;

	scrolled_window = gtk_scrolled_window_new (NULL, NULL);
	gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (scrolled_window),
			GTK_POLICY_ALWAYS, GTK_POLICY_ALWAYS);
	gtk_widget_set_usize(scrolled_window, x_size+25, y_size+25);
	gtk_box_pack_start(GTK_BOX(Parent), scrolled_window, TRUE, TRUE, 0);
	map_view->x_scroll_adj = gtk_scrolled_window_get_hadjustment
			(GTK_SCROLLED_WINDOW(scrolled_window));
	map_view->y_scroll_adj = gtk_scrolled_window_get_vadjustment
			(GTK_SCROLLED_WINDOW(scrolled_window));
	gtk_widget_show(scrolled_window);

	map_view->window = scrolled_window;

	image_widget = gtk_drawing_area_new();

	gtk_drawing_area_size (GTK_DRAWING_AREA (image_widget),
			x_size*100.0/map_view->zoom,
			y_size*100.0/map_view->zoom);

	gtk_signal_connect (GTK_OBJECT (image_widget), "expose_event",
			GTK_SIGNAL_FUNC(expose_event), map_view);

	gtk_signal_connect (GTK_OBJECT (image_widget), "motion_notify_event",
			GTK_SIGNAL_FUNC(motion_event), map_view);

	gtk_signal_connect (GTK_OBJECT (image_widget), "button_release_event",
			GTK_SIGNAL_FUNC(button_release_event), map_view);

	gtk_signal_connect (GTK_OBJECT (image_widget), "button_press_event",
			GTK_SIGNAL_FUNC(button_press_event), map_view);

	gtk_signal_connect (GTK_OBJECT (image_widget), "key_press_event",
				GTK_SIGNAL_FUNC(keyboard_press_event), map_view);

	gtk_signal_connect(GTK_OBJECT(image_widget), "configure_event",
			GTK_SIGNAL_FUNC(configure_event), map_view);

	gtk_signal_connect (GTK_OBJECT (map_view->x_scroll_adj), "value_changed",
			GTK_SIGNAL_FUNC(move_event), map_view);

	gtk_signal_connect (GTK_OBJECT (map_view->y_scroll_adj), "value_changed",
			GTK_SIGNAL_FUNC(move_event), map_view);

	gtk_widget_set_events (image_widget, GDK_EXPOSURE_MASK
			| GDK_LEAVE_NOTIFY_MASK
			| GDK_BUTTON_PRESS_MASK
			| GDK_BUTTON_RELEASE_MASK
			| GDK_POINTER_MOTION_MASK
			| GDK_POINTER_MOTION_HINT_MASK);

	gtk_scrolled_window_add_with_viewport
	(GTK_SCROLLED_WINDOW (scrolled_window), image_widget);

	map_view->image_widget = image_widget;

	gtk_widget_set_can_focus((GtkWidget*) map_view->image_widget, TRUE);
}

GtkMapViewer *
carmen_map_graphics_new_viewer(int x_size, int y_size, double initial_zoom) 
{
	/* GtkWidget is the storage type for widgets */
	GtkWidget *map_box;
	GtkWidget *zoom_box;
	GtkWidget *zoom_label;
	GtkWidget *scroll_bar;
	GtkMapViewer *new_map_view;

	new_map_view = (GtkMapViewer *)calloc(1, sizeof(GtkMapViewer));
	carmen_test_alloc(new_map_view);
	new_map_view->zoom = initial_zoom;

	map_box = gtk_vbox_new(FALSE, 0);
	gtk_container_border_width(GTK_CONTAINER(map_box), 0);
	gtk_widget_show(map_box);

	construct_image(x_size, y_size, map_box, new_map_view);

	zoom_box = gtk_hbox_new(FALSE, 0);
	gtk_box_pack_start(GTK_BOX(map_box), zoom_box, FALSE, FALSE, 0);
	gtk_widget_show(zoom_box);

	zoom_label = gtk_label_new(" Zoom");
	gtk_box_pack_start(GTK_BOX(zoom_box), zoom_label, FALSE, FALSE, 0);
	gtk_widget_show(zoom_label);

	new_map_view->zoom_adjustment = gtk_adjustment_new
			(new_map_view->zoom, 30.0, 110.0, 1.0, 10.0, 10.0);
	scroll_bar = gtk_hscale_new(GTK_ADJUSTMENT(new_map_view->zoom_adjustment));
	gtk_scale_set_value_pos(GTK_SCALE(scroll_bar), GTK_POS_LEFT);
	gtk_box_pack_start(GTK_BOX(zoom_box), scroll_bar, TRUE, TRUE, 0);
	gtk_widget_show(scroll_bar);

	gtk_signal_connect (GTK_OBJECT (new_map_view->zoom_adjustment),
			"value_changed", GTK_SIGNAL_FUNC (rescale_event),
			new_map_view);

	new_map_view->map_box = map_box;

	return new_map_view;
}

void
carmen_map_graphics_add_map(GtkMapViewer *map_view, carmen_map_p new_map, 
		int new_flags)
{
	carmen_world_point_t point;

	if ((map_view->internal_map || !new_map) &&
			(new_map->config.x_size != map_view->internal_map->config.x_size || new_map->config.y_size != map_view->internal_map->config.y_size))
		carmen_map_destroy(&(map_view->internal_map));

	if (new_map != NULL) {
		if (!map_view->internal_map)
			map_view->internal_map = carmen_map_clone(new_map);
		else
		{
			carmen_map_copy(map_view->internal_map, new_map);
			map_view->internal_map->config = new_map->config;
		}

		map_view->draw_flags = new_flags;
	}

	point.pose.x = (new_map->config.x_size*3/4)*new_map->config.resolution;
	point.pose.y = (new_map->config.y_size*3/4)*new_map->config.resolution;
	point.pose.theta = 0;
	point.map = map_view->internal_map;
	map_view->centre = point;

	redraw(map_view, 1, 0);
}

void 
carmen_map_graphics_add_drawing_func(GtkMapViewer *map_view, 
		carmen_graphics_mapview_drawing_func_t new_func)
{
	map_view->user_draw_routine = new_func;
}

void carmen_map_graphics_draw_point(GtkMapViewer *map_view, GdkColor *colour, 
		carmen_world_point_p world_point)
{
	carmen_point_t point;

	gdk_gc_set_foreground(map_view->drawing_gc, colour);

	if (world_point->map == NULL)
		return;

	if (world_to_screen(world_point, &point, map_view) == -1)
		return;

	gdk_draw_point(map_view->drawing_pixmap, map_view->drawing_gc, point.x, point.y);
}

void carmen_map_graphics_draw_image(GtkMapViewer *map_view, GdkImage *image, carmen_world_point_p world_point,
		int x_size, int y_size)
{
	GdkRectangle rect;
	carmen_point_t point;

//	gdk_gc_set_foreground(map_view->drawing_gc, colour);

	if (world_point->map == NULL)
		return;

	if (world_to_screen(world_point, &point, map_view) == -1)
		return;

	if (image == NULL)
		return;

	rect.x = carmen_round(point.x);
	rect.y = carmen_round(point.y);

	rect.width = x_size;
	rect.height = y_size;

	gdk_draw_image(map_view->drawing_pixmap, map_view->drawing_gc, image, 0, 0, rect.x - x_size/2, rect.y - y_size/2, x_size, y_size);
}

void carmen_map_graphics_draw_circle(GtkMapViewer *map_view, GdkColor *colour,
		int filled,
		carmen_world_point_p world_point,
		double radius)
{
	GdkRectangle rect;
	carmen_point_t point;

	gdk_gc_set_foreground(map_view->drawing_gc, colour);

	if (world_point->map == NULL)
		return;

	if (world_to_screen(world_point, &point, map_view) == -1)
		return;

	radius /= (map_view->internal_map->config.resolution/map_view->rescale_size);

	rect.x = carmen_round(point.x - radius);
	rect.y = carmen_round(point.y - radius);

	rect.width = carmen_round(2*radius);
	rect.height = carmen_round(2*radius);

	if (rect.width == 0)
		rect.width = 2;
	if (rect.height == 0)
		rect.height = 2;

	gdk_draw_arc(map_view->drawing_pixmap, map_view->drawing_gc,
			filled, rect.x, rect.y, rect.width, rect.height, 0, 360*64);
}

void carmen_map_graphics_draw_arc(GtkMapViewer *map_view, GdkColor *colour, 
		int filled,
		carmen_world_point_p world_point,
		double radius,
		int start, int delta)
{
	GdkRectangle rect;
	carmen_point_t point;

	gdk_gc_set_foreground(map_view->drawing_gc, colour);

	if (world_point->map == NULL)
		return;

	if (world_to_screen(world_point, &point, map_view) == -1)
		return;

	radius /= (map_view->internal_map->config.resolution/map_view->rescale_size);

	rect.x = carmen_round(point.x - radius);
	rect.y = carmen_round(point.y - radius);

	rect.width = carmen_round(2*radius);
	rect.height = carmen_round(2*radius);

	if (rect.width == 0)
		rect.width = 2;
	if (rect.height == 0)
		rect.height = 2;

	gdk_draw_arc(map_view->drawing_pixmap, map_view->drawing_gc,
			filled, rect.x, rect.y, rect.width, rect.height, start, delta);
}

void carmen_map_graphics_draw_line(GtkMapViewer *map_view, GdkColor *colour, 
		carmen_world_point_p start,
		carmen_world_point_p end)
{
	carmen_point_t p1, p2;

	if (start->map == NULL || end->map == NULL)
		return;

	if (world_to_screen(start, &p1, map_view) == -1)
		return;
	if (world_to_screen(end, &p2, map_view) == -1)
		return;

	gdk_gc_set_foreground(map_view->drawing_gc, colour);

	gdk_draw_line(map_view->drawing_pixmap, map_view->drawing_gc, p1.x,
			p1.y, p2.x, p2.y);
}

void carmen_map_graphics_draw_polygon(GtkMapViewer *map_view, GdkColor *colour,
		carmen_world_point_t *points,
		int num_points, int filled)
{
	static GdkPoint *poly = NULL;
	static int poly_length = 0;
	carmen_point_t p1;
	int i;

	if (points[0].map == NULL)
		return;

	if (poly != NULL && poly_length != num_points) {
		free(poly);
		poly = NULL;
	}

	if (poly == NULL) {
		poly = (GdkPoint *)calloc(num_points, sizeof(GdkPoint));
		carmen_test_alloc(poly);
	}

	for (i = 0; i < num_points; i++) {
		if (world_to_screen(points+i, &p1, map_view) == -1)
			return;
		poly[i].x = p1.x;
		poly[i].y = p1.y;
	}

	gdk_gc_set_foreground(map_view->drawing_gc, colour);

	gdk_draw_polygon(map_view->drawing_pixmap, map_view->drawing_gc, filled,
			poly, num_points);
}

void 
carmen_map_graphics_draw_rectangle(GtkMapViewer *map_view, GdkColor *colour, 
		int filled, carmen_world_point_p start,
		carmen_world_point_p end)
{
	carmen_point_t p1, p2;
	int x, y, width, height;

	if (start->map == NULL || end->map == NULL)
		return;

	if (world_to_screen(start, &p1, map_view) == -1)
		return;
	if (world_to_screen(end, &p2, map_view) == -1)
		return;

	width = abs(p1.x-p2.x);
	height = abs(p1.y-p2.y);
	x = carmen_fmin(p1.x, p2.x);
	y = carmen_fmin(p1.y, p2.y);

	gdk_gc_set_foreground(map_view->drawing_gc, colour);
	gdk_draw_rectangle(map_view->drawing_pixmap, map_view->drawing_gc, filled,
			x, y, width, height);

}

//void
//carmen_map_graphics_draw_string(GtkMapViewer *map_view, GdkColor *colour,
//		GdkFont *font, int x, int y,
//		const char *string)
//{
//	gdk_gc_set_foreground (map_view->drawing_gc, colour);
//	gdk_draw_string (map_view->drawing_pixmap, font, map_view->drawing_gc,
//			x, y, string);
//	return;
//}


void
carmen_map_graphics_draw_string(GtkMapViewer *map_view, GdkColor *colour,
		GdkFont *font, carmen_world_point_p start,
		const char *string)
{
	carmen_point_t screen_start;
	int x, y;

	if (start->map == NULL)
		return;

	if (world_to_screen(start, &screen_start, map_view) == -1)
		return;

	x = carmen_round(screen_start.x);
	y = carmen_round(screen_start.y);
	gdk_gc_set_foreground (map_view->drawing_gc, colour);
	gdk_draw_string (map_view->drawing_pixmap, font, map_view->drawing_gc,
			x, y, string);
	return;
}


void
carmen_map_graphics_draw_ellipse(GtkMapViewer *map_view, GdkColor *colour,
		carmen_world_point_p mean, double x_variance,
		double covariance, double y_variance,
		double k)
{
	double len;
	static GdkPoint poly[ELLIPSE_PLOTPOINTS];
	gint i;
	double discriminant, eigval1, eigval2,
	eigvec1x, eigvec1y, eigvec2x, eigvec2y;
	carmen_world_point_t point, e1, e2;
	carmen_point_t p1;

	gdk_gc_set_foreground(map_view->drawing_gc, colour);

	/* check for special case of axis-aligned */
	if (fabs(covariance) < (fabs(x_variance) + fabs(y_variance) + 1e-4) * 1e-4) {
		eigval1 = x_variance;
		eigval2 = y_variance;
		eigvec1x = 1.;
		eigvec1y = 0.;
		eigvec2x = 0.;
		eigvec2y = 1.;
	} else {

		/* compute axes and scales of ellipse */
		discriminant = sqrt(4*carmen_square(covariance) +
				carmen_square(x_variance - y_variance));
		eigval1 = .5 * (x_variance + y_variance - discriminant);
		eigval2 = .5 * (x_variance + y_variance + discriminant);
		eigvec1x = (x_variance - y_variance - discriminant) / (2.*covariance);
		eigvec1y = 1.;
		eigvec2x = (x_variance - y_variance + discriminant) / (2.*covariance);
		eigvec2y = 1.;

		/* normalize eigenvectors */
		len = sqrt(carmen_square(eigvec1x) + 1.);
		eigvec1x /= len;
		eigvec1y /= len;
		len = sqrt(carmen_square(eigvec2x) + 1.);
		eigvec2x /= len;
		eigvec2y /= len;
	}

	/* take square root of eigenvalues and scale -- once this is
     done, eigvecs are unit vectors along axes and eigvals are
     corresponding radii */
	if (eigval1 < 0 || eigval2 < 0) {
		return;
	}

	eigval1 = sqrt(eigval1) * k;
	eigval2 = sqrt(eigval2) * k;
	if (eigval1 < .01) eigval1 = .01;
	if (eigval2 < .01) eigval2 = .01;

	/* compute points around edge of ellipse */
	for (i = 0; i < ELLIPSE_PLOTPOINTS; i++) {
		double theta = M_PI * (-1 + 2.*i/ELLIPSE_PLOTPOINTS);
		double xi = cos(theta) * eigval1;
		double yi = sin(theta) * eigval2;

		point.pose.x = xi * eigvec1x + yi * eigvec2x + mean->pose.x;
		point.pose.y = xi * eigvec1y + yi * eigvec2y + mean->pose.y;
		point.map = map_view->internal_map;

		if (world_to_screen(&point, &p1, map_view) == -1)
			return;

		poly[i].x = p1.x;
		poly[i].y = p1.y;
	}

	/* finally we can draw it */
	gdk_draw_polygon(map_view->drawing_pixmap, map_view->drawing_gc, FALSE,
			poly, ELLIPSE_PLOTPOINTS);

	e1 = *mean;
	e1.pose.x = mean->pose.x + eigval1 * eigvec1x;
	e1.pose.y = mean->pose.y + eigval1 * eigvec1y;

	e2 = *mean;
	e2.pose.x = mean->pose.x - eigval1 * eigvec1x;
	e2.pose.y = mean->pose.y - eigval1 * eigvec1y;

	carmen_map_graphics_draw_line(map_view, colour, &e1, &e2);

	e1.pose.x = mean->pose.x + eigval2 * eigvec2x;
	e1.pose.y = mean->pose.y + eigval2 * eigvec2y;
	e2.pose.x = mean->pose.x - eigval2 * eigvec2x;
	e2.pose.y = mean->pose.y - eigval2 * eigvec2y;

	carmen_map_graphics_draw_line(map_view, colour, &e1, &e2);
}


void 
carmen_map_graphics_add_motion_event(GtkMapViewer *map_view, 
		carmen_graphics_mapview_callback_t
		motion_handler)
{
	map_view->motion_handler = motion_handler;
}

void 
carmen_map_graphics_add_button_release_event(GtkMapViewer *map_view, 
		carmen_graphics_mapview_callback_t
		button_release_handler)
{
	map_view->button_release_handler = button_release_handler;
}

void 
carmen_map_graphics_add_keyboard_press_event(GtkMapViewer *map_view,
		carmen_graphics_mapview_callback_t
		keyboard_press_handler)
{
	map_view->keyboard_press_handler = keyboard_press_handler;
}

void
carmen_map_graphics_add_button_press_event(GtkMapViewer *map_view, 
		carmen_graphics_mapview_callback_t
		button_press_handler)
{
	map_view->button_press_handler = button_press_handler;
}

void 
carmen_map_graphics_redraw(GtkMapViewer *map_view)
{
	static double time_of_last_redraw = 0;
	static double average = 0;
	static int count = 0;
	static double time_of_last_warning = 0;
	double difference;

	difference = carmen_get_time() - time_of_last_redraw;
	if (count > 0) {
		average += difference;
		if ((average / count) < 0.1 && carmen_get_time() -
				time_of_last_warning > 10) {
			carmen_warn("carmen_map_graphics_redraw requests average more "
					"than 10 Hz.\nX will not be able to keep up with these "
					"requests. This is a bug\nand needs to be fixed.\n\n");
			time_of_last_warning = carmen_get_time();
		}
	}
	else
		average = difference;

	redraw(map_view, 0, 0);

	time_of_last_redraw = carmen_get_time();
	count++;
}

void 
carmen_map_graphics_redraw_superimposed(GtkMapViewer *map_view)
{
	redraw(map_view, 1, 0);
}
