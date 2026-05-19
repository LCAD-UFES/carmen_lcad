#!/usr/bin/env python3
# encoding: utf-8

GRAPHICS_EDITOR = 'inkscape'
PROG_NOTES = \
'''
Notes:
    Lane left border RDDF files must be preffixed "lb_" or "LB_".
    Lane right border RDDF files must be preffixed "rb_" or "RB_".
    RDDF files in the rearward direction must be suffixed "_rw" or "_RW".F
'''
PROG_DESCRIPTION = \
f'''
Use case #1:
    edit_graph_inkscape.py   <rddf file 1>  <rddf file 2>  <...>   [ -i <png map image directory> ]   [ -o <output directory> ]
    edit_graph_inkscape.py   -rl <rddf file list (one per line)>   [ -i <png map image directory> ]   [ -o <output directory> ]
        input:  RDDF files and/or map images
        output1: the corresponding SVG file that may be manually edited using '{GRAPHICS_EDITOR}' graphics editor
        output2: (after graphically editing the SVG file): the corresponding RDDF files generated using the cubic Bezier curve algorithm

Use case #2:
    edit_graph_inkscape.py   -svg <SVG file>                       [ -o <output directory> ]
    edit_graph_inkscape.py   -sl  <SVG file list (one per line)>   [ -o <output directory> ]
        input:  SVG files
        output: the corresponding RDDF files generated using the cubic Bezier curve algorithm

Use case #3:
    edit_graph_inkscape.py   -a   <rddf file 1>  <rddf file 2>  <...>   [ -o <output directory> ]
    edit_graph_inkscape.py   -a   -rl <rddf file list (one per line)>   [ -o <output directory> ]
    edit_graph_inkscape.py   -a   -svg <SVG file>                       [ -o <output directory> ]
    edit_graph_inkscape.py   -a   -sl  <SVG file list (one per line)>   [ -o <output directory> ]
        input:  RDDF files or SVG files
        output: the analysis report of RDDF junctions (fork, merge, forward, backward), loop closures and steering angles (phi)
{PROG_NOTES}
'''

import sys, os, argparse
import math
import time
from datetime import datetime
from annotation_image import annotation_image
from xml.dom import minidom
from PIL import Image
from signal import signal, SIGINT
from copy import copy
from inspect import currentframe
import operator
import re
import subprocess


BEZIER_INCREMENT = 0.01         # Increment in meters to draw cubic Bezier curve points (number of points = meters/increment) 
VELOCITY = (10 / 3.6)           # Default x-axis velocity of the vehicle in m/s
GRAPH_RANGE = 150.0             # Defaul graph range in meters for computing the nearby lanes

RGB_COLORS = {'cyan':  (0, 255, 255), 'magenta': (255, 0, 255), 'yellow': (255, 255,   0), 'orange': (255, 165,   0), 
              'green': (0, 255,   0), 'red':     (255, 0,   0), 'blue':   (  0,   0, 255), 'gray':   (230, 230, 230)}
COLOR_LIST = ('magenta', 'green', 'orange', 'cyan', 'red', 'blue', 'gray', 'yellow')
color_index = 0
global parse, args

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __add__(self, p):
        return Point((self.x + p.x), (self.y + p.y))
    
    def __sub__(self, p):
        return Point((self.x - p.x), (self.y - p.y))
    
    def __neg__(self):
        return Point(-self.x, -self.y)
    
    def __eq__(self, p):
        return ((self.x == p.x) and (self.y == p.y))
    
    def __ne__(self, p):
        return not ((self.x == p.x) and (self.y == p.y))
    
    def __str__(self):
        return f'({self.x}, {self.y})'
    
    def mul(self, k):
        return Point((self.x * k), (self.y * k))
    
    def div(self, k):
        return Point((self.x / k), (self.y / k))
    
    def round(self, decimals=3):
        return Point(round(self.x, decimals), round(self.y, decimals))
    
    def norm(self):
        return math.sqrt((self.x * self.x) + (self.y * self.y))
    
    def unit(self):
        norm = self.norm()
        return self.div(norm) if norm != 0 else self
    
    def distance(self, p):
        return (self - p).norm()


def to_Point(p):
    if isinstance(p, tuple) or isinstance(p, list):
        return Point(*p)
    return p


class WayPoint:
    def __init__(self, point, theta, phi=None):
        self.point = to_Point(point)
        self.theta = theta
        self.phi = phi


class EndNode:
    def __init__(self, svg, index, rddf, waypoint_index, point=None):
        self.svg = svg
        self.index = index
        self.rddf = rddf
        self.waypoint_index = waypoint_index
        self.point = to_Point(point)


class JunctionNode:
    def __init__(self, rddf, waypoint_index, point, junction_type, rddf2, waypoint_index2, phi=None):
        self.rddf = rddf
        self.waypoint_index = waypoint_index
        self.point = to_Point(point)
        self.junction_type = junction_type
        self.rddf2 = rddf2
        self.waypoint_index2 = waypoint_index2
        self.phi = phi


class AnnotationNode:
    def __init__(self, name, type, code,theta, x,y,trash):
        self.name = name
        self.type = type
        self.code = code
        self.theta = theta
        self.x = x
        self.y = y
        self.trash = trash
        self.id = ""

class RDDF:
    def __init__(self, svg, index, filename, svg_path, forward=True):
        self.svg = svg
        self.index = index
        self.filename = filename
        self.svg_path = svg_path
        self.forward = forward
        self.meters = 0.0
        self.waypoints = []
        self.junctions = []
        self.bad_phi_waypoints = []


class CurvePoint:
    def __init__(self, svg_point, svg_orientation=None, point=None, orientation=None):
        self.svg_point = to_Point(svg_point)
        self.svg_orientation = to_Point(svg_orientation)
        self.point = to_Point(point)
        self.orientation = to_Point(orientation)


class SVGPath:
    def __init__(self, svg, index, label, forward=True, bezier_points=[], bezier_increment=BEZIER_INCREMENT):
        self.svg = svg
        self.index = index
        self.label = label
        self.forward = forward
        self.bezier_points = bezier_points
        self.bezier_increment = bezier_increment
        self.bezier_curve = []
        self.bezier_base_indexes = []


class SVG:
    def __init__(self, filename, origin=None, width=None, height=None):
        self.filename = filename
        self.origin = to_Point(origin)
        self.width = width
        self.height = height
        self.paths = []
        self.rddfs = []
        self.end_nodes = []


class Node:
    def __init__(self):
        self.id = 0
        self.id_ref_in_lane_graph = 0
        self.lane_id = 0
        self.type = ''  # 'm' for merge node, 'f' for fork node, 'e' for end of road node, 'i' for begin of road node, 'n' for normal node
        self.lon = 0.0
        self.lat = 0.0
        self.rddf_point = Astro_rddf_waypoint()
        self.edges = []
        self.edges_in = []
        self.nearby_lanes = []
        self.nearby_crossroads = []
        self.traffic_restrictions = 0  # LANE_LEFT_WIDTH | LANE_RIGHT_WIDTH | LEFT_MARKING | RIGHT_MARKING | LEVEL | YIELD | BIFURCATION
                                         #     6 bits      |      6 bits      |  3 bits enum |  3 bits enum  | 2 bits| 1 bit |   1 bit
        self.edge_index_ref_in_egdes_vector_of_lane_graph = 0


class Astro_robot_and_trailers_pose_t:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.num_trailers = 0
        self.trailer_theta = []

class Astro_rddf_waypoint:
    def __init__(self):
        self.pose = Astro_robot_and_trailers_pose_t()
        self.phi = 0.0
        self.max_velocity = 0.0
        self.timestamp = 0.0
        self.driver_velocity = 0.0

class Edge:
    def __init__(self):
        self.u = 0
        self.v = 0
        self.u_ref = 0
        self.v_ref = 0
        self.cost = 0

class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = []

class LaneGraphEdge:
    def __init__(self):
        self.u = 0
        self.v = 0
        self.u_ref_in_graph = 0
        self.v_ref_in_graph = 0
        self.cost = 0.0

class LaneGraphNode:
    def __init__(self):
        self.id = 0
        self.id_ref_in_graph = 0
        self.type = ''
        self.lon = 0.0
        self.lat = 0.0
        self.minimum_graph_node_id_in_lane_segment = []
        self.maximum_graph_node_id_in_lane_segment = []
        self.rddf_point = Astro_rddf_waypoint()
        self.edges = []
        self.edges_in = []

class LaneGraph:
    def __init__(self):
        self.nodes = []
        self.edges = []

class NearbyLane:
    def __init__(self):
        self.nearby_lane_id = 0
        self.initial_node_id = 0
        self.nearby_lane_size = 0

class Crossroad:
    def __init__(self):
        self.edge_a = Edge()
        self.edge_b = Edge()


def route_planner_get_lane_left_width (traffic_restrictions, new_traffic_restrictions):
    return float(((traffic_restrictions & 0x3ff) * 0.1) if new_traffic_restrictions == 1 else ((traffic_restrictions & 0x3f) * 0.1))

def route_planner_get_lane_right_width (traffic_restrictions, new_traffic_restrictions):
    return float((((traffic_restrictions & (0x3ff << 10)) >> 10) * 0.1) if new_traffic_restrictions == 1 else (((traffic_restrictions & (0x3f << 6)) >> 6) * 0.1))

def route_planner_set_lane_left_width (traffic_restrictions, lane_width):
    return float((traffic_restrictions & ~0x3ff) | ((int(lane_width * 10.0) & 0x3ff)))

def route_planner_set_lane_right_width (traffic_restrictions, lane_width):
    return float((traffic_restrictions & ~(0x3ff << 10)) | (((int(lane_width * 10.0) & 0x3ff) << 10)))


def get_color():
    global color_index
    index = color_index % len(COLOR_LIST)
    color = list(RGB_COLORS[COLOR_LIST[index]])
    if max(color) > 25:
        decay = (color_index // len(COLOR_LIST) * 80) % (max(color) - 26)
        decay_factor = 1.0 - float(decay) / max(color)
        for i in range(len(color)):
            color[i] = int(decay_factor * color[i] + 0.5)
    
    color_index += 1
    return tuple(color)


def round_limits(limits, round_unit = 70.0):
    try:
        (x_min, y_min, x_max, y_max) = limits
        x_min = math.floor(float(x_min) / round_unit) * round_unit
        y_min = math.floor(float(y_min) / round_unit) * round_unit
        x_max = math.ceil( float(x_max) / round_unit) * round_unit
        y_max = math.ceil( float(y_max) / round_unit) * round_unit
    except TypeError:
        return None
    
    rounded_limits = (x_min, y_min, x_max, y_max)
    return rounded_limits


def minval(a, b):
    if a is None:
        return b
    if b is None:
        return a
    return min(a, b)


def maxval(a, b):
    if a is None:
        return b
    if b is None:
        return a
    return max(a, b)


def get_filelist(filelist_name):
    filelist = []
    if filelist_name:
        with open(filelist_name) as fl:
            filelist = [ f.strip() for f in fl.readlines() if f.strip() and f.strip()[0] != '#' ]
    return filelist


def get_window_limits(window_origin_size_utm):
    try:
        (x_min, y_min, width, height) = window_origin_size_utm
        x_max = x_min + width
        y_max = y_min + height
    except TypeError:
        x_min = y_min = x_max = y_max = None
    return (x_min, y_min, x_max, y_max)


def update_loop_closures(loop_closures, filename, rddf, rddf_index):
    (xs, ys) = round_point(rddf[ 0][:2])
    (xf, yf) = round_point(rddf[-1][:2])
    delta_theta = delta_angle(rddf[1][2], rddf[-1][2])
    dist = distance(rddf[1][:2], rddf[-1][:2])
    dist = max(dist, 0.001)
    phi = steering_angle(args.front_rear, delta_theta, dist)
    loop_closures.append((rddf_index, filename, (xs, ys), (xf, yf), phi))


def loop_closure(rddf, loop_closures, filename):
    global rddf_length
    if len(rddf) <= 1:
        rddf_length = 0.0
    
    min_loop = (2.0 * math.pi) * (args.front_rear / math.tan(args.max_steering))   # C = 2*pi*R , tan(phi) = L/R => C = 2*pi*L/tan(phi)
    (xs, ys) = rddf[ 0][:2]
    (xf, yf) = rddf[-1][:2]
    (xp, yp) = rddf[-2][:2] if len(rddf) >= 3 else (xf, yf)
    dist_f_p = distance((xf, yf), (xp, yp))
    dist_f_s = distance((xf, yf), (xs, ys))
    dist_p_s = distance((xp, yp), (xs, ys))
    rddf_length += dist_f_p
    if not ((rddf_length > min_loop) and (round_point((xf, yf)) == round_point((xs, ys)) or max(dist_f_s, dist_p_s) < dist_f_p)):
        return False
    
    dist = 0.0
    for i in range(1, len(rddf)):
        (xi, yi) = rddf[i][:2]
        (xp, yp) = rddf[i - 1][:2]
        dist_i_p = distance((xi, yi), (xp, yp))
        dist += dist_i_p
        if dist > min_loop:
            break
    
    for i in reversed(range(i)):
        (xf, yf) = rddf[-1][:2]
        (xi, yi) = rddf[i][:2]
        (xn, yn) = rddf[i + 1][:2]
        dist_i_n = distance((xi, yi), (xn, yn))
        dist_i_f = distance((xi, yi), (xf, yf))
        dist_n_f = distance((xn, yn), (xf, yf))
        if round_point((xi, yi)) == round_point((xf, yf)) or max(dist_i_f, dist_n_f) < dist_i_n:
            break
    
    del rddf[:i]
    return True

def filename_split(filename, split_count, rddf_reverse):
    if split_count:
        (file_base, file_ext) = os.path.splitext(filename)
        file_split_text = file_base + f'_split{split_count}' + '_rw' * rddf_reverse + file_ext
        return file_split_text
    return filename


def update_rddf_list(rddf_list, loop_closures, filename, rddf, rddf_forward, found_loop_closure):
    if len(rddf) >= 4:
        (file_base, file_ext) = os.path.splitext(filename)
        filename_forward = (file_base[-3:].lower() != '_rw')
        if filename_forward and not rddf_forward:
            sys.stderr.write(f"warning: suffix '_rw' added to RDDF file '{filename}'\n")
            filename = file_base + '_rw' + file_ext
        if rddf_forward and not filename_forward:
            sys.stderr.write(f"warning: suffix '{file_base[-3:]}' removed from RDDF file '{filename}'\n")
            filename = file_base[:-3] + file_ext
        (xs, ys) = round_point(rddf[0][:2])
        (xf, yf) = round_point(rddf[-1][:2])
        if found_loop_closure or distance((xf, yf), (xs, ys)) <= (2.0 * args.gen_dist):
            update_loop_closures(loop_closures, filename, rddf, len(rddf_list))
        rddf_list.append((filename, rddf, rddf_forward))
        print(f"<--- RDDF file #{len(rddf_list)}: '{filename}' contains {len(rddf)} waypoints")
    else:
        sys.stderr.write(f"error: <--- RDDF file '{filename}' contains {len(rddf)} waypoint\n")


def get_rddf_list(filelist):
    rddf_list = []
    loop_closures = []
    (x_min, y_min, x_max, y_max) = get_window_limits(args.window)

    for filename in filelist:
        rddf = []
        found_loop_closure = False
        last_x = last_y = last_theta = last_rddf_forward = rddf_forward = None
        with open(filename) as f:
            for line_num, line in enumerate(f):
                try:
                    (x, y, theta) = (float(val) for val in line.split()[:3])
                except ValueError:
                    sys.stderr.write(f"warning: Line #{line_num + 1} was discarded from RDDF file '{filename}': invalid float data\n\t{line}\n")
                    continue
                if last_theta is not None:
                    rddf_angle = math.atan2(y - last_y, x - last_x)
                    rddf_forward = (math.cos(last_theta - rddf_angle) > 0.0)
                last_x, last_y, last_theta, last_rddf_forward = x, y, theta, rddf_forward
                rddf.append((x, y, theta))
                if args.window is None:
                    x_min = minval(x, x_min)
                    y_min = minval(y, y_min)
                    x_max = maxval(x, x_max)
                    y_max = maxval(y, y_max)
                if loop_closure(rddf, loop_closures, filename):
                    found_loop_closure = True
                    break
            update_rddf_list(rddf_list, loop_closures, filename, rddf, rddf_forward, found_loop_closure)

    if not rddf_list:
        (x_min, y_min, x_max, y_max) = get_window_limits(args.window)
        if filelist:
            sys.stderr.write('error: no valid RDDF files\n')

    rddf_limits = round_limits((x_min, y_min, x_max, y_max))
    return (rddf_list, rddf_limits, loop_closures)



def get_annotations_list(filename):
    trash_list = []
    annotations_list = []
    try:
        with open(filename) as f:
            for line_num, line in enumerate(f):
                try:
                    annotation_split = line.split()
                    if len(annotation_split) > 4:
                        if annotation_split[0][0] != '#':
                            for i in range(len(annotation_split)):
                                annotation_split[i] = annotation_split[i].strip()
                            annotation = AnnotationNode(annotation_split[0],annotation_split[1],annotation_split[2],float(annotation_split[3]),float(annotation_split[4]),float(annotation_split[5]),annotation_split[6:])
                            annotations_list.append((line_num,annotation))
                        else:
                            trash_list.append((line_num,line))
                    else:
                        trash_list.append((line_num,line))
                except:
                    sys.stderr.write(f"warning: Line #{line_num + 1} in annotation file '{filename}': invalid format  data\n\t{line}\n")
                    continue
    except:
        print("Arquivo de anotação não encontrado: ", filename)
    return (annotations_list,trash_list)


def intersection_area(box1, box2):
    (x1_min, y1_min, x1_max, y1_max) = box1 if not (box1 is None) else (None, None, None, None) 
    (x2_min, y2_min, x2_max, y2_max) = box2 if not (box2 is None) else (None, None, None, None)
    x_min = maxval(x1_min, x2_min)
    y_min = maxval(y1_min, y2_min)
    x_max = minval(x1_max, x2_max)
    y_max = minval(y1_max, y2_max)
    width  = float(x_max - x_min) if not (x_max is None) else 0.0
    height = float(y_max - y_min) if not (y_max is None) else 0.0
    if width <= 0.0 or height <= 0.0:
        return None
    
    x1_off = (x_min - x1_min) if not (x1_min is None) else None
    y1_off = (y_min - y1_min) if not (y1_min is None) else None
    x2_off = (x_min - x2_min) if not (x2_min is None) else None
    y2_off = (y_min - y2_min) if not (y2_min is None) else None
    return (width, height, x1_off, y1_off, x2_off, y2_off)


def get_image_list(imagedir, rddf_limits):
    image_list = []
    image_name_list = []
    mark_dict = {}
    mark_values = imagedir+"/mark_values.txt"
    if args.map_plot_inf:
        with open(mark_values, 'r') as file:
            lines = file.readlines()

        for line in lines:
            if "name" in line:
                line_split = line.split(',')
                name = line_split[0].split(':')[1].strip()
                
                x = line_split[1].split(":")[1].strip()
                y = line_split[2].split(":")[1].strip()
                key = name

                if key not in mark_dict:
                    mark_dict[key] = []
            elif key in mark_dict:
                line_split = line.split(',')
                
                row = px_to_unit(float(line_split[0]),'mm')
                col = px_to_unit(float(line_split[1]),'mm')
                value = line_split[2]

                mark_dict[key].append((int(row), int(col), int(value)))
            
    try:
        (x_min, y_min, x_max, y_max) = rddf_limits
    except TypeError:
        x_min = y_min = x_max = y_max = None
    
    imagedir_entries = sorted(os.listdir(imagedir), reverse=True)


    
    for entry in imagedir_entries:
        img_basename = os.path.basename(entry)
        img_basename_without_ext = os.path.splitext(img_basename)[0]
        img_fullpath = imagedir + '/' + img_basename
        if not os.path.isfile(img_fullpath):
            continue
        try:
            (x_low, y_low) = ( float(val) for val in img_basename_without_ext[1:].split('_') )
            img_type = img_basename_without_ext[:1]
        except ValueError:
            continue
        try:
            img = Image.open(img_fullpath)
            (width, height) = img.size
            img.close()
        except IOError:
            continue
        x_high = x_low + (width  * args.scale)
        y_high = y_low + (height * args.scale)
        image_limits = (x_low, y_low, x_high, y_high)
        if not intersection_area(rddf_limits, image_limits):
            continue

        if args.map_plot_inf:
            image_list.append((img_fullpath, image_limits, img_type,mark_dict[img_fullpath]))
        else:
            image_list.append((img_fullpath, image_limits, img_type, mark_dict))

        image_name_list.append(img_fullpath)
        if rddf_limits is None:
            x_min = minval(x_low,  x_min)
            y_min = minval(y_low,  y_min)
            x_max = maxval(x_high, x_max)
            y_max = maxval(y_high, y_max)
        
    print(f"{len(image_list)} images found in directory '{imagedir}'")
    
    window_limits = (x_min, y_min, x_max, y_max)
    if window_limits == (None, None, None, None):
        usage_exit('filename/-f/--filelist/-i/--imagedir/-w/--window', 'at least one area in UTM coordinates is required')
    
    show_width  = int((x_max - x_min) / args.scale)
    show_height = int((y_max - y_min) / args.scale)
    print(f'viewbox size is {show_width}x{show_height} px = {(x_max - x_min):.0f}x{(y_max - y_min):.0f} meters')

    return (image_list, window_limits)

def set_svg_attributes(svg, svg_file, window_limits):
    now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
    (x_min, y_min, x_max, y_max) = window_limits
    width  = svg_units(x_max - x_min)
    height = svg_units(y_max - y_min)
    svg.setAttribute('id', f'svg_{now}')
    svg.setAttribute('sodipodi:docname', svg_file)
    svg.setAttribute('width',  f"{width}{args.unit * (args.unit != 'px')}")
    svg.setAttribute('height', f"{height}{args.unit * (args.unit != 'px')}")
    svg.setAttribute('viewBox', f"0 0 {width} {height}")
    svg.setAttribute('xmlns:dc', 'http://purl.org/dc/elements/1.1/')
    svg.setAttribute('xmlns:cc', 'http://creativecommons.org/ns#')
    svg.setAttribute('xmlns:rdf', 'http://www.w3.org/1999/02/22-rdf-syntax-ns#')
    svg.setAttribute('xmlns:svg', 'http://www.w3.org/2000/svg')
    svg.setAttribute('xmlns', 'http://www.w3.org/2000/svg')
    svg.setAttribute('xmlns:xlink', 'http://www.w3.org/1999/xlink')
    svg.setAttribute('xmlns:sodipodi', 'http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd')
    svg.setAttribute('xmlns:inkscape', 'http://www.inkscape.org/namespaces/inkscape')
    svg.setAttribute('version', '1.1')
    svg.setAttribute('inkscape:version', '0.92.2 (unknown)')


def add_svg_metadata(doc, svg):
    now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
    metadata = doc.createElement('metadata')
    metadata.setAttribute('id', f'metadata_{now}')
    rdf = doc.createElement('rdf:RDF')
    cc_work = doc.createElement('cc:Work')
    cc_work.setAttribute('rdf:about', '')
    dc_format = doc.createElement('dc:format')
    dc_format.appendChild(doc.createTextNode('image/svg+xml'))
    cc_work.appendChild(dc_format)
    dc_type = doc.createElement('dc:type')
    dc_type.setAttribute('rdf:resource', 'http://purl.org/dc/dcmitype/StillImage')
    cc_work.appendChild(dc_type)
    dc_title = doc.createElement('dc:title')
    cc_work.appendChild(dc_title)
    rdf.appendChild(cc_work)
    metadata.appendChild(rdf)
    svg.appendChild(metadata)


def add_svg_defs(doc, svg):
    now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
    defs = doc.createElement('defs')
    defs.setAttribute('id', f'defs_{now}')
    marker = doc.createElement('marker')
    marker.setAttribute('id', 'marker_trapezium')
    marker.setAttribute('viewBox', '0 0 5 5')
    marker.setAttribute('refX', '0')
    marker.setAttribute('refY', '0')
    marker.setAttribute('markerUnits', 'strokeWidth')
    marker.setAttribute('markerWidth',  '5')
    marker.setAttribute('markerHeight', '5')
    marker.setAttribute('orient', 'auto')
    path = doc.createElement('path')
    path.setAttribute('d', 'M 0.0,-0.5 L 0.0,0.5 L -5.0,2.5 L -5.0,-2.5 z')
    path.setAttribute('fill', 'context-stroke')
    marker.appendChild(path)
    defs.appendChild(marker)
    svg.appendChild(defs)


def add_svg_namedview(doc, svg, window_limits):
    now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
    (x_min, y_min, x_max, y_max) = window_limits
    width  = svg_units(x_max - x_min)
    height = svg_units(y_max - y_min)
    zoom = min((1435.0 / unit_to_px(width, args.unit)), (900.0 / unit_to_px(height, args.unit))) - 0.01
    cx = (unit_to_px(width,  args.unit) / 2.0) + (150.0 / zoom)
    cy = (unit_to_px(height, args.unit) / 2.0)
    namedview = doc.createElement('sodipodi:namedview')
    namedview.setAttribute('id', f'namedview_{now}')
    namedview.setAttribute('inkscape:document-units', args.unit)
    namedview.setAttribute('inkscape:current-layer', svg.getAttribute('id'))
    namedview.setAttribute('inkscape:window-x', '0')
    namedview.setAttribute('inkscape:window-y', '0')
    namedview.setAttribute('inkscape:window-width',  f'{width}')
    namedview.setAttribute('inkscape:window-height', f'{height}')
    namedview.setAttribute('inkscape:zoom', f'{zoom:.2f}')
    namedview.setAttribute('inkscape:cx', f'{cx}')
    namedview.setAttribute('inkscape:cy', f'{cy}')
    namedview.setAttribute('inkscape:window-maximized', '1')
    namedview.setAttribute('inkscape:pageopacity', '0')
    namedview.setAttribute('inkscape:pageshadow', '2')
    namedview.setAttribute('pagecolor', '#ffffff')
    namedview.setAttribute('bordercolor', '#666666')
    namedview.setAttribute('borderopacity', '1')
    namedview.setAttribute('objecttolerance', '10')
    namedview.setAttribute('gridtolerance', '10')
    namedview.setAttribute('guidetolerance', '10')
    namedview.setAttribute('showgrid', 'false')
    svg.appendChild(namedview)


def add_svg_images(doc, svg, window_limits, image_list):
    (win_x_min, win_y_min, _, win_y_max) = window_limits
    group = group_img_type = group_img_mark_type = None
    now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
    if args.map_plot_inf:
        group_mark = doc.createElement('g')
        group_mark.setAttribute('inkscape:label', f'images_mark')
        group_mark.setAttribute('inkscape:group_markmode', 'layer')
        group_mark.setAttribute('style', 'display:inline')
        group_mark.setAttribute('sodipodi:insensitive', 'true')

    for (img_fullpath, image_limits, img_type, mark_list) in image_list:
        now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
        if group_img_type != img_type:
            group_img_type = img_type
            if group:
                svg.appendChild(group)
            group = doc.createElement('g')
            group.setAttribute('id', f'layer_{now}')
            group.setAttribute('inkscape:label', f'images_{group_img_type}')
            group.setAttribute('inkscape:groupmode', 'layer')
            group.setAttribute('style', 'display:inline')
            group.setAttribute('sodipodi:insensitive', 'true')

        img_basename = os.path.basename(img_fullpath)
        img_abspath = os.path.abspath(img_fullpath)
        (x_min, y_min, x_max, y_max) = image_limits
        x = svg_units(x_min - win_x_min)
        y = svg_units(win_y_max - y_max)  # SVG y-axis coordinates are top-down
        width  = svg_units(x_max - x_min)
        height = svg_units(y_max - y_min)
        image = doc.createElement('image')
        image.setAttribute('id', f'image_{now}')
        image.setAttribute('inkscape:label', f'image_{img_basename}')
        image.setAttribute('x', f'{x}')
        image.setAttribute('y', f'{y}')
        image.setAttribute('width',  f'{width}')
        image.setAttribute('height', f'{height}')
        image.setAttribute('xlink:href', f'file://{img_abspath}')
        image.setAttribute('sodipodi:absref', img_abspath)
        image.setAttribute('preserveAspectRatio', 'none')
        group.appendChild(image)
        cont = 0
        if args.map_plot_inf:
            for row,col,value in mark_list:
                x = svg_units(x_min - win_x_min) - col
                y = svg_units(win_y_max - y_max) - row
                
                now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
                circle = doc.createElement('circle')
                circle.setAttribute('id', f'mark_{now}')
                circle.setAttribute('cx', f'{x}')
                circle.setAttribute('cy', f'{y}')
                circle.setAttribute('r', f'{1}')
                circle.setAttribute('style', 'fill:#000000;stroke:#000000;stroke-width:0.1;')
                group_mark.appendChild(circle)
    if group:
        svg.appendChild(group)
    if args.map_plot_inf:
        if group_mark:
            svg.appendChild(group_mark)

def group_rddfs(rddf_list):
    group_label_dict = {'lb_':('left_borders', 2), 'rb_':('right_borders', 3)}
    path_color_dict = {}
    pi_width = len(str(len(rddf_list)))
    rddf_list_grouped = []
    for rddf_index, (rddf_file, rddf, rddf_forward) in enumerate(rddf_list):
        (group_label, group_order) = group_label_dict.get(os.path.basename(rddf_file)[:3].lower(), ('paths', 1))
        if group_label == 'paths':
            stroke_dasharray = 'none' if rddf_forward else '1.6,0.8'
            stroke_width = svg_units(args.stroke_width)
            path_key = os.path.realpath(rddf_file)
            if not (stroke_color := path_color_dict.get(path_key)):
                stroke_color = path_color_dict[path_key] = get_color()
        else: # borders
            stroke_dasharray = '0.4,0.8' if rddf_forward else '0.4,0.8,1.6,0.8'
            stroke_width = svg_units(args.stroke_width) * 0.5
            path_key = os.path.dirname(os.path.realpath(rddf_file)) + '/' + os.path.basename(rddf_file)[3:]
            if not (path_stroke_color := path_color_dict.get(path_key)):
                path_stroke_color = path_color_dict[path_key] = get_color()
            light_factor = 0.5
            stroke_color = tuple(round(255 * light_factor + color * (1 - light_factor)) for color in path_stroke_color)
        if not rddf_forward:
            group_label += '_rw'
            group_order *= 10
        rddf_order = rddf_index + group_order * pow(10, pi_width)
        rddf_list_grouped.append((rddf_order, group_label, rddf_index, rddf_file, rddf, rddf_forward, stroke_dasharray, stroke_width, stroke_color))

    return sorted(rddf_list_grouped, reverse=True)


def add_svg_paths(doc, svg, window_limits, rddf_list):
    (x_min, _, _, y_max) = window_limits
    rddf_list_grouped = group_rddfs(rddf_list)
    pi_width = len(str(len(rddf_list_grouped)))
    path_template = 'path{:0' + str(pi_width) + '}'
    group = path_group_label = None
    for (_, group_label, rddf_index, rddf_file, rddf, rddf_forward, stroke_dasharray, stroke_width, stroke_color) in rddf_list_grouped:
        now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
        if path_group_label != group_label:
            path_group_label = group_label
            if group:
                svg.appendChild(group)
            group = doc.createElement('g')
            group.setAttribute('id', f'layer_{now}')
            group.setAttribute('inkscape:label', group_label)
            group.setAttribute('inkscape:groupmode', 'layer')
            group.setAttribute('style', 'display:inline' + ';stroke-dashoffset:0;stroke-dasharray:' + stroke_dasharray)
            group.setAttribute('marker-end', 'url(#marker_trapezium)')
        path = doc.createElement('path')
        path.setAttribute('id', f'path_{now}')
        path.setAttribute('inkscape:label', path_template.format(rddf_index + 1) + '_' + os.path.basename(rddf_file))
        path.setAttribute('style', 'fill:none;stroke:#' + ('{:02x}' * len(stroke_color)).format(*stroke_color) + f';stroke-width:{stroke_width};' +
                          'stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;' + 
                          'stroke-dashoffset:0;stroke-dasharray:' + stroke_dasharray)
        path.setAttribute('sodipodi:nodetypes', 'c' + 'a' * (len(rddf) - 2) + 'c')
        path.setAttribute('inkscape:connector-curvature', '0')
        (x0, y0, theta0) = rddf[0]
        theta0 = theta0 if rddf_forward else (theta0 + math.pi)
        x0_units = svg_units(x0 - x_min)
        y0_units = svg_units(y_max - y0)  # SVG y-axis coordinates are top-down
        d = f'M {x0_units},{y0_units} C '
        for i in range(1, len(rddf)):
            (x1, y1, theta1) = rddf[i]
            theta1 = theta1 if rddf_forward else (theta1 + math.pi)
            handle_length = distance((x0, y0), (x1, y1)) / 3
            hx0 = x0 + handle_length * math.cos(theta0)
            hy0 = y0 + handle_length * math.sin(theta0)
            hx1 = x1 - handle_length * math.cos(theta1)
            hy1 = y1 - handle_length * math.sin(theta1)
            hx0_units = svg_units(hx0 - x_min)
            hy0_units = svg_units(y_max - hy0)  # SVG y-axis coordinates are top-down
            hx1_units = svg_units(hx1 - x_min)
            hy1_units = svg_units(y_max - hy1)  # SVG y-axis coordinates are top-down
            x1_units  = svg_units(x1 - x_min)
            y1_units  = svg_units(y_max - y1)   # SVG y-axis coordinates are top-down
            d += f'\n{hx0_units},{hy0_units} {hx1_units},{hy1_units} {x1_units},{y1_units} '
            (x0, y0, theta0) = (x1, y1, theta1)
        path.setAttribute('d', d)
        group.appendChild(path)
    if group:
        svg.appendChild(group)


def add_svg_loop_closures(doc, svg, window_limits, loop_closures):
    (x_min, _, _, y_max) = window_limits
    group = None
    pi_width = len(str(max(data[0] for data in loop_closures) if loop_closures else 0))
    path_template = 'p{:0' + str(pi_width) + '}'
    for (rddf_index, filename, (xs, ys), (xf, yf), _) in reversed(loop_closures):
        if "lb_" not in filename and "rb" not in filename:
            now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
            if group is None:
                group = doc.createElement('g')
                group.setAttribute('id', f'layer_{now}')
                group.setAttribute('inkscape:label', 'loop_closures')
                group.setAttribute('inkscape:groupmode', 'layer')
                group.setAttribute('style', 'display:inline')
            cx = svg_units((xs + xf) / 2 - x_min)
            cy = svg_units(y_max - (ys + yf) / 2)  # SVG y-axis coordinates are top-down
            rgb_color = 'rgb(255,0,0)' if ((xs, ys) != (xf, yf)) else 'rgb(0,255,0)'
            circle = doc.createElement('circle')
            circle.setAttribute('id', f'circle_{now}')
            circle.setAttribute('inkscape:label', 'loop_' + path_template.format(rddf_index + 1) + '_' + os.path.basename(filename))
            circle.setAttribute('cx', f'{cx}')
            circle.setAttribute('cy', f'{cy}')
            circle.setAttribute('r',  '10')
            circle.setAttribute('style', f'fill:none;stroke:{rgb_color};stroke-width:1;stroke-miterlimit:4;stroke-dasharray:none')
            group.appendChild(circle)
    if group:
        svg.appendChild(group)


def get_junctions_from_svg(svg_data):
    pi_width = len(str(len(svg_data.rddfs)))
    ni_width = len(str(max(max(j.waypoint_index if j.waypoint_index != -1 else (len(j.rddf.waypoints) - 1) \
                               for j in rddf.junctions) if rddf.junctions else 0 for rddf in svg_data.rddfs) if svg_data.rddfs else 0))
    junction_template = 'p{:0' + str(pi_width) + '}_n{:0' + str(ni_width) + '}_p{:0' + str(pi_width) + '}_n{:0' + str(ni_width) + '}'
    junction_dict = {}
    for rddf in svg_data.rddfs:
        if "lb_" not in rddf.filename and "rb_" not in rddf.filename:
            for j in rddf.junctions:
                if not (j.phi is None):
                    w_index  = j.waypoint_index  if j.waypoint_index  != -1 else (len(j.rddf.waypoints)  - 1)
                    w_index2 = j.waypoint_index2 if j.waypoint_index2 != -1 else (len(j.rddf2.waypoints) - 1)
                    key1 = junction_template.format(j.rddf.index  + 1, w_index,  j.rddf2.index + 1, w_index2)
                    key2 = junction_template.format(j.rddf2.index + 1, w_index2, j.rddf.index  + 1, w_index )
                    value1 = (j.point, j.rddf2.waypoints[w_index2].point, j.phi)
                    value2 = (j.rddf2.waypoints[w_index2].point, j.point, j.phi)
                    if key1 < key2:
                        junction_dict[key1] = value1
                    else:
                        junction_dict[key2] = value2
    return junction_dict


def add_svg_junctions(doc, svg, window_limits, junction_dict):
    (x_min, _, _, y_max) = window_limits
    group = None
    for key, (point, point2, _) in sorted(junction_dict.items(), reverse=True):
        now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
        if group is None:
            group = doc.createElement('g')
            group.setAttribute('id', f'layer_{now}')
            group.setAttribute('inkscape:label', 'junctions')
            group.setAttribute('inkscape:groupmode', 'layer')
            group.setAttribute('style', 'display:inline')
        rgb_color = 'rgb(255,0,0)' if (point != point2) else 'rgb(0,255,0)'
        cx = svg_units((point.x + point2.x) / 2 - x_min)
        cy = svg_units(y_max - (point.y + point2.y) / 2)  # SVG y-axis coordinates are top-down
        polygon = doc.createElement('polygon')
        polygon.setAttribute('id', f'polygon_{now}')
        polygon.setAttribute('inkscape:label', f'junction_{key}')
        polygon.setAttribute('points', f'{cx - 5},{cy} {cx},{cy - 10} {cx + 5},{cy} {cx},{cy + 10}')
        polygon.setAttribute('style', f'fill:none;stroke:{rgb_color};stroke-width:1;stroke-miterlimit:4;stroke-dasharray:none')
        group.appendChild(polygon)
    if group:
        svg.appendChild(group)


def add_svg_annotations(doc, svg, window_limits, annotations_list):
    (x_min, _, _, y_max) = window_limits
    group = None
    group_images = None

    for annotation in annotations_list:
        annotation = annotation[1]
       
        now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')

        if group is None and group_images is None:
            group = doc.createElement('g')
            group.setAttribute('id', f'layer_{now}')
            group.setAttribute('inkscape:label', 'annotation')
            group.setAttribute('inkscape:groupmode', 'layer')
            group.setAttribute('style', 'display:inline')
            group.setAttribute('sodipodi:insensitive', 'true')

            group_images = doc.createElement('g')
            group_images.setAttribute('id', f'layer_{now}')
            group_images.setAttribute('inkscape:label', 'annotation_images')
            group_images.setAttribute('inkscape:groupmode', 'layer')
            group_images.setAttribute('style', 'display:inline')
            group_images.setAttribute('sodipodi:insensitive', 'true')

        if (int(annotation.type) == 3 or int(annotation.type) == 14 or int(annotation.type) == 22):
            if len(annotation.trash) <= 2:
                circle = doc.createElement('circle')
                circle.setAttribute('id', f'annotation_{now}')
                circle.setAttribute('inkscape:label', f'{annotation.name}')
                circle.setAttribute('cx', f'{svg_units(annotation.x - x_min)}')
                circle.setAttribute('cy', f'{svg_units(y_max - annotation.y)}')
                circle.setAttribute('r', f'{svg_units(annotation.trash[0])}')
                circle.setAttribute('style', 'fill:none;stroke:#000000;stroke-width:0.4;')
                annotation.id = f'annotation_{now}'

                group.appendChild(circle)

            if len(annotation.trash) > 2:
                poli_points = []
                
                for i in range(2, len(annotation.trash), 2):
                    try:
                        x_units  = svg_units(float(annotation.trash[i]) - x_min)
                        y_units  = svg_units(y_max - float(annotation.trash[i + 1]))  

                        poli_points.append(f"{x_units},{y_units}")
                    except:
                        print("Está faltando algum ponto para delimitar a faixa de pedestre.")

                points = ' '.join(poli_points)

                polyLine = doc.createElement('path')
                polyLine.setAttribute('id', f'annotation_{now}')
                polyLine.setAttribute('inkscape:label', f'{annotation.name}')
                polyLine.setAttribute('d',f'M {points} Z')
                polyLine.setAttribute('style', f'fill:none;stroke:#000000;stroke-width:0.4);')

                annotation.id = f'annotation_{now}'

                group.appendChild(polyLine)

            try:
                image = doc.createElement('image')
                image.setAttribute('x', f'{svg_units(annotation.x - x_min)}')
                image.setAttribute('y',f'{svg_units(y_max - annotation.y)}')
                image.setAttribute('width', str(2))
                image.setAttribute('height', str(2))
                image.setAttribute('id', f'annotation_image_{now}')
                image.setAttribute('inkscape:label', f'annotation_image_{annotation.name}_{annotation.type}_{annotation.code}')
                image.setAttribute('href', str(annotation_image[int(annotation.type)][int(annotation.code)]))
                group_images.appendChild(image)
            except:
                print("Annotação não contem imagem")
        else:     
            theta0 = annotation.theta + math.pi/2

            xs = (annotation.x)
            xf = (annotation.x)

            ys = (annotation.y)
            yf = (annotation.y)

            handle_length = 6
            xs = xs + handle_length * math.cos(theta0)
            ys = ys + handle_length * math.sin(theta0)

            xf = xf - handle_length * math.cos(theta0)
            yf = yf - handle_length * math.sin(theta0)

            x1_units  = svg_units(xs - x_min)
            x2_units  = svg_units(xf - x_min)

            y1_units  = svg_units(y_max - ys)
            y2_units  = svg_units(y_max - yf)  

            arrow_width = 0.5

            x1_base = xs + arrow_width * math.cos(annotation.theta)
            y1_base = ys + arrow_width * math.sin(annotation.theta)
            x2_base = xf + arrow_width * math.cos(annotation.theta)
            y2_base = yf + arrow_width * math.sin(annotation.theta)

            x1_units_2  = svg_units(x1_base - x_min)
            x2_units_2  = svg_units(x2_base - x_min)

            y1_units_2  = svg_units(y_max - y1_base)
            y2_units_2  = svg_units(y_max - y2_base)  

            xs = (annotation.x)
            xf = (annotation.x)

            ys = (annotation.y)
            yf = (annotation.y)

            handle_length = 0.8
            xs = xs + handle_length * math.cos(theta0)
            ys = ys + handle_length * math.sin(theta0)

            xf = xf - handle_length * math.cos(theta0)
            yf = yf - handle_length * math.sin(theta0)

            x1_base = xs + arrow_width * math.cos(annotation.theta)
            y1_base = ys + arrow_width * math.sin(annotation.theta)
            x2_base = xf + arrow_width * math.cos(annotation.theta)
            y2_base = yf + arrow_width * math.sin(annotation.theta)

            x1_mini_base  = svg_units(x1_base - x_min)
            x2_mini_base  = svg_units(x2_base - x_min)

            y1_mini_base  = svg_units(y_max - y1_base)
            y2_mini_base  = svg_units(y_max - y2_base)  

            handle_length = 2.0
            xs = annotation.x + handle_length * math.cos(annotation.theta)
            ys = annotation.y + handle_length * math.sin(annotation.theta)

            x1_units_end  = svg_units(xs - x_min)
            y1_units_end  = svg_units(y_max - ys)
             
            points = f'{x1_mini_base},{y1_mini_base} {x1_units_end},{y1_units_end} {x2_mini_base},{y2_mini_base} {x1_units_2},{y1_units_2} {x1_units},{y1_units} {x2_units},{y2_units} {x2_units_2},{y2_units_2}'
            polygon = doc.createElement('polygon')
            polygon.setAttribute('id', f'annotation_{now}')
            polygon.setAttribute('inkscape:label', f'{annotation.name}')
            polygon.setAttribute('points',f'{points}')
            polygon.setAttribute('style', f'fill:#ff6961;stroke-width:1);')
            annotation.id = f'annotation_{now}'

            group.appendChild(polygon)

            try:
                image = doc.createElement('image')
                image.setAttribute('x', f'{svg_units(x2_units)}')
                image.setAttribute('y',f'{svg_units(y2_units)}')
                image.setAttribute('width', str(2))
                image.setAttribute('height', str(2))
                image.setAttribute('id', f'annotation_image_{now}')
                image.setAttribute('inkscape:label', f'annotation_image_{annotation.name}_{annotation.type}_{annotation.code}')
                image.setAttribute('href', str(annotation_image[int(annotation.type)][int(annotation.code)]))
                group_images.appendChild(image)
            except:
                print("Annotação não contem imagem")

    if group:
        svg.appendChild(group)
        svg.appendChild(group_images)


def add_svg_bad_phis(doc, svg, window_limits, svg_data, junction_dict, loop_closures):
    (x_min, y_min, x_max, y_max) = window_limits
    group = None
    bad_phis = []
    pi_width = len(str(len(svg_data.rddfs)))
    ni_width = len(str(max(max(rddf.bad_phi_waypoints) if rddf.bad_phi_waypoints else 0 for rddf in svg_data.rddfs) if svg_data.rddfs else 0))
    cx_width = len(str(int(svg_units(x_max - x_min))))
    cy_width = len(str(int(svg_units(y_max - y_min))))
    path_template = 'p{:0' + str(pi_width) + '}'
    node_template = 'p{:0' + str(pi_width) + '}_n{:0' + str(ni_width) + '}'
    point_template = '{:0' + str(cx_width + 3) + '.2f}_{:0' + str(cy_width + 3) + '.2f}'
    for rddf in svg_data.rddfs:
        if "lb_" not in rddf.filename and "rb" not in rddf.filename:
            for waypoint_index in rddf.bad_phi_waypoints:
                w = rddf.waypoints[waypoint_index]
                key = node_template.format(rddf.index + 1, waypoint_index if waypoint_index != -1 else (len(rddf.waypoints) - 1))
                rgb_color = 'rgb(0,0,255)'
                bad_phis.append((key, w.point, w.phi, rgb_color))
    for (rddf_index, filename, (xs, ys), (xf, yf), phi) in loop_closures:
        if "lb_" not in filename and "rb" not in filename:
            if is_bad_phi(phi):
                key = 'loop_' + path_template.format(rddf_index + 1) + '_' + os.path.basename(filename)
                rgb_color = 'rgb(255,0,0)' if ((xs, ys) != (xf, yf)) else 'rgb(0,255,0)'
                bad_phis.append((key, (Point(xs, ys) + Point(xf, yf)).div(2), phi, rgb_color))
    for key, (point, point2, phi) in junction_dict.items():
        if is_bad_phi(phi):
            key = 'junction_' + key
            rgb_color = 'rgb(255,0,0)' if (point != point2) else 'rgb(0,255,0)'
            bad_phis.append((key, (point + point2).div(2), phi, rgb_color))
    for (key, point, phi, rgb_color) in reversed(bad_phis):
        now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
        if group is None:
            group = doc.createElement('g')
            group.setAttribute('id', f'layer_{now}')
            group.setAttribute('inkscape:label', 'bad_phis')
            group.setAttribute('inkscape:groupmode', 'layer')
            group.setAttribute('style', 'display:inline')
        cx = svg_units(point.x - x_min)
        cy = svg_units(y_max - point.y)  # SVG y-axis coordinates are top-down
        cx_view = cx
        cy_view = svg_units(point.y - y_min)
        rect = doc.createElement('rect')
        rect.setAttribute('id', f'rect_{now}')
        rect.setAttribute('inkscape:label', 'bad_phi_' + point_template.format(cx_view, cy_view) + '_' + key)
        rect.setAttribute('x', f'{cx - 5}')
        rect.setAttribute('y', f'{cy - 5}')
        rect.setAttribute('width',  '10')
        rect.setAttribute('height', '10')
        rect.setAttribute('style', f'fill:none;stroke:{rgb_color};stroke-width:1;stroke-miterlimit:4;stroke-dasharray:none')
        group.appendChild(rect)
    if group:
        svg.appendChild(group)


def get_svg_filename(x_origin, y_origin):
    svg_filename = f'@{x_origin:.0f}_{y_origin:.0f}.svg'
    return svg_filename


def write_svg(rddf_list, image_list, window_limits, loop_closures, svg_data, annotations_list):
    (x_min, y_min, _, _) = window_limits
    svg_filename = get_svg_filename(x_min, y_min)
    svg_file = args.outputdir + '/' + svg_filename
    doc = minidom.Document()
    svg = doc.createElement('svg')
    set_svg_attributes(svg, svg_filename, window_limits)
    add_svg_metadata(doc, svg)
    add_svg_defs(doc, svg)
    add_svg_namedview(doc, svg, window_limits)
    add_svg_images(doc, svg, window_limits, image_list)
    junction_dict = get_junctions_from_svg(svg_data)
    add_svg_bad_phis(doc, svg, window_limits, svg_data, junction_dict, loop_closures)
    add_svg_junctions(doc, svg, window_limits, junction_dict)
    add_svg_annotations(doc, svg, window_limits, annotations_list)
    add_svg_loop_closures(doc, svg, window_limits, loop_closures)
    add_svg_paths(doc, svg, window_limits, rddf_list)
    doc.appendChild(svg)
    with open(svg_file, 'w') as svg_f:
        svg_f.write(doc.toprettyxml())
    doc.unlink()
    print(f"SVG file '{svg_file}' may be manually edited by '{GRAPHICS_EDITOR}' graphics editor...{PROG_NOTES}")
    return svg_file


#https://wiki.inkscape.org/wiki/index.php/Units_In_Inkscape
def unit_to_px(val, unit):
    if unit == 'mm':
        val *= (96.0 / 25.4)
    elif unit == 'cm':
        val *= (96.0 / 25.4 * 10.0)
    elif unit == 'm':
        val *= (96.0 / 25.4 * 1000.0)
    elif unit == 'in':
        val *= (96.0)
    elif unit == 'pc':
        val *= (96.0 /  6.0)
    elif unit == 'pt':
        val *= (96.0 / 72.0)
    return val


def px_to_unit(val, unit):
    if unit == 'mm':
        val *= (25.4 / 96.0)
    elif unit == 'cm':
        val *= (25.4 / 96.0 / 10.0)
    elif unit == 'm':
        val *= (25.4 / 96.0 / 1000.0)
    elif unit == 'in':
        val *= ( 1.0 / 96.0)
    elif unit == 'pc':
        val *= ( 6.0 / 96.0)
    elif unit == 'pt':
        val *= (72.0 / 96.0)
    return val


def unit_convert(val_unit):
    if is_float(val_unit):
        val = float(val_unit)
        unit = 'px'
    else: 
        val = float(val_unit[:-2])
        unit = val_unit[-2:]
    if unit == args.unit:
        val_svg = val
    else:       
        val_px = unit_to_px(val, unit)
        val_svg = px_to_unit(val_px, args.unit)    
    return val_svg


def svg_units(val_meters):
    if args.unit == 'mm':
        val_svg_units = val_meters
    else:
        val_svg_units = val_meters / args.scale
    return val_svg_units


def meters(val_svg_units):
    if args.unit == 'mm':
        val_meters = val_svg_units 
    else:
        val_meters = val_svg_units * args.scale
    return val_meters


def svg_to_utm(svg_point, svg_height, origin):
    point = copy(svg_point)
    point.y = svg_height - point.y              # SVG y-axis orientation is downwards, but UTM y-axis orientation is upwards
    point.x = meters(point.x) + origin.x
    point.y = meters(point.y) + origin.y
    return point


def convert_coordinates_from_svg_to_utm(svg_path):
    curve = svg_path.bezier_curve
    for i in range(len(curve)):
        point = svg_to_utm(curve[i].svg_point, svg_path.svg.height, svg_path.svg.origin)
        curve[i].point = point
        if curve[i].svg_orientation:
            orientation = copy(curve[i].svg_orientation)
            orientation.y = -orientation.y      # SVG y-axis orientation is downwards, but UTM y-axis orientation is upwards
            curve[i].orientation = orientation
    return (curve)


def normalize(point):
    if point == None:
        return None
    (x, y) = point
    (norm_x, norm_y) = (0.0, 0.0)
    norm = math.sqrt(x * x + y * y)
    if norm != 0.0:
        norm_x = x / norm
        norm_y = y / norm
    return (norm_x, norm_y)


def delta(point1, point2):
    if point1 == None:
        delta_point = point2
    elif point2 == None:
        delta_point = point1
    else:
        (x1, y1) = point1
        (x2, y2) = point2
        delta_point = ((x1 - x2), (y1 - y2))
    n_point = normalize(delta_point)
    return n_point


def avg(point1, point2):
    if point1 == None:
        avg_point = point2
    elif point2 == None:
        avg_point = point1
    else:
        (x1, y1) = point1
        (x2, y2) = point2
        avg_point = ((x1 + x2) / 2, (y1 + y2) / 2)
    n_point = normalize(avg_point)
    return n_point


def avg_base_point(bezier, index, last_orientation, next_orientation):
    last_base = next_base = None
    if index > 0 and bezier[index] != bezier[index - 1]:
        last_base = (bezier[index] - bezier[index - 1]).unit()
    if index < (len(bezier) - 1) and bezier[index + 1] != bezier[index]:
        next_base = (bezier[index + 1] - bezier[index]).unit()
    if last_base is None and next_base is None:
        last_base = last_orientation
        next_base = next_orientation
    if last_base is None:
        last_base = next_base
    if next_base is None:
        next_base = last_base
    orientation = (last_base + next_base).div(2).unit()
    return orientation


def ratio(point1, point2, factor):
    if point1 == None:
        return point2
    elif point2 == None:
        return point1
    (x1, y1) = point1
    (x2, y2) = point2
    r_x = x1 + ((x2 - x1) * factor)
    r_y = y1 + ((y2 - y1) * factor)
    return (r_x, r_y)


def distance(point1, point2):
    if not point1 is None:
        (x1, y1) = point1
    else:
        (x1, y1) = (0.0, 0.0)
    if not point2 is None:
        (x2, y2) = point2
    else:
        (x2, y2) = (0.0, 0.0)
    (dx, dy) = ((x1 - x2), (y1 - y2))
    dist = math.sqrt(dx * dx + dy * dy)
    return dist

def make_straight_line(points, new_point):
    points.append(points[-1]) # Repeat last point just to fake the cubic Bezier algorithm
    points.append(new_point)
    last_abs_point_i = len(points) - 1
    points.append(points[-1]) # Repeat new point just to fake the Bezier algorithm
    return last_abs_point_i

#https://developer.mozilla.org/en-US/docs/Web/SVG/Tutorial/Paths
def get_bezier_points_from_svg_path(path_label, path_d):
    letter = None
    count = n_ms = 0
    points = [] # list of (x,y) in SVG coordinates
    last_abs_point_i = 0
    total_errors = errors = 0
    for p in path_d.split():
        if len(p) == 1:
            if p in 'MmCcLlHhVvZz':
                if p != letter:
                    count = n_ms = 0
                letter = p
                if p in 'Zz': # Draw straight line to the initial point
                    last_abs_point_i = make_straight_line(points, points[0])
            else:
                errors += 1
        else:  
            if letter in 'Mm': # Move cursor to (x,y), lowercase = relative coordinates, uppercase = absolute coordinates
                pt = p.split(',')
                if n_ms == 0:
                    if len(pt) == 2:
                        points.append(Point(float(pt[0]), float(pt[1])))
                        last_abs_point_i = len(points) - 1
                        n_ms += 1
                    else:
                        errors += 1             
                else: # In case we have multiple coordinates following (m or M) draw straight lines as if it were (l or L)
                    if letter == 'm' and len(pt) == 2:
                        x = float(pt[0]) + points[last_abs_point_i].x
                        y = float(pt[1]) + points[last_abs_point_i].y
                        last_abs_point_i = make_straight_line(points, Point(x, y))
                    elif letter == 'M' and len(pt) == 2:
                        last_abs_point_i = make_straight_line(points, Point(float(pt[0]), float(pt[1])))
                    else:
                        errors += 1             
            elif letter == 'c': # Cubic Bezier curve in relative coordinates
                count += 1
                pt = p.split(',')
                if len(pt) == 2:                
                    x = float(pt[0]) + points[last_abs_point_i].x
                    y = float(pt[1]) + points[last_abs_point_i].y
                    points.append(Point(x, y))     
                    if count % 3 == 0:
                        last_abs_point_i = len(points) - 1
                else:
                    errors +=1
            elif letter == 'C': # Cubic Bezier curve in absolute coordinates
                count += 1            
                pt = p.split(',') 
                if len(pt) == 2:
                    points.append(Point(float(pt[0]), float(pt[1])))
                    last_abs_point_i = len(points) - 1
                else:
                    errors +=1                            
            elif letter in 'lhv': # Draw straight line to next point (x,y) in relative coordinates
                pt = p.split(',')
                if letter == 'l' and len(pt) == 2:
                    x = float(pt[0]) + points[last_abs_point_i].x
                    y = float(pt[1]) + points[last_abs_point_i].y
                    last_abs_point_i = make_straight_line(points, Point(x, y))
                elif letter == 'h' and len(pt) == 1:
                    x = float(pt[0]) + points[last_abs_point_i].x
                    y = points[last_abs_point_i].y
                    last_abs_point_i = make_straight_line(points, Point(x, y))
                elif letter == 'v' and len(pt) == 1:
                    x = points[last_abs_point_i].x
                    y = float(pt[0]) + points[last_abs_point_i].y
                    last_abs_point_i = make_straight_line(points, Point(x, y))
                else:
                    errors += 1             
            elif letter in 'LHV': # Draw straight line to next point (x,y) in absolute coordinates
                pt = p.split(',')
                if letter == 'L' and len(pt) == 2:
                    last_abs_point_i = make_straight_line(points, Point(float(pt[0]), float(pt[1])))
                elif letter == 'H' and len(pt) == 1:
                    last_abs_point_i = make_straight_line(points, Point(float(pt[0]), points[last_abs_point_i].y))
                elif letter == 'V' and len(pt) == 1:
                    last_abs_point_i = make_straight_line(points, Point(points[last_abs_point_i].x, float(pt[0])))
                else:
                    errors += 1             
            else:
                errors += 1
        if errors > 0:
            sys.stderr.write(f'error: unexpected SVG path token: {p}\n')
        total_errors += errors
        errors = 0
    if total_errors > 0:
        sys.stderr.write(f"{total_errors} error{'s' * (total_errors > 1)} in SVG path label={path_label} d=" + 
                         (path_d if len(path_d) < 1000 else (path_d[:500] + '     (...)     ' + path_d[-500:])) + '\n') 
    return points


#https://stackoverflow.com/questions/15857818/python-svg-parser
def get_paths_from_svg(svg):
    doc = minidom.parse(svg.filename)
    svg_element = doc.getElementsByTagName('svg')
    if svg_element:
        width_units  = svg_element[0].getAttribute('width')
        height_units = svg_element[0].getAttribute('height')
        svg.width  = unit_convert(width_units)
        svg.height = unit_convert(height_units)
        for i, path_element in enumerate(svg_element[0].getElementsByTagName('path')):
            if path_element.parentNode.nodeName == 'marker':
                continue
            path_label = path_element.getAttribute('inkscape:label')
            if not path_label:
                path_label = path_element.getAttribute('id')
            if("ANNOTATION" not in path_label.split("_") or "TYPE" not in path_label.split("_")):
                (file_base, file_ext) = os.path.splitext(path_label)
                path_forward = (file_base[-3:].lower() != '_rw')
                path_d = path_element.getAttribute('d')
                bezier_points = get_bezier_points_from_svg_path(path_label, path_d)
                svg.paths.append(SVGPath(svg, len(svg.paths), path_label, path_forward, bezier_points))
    doc.unlink()
    return (svg.paths, svg.width, svg.height)

def rotate_point(point, rotation_angle, center_point):
    radians = math.radians(rotation_angle)
    
    relative_x = point[0] - center_point[0]
    relative_y = point[1] - center_point[1]
    
    new_x = relative_x * math.cos(radians) - relative_y * math.sin(radians) + center_point[0]
    new_y = relative_x * math.sin(radians) + relative_y * math.cos(radians) + center_point[1]
    
    return new_x, new_y, radians


def median_point_annotation(v1, v2, v3):
    x_medio = (v1[0] + v2[0] + v3[0]) / 3
    y_medio = (v1[1] + v2[1] + v3[1]) / 3
    return (x_medio, y_medio)


def get_annotations_from_svg(svg):
    doc = minidom.parse(svg.filename)
    svg_element = doc.getElementsByTagName('svg')
    annotations = [] 
    center_x = 0
    center_y = 0

    if svg_element:
        width_units  = svg_element[0].getAttribute('width')
        height_units = svg_element[0].getAttribute('height')
        svg.width  = unit_convert(width_units)
        svg.height = unit_convert(height_units)

        group_elements = doc.getElementsByTagName('g')

        for group_element in group_elements:
            if group_element.getAttribute('inkscape:label') == 'annotation':
                for child_element in group_element.childNodes:
                    if child_element.nodeType == child_element.ELEMENT_NODE:
                        tag_name = child_element.tagName

                        if tag_name == "circle" or tag_name == "ellipse":
                            raio = 0
                            x = float(child_element.getAttribute('cx'))
                            y = float(child_element.getAttribute('cy'))
                            name = str(child_element.getAttribute('inkscape:label')).split("_##")[0]
                            if(child_element.getAttribute('r')):
                                raio = child_element.getAttribute('r')
                            if (child_element.getAttribute('rx')):
                                raio = child_element.getAttribute('rx')
                            
                            annotations.append((name,x,y,raio,"circle"))

                        if tag_name == "path":
                            name = str(child_element.getAttribute('inkscape:label')).split("_##")[0]
                            path_d = child_element.getAttribute('d')
                            annotations.append((name,path_d,"path"))
                         
                        if tag_name == "polygon":
                            name = str(child_element.getAttribute('inkscape:label')).split("_##")[0]
                            points = child_element.getAttribute('points')
                            point_split = points.split(" ")

                            x1_left,y1_left = point_split[4].split(",")
                            x1_right,y1_right = point_split[6].split(",")
                            x1_units_end,y1_units_end = point_split[2].split(",")
                            median_point_x, median_point_y = median_point_annotation([float(x1_left),float(y1_left)],[float(x1_right),float(y1_right)],[float(x1_units_end),float(y1_units_end)])

                            if(child_element.getAttribute('transform')):
                                splited_transform = child_element.getAttribute('transform').split("(")
                                transform_type = splited_transform[0]

                                if(transform_type == "rotate"):
                                    rotate,center_x,center_y =  splited_transform[1].split(',')
                                    center_y = center_y.strip(')')

                                    median_point_x,median_point_y, radians = rotate_point([float(median_point_x),float(median_point_y)],float(rotate),[float(center_x),float(center_y)])
                                
                                    annotations.append((name,median_point_x,median_point_y,radians,"other")) 


                                if(transform_type == "translate"):     
                                    center_x,center_y =  splited_transform[1].split(',')
                                    center_y = center_y.strip(')')

                                    median_point_x = float(median_point_x) + float(center_x)
                                    median_point_y = float(median_point_y) + float(center_y)
                                    
                                    annotations.append((name,median_point_x,median_point_y,"other")) 
                            else:
                                annotations.append((name,median_point_x,median_point_y,"without_transform")) 
        
    doc.unlink()
    return annotations


def get_bezier_curve(svg_path):
    points = svg_path.bezier_points
    increment = svg_path.bezier_increment
    curve = svg_path.bezier_curve 
    last_orientation = next_orientation = None

    if (len(points) - 1) % 3 != 0:
        sys.stderr.write(f'error: SVG path {svg_path.label}: invalid number of Bezier points: {len(points)} (must be 1 + multiple of 3)\n')
        return (curve)

    number_of_base_points = (len(points) - 1) // 3
    for i in range(number_of_base_points):
        p0 = points[i * 3]
        p1 = points[i * 3 + 1]
        p2 = points[i * 3 + 2]
        p3 = points[i * 3 + 3]
        dist = p3.distance(p0)
        number_of_calculated_points = int(float(meters(dist)) / increment)
        if number_of_calculated_points:
            svg_path.bezier_base_indexes.append(len(curve))
        for j in range(number_of_calculated_points):
            factor = float(j) / number_of_calculated_points 
            # The Green Lines
            pa = p0 + (p1 - p0).mul(factor)
            pb = p1 + (p2 - p1).mul(factor)
            pc = p2 + (p3 - p2).mul(factor)
            # The Blue Line
            pm = pa + (pb - pa).mul(factor)
            pn = pb + (pc - pb).mul(factor)
            # The Black Dot
            p  = pm + (pn - pm).mul(factor)
            curve.append(CurvePoint(p))

            if j == 1:
                next_orientation = (curve[-1].svg_point - curve[-2].svg_point).unit()
                curve[-2].svg_orientation = avg_base_point(points, (i * 3), last_orientation, next_orientation)
                last_orientation = next_orientation
            elif j > 1:
                next_orientation = (curve[-1].svg_point - curve[-2].svg_point).unit()
                curve[-2].svg_orientation = (last_orientation + next_orientation).div(2).unit()
                last_orientation = next_orientation
        if number_of_calculated_points:
            next_orientation = (p3 - curve[-1].svg_point).unit()
            curve[-1].svg_orientation = (last_orientation + next_orientation).div(2).unit()
            last_orientation = next_orientation
    if curve:
        svg_path.bezier_base_indexes.append(len(curve))
        svg_orientation = avg_base_point(points, (i * 3 + 3), last_orientation, None)
        curve.append(CurvePoint(points[-1], svg_orientation))
    else:
        sys.stderr.write(f'error: SVG path {svg_path.label}: invalid Bezier curve: length < {increment} meter\n')
    return (curve)


def get_rddf_filename_from_svg_path_label(svg_path_label, svg_filename):
    pattern = re.compile('path\d*_.+') # 'path', 0-n digits, '_', 1-n characters
    if pattern.match(svg_path_label):
        path_tokens = svg_path_label.split('_')
        rddf_file = args.outputdir + '/' + svg_path_label[(len(path_tokens[0]) + 1):]
    else:
        svg_basename = os.path.basename(svg_filename)
        rddf_file = args.outputdir + '/rddf_' + svg_basename[1:-4] + '_' + svg_path_label + '.txt'
    return rddf_file

def find_lane(lane1,lane2, list ,side, rest = None):
    rddf_result = None
    if side < 0:
        for rddf in list:
            name_splited = rddf.filename.split(".")
            name_splited = name_splited[0].split("/")
            name_splited = name_splited[-1].split("_")
            if "lb" in name_splited:
                if int(name_splited[1]) == int(lane1) and int(name_splited[2]) == int(lane2):
                    rddf_result = rddf
                    if len(name_splited) > 2:
                        if rest == name_splited[3:]:
                            return rddf     
    if side > 0:
        for rddf in list:
            name_splited = rddf.filename.split(".")
            name_splited = name_splited[0].split("/")
            name_splited = name_splited[-1].split("_")
            if "rb" in name_splited:
                if int(name_splited[1]) == int(lane1) and int(name_splited[2]) == int(lane2):
                    rddf_result = rddf
                    if len(name_splited) > 2:
                        if rest == name_splited[3:]:
                            return rddf
    return rddf_result
def create_list_rddfs (svg):
    name_default = f"{args.outputdir}/rddfs_list"
    name_left_default = f"{args.outputdir}/rddfs_list_l"
    name_right_default = f"{args.outputdir}/rddfs_list_r"
    full_direcories = f"{args.outputdir}/full_directories.txt"

    if (os.path.isfile((name_default+".txt"))):
        date_format = '%Y_%m_%d_%H_%M_%S_%f'
        current_date = datetime.now().strftime(date_format)
        name_default = f"{name_default}_{current_date}.txt"
        name_left_default = f"{name_left_default}_{current_date}.txt"
        name_right_default = f"{name_right_default}_{current_date}.txt"
    else:
        name_left_default += ".txt"
        name_right_default += ".txt"
        name_default += ".txt"
    try:
        file = open(name_default, 'w')
        file_rb = open(name_right_default, 'w')
        file_lb = open(name_left_default, 'w')

    except EOFError as err:
        sys.stderr.write(f"Erro ao tentar criar o arquivo {name_default}, error: {err}.")
    file_full = open(full_direcories, 'w')

    path_list_rddf = os.path.dirname(os.path.abspath(name_default))

    rddfs_ordered = sorted(svg.rddfs, key=lambda rddf: len(rddf.waypoints), reverse=True)

    for name in rddfs_ordered:
        name_splited = name.filename.split(".")
        name_splited = name_splited[0].split("/")
        name_splited = name_splited[-1].split("_")
        
        if "rddf" in name_splited:
            file.write(f"{path_list_rddf}/{os.path.basename(name.filename)}\n")
            try:
                if len(name_splited) > 4:
                    name_left = find_lane(name_splited[2],name_splited[3],rddfs_ordered,-1, name_splited[4:])
                    name_right = find_lane(name_splited[2],name_splited[3],rddfs_ordered,1, name_splited[4:])
                else:
                    name_left = find_lane(name_splited[2],name_splited[3],rddfs_ordered,-1)
                    name_right = find_lane(name_splited[2],name_splited[3],rddfs_ordered,1)
                if name_right and name_left:
                    file_lb.write(f"{path_list_rddf}/{os.path.basename(name_left.filename)}\n")
                    file_rb.write(f"{path_list_rddf}/{os.path.basename(name_right.filename)}\n")
            except:
                print("lane_left ou lane_right foi encontrado")

        file_full.write(f"{path_list_rddf}/{os.path.basename(name.filename)}\n")
        
    file.close()
    file_full.close()

    if args.graph_filename:
        file_rb.close()
        file_lb.close()


def get_rddfs_from_svg_paths(svg):
    rddf_dict = {}
    rddf_list = []

    for order, svg_path in enumerate(svg.paths):
        rddf_file = get_rddf_filename_from_svg_path_label(svg_path.label, svg.filename)
        rddf_list.append(rddf_file)
        (rddf_file_name, rddf_file_ext) = os.path.splitext(rddf_file)
        repetition = 0
        while rddf_file in rddf_dict:
            repetition += 1
            rddf_file = f'{rddf_file_name}({repetition}){rddf_file_ext}'
        if os.path.isfile(rddf_file) and not args.overwrite and not args.analysis_only:
            sys.stderr.write(f'warning: SVG path label="{svg_path.label}": RDDF file \'{rddf_file}\' already exists\n')
            sys.stderr.write('do you want to overwrite it? [y/N]: ')
            answer = input()
            if not answer or not answer in 'Yy':
                sys.stderr.write('skipped by user request\n')
                continue
        rddf_dict[rddf_file] = (order, svg_path)
    rddfs = sorted(rddf_dict.items(), key=operator.itemgetter(1), reverse=True)
    for i, (rddf_file, (_, svg_path)) in enumerate(rddfs):
        rddf = RDDF(svg, i, rddf_file, svg_path, svg_path.forward)
        svg.rddfs.append(rddf)
    
    create_list_rddfs(svg)

    return svg.rddfs

def round_point(point, decimals=3):
    (x, y) = point
    x_round = round(x, decimals)
    y_round = round(y, decimals)
    return (x_round, y_round)

def get_end_nodes(svg):
    for i, rddf in enumerate(svg.rddfs):
        start  = svg_to_utm(rddf.svg_path.bezier_points[ 0], svg.height, svg.origin)
        finish = svg_to_utm(rddf.svg_path.bezier_points[-1], svg.height, svg.origin)
        svg.end_nodes.append(EndNode(svg, (2 * i    ), rddf,  0, start.round()))
        svg.end_nodes.append(EndNode(svg, (2 * i + 1), rddf, -1, finish.round()))
    return svg.end_nodes

def get_theta(orientation):
    theta = math.atan2(orientation.y, orientation.x)
    return theta

def get_waypoint_theta(poses, index):
    point = to_Point(poses[index][:2])
    last_point = to_Point(poses[index - 1][:2]) if index > 0 else None
    next_point = to_Point(poses[index + 1][:2]) if index < (len(poses) - 1) else None
    last_orientation = (point - last_point).unit() if not (last_point is None) else None
    next_orientation = (next_point - point).unit() if not (next_point is None) else None
    try:
        orientation = (last_orientation + next_orientation).div(2).unit()
    except (TypeError, AttributeError):
        orientation = last_orientation if next_orientation is None else next_orientation
    theta = get_theta(orientation) if not (orientation is None) else 0.0
    return theta

def append_waypoint_to_rddf(curve_point, waypoints, forward=True):
    theta = get_theta(curve_point.orientation)
    if not forward:
        theta = normalize_angle(theta + math.pi)
    waypoint = WayPoint(curve_point.point, theta)
    waypoints.append(waypoint)


def append_end_node_to_rddf(curve_point, waypoints, forward=True):
    theta = get_theta(curve_point.orientation)
    if not forward:
        theta = normalize_angle(theta + math.pi)
    waypoint = WayPoint(curve_point.point.round(), theta)
    waypoints.append(waypoint)


def get_junction_type(waypoint_index, node_waypoint_index):
    if node_waypoint_index == 0:
        if waypoint_index == -1:
            junction_t = 'forward'
            junction_node_t = 'backward'
        else:
            junction_t = junction_node_t = 'fork'
    elif node_waypoint_index == -1:
        if waypoint_index == 0:
            junction_t = 'backward'
            junction_node_t = 'forward'
        else:
            junction_t = junction_node_t = 'merge'
    else:
        raise ValueError(f'junction_type: unexpected node_waypoint_index: {node_waypoint_index} (must be 0 or -1)')
    return (junction_t, junction_node_t)


def find_junction(end_node, rddf):
    if end_node.rddf != rddf:
        for junction in rddf.junctions:
            if junction.rddf2 == end_node.rddf and junction.waypoint_index2 == end_node.waypoint_index:
                return junction
    return None


def find_closest_point(rddf, point, start=0, finish=-1):
    waypoint_index1 = start  + (0 if start  >= 0 else len(rddf.waypoints)) 
    point1 = rddf.waypoints[waypoint_index1].point
    dist1 = point1.distance(point)
    waypoint_index2 = finish + (0 if finish >= 0 else len(rddf.waypoints))
    point2 = rddf.waypoints[waypoint_index2].point
    dist2 = point2.distance(point)
    while waypoint_index1 < (waypoint_index2 - 1):
        waypoint_index = (waypoint_index1 + waypoint_index2) // 2
        if dist1 > dist2:
            waypoint_index1 = waypoint_index
            point1 = rddf.waypoints[waypoint_index1].point
            dist1 = point1.distance(point)
        else:
            waypoint_index2 = waypoint_index
            point2 = rddf.waypoints[waypoint_index2].point
            dist2 = point2.distance(point)
    if dist1 > dist2:
        (waypoint_index1, point1, dist1) = (waypoint_index2, point2, dist2)
    if waypoint_index1 == (len(rddf.waypoints) - 1):
        waypoint_index1 = -1
    return (waypoint_index1, point1, dist1)


def get_inexact_junctions(rddf):
    for end_node in rddf.svg.end_nodes:
        if "lb_" not in rddf.filename and "rb" not in rddf.filename:
            if end_node.rddf != rddf and find_junction(end_node, rddf) is None:
                mid_index = len(rddf.waypoints) // 2
                (waypoint_index,  point,  dist)  = find_closest_point(rddf, end_node.point, 0, mid_index)
                (waypoint_index2, point2, dist2) = find_closest_point(rddf, end_node.point, mid_index, -1)
                if dist > dist2:
                    (waypoint_index, point, dist) = (waypoint_index2, point2, dist2)
                if dist <= (2.0 * args.gen_dist):
                    (junction_t, junction_node_t) = get_junction_type(waypoint_index, end_node.waypoint_index)
                    rddf.junctions.append(JunctionNode(rddf, waypoint_index, point, junction_t, end_node.rddf, end_node.waypoint_index))
                    if not waypoint_index in (0, -1): 
                        end_node.rddf.junctions.append(JunctionNode(end_node.rddf, end_node.waypoint_index, end_node.point, junction_node_t, rddf, waypoint_index))

def junction_node(point, rddf, waypoint_index=None):
    rpoint = point.round()
    count = 0
    if "lb_" not in rddf.filename and "rb" not in rddf.filename:
        for end_node in rddf.svg.end_nodes:
            if end_node.rddf != rddf and end_node.point == rpoint:
                count += 1
                if waypoint_index is None:
                    break
                (junction_t, junction_node_t) = get_junction_type(waypoint_index, end_node.waypoint_index)
                rddf.junctions.append(JunctionNode(rddf, waypoint_index, rpoint, junction_t, end_node.rddf, end_node.waypoint_index))
                if not waypoint_index in (0, -1): 
                    end_node.rddf.junctions.append(JunctionNode(end_node.rddf, end_node.waypoint_index, end_node.point, junction_node_t, rddf, waypoint_index))
    return count


def get_rddf_from_bezier_curve(rddf, target_dist):
    waypoints = rddf.waypoints
    curve = rddf.svg_path.bezier_curve
    base_indexes = rddf.svg_path.bezier_base_indexes
    for j in range(len(base_indexes) - 1):
        base_index = base_indexes[j]
        next_base_index = base_indexes[j + 1]
        append_end_node_to_rddf(curve[base_index], waypoints, rddf.forward)
        junction_node(curve[base_index].point, rddf, (len(waypoints) - 1))
        rddf.meters += waypoints[-1].point.distance(waypoints[-2].point) if j > 0 else 0.0
        dist_base_points = 0.0
        for i in range(base_index, next_base_index):
            dist_base_points += curve[i].point.distance(curve[i + 1].point)
        waypoint_count = int(round(dist_base_points / target_dist))
        if waypoint_count > 1:
            dist_points = dist_base_points / waypoint_count
            dist = 0.0
            waypoints_since_last_base = 1
            for i in range((base_index + 1), next_base_index):
                if waypoints_since_last_base == waypoint_count:
                    break
                dist += curve[i].point.distance(curve[i - 1].point)
                if dist >= (dist_points * waypoints_since_last_base):
                    append_waypoint_to_rddf(curve[i], waypoints, rddf.forward)
                    waypoints_since_last_base += 1
                    rddf.meters += waypoints[-1].point.distance(waypoints[-2].point)
        if next_base_index == base_indexes[-1]:
            append_end_node_to_rddf(curve[next_base_index], waypoints, rddf.forward)
            junction_node(curve[next_base_index].point, rddf, -1)
            rddf.meters += waypoints[-1].point.distance(waypoints[-2].point)
    if len(waypoints) < 2:
        sys.stderr.write(f'error: SVG path {rddf.svg_path.label}: RDDF length is {rddf.meters:.2f} meter\n')
    return (waypoints, rddf.meters)


def normalize_angle(angle):
    norm_angle = angle - (2.0 * math.pi) * math.floor((angle + math.pi) / (2.0 * math.pi))
    return norm_angle


def delta_angle(angle1, angle2):
    delta = angle1 - angle2
    norm_delta = normalize_angle(delta)
    return norm_delta


def steering_angle(front_rear_dist_L, delta_theta, dist_l):
    if delta_theta is None:
        return None
    phi = math.atan2(front_rear_dist_L * delta_theta, dist_l)   #  phi = atan(L/R) , l = R * dtetha  =>  phi = atan(L * dtheta / l)
    return phi


def is_bad_phi(phi):
    is_bad = False
    if phi:
        is_bad = (abs(phi) > abs(args.max_steering))
    return is_bad


def bad_phi_warning(phi):
    warning = ''
    if is_bad_phi(phi):
        warning = f'bad steering angle: |{degrees(phi):7.2f}°| > {degrees(args.max_steering):.2f}°'
    return warning


def bad_phi(waypoint, index, bad_phi_waypoints):
    if is_bad_phi(waypoint.phi):
        bad_phi_waypoints.append(index)

def write_rddf(rddf, rddf_left = None, rddf_right = None):
    if not rddf.waypoints:
        return
    
    last_point = last_theta = None
    v = abs(VELOCITY) if rddf.forward else -abs(VELOCITY)
    timestamp = time.time()

    if not args.analysis_only:
        rddf_f = open(rddf.filename, 'w')

    if(rddf_left and rddf_right):
        rddf_left_f = open(rddf_left.filename, 'w')
        rddf_right_f = open(rddf_right.filename, 'w')
        points_right = {point.theta: point for point in rddf_right.waypoints}
        points_left = {point.theta: point for point in rddf_left.waypoints}

    for index, waypoint in enumerate(rddf.waypoints):
        delta_theta = delta_angle(waypoint.theta, last_theta) if last_theta is not None else 1.0
        dist = waypoint.point.distance(last_point) if last_point is not None else args.gen_dist
        waypoint.phi = steering_angle(args.front_rear, delta_theta, dist)
        bad_phi(waypoint, index, rddf.bad_phi_waypoints)
        timestamp += abs(dist / v)
        nearest_point_r = None
        nearest_point_l = None
        if(rddf_left and rddf_right):

            nearest_point_r = min(points_right.values(), key=lambda p: waypoint.point.distance(p.point)) if points_right else None
            nearest_point_l = min(points_left.values(), key=lambda p: waypoint.point.distance(p.point)) if points_left else None

            if nearest_point_r:
                nearest_point_r.phi = 0
            if nearest_point_l:
                nearest_point_l.phi = 0
        if not args.analysis_only:
            rddf_f.write(f'{waypoint.point.x:.6f}\t{waypoint.point.y:.6f}\t{waypoint.theta:.6f}\t{v:.6f}\t{waypoint.phi:.6f}\t{timestamp:.6f}\n')
            if nearest_point_l:
                rddf_left_f.write(f'{nearest_point_l.point.x:.6f}\t{nearest_point_l.point.y:.6f}\t{nearest_point_l.theta:.6f}\t{v:.6f}\t{nearest_point_l.phi:.6f}\t{timestamp:.6f}\n')
            if nearest_point_r:
                rddf_right_f.write(f'{nearest_point_r.point.x:.6f}\t{nearest_point_r.point.y:.6f}\t{nearest_point_r.theta:.6f}\t{v:.6f}\t{nearest_point_r.phi:.6f}\t{timestamp:.6f}\n')
        
        last_theta = waypoint.theta
        last_point = waypoint.point

    if args.analysis_only:
        print(f"---> Analysis only ***** RDDF file '{rddf.filename}' not generated")
        print(f"---> Analysis only ***** RDDF file '{rddf_left.filename}' not generated")
        print(f"---> Analysis only ***** RDDF file '{rddf_right.filename}' not generated")

    else:
        rddf_f.close()
        print(f"---> RDDF file '{rddf.filename}' generated")

        if(rddf_left and rddf_right):
            rddf_left_f.close()
            rddf_right_f.close()
            print(f"---> RDDF LEFT file '{rddf_left.filename}' generated")
            print(f"---> RDDF RIGHT file '{rddf_right.filename}' generated")


def ajustar_intervalo(angulo):
    # Ajusta o ângulo para o intervalo de -180 a 180 graus
    while angulo <= -180:
        angulo += 360
    while angulo > 180:
        angulo -= 360
    return angulo


def write_annotation(svg):
    if(args.annotation):
        print(args.annotation)
        annotations_list,trash_list = get_annotations_list(args.annotation)
        if annotations_list == []:
            print("Arquivo de anotação não encontrado: ", args.annotation)
        annotations = get_annotations_from_svg(svg)

        dict1 = dict(annotations_list)
        dict2 = dict(trash_list)

        chaves_unicas = set(dict1) | set(dict2)

        uniao_lista = [(chave, dict1.get(chave, None) or dict2.get(chave, None)) for chave in sorted(chaves_unicas)]

        cont_new_pos = 0
        write_str = ""
        for line in uniao_lista:
            content = line[1]
            if type(content) == str:
                write_str+= content
            else:
                new_annotation = annotations[cont_new_pos]
                name = new_annotation[0]

                if(new_annotation[-1] == "other"):
                    point = svg_to_utm(Point(float(new_annotation[1]),float(new_annotation[2])), svg.height, svg.origin)
                    if(len(new_annotation) == 5):                    
                        grau_ori = math.degrees(content.theta)
                        rotate = math.degrees(new_annotation[3])

                        new_grau = grau_ori - rotate

                        write_str += f"{name}\t{content.type}\t{content.code}\t{math.radians(ajustar_intervalo(new_grau))}\t{point.x}\t{point.y}\t{content.trash[0]}\n"
                    else:
                        write_str += f"{name}\t{content.type}\t{content.code}\t{content.theta}\t{point.x}\t{point.y}\t{content.trash[0]}\n"
                    cont_new_pos+=1

                elif(new_annotation[-1] == "circle"):  
                    point = svg_to_utm(Point(float(new_annotation[1]),float(new_annotation[2])), svg.height, svg.origin)
                    if(len(new_annotation) == 5): 
                        content.trash[-1] = new_annotation[3]
                        content.trash = "\t".join(content.trash)
                        write_str += f"{name}\t{content.type}\t{content.code}\t{content.theta}\t{point.x}\t{point.y}\t{content.trash}\n"
                    else:
                        write_str += f"{name}\t{content.type}\t{content.code}\t{content.theta}\t{point.x}\t{point.y}\t{content.trash[0]}\n"
                    cont_new_pos+=1

                elif(new_annotation[-1] == "path"):  
                    poli_points = []
                    path_d = new_annotation[1]
                    
                    bezier_points = get_bezier_points_from_svg_path("Area de pedestre", path_d)

                    for i in range(0, len(bezier_points)):

                        point = svg_to_utm(Point(bezier_points[i].x,bezier_points[i].y), svg.height, svg.origin)
                        if i > 0:
                            if(point.distance(last_point) > 0.6):
                                poli_points.append(f"{point.x}\t{point.y}")

                        last_point = point

                    content.trash = content.trash[0:2]
                    content.trash[1] = str(len(poli_points))
                    
                    content.trash.append("\t".join(poli_points))
                    trash = "\t".join(content.trash)

                    write_str += f"{name}\t{content.type}\t{content.code}\t{content.theta}\t{point.x}\t{point.y}\t{trash}\n"
                    
                    cont_new_pos+=1

                elif( new_annotation[-1] == "without_transform" ):  
                    content_joined = ", ".join(map(str, content.trash))

                    write_str += f"{name}\t{content.type}\t{content.code}\t{content.theta}\t{content.x}\t{content.y}\t{content_joined}\n"
                    
                    cont_new_pos+=1
                    
        annotation_file = open(args.outputdir + "/" +os.path.basename(args.annotation), 'w')
        annotation_file.write(write_str)
        print(f"---> ANNOTATION file '{args.annotation}' generated")
        annotation_file.close()

def get_junctions_phi(rddfs):
    for rddf in rddfs:
        for junction in rddf.junctions:
            point = junction.point
            theta = rddf.waypoints[junction.waypoint_index].theta
            if junction.waypoint_index == 0: # (fork, backward)
                previous_waypoint = junction.rddf2.waypoints[junction.waypoint_index2 - (1 if (junction.waypoint_index2 != 0) else 0)]
                dist = point.distance(previous_waypoint.point) if (point != previous_waypoint.point) else args.gen_dist
                delta_theta = delta_angle(theta, previous_waypoint.theta)
                junction.phi = steering_angle(args.front_rear, delta_theta, dist)
            elif junction.waypoint_index == -1: # (merge, forward)
                next_waypoint = junction.rddf2.waypoints[junction.waypoint_index2 + (1 if (junction.waypoint_index2 != -1) else 0)]
                dist = point.distance(next_waypoint.point) if (point != next_waypoint.point) else args.gen_dist
                delta_theta = delta_angle(next_waypoint.theta, theta)
                junction.phi = steering_angle(args.front_rear, delta_theta, dist)


def degrees(radian):
    deg = radian * 180.0 / math.pi
    return deg


def order_by_rddf_index(rddf_data):
    if isinstance(rddf_data, JunctionNode):
        rddf_index = rddf_data.rddf.index
        waypoint_index = rddf_data.waypoint_index
    else:
        rddf_index = rddf_data[0]
        waypoint_index = rddf_data[1]
    waypoint_index += 0 if (waypoint_index >= 0) else sys.maxsize
    key = f'{rddf_index:>20} ; {waypoint_index:>20}'  
    return key


def reedit_commandline(rddfs, utm_window):
    cmd = f'{sys.argv[0]} '
    if args.imagedir != parser.get_default('imagedir'):
        cmd += f' -i {args.imagedir} '
    if args.outputdir != parser.get_default('outputdir'):
        cmd += f' -o {args.outputdir} '
    if args.overwrite and not parser.get_default('overwrite'):
        cmd += ' -x '
    if list(utm_window) != parser.get_default('window'):
        cmd += ' --window  {}  {}  {}  {} '.format(*utm_window)
    if args.unit != parser.get_default('unit'):
        cmd += f' --unit {args.unit} '
    if args.stroke_width != parser.get_default('stroke_width'):
        cmd += f' --stroke_width {args.stroke_width} '
    if args.scale != parser.get_default('scale'):
        cmd += f' --scale {args.scale} '
    if args.gen_dist != parser.get_default('gen_dist'):
        cmd += f' --gen_dist {args.gen_dist} '
    if args.rem_dist != parser.get_default('rem_dist'):
        cmd += f' --rem_dist {args.rem_dist} '
    if args.front_rear != parser.get_default('front_rear'):
        cmd += f' --front_rear {args.front_rear} '
    if args.max_steering != parser.get_default('max_steering'):
        cmd += f' --max_steering {args.max_steering} '
    for rddf in rddfs:
        cmd += " '{}' ".format(rddf.filename.replace('\'', '\'\\\'\''))
    return cmd


def generate_graph_commandline(rddfs):
    cmd =  '# You may modify the content of GRAPH and RANGE:\n'
    default_filename = f'rddf_{datetime.now().strftime("%Y%m%d")}.txt'
    rddf_filename = os.path.basename(default_filename)
    graph_range = GRAPH_RANGE if (GRAPH_RANGE % 1) != 0 else int(GRAPH_RANGE)
    graph_filename = args.outputdir + '/' + rddf_filename.replace('rddf', 'graph', 1).strip('.txt') + f'_{graph_range}m.gr'
    cmd += f'GRAPH="--graph_dir {graph_filename}";  '
    cmd += f'RANGE="--nearby_lane_range {graph_range}";  '
    filenames = []
    for rddf in rddfs:
        filenames.append(os.path.abspath(rddf.filename))
    cmd += 'echo -e \'' + '\\n'.join([ f.replace('\'', '\'\\\'\'') for f in filenames ]) + '\' > rddf_files_list.txt;  '
    cmd += '$CARMEN_HOME/bin/road_network_generator  --rddf_list rddf_files_list.txt  $GRAPH  $RANGE'
    return cmd


def tee_print(outfile, outtext):
    outfile.write(outtext + '\n')
    if not outfile in (sys.stdout, sys.stderr):
        print(outtext)


def get_index_width(rddfs):
    max_length = 0
    for rddf in rddfs:
        max_length = max(len(rddf.waypoints), max_length)
    index_width = len(str(max_length - 1))
    return index_width


def print_summary(svg, utm_window):
    now = datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
    outfilename = args.outputdir + f'/report_{os.path.basename(svg.filename)}_{now}.txt'
    (xo, yo, ww, wh) = utm_window
    index_width = get_index_width(svg.rddfs)
    line = (' ' * 5) + '[{:>' + str(index_width) + '}]  {:20}  {:10}  (x,y)  {:.6f}\t{:.6f}\t ({:>8.2f}, {:>8.2f})  {}'
    with open(outfilename, 'w') as of:
        tee_print(of, '\nGraph Summary Report\n--------------')
        tee_print(of, f"SVG file: '{svg.filename}'")
        for rddf in svg.rddfs:
            if not rddf.waypoints:
                break
            waypoints = rddf.waypoints
            tee_print(of, f"\nRDDF file #{rddf.index + 1}: '{rddf.filename}'")
            tee_print(of, ('{:>' + str(index_width + 1) + '} waypoints in ').format(len(waypoints)) + f'{rddf.meters:.2f} meters')
            (xs, ys) = (waypoints[ 0].point.x, waypoints[ 0].point.y) 
            (xf, yf) = (waypoints[-1].point.x, waypoints[-1].point.y)
            tee_print(of, line.format(0, 'start point', '', xs, ys, svg_units(xs - xo), svg_units(wh - (ys - yo)), ''))
            tee_print(of, line.format((len(waypoints) - 1), 'finish point', '', xf, yf, svg_units(xf - xo), svg_units(wh - (yf - yo)), ''))
            if distance((xf, yf), (xs, ys)) <= (2.0 * args.gen_dist):
                phi = None
                inexact = 'inexact ' * ((xs, ys) != (xf, yf))
                if (xs, ys) == (xf, yf):
                    delta_theta = delta_angle(waypoints[1].theta, waypoints[-1].theta)
                    dist = waypoints[1].point.distance(waypoints[-1].point) 
                    phi = steering_angle(args.front_rear, delta_theta, dist)
                tee_print(of, line.format((len(waypoints) - 1), inexact + 'loop closure', '', 
                    xf, yf, svg_units(xf - xo), svg_units(wh - (yf - yo)), bad_phi_warning(phi)))
            for junction in sorted(rddf.junctions, key=order_by_rddf_index):
                (x, y) = (junction.point.x, junction.point.y)
                waypoint_index = junction.waypoint_index + (0 if junction.waypoint_index >= 0 else len(waypoints))
                inexact = 'inexact ' * (junction.point != junction.rddf2.waypoints[junction.waypoint_index2].point)
                tee_print(of, line.format(waypoint_index, inexact + junction.junction_type, f'RDDF #{(junction.rddf2.index + 1):<4}', 
                    x, y, svg_units(x - xo), svg_units(wh - (y - yo)), bad_phi_warning(junction.phi)))
            for waypoint_index in rddf.bad_phi_waypoints:
                (x, y, phi) = (waypoints[waypoint_index].point.x, waypoints[waypoint_index].point.y, waypoints[waypoint_index].phi)
                tee_print(of, line.format(waypoint_index, 'bad waypoint', '',  x, y, svg_units(x - xo), svg_units(wh - (y - yo)), bad_phi_warning(phi)))
        if not args.analysis_only:
            rddfs_ordered = sorted(svg.rddfs, key=lambda rddf: len(rddf.waypoints), reverse=True)
            tee_print(of, f"\nCommand for re-editing the RDDF{'s' * (len(svg.rddfs) > 1)}:\n{reedit_commandline(rddfs_ordered, utm_window)}")
            tee_print(of, f"\nCommand for generating the graph file:\n{generate_graph_commandline(rddfs_ordered)}")
    print(f"\nSee Graph Summary Report in '{outfilename}'")


def lineno():
    cf = currentframe()
    return cf.f_back.f_lineno


def reset_time():
    global t
    t = time.time()


def elapsed_time(ref):
    global t
    t1 = time.time()
    print(f'{ref}: {(t1 - t):.2f}')
    t = t1

def process_svg(svg_filename):
    global count_files, total_files

    count_files += 1
    count_msg = f'({count_files} of {total_files})' * (total_files > 1)
    if not os.path.isfile(svg_filename):
        sys.stderr.write(f"error: skipped SVG file '{svg_filename}': file not found\n")
        return
    svg_basename = os.path.basename(svg_filename)
    coord = svg_basename[1:-4].split('_')
    if len(coord) < 2 or not is_float(coord[:2]) or svg_basename[-4:] != '.svg':
        sys.stderr.write(f"error: skipped SVG file '{svg_filename}': expected filename pattern: @<x>_<y>*.svg\n")
        return
    origin = [ float(val) for val in coord[:2] ]
    svg = SVG(svg_filename, origin)
    write_annotation(svg)
    if not input('\nProceed with the generation of RDDF files? [Y/n]: ') in 'Yy':
        print()
        return
    get_paths_from_svg(svg)
    get_rddfs_from_svg_paths(svg)
    if not svg.rddfs:
        sys.stderr.write(f"error: SVG file '{svg_filename}': no RDDF was generated\n")
        return
    print(f"Processing SVG file: '{svg_filename}'  {count_msg}")
    get_end_nodes(svg)

    left_rddfs = []
    right_rddfs = []
    lane_rddfs = []

    for obj in svg.rddfs:
        if "lb_" in obj.filename:
            left_rddfs.append(obj)
        elif "rb_" in obj.filename:
            right_rddfs.append(obj)
        elif "ANNOTATION" not in obj.filename or "TYPE" not in obj.filename:
            lane_rddfs.append(obj)

    for i in range(len(lane_rddfs)):
        rddf = lane_rddfs[i]

        get_bezier_curve(rddf.svg_path)

        if rddf.svg_path.bezier_curve:
            convert_coordinates_from_svg_to_utm(rddf.svg_path)
            get_rddf_from_bezier_curve(rddf, args.gen_dist)

        if (left_rddfs and right_rddfs):
            rddf_left = left_rddfs[i]
            rddf_right = right_rddfs[i]
            get_bezier_curve(rddf_left.svg_path)
            get_bezier_curve(rddf_right.svg_path)

            convert_coordinates_from_svg_to_utm(rddf_left.svg_path)
            get_rddf_from_bezier_curve(rddf_left, args.gen_dist)

            convert_coordinates_from_svg_to_utm(rddf_right.svg_path)
            get_rddf_from_bezier_curve(rddf_right, args.gen_dist)

            get_inexact_junctions(rddf_left)
            get_inexact_junctions(rddf_right)

            if (rddf_left.svg_path.bezier_curve and rddf_right.svg_path.bezier_curve):
                write_rddf(rddf, rddf_left, rddf_right)
            else:
                write_rddf(rddf)
                get_inexact_junctions(rddf)

        else:
            write_rddf(rddf)
 
            get_inexact_junctions(rddf)

    utm_window = origin + [ meters(svg.width), meters(svg.height) ]
    
    get_junctions_phi(svg.rddfs)

    
    print_summary(svg, utm_window)


def shutdown(signum, *frame):
    del frame
    sys.exit(signum)


def usage_exit(argument = None, msg = None, value = None):
    if argument is None and msg is None:
        parser.print_help(sys.stderr)
        print()
    else:
        parser.print_usage(sys.stderr)
        full_msg = f'{os.path.basename(sys.argv[0])}: error'
        if argument:
            full_msg += f': argument {argument}'
        if msg:
            full_msg += f': {msg}'
        if not value is None:
            full_msg += f": '{value}'"
        sys.stderr.write(full_msg + '\n')
    sys.exit(1)


def sanity_check(rddf_list, window_limits):
    (x_min, y_min, _, _) = window_limits
    svg_filename = get_svg_filename(x_min, y_min)
    origin = [ x_min, y_min ]
    svg = SVG(svg_filename, origin)
    for i, (rddf_file, rddf_poses, rddf_forward) in enumerate(rddf_list):
        rddf = RDDF(svg, i, rddf_file, None, rddf_forward)
        svg.rddfs.append(rddf)
        (x_start,  y_start,  _) = rddf_poses[ 0]
        (x_finish, y_finish, _) = rddf_poses[-1]
        svg.end_nodes.append(EndNode(svg, (2 * i    ), rddf,  0, Point(x_start,  y_start ).round()))
        svg.end_nodes.append(EndNode(svg, (2 * i + 1), rddf, -1, Point(x_finish, y_finish).round()))
    for i, (rddf_file, rddf_poses, rddf_forward) in enumerate(rddf_list):
        rddf = svg.rddfs[i]
        last_point = last_theta = None
        for i_pose, (x, y, theta) in enumerate(rddf_poses):
            point = Point(x, y)
            waypoint = WayPoint(point, theta)
            rddf.waypoints.append(waypoint)
            index = i_pose if i_pose != (len(rddf_poses) - 1) else -1
            if junction_node(point, rddf, index) or index in (0, -1):
                waypoint.point = point.round()
            delta_theta = delta_angle(waypoint.theta, last_theta) if not (last_theta is None) else 0.0
            dist = waypoint.point.distance(last_point) if not (last_point is None) else args.gen_dist
            waypoint.phi = steering_angle(args.front_rear, delta_theta, dist)
            bad_phi(waypoint, index, rddf.bad_phi_waypoints)
            rddf.meters += dist if not (last_point is None) else 0.0
            last_theta = waypoint.theta
            last_point = waypoint.point
        get_inexact_junctions(rddf)
    get_junctions_phi(svg.rddfs)
    return svg


def get_junction_nodes(rddf):
    junction_nodes = [ 0, (len(rddf.waypoints) - 1) ]
    for junction in rddf.junctions:
        index = junction.waypoint_index + (0 if junction.waypoint_index >= 0 else len(rddf.waypoints))
        if not index in junction_nodes:
            junction_nodes.append(index)
    return sorted(junction_nodes)


def remove_poses_from_rddfs(rddf_list, svg, target_dist):
    print(f'\nPerforming the removal of poses from RDDFs using criterium: target distance between nodes is {target_dist} meters:')
    for i_list, (rddf_file, rddf_poses, rddf_forward) in enumerate(rddf_list):
        rddf = svg.rddfs[i_list]
        rddf_poses_kept = []
        base_indexes = get_junction_nodes(rddf)
        for j in range(len(base_indexes) - 1):
            base_index = base_indexes[j]
            next_base_index = base_indexes[j + 1]
            rddf_poses_kept.append(rddf_poses[base_index])
            dist_base_points = 0.0
            for i in range(base_index, next_base_index):
                dist_base_points += rddf.waypoints[i].point.distance(rddf.waypoints[i + 1].point)
            waypoint_count = int(round(dist_base_points / target_dist))
            if waypoint_count > 1:
                dist_points = dist_base_points / waypoint_count
                dist = 0.0
                waypoints_since_last_base = 1
                for i in range((base_index + 1), next_base_index):
                    if waypoints_since_last_base == waypoint_count:
                        break
                    dist += rddf.waypoints[i].point.distance(rddf.waypoints[i - 1].point)
                    if dist >= (dist_points * waypoints_since_last_base):
                        rddf_poses_kept.append(rddf_poses[i])
                        waypoints_since_last_base += 1
            if next_base_index == base_indexes[-1]:
                rddf_poses_kept.append(rddf_poses[next_base_index])

        remove_count = len(rddf_poses) - len(rddf_poses_kept)
        remove_msg = '(unchanged)' if remove_count == 0 else f'({remove_count} poses removed out of {len(rddf_poses)})'
        print(f"<--- RDDF file #{i_list + 1}: '{rddf_file}' now contains {len(rddf_poses_kept)} waypoints  {remove_msg}")
        rddf_list[i_list] = (rddf_file, rddf_poses_kept, rddf_forward)


def make_svg():
    annotations_list = []

    rddf_filelist = get_filelist(args.rddf_filelist)
    (rddf_list, rddf_limits, loop_closures) = get_rddf_list(args.rddf_filenames + rddf_filelist)

    if(args.annotation):
        annotations_list,_ = get_annotations_list(args.annotation)

    (image_list, window_limits) = get_image_list(args.imagedir, rddf_limits)

    (x_min, y_min, x_max, y_max) = window_limits
    utm_window = [ x_min, y_min, (x_max - x_min), (y_max - y_min) ]
    print(f'--window  {utm_window[0]}  {utm_window[1]}  {utm_window[2]}  {utm_window[3]}  --unit {args.unit}  --scale {args.scale}')
    for (rddf_index, filename, (xs, ys), (xf, yf), _) in loop_closures:
        inexact = ((xs, ys) != (xf, yf)) * 'inexact '
        print(f'{inexact}loop closure:  (x,y)  start: ({svg_units(xs - x_min):>8.2f}, {svg_units(y_max - ys):>8.2f})  ' + 
              f'finish: ({svg_units(xf - x_min):>8.2f}, {svg_units(y_max - yf):>8.2f})  RDDF file #{rddf_index + 1}: {os.path.basename(filename)}')
        
    svg = sanity_check(rddf_list, window_limits)
    if args.rem_dist > 0.0:
        remove_poses_from_rddfs(rddf_list, svg, args.rem_dist)
        svg = sanity_check(rddf_list, window_limits)

    print_summary(svg, utm_window)
    
    if args.analysis_only:
        svg_file = args.outputdir + '/' + get_svg_filename(x_min, y_min)
        print(f"\n---> Analysis only ***** SVG file '{svg_file}' not generated\n")
        sys.exit(0)

    svg_file = write_svg(rddf_list, image_list, window_limits, loop_closures, svg, annotations_list)
    
    if not os.popen(f'which {GRAPHICS_EDITOR}').read():
        sys.stderr.write(f'{PROG_DESCRIPTION}\n\n')
        usage_exit(msg=f"'{GRAPHICS_EDITOR}' graphics editor is not installed")
    
    result_code = os.system(f'{GRAPHICS_EDITOR} {svg_file} 2>/dev/null ; exit $?')
    if not os.WIFEXITED(result_code) or os.WEXITSTATUS(result_code) != 0:
        shutdown(os.WEXITSTATUS(result_code))

    create_list_rddfs(svg)

    return svg_file

def read_graph_file (f_graph, has_alternative_paths, new_traffic_restrictions):
    print("read_graph infos...")
    trash = ""
    qtd_graph_nodes = 0
    qtd_node_edges = 0
    qtd_node_edges_in = 0
    qtd_node_nearby_lanes = 0
    qtd_node_nearby_crossroads = 0
    qtd_graph_edges = 0
    graph = Graph()
    cont_loop = 0
    line_1 = f_graph.readline()
    first_line = line_1.strip()

    if first_line[0] == '#':
        new_traffic_restrictions = 1
        number_of_character_in_string = first_line.count(' ')
        if number_of_character_in_string > 1:
            has_alternative_paths = 1
        else:
            has_alternative_paths = 0

        trash, qtd_graph_nodes = f_graph.readline().strip().split()
    else:
        new_traffic_restrictions = 0
        has_alternative_paths = 0
        f_graph.seek(0)
        trash, qtd_graph_nodes = f_graph.readline().strip().split()

    qtd_graph_nodes = int(qtd_graph_nodes)
    print(trash, qtd_graph_nodes)

    i = 0
    while i < qtd_graph_nodes:
        n = Node()
        e = Edge()
        e_in = Edge()
        near = NearbyLane()
        cross = Crossroad()
        
        trash, n.id, n.type, n.lane_id, n.rddf_point.pose.x, n.rddf_point.pose.y, n.rddf_point.pose.theta, n.rddf_point.driver_velocity, n.rddf_point.phi, n.rddf_point.timestamp, n.lat, n.lon, n.traffic_restrictions = f_graph.readline().strip().split()
        trash, n.edge_index_ref_in_egdes_vector_of_lane_graph = f_graph.readline().strip().split()

        if trash[:4] != "EDGE":
            qtd_node_edges = int(n.edge_index_ref_in_egdes_vector_of_lane_graph)
            n.edge_index_ref_in_egdes_vector_of_lane_graph = -1
        else:
            trash, qtd_node_edges = f_graph.readline().strip().split()

        qtd_node_edges = int(qtd_node_edges)
        j = 0

        
        # print("||||||||||")

        while j < qtd_node_edges:
            trash, e.u, e.v, e.cost = f_graph.readline().strip().split()
            n.edges.append(e)
            
            j += 1
        
        trash, qtd_node_edges_in = f_graph.readline().strip().split()
        qtd_node_edges_in = int(qtd_node_edges_in)
        j = 0
        while j < qtd_node_edges_in:
            trash, e_in.u, e_in.v, e_in.cost = f_graph.readline().strip().split()
            n.edges_in.append(e_in)
            j += 1

        trash, qtd_node_nearby_lanes = f_graph.readline().strip().split()
        qtd_node_nearby_lanes = int(qtd_node_nearby_lanes)
        j = 0
        while j < qtd_node_nearby_lanes:
            trash, near.nearby_lane_id, near.initial_node_id, near.nearby_lane_size = f_graph.readline().strip().split()
            n.nearby_lanes.append(near)
            j += 1

        trash, qtd_node_nearby_crossroads = f_graph.readline().strip().split()
        qtd_node_nearby_crossroads = int(qtd_node_nearby_crossroads)
        j = 0
        while j < qtd_node_nearby_crossroads:
            trash, cross.edge_a.u, cross.edge_a.v, cross.edge_b.u, cross.edge_b.v = f_graph.readline().strip().split()
            n.nearby_crossroads.append(cross)
            j += 1

        graph.nodes.append(n)

        n = Node()


        i += 1

    trash, qtd_graph_edges = f_graph.readline().strip().split()
    qtd_graph_edges = int(qtd_graph_edges)
    k = 0
    while k < qtd_graph_edges:
        e = Edge()
        trash, e.u, e.v, e.cost = f_graph.readline().strip().split()
        init = int(e.u)
        fim = int(e.v)
        if((init - fim > 1 or init - fim <-1) and init - fim != 0):
            cont_loop+=1
        else:
            graph.edges.append(e)
        k += 1
    print("Loops: ",cont_loop)
    print("returning graph")
    return graph

def read_lane_graph_file (f_graph):
    trash = ""
    qtd_lane_graph_nodes = 0
    qtd_lane_graph_node_edges = 0
    qtd_lane_graph_node_edges_in = 0
    qtd_lane_graph_edges = 0
    qtd_minimum_graph_node_id_in_lane_segment = 0
    qtd_maximum_graph_node_id_in_lane_segment = 0
    lane_graph = LaneGraph()

    trash, qtd_lane_graph_nodes = f_graph.readline().strip().split()
    qtd_lane_graph_nodes = int(qtd_lane_graph_nodes)
    i = 0

    while i < qtd_lane_graph_nodes:
        n = LaneGraphNode()
        e = LaneGraphEdge()
        
        trash, n.id, n.id_ref_in_graph, n.type, n.rddf_point.pose.x, n.rddf_point.pose.y, n.rddf_point.pose.theta, n.rddf_point.driver_velocity, n.rddf_point.phi, n.rddf_point.timestamp, n.lat, n.lon = f_graph.readline().strip().split()
        n.id, n.id_ref_in_graph = int(n.id), int(n.id_ref_in_graph)
        n.rddf_point.pose.x, n.rddf_point.pose.y, n.rddf_point.pose.theta = float(n.rddf_point.pose.x), float(n.rddf_point.pose.y), float(n.rddf_point.pose.theta)
        n.rddf_point.driver_velocity, n.rddf_point.phi, n.rddf_point.timestamp = float(n.rddf_point.driver_velocity), float(n.rddf_point.phi), float(n.rddf_point.timestamp)
        n.lat, n.lon = float(n.lat), float(n.lon)

        qtd_minimum_graph_node_id_in_lane_segment = int(f_graph.readline().strip().split()[1])
        j = 0
        while j < qtd_minimum_graph_node_id_in_lane_segment:
            trash, val = f_graph.readline().strip().split()
            n.minimum_graph_node_id_in_lane_segment.append(int(val))
            j += 1

        qtd_maximum_graph_node_id_in_lane_segment = int(f_graph.readline().strip().split()[1])
        j = 0
        while j < qtd_maximum_graph_node_id_in_lane_segment:
            trash, val = f_graph.readline().strip().split()
            n.maximum_graph_node_id_in_lane_segment.append(int(val))
            j += 1

        qtd_lane_graph_node_edges = int(f_graph.readline().strip().split()[1])
        j = 0
        while j < qtd_lane_graph_node_edges:
            trash, e.u, e.v, e.u_ref_in_graph, e.v_ref_in_graph, e.cost = f_graph.readline().strip().split()
            e.u, e.v, e.u_ref_in_graph, e.v_ref_in_graph = int(e.u), int(e.v), int(e.u_ref_in_graph), int(e.v_ref_in_graph)
            e.cost = float(e.cost)
            n.edges.append(e)
            j += 1

        qtd_lane_graph_node_edges_in = int(f_graph.readline().strip().split()[1])
        j = 0
        while j < qtd_lane_graph_node_edges_in:
            trash, e.u, e.v, e.u_ref_in_graph, e.v_ref_in_graph, e.cost = f_graph.readline().strip().split()
            e.u, e.v, e.u_ref_in_graph, e.v_ref_in_graph = int(e.u), int(e.v), int(e.u_ref_in_graph), int(e.v_ref_in_graph)
            e.cost = float(e.cost)
            n.edges_in.append(e)
            j += 1

        lane_graph.nodes.append(n)
        n.edges.clear()
        i += 1

    trash, qtd_lane_graph_edges = f_graph.readline().strip().split()
    qtd_lane_graph_edges = int(qtd_lane_graph_edges)
    k = 0
    while k < qtd_lane_graph_edges:
        e = LaneGraphEdge()
        trash, e.u, e.v, e.cost = f_graph.readline().strip().split()
        e.u, e.v = int(e.u), int(e.v)
        e.cost = float(e.cost)
        lane_graph.edges.append(e)
        k += 1

    return lane_graph


def get_graph_from_file (graph, lane_graph, filename):
    has_alternative_paths = -1
    new_traffic_restrictions = -1 
    with open(filename, 'r') as f_graph:
        graph = read_graph_file(f_graph, has_alternative_paths, new_traffic_restrictions)
        if f_graph:
            lane_graph = read_lane_graph_file(f_graph)
            has_lane_graph = 1
            for i in range(len(lane_graph.nodes)):
                graph.nodes[lane_graph.nodes[i].id_ref_in_graph].id_ref_in_lane_graph = lane_graph.nodes[i].id
                # print("node", lane_graph.nodes[i].id_ref_in_graph, "has id_ref_in_lane_graph", graph.nodes[lane_graph.nodes[i].id_ref_in_graph].id_ref_in_lane_graph)
        f_graph.close()
    return graph, lane_graph


def normalize_theta (theta):
    if -math.pi <= theta < math.pi:
        return theta

    multiplier = (int)(theta / (2 * math.pi))
    theta = theta - multiplier * 2 * math.pi
    if theta >= math.pi:
        theta -= 2 * math.pi
    if theta < -math.pi:
        theta += 2 * math.pi

    return float(theta)


def write_rddf_point(rddf, point, side_angle, traffic_restrictions_lr):
    x = 0.0
    y = 0.0
    if math.fabs(side_angle) == 0:
        rddf.write(f"{point.pose.x}\t{point.pose.y}\t{point.pose.theta}\t{point.driver_velocity}\t{point.phi}\t{point.timestamp}\n")
    else:
        node_side_angle = normalize_angle(float(point.pose.theta) + side_angle)
        
        x = float(point.pose.x )+ (traffic_restrictions_lr * math.cos(node_side_angle))
        y = float(point.pose.y) + (traffic_restrictions_lr * math.sin(node_side_angle))

        rddf.write(f"{x}\t{y}\t{point.pose.theta}\t{point.driver_velocity}\t{point.phi}\t{point.timestamp}\n")

def extract_all_rddfs_from_lane_graph (lane_graph,graph,output_dir):
    extract_lane_border = False  # Set this flag as needed
    right_side_angle = math.pi / 2.0
    left_side_angle = -math.pi / 2.0
    traffic_restrictions = 0
    new_traffic_restrictions =  -1
    traffic_restrictions_left = 0
    traffic_restrictions_right = 0
    result_left = ""
    result_right = ""
    result = ""

    for i, initial_node in enumerate(lane_graph.nodes):
        for j, next_node_id in enumerate(initial_node.minimum_graph_node_id_in_lane_segment):
            end_node_id = initial_node.maximum_graph_node_id_in_lane_segment[j]
            rddf_filename = f"{output_dir}/rddf_lane_{initial_node.id}_{graph.nodes[end_node_id].id_ref_in_lane_graph}.txt"
            rddf = open(rddf_filename, "w")

            if not rddf:
                exit(f"\nCould not open rddf file for writing: {rddf_filename}\n\n")

            if extract_lane_border:
                rddf_left_filename = f"{output_dir}/lb_{initial_node.id}_{graph.nodes[end_node_id].id_ref_in_lane_graph}.txt"
                rddf_right_filename = f"{output_dir}/rb_{initial_node.id}_{graph.nodes[end_node_id].id_ref_in_lane_graph}.txt"

                rddf_left = open(rddf_left_filename, "w")
                rddf_right = open(rddf_right_filename, "w")

                traffic_restrictions = graph.nodes[next_node_id].traffic_restrictions
                traffic_restrictions_left = route_planner_get_lane_left_width(int(traffic_restrictions), new_traffic_restrictions)
                traffic_restrictions_right = route_planner_get_lane_right_width(int(traffic_restrictions), new_traffic_restrictions)

                write_rddf_point(rddf_left, initial_node.rddf_point, left_side_angle, traffic_restrictions_left)
                write_rddf_point(rddf_left, graph.nodes[next_node_id].rddf_point, left_side_angle, traffic_restrictions_left)

                write_rddf_point(rddf_right, initial_node.rddf_point, right_side_angle, traffic_restrictions_right)
                write_rddf_point(rddf_right, graph.nodes[next_node_id].rddf_point, right_side_angle, traffic_restrictions_right)

            write_rddf_point(rddf, initial_node.rddf_point, 0.0, graph.nodes[next_node_id].traffic_restrictions)
            write_rddf_point(rddf, graph.nodes[next_node_id].rddf_point, 0.0, graph.nodes[next_node_id].traffic_restrictions)

            while next_node_id != end_node_id:
                next_node_id = int(graph.nodes[next_node_id].edges[0].v)
                if extract_lane_border:
                    traffic_restrictions = graph.nodes[next_node_id].traffic_restrictions

                    traffic_restrictions_left = route_planner_get_lane_left_width(int(traffic_restrictions), new_traffic_restrictions)
                    traffic_restrictions_right = route_planner_get_lane_right_width(int(traffic_restrictions), new_traffic_restrictions)

                    write_rddf_point(rddf_left, graph.nodes[next_node_id].rddf_point, left_side_angle, traffic_restrictions_left)
                    write_rddf_point(rddf_right, graph.nodes[next_node_id].rddf_point, right_side_angle, traffic_restrictions_right)
                    write_rddf_point(rddf, graph.nodes[next_node_id].rddf_point, 0.0, graph.nodes[next_node_id].traffic_restrictions)
                else:
                    write_rddf_point(rddf, graph.nodes[next_node_id].rddf_point, 0.0, graph.nodes[next_node_id].traffic_restrictions)

            result += f"{rddf_filename}\n"
            rddf.close()

            if extract_lane_border:
                result_left += f"{rddf_left_filename}\n"
                result_right += f"{rddf_right_filename}\n"
                rddf_left.close()
                rddf_right.close()

    rddf_lists_rddf_filename = f"{output_dir}/rddfs_list.txt"
    rddf_lists_rddf_left_filename = f"{output_dir}/rddfs_list_l.txt"
    rddf_lists_rddf_right_filename = f"{output_dir}/rddfs_list_r.txt"

    linhas_result = result.strip().split('\n')
    linhas_result_left = result_left.strip().split('\n')
    linhas_result_right = result_right.strip().split('\n')

    # Conjunto para armazenar diretórios únicos
    diretorios_result = set()
    diretorios_result_left = set()
    diretorios_result_right = set()

    # Iterando pelas linhas e adicionando diretórios ao conjunto
    for linha in linhas_result:
        diretorios_result.add(linha)
        
    for linha in linhas_result_left:
        diretorios_result_left.add(linha)

    for linha in linhas_result_right:
        diretorios_result_right.add(linha)

    diretorios_result = '\n'.join(diretorios_result) + '\n'
    diretorios_result_left = '\n'.join(diretorios_result_left) + '\n'
    diretorios_result_right = '\n'.join(diretorios_result_right)  + '\n'

    list_dir_borders_rddfs = open(rddf_lists_rddf_filename, "w")
    list_dir_borders_rddfs.write(diretorios_result)
    list_dir_borders_rddfs.close()

    if extract_lane_border:
        list_dir_borders_rddfs_left = open(rddf_lists_rddf_left_filename, "w")
        list_dir_borders_rddfs_right = open(rddf_lists_rddf_right_filename, "w")

        list_dir_borders_rddfs_left.write(diretorios_result_left)
        list_dir_borders_rddfs_right.write(diretorios_result_right)

        list_dir_borders_rddfs_left.close()
        list_dir_borders_rddfs_right.close()
    else:
        diretorios_result = diretorios_result[0:len(diretorios_result)-1]
    full_directories_filename = f"{output_dir}/full_directories.txt"
    full_directories = open(full_directories_filename, "w")
    full_directories.write(diretorios_result)
    full_directories.write(diretorios_result_left)
    full_directories.write(diretorios_result_right)
    full_directories.close()


def is_float(var):
    if isinstance(var, list) or isinstance(var, tuple):
        for element in var:
            if not is_float(element):
                return False
    else:
        try:
            float(var)
        except ValueError:
            return False
    return True


def _positive_float(s):
    if not is_float(s) or float(s) <= 0.0:
        raise argparse.ArgumentTypeError(f"invalid positive float value: '{s}'")
    return float(s)


def _dir(s):
    if not os.path.isdir(s):
        raise argparse.ArgumentTypeError(f"directory not found: '{s}'")
    return s


def _file(s):
    if not os.path.isfile(s):
        raise argparse.ArgumentTypeError(f"file not found: '{s}'")
    return s


def create_default_outputdir ():
    global args

    prefix_name = "output"
    reference_output = args.outputdir

    if (not args.default_outputdir):
        if not os.path.isdir(reference_output):
            parser.print_usage()
            sys.stderr.write(f"{parser.prog}: error: argument -o/--outputdir: directory not found: '{reference_output}'")
            exit(0)

        return

    if (not os.path.isdir(reference_output) or reference_output == "."):
        if (reference_output == "."):
            date_format = '%Y_%m_%d_%H_%M_%S_%f'
            current_date = datetime.now().strftime(date_format)
            reference_output = f"{prefix_name}_{current_date}"
            args.outputdir = reference_output

        try:
            os.mkdir(reference_output)

        except Exception as e:
            sys.stderr.write(f"Error when trying to create the folder: {reference_output} (error: {e})")
            exit(0)

def read_parameters():
    global parser, args
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-a', '--analysis_only', help='print analysis of given RDDF or SVG files and stop  (default: False)', action='store_true', dest='analysis_only', default=False)
    parser.add_argument('-i', '--imagedir', help='image directory  (default: .)', type=_dir, default='.')
    parser.add_argument('-o', '--outputdir', help='output directory  (default: .)', default='.')
    parser.add_argument('-dout', '--default_outputdir', help='enables automatic creation of output directories (default: False)', action='store_true', dest="default_outputdir", default=False)
    parser.add_argument('-x', '--overwrite', help='overwrite existing RDDF files  (default: False)', action='store_true', dest='overwrite', default=False)
    parser.add_argument('-w', '--window', help='window origin and size in UTM meters: x y width height', type=float, nargs=4)
    parser.add_argument('-u', '--unit', help='SVG document unit  (default: mm)', choices=('mm', 'px'), default='mm')
    parser.add_argument('-sw', '--stroke_width', help='SVG line stroke width in meters  (default: 0.4)', type=_positive_float, default=0.4)
    parser.add_argument('-s', '--scale', help='image pixel scale in meters  (default: 0.2)', type=_positive_float, default=0.2)
    parser.add_argument('-d', '--gen_dist', help='RDDF distance between waypoints in meters (used for generation)  (default: 2.0)', type=_positive_float, default=0.5)
    parser.add_argument('-rd', '--rem_dist', help='SVG path distance between nodes in meters (used for removal)  (default: 0.0)', type=_positive_float, default=0.5)
    parser.add_argument('-fr', '--front_rear', help='distance between front and rear axles of the vehicle in meters  (default: 2.625)', type=_positive_float, default=2.625)
    parser.add_argument('-ms', '--max_steering', help='maximum steering angle of the vehicle in radians  (default: 0.5237)', type=_positive_float, default=0.5237)
    parser.add_argument('-svg', '--svg_filename', help='SVG filename', type=_file)
    parser.add_argument('-sl', '--svg_filelist', help='text file containing a list of SVG filenames (one per line)', type=_file)
    parser.add_argument('-rl', '--rddf_filelist', help='text file containing a list of RDDF filenames (one per line)', type=_file)
    parser.add_argument('-process', '--process', help='Process.ini file entry for catch graph map and annotations', type=_file)
    parser.add_argument('rddf_filenames', help='RDDF filenames (separated by spaces)', type=_file, nargs='*')
    parser.add_argument('-annotation','-annotation', help='annotations file directory (default: .)', type=_file)
    parser.add_argument('-gr','--graph_filename', help='graph file directory (default: .)', type=_file)
    parser.add_argument('-map_plot_inf','--map_plot_inf', help='execute build complete map for cat offline map and height map(default: False)',  default=False)
    parser.add_argument('-smooth_borders','--smooth_borders', help='apply smooth_rddf on borders (default: False)',  default=True)
    args = parser.parse_args()
    
    if args.process:
        regex_patterns = [
            r'^SET MAP_PATH=(.*)$',
            r'^SET GRAPH_PATH=(.*)$',
            r'^SET ANNO_PATH=(.*)$',
            r'^SET ANNOTATION_PATH=(.*)$',
            r'^SET PATH_CENTRAL=(.*)$'
        ]

        paths = []
        save_map_image = "../grid_mapping/save_map_images"
        images_dir = ""
        map_dir = ""
        path_central = ""
        with open(args.process, 'r') as file:
            for line in file:
                for pattern in regex_patterns:
                    match = re.match(pattern, line)
                    if match:
                        paths.append((pattern,match.group(1)))
                        break
        cwd = os.getcwd()
        cwd_split = cwd.split("/")
        
        if str(cwd_split[-1]) == "bin":
            save_map_image = "./save_map_images"

        for path in paths:
            if "PATH_CENTRAL" in path[0]:
                if str(cwd_split[-1]) == "bin":
                     path_central = path[1].strip()
                else:
                    path_central = "../../bin/"+ path[1].strip()

            if "MAP_PATH" in path[0]:
                if(path_central != "" and "PATH_CENTRAL" in path[1]):
                    map_path = path[1].strip()
                    map_dir = map_path.replace("${PATH_CENTRAL}",path_central)
                elif(path[1].strip()[0] == "/"):
                    map_dir = path[1].strip()
                else:
                    if str(cwd_split[-1]) == "bin":
                         map_dir = path[1].strip()
                    else:
                        map_dir = path[1].strip()
                        map_dir = "../../bin/"+map_dir

            if "GRAPH_PATH" in path[0]:
                if(path_central != "" and "PATH_CENTRAL" in path[1]):
                    graph_path = path[1].strip()
                    args.graph_filename = graph_path.replace("${PATH_CENTRAL}",path_central)
                elif(path[1].strip()[0] == "/"):
                    args.graph_filename = path[1].strip()
                else:
                    if str(cwd_split[-1]) == "bin":
                         args.graph_filename = path[1].strip()
                    else:
                        args.graph_filename = path[1].strip()
                        args.graph_filename = "../../bin/"+args.graph_filename

            if "ANNOTATION_PATH" in path[0] or "ANNO_PATH" in path[0]:
                if(path_central != "" and "PATH_CENTRAL" in path[1]):
                    annotation_path = path[1].strip()
                    args.annotation  = annotation_path.replace("${PATH_CENTRAL}",path_central)
                elif(path[1].strip()[0] == "/"):
                    args.annotation = path[1].strip()
                else:
                    if str(cwd_split[-1]) == "bin":
                         args.annotation = path[1].strip()
                    else:
                        args.annotation = path[1].strip()
                        args.annotation = "../../bin/"+args.annotation

        if map_dir == "" or args.graph_filename == None:
            with open(args.process, 'r') as arquivo:
                linha_split = []
                for linha in arquivo:
                    linha = linha.strip()
                    if linha.startswith("mapper") and map_dir == "":
                        linha_split = linha.split()
                        index = linha_split.index("-map_path")
                        map_dir = linha_split[index+1].strip()

                    if linha.startswith("route_planner") and args.graph_filename == None:
                        linha_split = linha.split()
                        index = linha_split.index("--graph")
                        args.graph_filename = linha_split[index+1].strip()
                        if args.annotation == None:
                            args.annotation = linha_split[index+2].strip()
            if map_dir == "" or args.graph_filename == None:
                print("Para usar a funcionalidade --process é necessario conter no .ini ao menos o caminho de um grapho e de um mapa!\n")

        if map_dir:
            try:
                images_dir = args.outputdir + "/imgs"

                if not os.path.exists(images_dir):
                    os.makedirs(images_dir)
                    print(f"O diretório '{images_dir}' foi criado com sucesso.")
                else:
                    print(f"O diretório '{images_dir}' já existe.")
                subprocess.run([save_map_image,"-input_dir", map_dir, "-out_dir", images_dir])
                args.imagedir = images_dir

            except FileNotFoundError:
                print("Ocorreu algum erro durante a execução do programa ./save_map_images")
    if len(sys.argv) == 1:
        usage_exit()

    if args.window:
        (_, _, width, height) = args.window 
        if width <= 0.0 or height <= 0.0:
            usage_exit('-w/--window', 'invalid positive float value (width, height)', (width, height))
   
    output_dir = args.outputdir + "/extract"

    if args.graph_filename:

        # graph = Graph()
        # lane_graph = LaneGraph()

        # graph,lane_graph = get_graph_from_file(graph,lane_graph,args.graph_filename)

        output_dir = args.outputdir + "/extract"

        if (os.path.isdir(output_dir)):
            for arquivo in os.listdir(output_dir):
                caminho_completo = os.path.join(output_dir, arquivo)
                if os.path.isfile(caminho_completo):
                    os.remove(caminho_completo)
            os.rmdir(output_dir)
            os.mkdir(output_dir)
        else:
            os.mkdir(output_dir)

        extract_rddfs_from_graph_v2 = "./extract_rddfs_from_graph_v2"
        subprocess.run([extract_rddfs_from_graph_v2, args.graph_filename, "-o", output_dir, "-extract_lane_border"])
        time.sleep(5)
        args.rddf_filelist = output_dir+"/full_directories.txt"

        print(args.graph_filename)
        if (args.smooth_borders):
            # extract_all_rddfs_from_lane_graph(lane_graph,graph,output_dir)
            try:
                with open(args.rddf_filelist, 'r') as file:
                    lines = file.readlines()
                    for line in lines:
                        if 'lb_' in line or 'rb_' in line:
                            smooth = "./smooth_rddf"
                            line = line.strip()
                            subprocess.run([smooth, line, line])
            except:
                print("Não foi possivel suavizar as bordas")
                   
    if args.rddf_filenames or args.rddf_filelist:
        msg = 'SVG option is not allowed together with RDDF files'
        if (args.svg_filename):
            usage_exit('-svg/--svg_filename', msg, args.svg_filename)
        if (args.svg_filelist):
            usage_exit('-sl/--svg_filelist' , msg, args.svg_filelist)

def main():
    global count_files, total_files
    print()
    signal(SIGINT, shutdown)
    read_parameters()
    create_default_outputdir()
    
    svg_filelist = get_filelist(args.svg_filelist)

    if args.svg_filename:
        svg_filelist += [ args.svg_filename ]
    
    if not (args.svg_filename or args.svg_filelist):
        svg_filelist = [ make_svg() ]

    (count_files, total_files) = (0, len(svg_filelist))
    for f in svg_filelist:
        process_svg(f)
    print()


if __name__ == '__main__':
    main()
