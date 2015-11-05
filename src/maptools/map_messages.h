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


/** @addtogroup maptools **/
// @{

/** \file map_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef CARMEN_MAP_MESSAGES_H
#define CARMEN_MAP_MESSAGES_H

#include <carmen/map.h>


#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_HMAP_REQUEST_NAME         "carmen_hmap_request"
typedef carmen_default_message carmen_hmap_request_message;
#define CARMEN_GRIDMAP_REQUEST_NAME      "carmen_gridmap_request"
typedef carmen_default_message carmen_gridmap_request_message;
#define CARMEN_PLACELIST_REQUEST_NAME    "carmen_placelist_request"
typedef carmen_default_message carmen_placelist_request_message;
#define CARMEN_OFFLIMITS_REQUEST_NAME    "carmen_offlimits_request"
typedef carmen_default_message carmen_offlimits_request_message;
#define CARMEN_GLOBAL_OFFSET_REQUEST_NAME "carmen_global_offset_request"
typedef carmen_default_message carmen_global_offset_request_message;

typedef struct {
  carmen_hmap_t hmap;
  double timestamp;
  char *host;
} carmen_hmap_message;

#define CARMEN_MAP_HMAP_NAME "carmen_hmap_message"
#define CARMEN_MAP_HMAP_FMT  "{{int, <string:1>, int, <{int, int, <int:2>, int, <{double, double, double}:4>}:3>},double,string}"

typedef struct {
  unsigned char * map;
  int size;
  int compressed;
  carmen_map_config_t config;
  
  char *err_mesg;
  double timestamp;
  char *host;
} carmen_grid_map_message;

#define CARMEN_MAP_GRIDMAP_NAME    "carmen_grid_map_message"
#define CARMEN_MAP_GRIDMAP_UPDATE_NAME    "carmen_grid_map_update_message"
#define CARMEN_MAP_GRIDMAP_FMT     "{<char:2>, int, int, {int, int, double, [byte:64], string, double, double}, string, double, string}"



typedef struct {
  char *name;
  double timestamp;
  char *host;
} carmen_named_gridmap_request;

#define CARMEN_NAMED_GRIDMAP_REQUEST_NAME    "carmen_named_gridmap_request"
#define CARMEN_NAMED_GRIDMAP_REQUEST_FMT     "{string,double,string}"

typedef struct {  
  carmen_place_p places;
  int num_places;
  double timestamp;
  char *host;
} carmen_map_placelist_message;

#define CARMEN_MAP_PLACELIST_NAME    "carmen_placelist_message"
#define CARMEN_MAP_PLACELIST_FMT     "{<{int,int,[char:22],double,double,double,double,double,double}:2>,int,double,string}"

typedef struct {
  char *name;
  double timestamp;
  char *host;
} carmen_map_named_placelist_request;

#define CARMEN_NAMED_PLACELIST_REQUEST_NAME    "carmen_named_placelist_request"
#define CARMEN_NAMED_PLACELIST_REQUEST_FMT     "{string,double,string}"

typedef struct {  
  carmen_offlimits_p offlimits_list;
  int list_length;
  double timestamp;
  char *host;
} carmen_map_offlimits_message;

#define CARMEN_MAP_OFFLIMITS_NAME    "carmen_offlimits_message"
#define CARMEN_MAP_OFFLIMITS_FMT     "{<{int,int,int,int,int}:2>,int,double,string}"

typedef struct {
  char *name;
  double timestamp;
  char *host;
} carmen_map_named_offlimits_request;

#define CARMEN_NAMED_OFFLIMITS_REQUEST_NAME    "carmen_named_offlimits_request"
#define CARMEN_NAMED_OFFLIMITS_REQUEST_FMT     "{string,double,string}"

typedef struct {  
  carmen_global_offset_t global_offset;
  double timestamp;
  char *host;
} carmen_map_global_offset_message;

#define CARMEN_MAP_GLOBAL_OFFSET_NAME  "carmen_global_offset_message"
#define CARMEN_MAP_GLOBAL_OFFSET_FMT   "{{double,double,double},double,string}"

typedef struct {
  char *name;
  double timestamp;
  char *host;
} carmen_map_named_global_offset_request;

#define CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_NAME "carmen_named_global_offset_request"
#define CARMEN_NAMED_GLOBAL_OFFSET_REQUEST_FMT  "{string,double,string}"

typedef struct {
  char *zone_name;
  double timestamp;
  char *host;
} carmen_map_zone_message;

#define CARMEN_MAP_ZONE_NAME                   "carmen_map_zone_message"
#define CARMEN_MAP_ZONE_FMT                    "{string,double,string}"

typedef struct {
  char *zone_name;
  double timestamp;
  char *host;
} carmen_map_change_map_zone_request;

#define CARMEN_CHANGE_MAP_ZONE_REQUEST_NAME    "carmen_change_map_zone_request"
#define CARMEN_CHANGE_MAP_ZONE_REQUEST_FMT     "{string,double,string}"

typedef struct {
  char *err_msg;
  double timestamp;
  char *host;
} carmen_map_change_map_zone_response;

#define CARMEN_CHANGE_MAP_ZONE_RESPONSE_NAME   "carmen_change_map_zone_response"
#define CARMEN_CHANGE_MAP_ZONE_RESPONSE_FMT    "{string,double,string}"


#ifdef __cplusplus
}
#endif

#endif
// @}
