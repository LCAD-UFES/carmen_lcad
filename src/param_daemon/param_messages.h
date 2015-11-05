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

/** @addtogroup param_daemon **/
// @{

/** \file param_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef CARMEN_PARAM_MESSAGES_H
#define CARMEN_PARAM_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {CARMEN_PARAM_OK, CARMEN_PARAM_NOT_FOUND, CARMEN_PARAM_NOT_INT, 
	      CARMEN_PARAM_NOT_DOUBLE, CARMEN_PARAM_NOT_ONOFF,
	      CARMEN_PARAM_NOT_FILE, CARMEN_PARAM_FILE_ERR} carmen_param_status_t;

#define CARMEN_PARAM_QUERY_ALL_NAME       "carmen_param_query_all"
#define CARMEN_PARAM_QUERY_INT_NAME       "carmen_param_query_int"
#define CARMEN_PARAM_QUERY_DOUBLE_NAME    "carmen_param_query_double"
#define CARMEN_PARAM_QUERY_ONOFF_NAME     "carmen_param_query_onoff"
#define CARMEN_PARAM_QUERY_STRING_NAME    "carmen_param_query_string"
#define CARMEN_PARAM_VERSION_QUERY_NAME   "carmen_param_query_version"
#define CARMEN_PARAM_REREAD_COMMAND_NAME   "carmen_param_reread_command"

typedef carmen_default_message carmen_param_query_version_message;
typedef carmen_default_message carmen_param_reread_command_message;

typedef struct {
  char *module_name;
  char *variable_name;
  double timestamp;
  char *host;
} carmen_param_query_message;

#define CARMEN_PARAM_QUERY_ROBOT_NAME  "carmen_param_query_robot"
#define CARMEN_PARAM_QUERY_FMT     "{string,string,double,string}"

  /** This message reports what robot (i.e., what parameter set) has been
      loaded into the param_daemon. 
  */

typedef struct {
  char *robot;
  carmen_param_status_t status;
  double timestamp;
  char *host;
} carmen_param_response_robot_message;

#define CARMEN_PARAM_RESPONSE_ROBOT_NAME  "carmen_param_respond_robot"
#define CARMEN_PARAM_RESPONSE_ROBOT_FMT  "{string,int,double,string}"

#define CARMEN_PARAM_QUERY_MODULES_NAME   "carmen_param_query_modules"

  /** This message reports a complete list of module names (as determined from
      the ini file by the set of variables with a prepended module name. The
      message fields are undefined if status is not CARMEN_PARAM_OK. 
  */

typedef struct {
  char **modules;
  int num_modules;
  carmen_param_status_t status;
  double timestamp;
  char *host;
} carmen_param_response_modules_message;

#define CARMEN_PARAM_RESPONSE_MODULES_NAME  "carmen_param_respond_modules"
#define CARMEN_PARAM_RESPONSE_MODULES_FMT  "{<string:2>, int, int,double,string}"

  /** This message reports all the variable names and values currently
      associated with a particular module. This message is generally only
      emitted in response to a request for all variables. 
  */

typedef struct {
  char *module_name;
  int list_length;                        /**< variables, values and expert
					     are of this length, corresponding
					     to the number of variables of
					     this module type. */
  char **variables;
  char **values;                          /**< Note that for consistency, all
					     values are returned as strings,
					     even if they can be parsed as
					     on/off, doubles, etc. */
  int *expert;                            /**< Can be used to determine which
					     parameters are expert and which
					     are not. For each variable,
					     expert is 1 if the variable was
					     labelled as an "expert" variable
					     in the ini file, 0 otherwise. */
  carmen_param_status_t status;           /**< Has value CARMEN_PARAM_OK if
                                               the fields in this message are
                                               well-defined. All preceding
                                               fields are undefined if this
                                               field is not
                                               CARMEN_PARAM_OK. */ 
  double timestamp;
  char *host;
} carmen_param_response_all_message;

#define CARMEN_PARAM_RESPONSE_ALL_NAME     "carmen_param_respond_all"
#define CARMEN_PARAM_RESPONSE_ALL_FMT "{string, int, <string:2>, <string:2>, <int:2>, int, double, string}"

  /** This message reports the current value for a specific variable, assumed
      to be an integer. Generally emitted in response to a query. All fields
      are undefined if status is not CARMEN_PARAM_OK, for example, if the
      query did not match a variable name, or if the variable did not appear
      to be a well-formed integer. 
  */

typedef struct {
  char *module_name;                /**< The queried variable's module */
  char *variable_name;              /**< The queried variable's name */
  int value;                        /**< The queried variable's value, if it
				       can be parsed as an integer. */
  int expert;                       /**< 1 if this variable was labelled as an
				       "expert" variable, 0 otherwise. */
  carmen_param_status_t status;     /**< If status is not CARMEN_PARAM_OK, all
				       previous fields are not defined. */
  double timestamp;
  char *host;
} carmen_param_response_int_message;

#define CARMEN_PARAM_RESPONSE_INT_NAME    "carmen_param_respond_int"
#define CARMEN_PARAM_RESPONSE_INT_FMT     "{string, string, int, int, int, double, string}"

  /** This message reports the current value for a specific variable, assumed
      to be a double. Generally emitted in response to a query. All fields
      are undefined if status is not CARMEN_PARAM_OK, for example, if the
      query did not match a variable name, or if the variable did not appear
      to be a well-formed double. 
  */

typedef struct {
  char *module_name;                /**< The queried variable's module */
  char *variable_name;              /**< The queried variable's name */
  double value;                     /**< The queried variable's value, if it
				       can be parsed as a double. */
  int expert;                       /**< 1 if this variable was labelled as an
				       "expert" variable, 0 otherwise. */
  carmen_param_status_t status;     /**< If status is not CARMEN_PARAM_OK, all
				       previous fields are not defined. */
  double timestamp;
  char *host;
} carmen_param_response_double_message;

#define CARMEN_PARAM_RESPONSE_DOUBLE_NAME    "carmen_param_respond_double"
#define CARMEN_PARAM_RESPONSE_DOUBLE_FMT     "{string, string, double, int, int, double, string}"

  /** This message reports the current value for a specific variable, assumed
      to be on/off. Generally emitted in response to a query. All fields
      are undefined if status is not CARMEN_PARAM_OK, for example, if the
      query did not match a variable name, or if the variable did not appear
      to be a well-formed on/off value. 
  */

typedef struct {
  char *module_name;                /**< The queried variable's module */
  char *variable_name;              /**< The queried variable's name */
  int value;                        /**< The queried variable's value, if it
				          can be parsed as on/off. 0 if the
				          variable is off, 1 if it is on. */
  int expert;                       /**< 1 if this variable was labelled as an
				       "expert" variable, 0 otherwise. */
  carmen_param_status_t status;     /**< If status is not CARMEN_PARAM_OK, all
				       previous fields are not defined. */
  double timestamp;
  char *host;
} carmen_param_response_onoff_message;

#define CARMEN_PARAM_RESPONSE_ONOFF_NAME    "carmen_param_respond_onoff"
#define CARMEN_PARAM_RESPONSE_ONOFF_FMT     "{string, string, int, int, int, double, string}"

  /** This message reports the current value for a specific
      variable. Generally emitted in response to a query. All fields are
      undefined if status is not CARMEN_PARAM_OK, for example, if the query
      did not match a variable name. 
  */

typedef struct {
  char *module_name;                /**< The queried variable's module */
  char *variable_name;              /**< The queried variable's name */
  char *value;                      /**< The queried variable's value. */ 
  int expert;                       /**< 1 if this variable was labelled as an
				       "expert" variable, 0 otherwise. */
  carmen_param_status_t status;     /**< If status is not CARMEN_PARAM_OK, all
				       previous fields are not defined. */
  double timestamp;
  char *host;
} carmen_param_response_string_message;

#define CARMEN_PARAM_RESPONSE_STRING_NAME    "carmen_param_respond_string"
#define CARMEN_PARAM_RESPONSE_STRING_FMT     "{string, string, string, int, int, double, string}"

  /** This message sets the variable to a new value, and is generally sent to
      param_daemon using libparam_interface.a 
  */

typedef struct {
  char *module_name;                /**< The module name of the variable to
					set. */ 
  char *variable_name;              /**< The name of the variable to
					set. */ 
  char *value;                      /**< The new value to assign to the
					variable. */ 
  double timestamp;
  char *host;
} carmen_param_set_message;

#define CARMEN_PARAM_SET_NAME    "carmen_param_set"
#define CARMEN_PARAM_SET_FMT     "{string, string, string, double, string}"

typedef carmen_param_response_string_message carmen_param_variable_change_message;

#define CARMEN_PARAM_VARIABLE_CHANGE_NAME    "carmen_param_variable_change"
#define CARMEN_PARAM_VARIABLE_CHANGE_FMT     "{string, string, string, int, int, double, string}"

  /** This message reports the current version of carmen. Used to
      ensure that modules from incompatible versions of carmen are not being
      used together.   
  */

typedef struct {
  int major;
  int minor;
  int revision;
  double timestamp;
  char *host;
} carmen_param_version_message;

#define CARMEN_PARAM_VERSION_NAME "carmen_param_version"
#define CARMEN_PARAM_VERSION_FMT  "{int, int, int, double, string}"

/**
 * This message will be sent out whenever the param_daemon starts or rereads
 * the whole paramfile
 */
typedef struct {
  double timestamp;
  char* host;
} carmen_param_started_message;
#define CARMEN_PARAM_STARTED_NAME "carmen_param_started"
#define CARMEN_PARAM_STARTED_FMT  "{double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
