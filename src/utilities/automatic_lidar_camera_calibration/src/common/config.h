#ifndef _CONFIG_H
#define _CONFIG_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _Config Config;

/* Parses a file and returns a handle to the contents.  The file should
 * already be opened and passed in as f.  filename is used merely for
 * printing error messages if the need should arise. */
Config *
config_parse_file (FILE * f, char * filename);

/* Parses the default DGC configuration file (config/master.cfg) and
 * returns a handle to it.  If the environment variable DGC_CONFIG_PATH
 * is set, that path is used instead. */
Config *
config_parse_default (void);

/**
 * Copies the filename of the default config file into buf.
 * In the future, it might also indicate that LC is to be used for the config
 * source
 *
 * Returns: 0 on success, -1 if buf isn't big enough
 */
int config_get_default_src (char *buf, int buflen);

/* Frees a configuration handle */
void
config_free (Config * conf);

/* Prints the contents of a parsed configuration file. */
int
config_print (Config * conf);

/**
 * Returns: 1 if the key is present, 0 if not
 */
int config_has_key (Config *conf, const char *key);

/* Return number of top-level keys in container named by containerKey.
 * Return -1 if container key not found.
 */
int
config_get_num_subkeys (Config * conf, const char * containerKey);

/* Fetch all top-level keys in container named by containerKey.
 *
 * Returns a newly allocated, NULL-terminated, array of strings.  This array
 * should be freed by the caller (e.g. with g_strfreev)
 */
char **
config_get_subkeys (Config * conf, const char * containerKey);
                     


/* These four functions search for a key (i.e. "sick.front.pos") and fetch
 * the value associated with that key into val, converting it to the specified
 * type.  If the conversion is not possible or the key is not found, an error
 * is returned.  If the key contains an array of multiple values, the first
 * value is used.  Return 0 on success, -1 on failure. */
int
config_get_int (Config * conf, const char * key, int * val);
int
config_get_boolean (Config * conf, const char * key, int * val);
int
config_get_double (Config * conf, const char * key, double * val);
int
config_get_str (Config * conf, const char * key, char ** val);

double config_get_double_or_fail (Config *conf, const char *key);

/* These four functions do the same thing as the previous four, except they
 * return the value associated with the key.  In case of error, the default
 * value def is returned instead. */
int
config_get_int_or_default (Config * conf, const char * key, int def);
int
config_get_boolean_or_default (Config * conf, const char * key, int def);
double
config_get_double_or_default (Config * conf, const char * key, double def);
char *
config_get_str_or_default (Config * conf, const char * key, char * def);

char *config_get_str_or_fail (Config *conf, const char *key);

/* These four functions fetch the entire array associated with a key into vals.
 * vals is an array passed in by the caller of length len elements.  The
 * number of elements actually stored in vals is returned.  Also, -1 is
 * returned on error. */
int
config_get_int_array (Config * conf, const char * key, int * vals, int len);
int
config_get_boolean_array (Config * conf, const char * key, int * vals, int len);
int
config_get_double_array (Config * conf, const char * key, double * vals, int len);
int
config_get_str_array (Config * conf, const char * key, char ** vals, int len);

/**
 * Returns: the number of elements in the specified array, or -1 if the key
 * does not correspond to an array
 */
int config_get_array_len (Config *conf, const char * key);

/* Allocates and returns a NULL-terminated array of strings. */
char **
config_get_str_array_alloc (Config * conf, const char * key);

void config_str_array_free ( char **data);
  
/* Creates a new config struct */
Config *
config_alloc( void );


/* These four functions search for a key, or create it if it does not exist.
 * They then store a single value associated with that key.
 * Return 0 on success, -1 on failure. */
int
config_set_int (Config * conf,
                const char * key,
                int val);
int
config_set_boolean (Config * conf,
                    const char * key,
                    int val);
int
config_set_double (Config * conf,
                   const char * key,
                   double val);
int
config_set_str (Config * conf,
                const char * key,
                char * val);

/* These four functions search for a key, or create it if it does not exist.
 * They store the entire input array contained in vals to that key.
 * vals is an array passed in by the caller of length len elements.  The
 * Return 0 on success, -1 on failure. */
int
config_set_int_array (Config * conf,
                      const char * key,
                      int * vals,
                      int len);
int
config_set_boolean_array (Config * conf,
                          const char * key,
                          int * vals,
                          int len);
int
config_set_double_array (Config * conf,
                         const char * key,
                         double * vals,
                         int len);
int
config_set_str_array (Config * conf,
                      const char * key,
                      char ** vals,
                      int len);


#ifdef __cplusplus
}
#endif

#endif
