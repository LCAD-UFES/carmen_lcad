
#ifndef CARMEN_USER_PREFERENCES_H
#define CARMEN_USER_PREFERENCES_H

#ifdef __cplusplus
extern "C" {
#endif

#define USER_DEFAULT_FILENAME	"user_preferences.ini"

#define USER_PARAM_TYPE_INT      1
#define USER_PARAM_TYPE_DOUBLE   2
#define USER_PARAM_TYPE_ONOFF    3
#define USER_PARAM_TYPE_STRING   4
#define USER_PARAM_TYPE_FILE     5
#define USER_PARAM_TYPE_DIR      6


typedef struct
{
	const char *variable;
	char type;
	void *value;
}
	user_param_t, *user_param_p;


void user_preferences_read(const char *filename, const char *module, user_param_t *param_list, int num_items);

void user_preferences_read_commandline(int argc, char **argv, user_param_t *param_list, int num_items);

void user_preferences_save(const char *filename, const char *module, user_param_t *param_list, int num_items);


#ifdef __cplusplus
}
#endif

#endif
