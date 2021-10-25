
#ifndef CARGO_MESSAGES_H
#define CARGO_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
	int cargo_id;
	int cargo_type;
	carmen_point_t pose;
} carmen_cargo_database_line_t;

typedef struct
{
	int num_cargos;
	carmen_cargo_database_line_t *cargos;
	double timestamp;
	char *host;
} carmen_cargo_cargos_message;

#define	CARMEN_CARGO_CARGOS_MESSAGE_NAME	"carmen_cargo_cargos"
#define	CARMEN_CARGO_CARGOS_MESSAGE_FMT		"{int,<{int,int,{double,double,double}}:1>,double,string}"

#ifdef __cplusplus
}
#endif

#endif
