#include <stdafx.h>

#ifdef STANDARD
#include <stdio.h>
#include <string.h>
#else
#include <my_global.h>
#include <my_sys.h>
#endif

#include <mysql.h>
#include <m_ctype.h>
#include <m_string.h>                // To get strmov()


//Function to return the distance between two keypoints

//Make with 'g++ -fPIC -I/usr/include/mysql -I/usr/include/libmodplug -O -pipe -o libObjRecUtil.so -shared -lmysqlclient ObjRecUtil.cc'
//Install with CREATE FUNCTION kp_dist RETURNS INTEGER SONAME "libObjRecUtil.so";


/* These must be right or mysqld will not find the symbol! */
extern "C" {
my_bool kp_dist_init( UDF_INIT* initid, UDF_ARGS* args, char* message );
void kp_dist_deinit( UDF_INIT* initid );
longlong kp_dist(UDF_INIT * initid, UDF_ARGS *args, char *is_null, char * /*error*/ );
}

my_bool kp_dist_init( UDF_INIT* initid, UDF_ARGS* args, char* message )
{
  uint i;

  if (args->arg_count != 2)
  {
    strcpy(message,"kp_dist must have two arguments");
    return 1;
  }

  initid->maybe_null=1;                /* The result may be null */
  initid->decimals=2;                /* We want 2 decimals in the result */
  initid->max_length=6;                /* 3 digits + . + 2 decimals */
  return 0;
}

void kp_dist_deinit( UDF_INIT* initid )
{
}

longlong kp_dist(UDF_INIT *initid __attribute__((unused)), UDF_ARGS *args,
                     char *is_null, char *error __attribute__((unused)))
{
  longlong dist = 0;

  if (args->args[0] == NULL || args->args[1] == NULL ||
      args->lengths[0] < 128 || args->lengths[1] < 128
     )
  {
    *is_null = 1;
    return dist;
  }

  char* kp1 = args->args[0];
  char* kp2 = args->args[1];

  for(int i=0; i<128; i++)
  {
    dist += (kp1[i]-kp2[i])*(kp1[i]-kp2[i]);
  }

  return dist;

}
