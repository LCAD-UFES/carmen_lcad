#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>

#include "gps.h"
#include "gps-io.h"


int
gps_parse_gga( char * line, int num_chars )
{
  char * ptr;
  int    i;
  for (i=1; i<num_chars-1; i++) {
    if (line[i]=='$' || line[i]=='*')
      return(FALSE);
  }
  if (num_chars>0 && carmen_extern_gpgga_ptr!=NULL) {

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->utc = strtod(ptr, NULL);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->latitude_dm = strtod(ptr, NULL);
    carmen_extern_gpgga_ptr->latitude = carmen_global_convert_degmin_to_double(carmen_extern_gpgga_ptr->latitude_dm);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->lat_orient = ptr[0];

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->longitude_dm = strtod(ptr, NULL);
    carmen_extern_gpgga_ptr->longitude = carmen_global_convert_degmin_to_double(carmen_extern_gpgga_ptr->longitude_dm);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->long_orient = ptr[0];

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->gps_quality = atoi(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->num_satellites = atoi(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->hdop = strtod(ptr, NULL);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->sea_level = strtod(ptr, NULL);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->altitude = strtod(ptr, NULL);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->geo_sea_level = strtod(ptr, NULL);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->geo_sep = strtod(ptr, NULL);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->data_age = atoi(ptr);

    return(TRUE);
  }
  return(FALSE);
}


int
gps_parse_hdt(char *line, int num_chars)
{
  char *ptr;
  int i;
  for (i = 1; i < num_chars-1; i++)
  {
    if (line[i] == '$' || line[i] == '*')
      return(FALSE);
  }
  if (num_chars > 0 && carmen_extern_gphdt_ptr != NULL)
  {
    ptr = strsep(&line, ",");
    if (ptr == NULL)
      return(FALSE);

    ptr = strsep(&line, ",");
    if (ptr == NULL)
      return(FALSE);

    if (strlen(ptr) > 0)
    {
      double heading = strtod(ptr, NULL);
      heading = carmen_degrees_to_radians(-90.0 - heading);
      heading = carmen_normalize_theta(heading);
      carmen_extern_gphdt_ptr->heading = heading;
      carmen_extern_gphdt_ptr->valid = 1;
    }
    else
    {
      carmen_extern_gphdt_ptr->heading = 0.0;
      carmen_extern_gphdt_ptr->valid = 0;
    }

    return(TRUE);
  }
  return(FALSE);
}


int
gps_parse_rmc(char * line, int num_chars)
{
  char * ptr;
  int    i;
  for (i=1; i<num_chars-1; i++) {
    if (line[i]=='$' || line[i]=='*')
      return(FALSE);
  }
  if (num_chars>0 && carmen_extern_gprmc_ptr!=NULL) {

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->utc = strtod(ptr, NULL);
 
    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->validity = (ptr[0] == 'A')?1:0;

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->latitude_dm = strtod(ptr, NULL);
    carmen_extern_gprmc_ptr->latitude = carmen_global_convert_degmin_to_double(carmen_extern_gprmc_ptr->latitude_dm);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->lat_orient = ptr[0];

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->longitude_dm = strtod(ptr, NULL);
    carmen_extern_gprmc_ptr->longitude = carmen_global_convert_degmin_to_double(carmen_extern_gprmc_ptr->longitude_dm);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->long_orient = ptr[0];

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->speed = carmen_knots_to_meters_per_second(strtod(ptr, NULL));

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->true_course = carmen_degrees_to_radians(atoi(ptr));

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->date = atoi(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->variation = carmen_degrees_to_radians(atoi(ptr));

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gprmc_ptr->var_dir =ptr[0];

    return(TRUE);
  }
  return(FALSE);
}

int
carmen_gps_parse_data(char *line, int num_chars)
{
  if (num_chars >= 6)
  {
    if ((strncmp("$GPGGA", line, 6) == 0) || (strncmp("$GNGGA", line, 6) == 0))
    {
      if (gps_parse_gga(line, num_chars))
    	  return (GGA_MESSAGE);
      else
    	  return (FALSE);
    }
    else if ((strncmp("$GNHDT", line, 6) == 0) || (strncmp("$GPHDT", line, 6)))
    {
      if (gps_parse_hdt(line, num_chars))
    	  return (HDT_MESSAGE);
      else
    	  return (FALSE);
    }
    else if ((strncmp("$GPRMC", line, 6) == 0) || (strncmp("$GNRMC", line, 6) == 0))
    {
      if (gps_parse_rmc(line, num_chars))
    	  return (RMC_MESSAGE);
      else
    	  return (FALSE);
    }
  }
  return(FALSE);
}

