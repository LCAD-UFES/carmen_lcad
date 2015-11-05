#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include <carmen/carmen.h>
#include <carmen/logtools.h>
#include <carmen/logtools_graphics.h>
#include <carmen/map_io.h>
#include "log2pic.h"

log2pic_settings_t  settings = {
  /* enum FORMAT_TYPE     format; */
  GRAPHICS,
  /* int       display_arrow; */
  FALSE,
  /* RGB       bg; */
  { 0.9, 0.9, 0.9 },
  /*  char      bgcolor[MAX_STRING_LENGTH]; */
  "#e6e6e6",
  /*  double    darken; */
  1.0,
  /*  int       showpath; */
  FALSE,
  /*  char      pathcolor[MAX_STRING_LENGTH]; */
  "red",
  /*  double    pathwidth; */
  2.0,
  /*  double    rotation_angle; */
  DEFAULT_ROTATION_ANGLE,
  /*  logtools_vector2_t   rotation_center; */
  {0.0,0.0},
  /*  char      infilename[MAX_STRING_LENGTH]; */
  "in.rec",
  /*  char      outfilename[MAX_STRING_LENGTH]; */
  "out.png",
  /*  char      filetemplate[MAX_STRING_LENGTH]; */
  "dump",
  /* double     usable_range; */
  DEFAULT_MAX_USABLE_RANGE,
  /* double     max_range; */
  DEFAULT_MAX_RANGE,
  /* double     zoom; */
  DEFAULT_ZOOM,  
  /* double     resolution_x; */
  DEFAULT_RESOLUTION,
  /* double     resolution_y; */
  DEFAULT_RESOLUTION,
  /* int        utm_correct; */
  FALSE,
  /* int        google_correct; */
  FALSE,
  /* int        google_zoom; */
  0,
  /* double     border; */
  DEFAULT_BORDER,
  /* double     unknown_val; */
  MAP_STD_VAL,
  /* int        use_odds_model; */
  FALSE,
  /* int        static_prob; */
  STATIC_PROB,
  /* int        dynamic_prob; */
  DYNAMIC_PROB,
  /* int        flip; */
  FALSE,
  /* int        animation; */
  FALSE,
  /* double     anim_step; */
  50.0,
  /* int        anim_skip; */
  0,
  /* int        laser_id; */
  DEFAULT_LASER_ID,
  /* int        endpoints; */
  FALSE,
  /* int        from; */
  -1,
  /* int        to; */
  -1,
  /* int        convolve; */
  FALSE,
  /* int        kernel_size; */
  5,
  /* int        integrate_scans; */
  TRUE,
  /* int        set_size; */
  FALSE,  
  /* int        crop_size; */
  FALSE,  
  /* double     min_x; */
  0.0,
  /* double     min_y; */
  0.0,
  /* double     max_x; */
  0.0,
  /* double     max_y; */
  0.0,
  /* int        set_pos; */
  FALSE,
  /* double     pos_x; */
  0.0,
  /* double     pos_y; */
  0.0,
  /* double     pos_o; */
  0.0,
  /* int        bgfile; */
  FALSE,
  /* IMAGE_TYPE background; */
  { 0, 0, { 0, 0 }, NULL },
  /*   int      gpspath; */
  FALSE,
  /* logtools_vector2_t  bgoffset; */
  {0.0, 0.0},
  /* logtools_rpos2_t              posstart; */
  {0.0, 0.0, 0.0}
};

void
fast_grid_line( logtools_ivector2_t start, logtools_ivector2_t end, logtools_grid_line_t *line )
{
  int dy = end.y - start.y;
  int dx = end.x - start.x;
  int stepx, stepy;
  int fraction, cnt = 0;
  
  if (dy < 0) {
    dy = -dy;
    stepy = -1;
  } else {
    stepy = 1;
  }
  if (dx < 0) {
    dx = -dx;
    stepx = -1;
  } else {
    stepx = 1;
  }
  
  dy <<= 1;
  dx <<= 1;

  line->grid[cnt++]=start;
  
  if (dx > dy) {
    fraction = dy - (dx >> 1);
    while (start.x != end.x) {
      if (fraction >= 0) {
	start.y += stepy;
	fraction -= dx;
      }
      start.x += stepx;
      fraction += dy;
      line->grid[cnt++]=start;
    }
  } else {
    fraction = dx - (dy >> 1);
    while (start.y != end.y) {
      if (fraction >= 0) {
	start.x += stepx;
	fraction -= dy;
      }
      start.y += stepy;
      fraction += dx;
      line->grid[cnt++]=start;
    }
  }
  line->numgrids = cnt;
}

void
grid_line_core( logtools_ivector2_t start, logtools_ivector2_t end, logtools_grid_line_t *line )
{
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

  int cnt = 0;

  dx = abs(end.x-start.x); dy = abs(end.y-start.y);
  
  if (dy <= dx) {
    d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x; y = end.y;
      ydirflag = (-1);
      xend = start.x;
    } else {
      x = start.x; y = start.y;
      ydirflag = 1;
      xend = end.x;
    }
    line->grid[cnt].x=x;
    line->grid[cnt].y=y;
    cnt++;
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y++; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    } else {
      while (x < xend) {
	x++;
	if (d <0) {
	  d+=incr1;
	} else {
	  y--; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    }		
  } else {
    d = 2*dx - dy;
    incr1 = 2*dx; incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y; x = end.x;
      yend = start.y;
      xdirflag = (-1);
    } else {
      y = start.y; x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    line->grid[cnt].x=x;
    line->grid[cnt].y=y;
    cnt++;
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x++; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    } else {
      while (y < yend) {
	y++;
	if (d <0) {
	  d+=incr1;
	} else {
	  x--; d+=incr2;
	}
	line->grid[cnt].x=x;
	line->grid[cnt].y=y;
	cnt++;
      }
    }
  }
  line->numgrids = cnt;
}

void
grid_line( logtools_ivector2_t start, logtools_ivector2_t end, logtools_grid_line_t *line ) {
  int i,j;
  int half;
  logtools_ivector2_t v;
  grid_line_core( start, end, line );
  if ( start.x!=line->grid[0].x ||
       start.y!=line->grid[0].y ) {
    half = line->numgrids/2;
    for (i=0,j=line->numgrids - 1;i<half; i++,j--) {
      v = line->grid[i];
      line->grid[i] = line->grid[j];
      line->grid[j] = v;
    }
  }
}
     
int
log2pic_map_pos_from_vec2( logtools_vector2_t pos,
			   logtools_grid_map2_t *map,
			   logtools_vector2_t *v )
{
  v->x = (map->center.x + (pos.x-map->offset.x)/settings.resolution_x);
  v->y = (map->center.y + (pos.y-map->offset.y)/settings.resolution_y);
  if (v->x<0) {
    return(FALSE);
  } else if (v->x>=map->mapsize.x) {
    return(FALSE);
  }
  if (v->y<0) {
    return(FALSE);
  } else if (v->y>=map->mapsize.y) {
    return(FALSE);
  }
  return(TRUE);
}

int
log2pic_map_pos_from_rpos( logtools_rpos2_t rpos, logtools_grid_map2_t *map,
			   logtools_vector2_t *v )
{
  logtools_vector2_t pos;
  pos.x = rpos.x;
  pos.y = rpos.y;
  return(log2pic_map_pos_from_vec2( pos, map, v));
}

int
log2pic_imap_pos_from_vec2( logtools_vector2_t pos,
			    logtools_grid_map2_t *map, logtools_ivector2_t *iv )
{
  logtools_vector2_t v;
  int ret = log2pic_map_pos_from_vec2( pos, map, &v );
  iv->x = (int)v.x;
  iv->y = (int)v.y;
  return(ret);
}

int
log2pic_imap_pos_from_rpos( logtools_rpos2_t rpos, logtools_grid_map2_t *map,
			    logtools_ivector2_t *iv )
{
  logtools_vector2_t v;
  int ret = log2pic_map_pos_from_rpos( rpos, map, &v );
  iv->x = (int)v.x;
  iv->y = (int)v.y;
  return(ret);
}

void
log2pic_simple_convolve_map( logtools_grid_map2_t *map, logtools_gauss_kernel_t kernel )
{
  int x, y, k, hk;
  double ksum;
  
  hk = ( kernel.len - 1 ) / 2;
  for (x=hk;x<map->mapsize.x-hk;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      ksum = 0.0;
      for (k=0;k<kernel.len;k++) {
	ksum += ( kernel.val[k] * map->mapprob[x+k-hk][y] );
      }
      map->calc[x][y] = ksum;
      if (map->calc[x][y]>1.0)
	map->calc[x][y]=1.0;
    }
  }
  for (x=0;x<map->mapsize.x;x++) {
    for (y=hk;y<map->mapsize.y-hk;y++) {
      ksum = 0.0;
      for (k=0;k<kernel.len;k++) {
	ksum += ( kernel.val[k] * map->calc[x][y+k-hk] );
      }
      map->mapprob[x][y] = ksum;
      if (map->mapprob[x][y]>0.0) {
	if (map->mapprob[x][y]>1.0)
	  map->mapprob[x][y]=1.0;
	map->mapsum[x][y]++;
      }
    }
  }
}

void
log2pic_map_integrate_scan( logtools_grid_map2_t * map, logtools_lasersens2_data_t data,
			    double max_range, double max_usable  )
{
  static int            first_time = TRUE; 
  static logtools_grid_line_t      line;
  static int            max_num_linepoints = 0;
  int                   i, j, x, y;
  logtools_ivector2_t              start, end;
  logtools_vector2_t    abspt;
  logtools_rmove2_t                nomove = {0.0, 0.0, 0.0};
  
  if (first_time) {
    max_num_linepoints =
      3 * ( max_range / map->resolution );
    line.grid =
      (logtools_ivector2_t *) malloc( max_num_linepoints *
				      sizeof(logtools_ivector2_t) );
    first_time = FALSE;
  }

  if (settings.integrate_scans) {
    for (j=0;j<data.laser.numvalues;j++) {
      if (data.laser.val[j] <= max_usable ) {
	if (settings.endpoints) {
	  if (data.laser.val[j] <= max_range ) {
	    abspt = logtools_compute_laser_points( data.estpos,
						   data.laser.val[j]+
						   (map->resolution),
						   nomove,
						   data.laser.angle[j] );
	    log2pic_imap_pos_from_vec2( abspt, map, &end );
	    map->maphit[end.x][end.y]++;
	    map->mapsum[end.x][end.y]++;
	  }
	} else {
	  if (data.laser.val[j] > max_range ) {
	    abspt = logtools_compute_laser_points( data.estpos,
						   max_range,
						   nomove,
						   data.laser.angle[j] );
	    log2pic_imap_pos_from_vec2( abspt, map, &end );
	  } else {
	    abspt = logtools_compute_laser_points( data.estpos,
						   data.laser.val[j]+
						   (map->resolution),
						   nomove,
						   data.laser.angle[j] );
	    log2pic_imap_pos_from_vec2( abspt, map, &end );
	  }
	  log2pic_imap_pos_from_rpos( data.estpos, map, &start );
	  //grid_line( start, end, &line );
	  fast_grid_line( start, end, &line );
	  for (i=0;i<line.numgrids;i++) {
	    x = line.grid[i].x;
	    y = line.grid[i].y;
	    if ( x>=0 && x<map->mapsize.x &&
		 y>=0 && y<map->mapsize.y ) {
	      if (data.laser.val[j]<=max_range ) {
		if (i>=line.numgrids-2) {
		  map->maphit[x][y]++;
		}
		map->mapsum[x][y]++;
	      } else {
		if (i<line.numgrids-1) {
		  map->mapsum[x][y]++;
		}
	      }
	    }
	  }
	}
      }
    }
  }
}
  
void
clear_map( logtools_grid_map2_t * map, logtools_rpos2_t pos )
{
  int x, y;
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
      map->calc[x][y]    = 0.0;
    }
  }
  map->offset     = pos;
}

void
map_initialize( logtools_grid_map2_t *map, int sx, int sy, int center_x, int center_y,
		double zoom, double resolution, logtools_rpos2_t start )
{
  int x, y;

  map->mapsize.x  = sx;
  map->mapsize.y  = sy;
  map->resolution = resolution;
  map->zoom       = zoom;
  map->offset     = start;

  fprintf( stderr, "# INFO: allocating memory ... " );
  map->maphit   = mdalloc( 2, sizeof(float),  sx, sy );
  map->mapsum   = mdalloc( 2, sizeof(short),  sx, sy );
  map->mapprob  = mdalloc( 2, sizeof(float), sx, sy );
  map->calc     = mdalloc( 2, sizeof(float), sx, sy );
  fprintf( stderr, "done\n" );
  map->center.x = center_x;
  map->center.y = center_y;

  fprintf( stderr, "# INFO: map:            %d %d\n",
	   map->mapsize.x, map->mapsize.y );
  fprintf( stderr, "# INFO: center:         %.1f %.1f\n",
	   map->center.x, map->center.y );
  fprintf( stderr, "# INFO: resolution:      %.2f cm\n",
	   map->resolution );
  fprintf( stderr, "# INFO: real-size:      [%.2f %.2f] [%.2f %.2f] m\n",
	   -sx*map->resolution / 100.0, sx*map->resolution / 100.0,
	   -sy*map->resolution / 100.0, sy*map->resolution / 100.0 );

  for (x=0;x<sx;x++) {
    for (y=0;y<sy;y++) {
      map->mapprob[x][y] = -1;
      map->calc[x][y]    = 0.0;
      map->maphit[x][y]  = 0.0;
      map->mapsum[x][y]  = 0;
    }
  }
}

void
printUnknown( FILE *fp, int n)
{
   while (n-- > 0)
      fprintf( fp, "-1 ");
}

void
log2pic_write_plot2d_data( logtools_log_data_t *rec )
{
  int       i, j;
  FILE    * ofp;

  if ((ofp = fopen( settings.outfilename, "w")) == 0){
    fprintf(stderr, "# ERROR: can't write data file %s\n",
	    settings.outfilename );
    return;
  }
  
  fprintf(stderr, "# INFO: write 2d data file %s\n",
	  settings.outfilename );
  for (i=0; i<rec->numlaserscans; i++) {
    if (rec->lsens[i].id==settings.laser_id) {
      for (j=0;j<rec->lsens[i].laser.numvalues;j++) {
	if (rec->lsens[i].laser.val[j]<settings.max_range) {
	  fprintf(ofp, "%f %f\n", 
		  rec->lsens[i].coord[j].abspt.x,
		  rec->lsens[i].coord[j].abspt.y );
	}
      }
    }
  }
}

void
log2pic_write_plot3d_data( logtools_grid_map2_t *map )
{
  FILE    * ofp;
  int       x, y;
  
  if ((ofp = fopen( settings.outfilename, "w")) == 0){
    fprintf(stderr, "# ERROR: can't write data file %s\n",
	    settings.outfilename );
    return;
  }

  for (x = 0; x < map->mapsize.x; x++){
    for (y = 0; y < map->mapsize.y; y++) {
      if (map->mapsum[map->mapsize.x-x-1][map->mapsize.y-y-1] == 0)
	fprintf(ofp, "%d %d 0.5\n", x, y );
      else
	fprintf(ofp, "%d %d %.3f\n", x, y,
		1.0 * map->maphit[map->mapsize.x-x-1][map->mapsize.y-y-1] /
		(double) map->mapsum[map->mapsize.x-x-1][map->mapsize.y-y-1] );
    }
    fprintf(ofp, "\n" );
  }
 
  fclose(ofp);
} 
  
double
rpos2_length( logtools_rpos2_t pos1, logtools_rpos2_t pos2 )
{
  return( sqrt( ( (pos1.x-pos2.x) * (pos1.x-pos2.x) ) +
		( (pos1.y-pos2.y) * (pos1.y-pos2.y) ) ) );
}


void
log2pic_map_compute_probs( logtools_grid_map2_t * map, double unknown_val )
{
  int     x, y, occ_cells, free_cells;
  double  odds, logodds, s_prob = 0.0, d_prob = 0.0;
  if (settings.use_odds_model) {
    s_prob  = log( settings.static_prob /(1.0-settings.static_prob) );
    d_prob  = log( settings.dynamic_prob /(1.0-settings.dynamic_prob) );
  }
  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      if (map->mapsum[x][y]>0) {
	if (settings.use_odds_model) {
	  occ_cells  = map->maphit[x][y];
	  free_cells = map->mapsum[x][y]-map->maphit[x][y];
	  logodds = ( occ_cells * s_prob +  free_cells * d_prob );
	  odds = exp(logodds);
	  map->mapprob[x][y] = (odds / (1+odds));
	} else {
	  map->mapprob[x][y] =
	    ( settings.darken * map->maphit[x][y] /
	      (double) ( map->mapsum[x][y] ) );
	  if (map->mapprob[x][y] > 0.95 )
	    map->mapprob[x][y] = 0.95;
	}
      } else {
	if (settings.format==GRAPHICS) {
	  map->mapprob[x][y] = unknown_val;
	} else {
	  if (map->mapprob[x][y] < 0) {
	    map->mapprob[x][y] = unknown_val;
	  } else {
	    map->mapsum[x][y] = 1;
	  }
	}
      }
    }
  }
}

void
log2pic_compute_map( logtools_log_data_t rec, logtools_grid_map2_t * map )
{
  logtools_gauss_kernel_t     kernel;
  int              i, idx;
  for (i=0; i<rec.numentries; i++) {
    idx = rec.entry[i].index;
    if (rec.entry[i].type==LASER_VALUES) {
      if (rec.lsens[idx].id==settings.laser_id) {
	log2pic_map_integrate_scan( map, rec.lsens[idx], settings.max_range,
				    settings.usable_range );
      }
    }
  }
  if (settings.endpoints) {
    log2pic_map_compute_probs( map, 0.0 );
  } else {
    log2pic_map_compute_probs( map, settings.unknown_val );
  }

  if (settings.convolve) {
    kernel = logtools_compute_gauss_kernel( settings.kernel_size );
    log2pic_simple_convolve_map( map, kernel );
  }
}   

void
log2pic_read_carmen_map( char * filename, logtools_grid_map2_t * map, double zoom )
{
  carmen_map_t      carmen_map;
  int               x, y;
  logtools_rpos2_t  nullpos = {0.0, 0.0, 0.0};
  char              description[MAX_STRING_LENGTH];
  char              username[MAX_STRING_LENGTH];
  char              origin[MAX_STRING_LENGTH];
  time_t            creation_time;
  
  fprintf( stderr, "# read carmen map %s ... ", filename );
  carmen_map_read_gridmap_chunk( filename, &carmen_map );
  carmen_map_read_creator_chunk( filename, &creation_time, username,
				 origin, description );
  fprintf( stderr, "done\n" );
  fprintf( stderr, "#####################################################################\n" );

  map_initialize( map, carmen_map.config.x_size,
		  carmen_map.config.y_size, 0, 0, zoom,
		  carmen_map.config.resolution*100.0, nullpos );

  for (x=0;x<map->mapsize.x;x++) {
    for (y=0;y<map->mapsize.y;y++) {
      map->mapprob[x][y] = carmen_map.map[x][y];
    }
  }
}

void
log2pic_write_bee_map( logtools_grid_map2_t *map, int clip )
{
  FILE    * ofp;
  
  int       globalSizeX = 0, globalSizeY = 0;
  int       extendedSizeX = 0, extendedSizeY = 0; 
  int       top = 0, bottom = 0;
  int       left = 0, right = 0, x, y;

  if ((ofp = fopen( settings.outfilename, "w")) == 0){
    fprintf(stderr, "# ERROR: can't write data map file %s\n",
	    settings.outfilename );
    return;
  }

  if (!clip) {
    if (map->mapsize.x < MINIMUM_MAP_SIZE)
      extendedSizeX = MINIMUM_MAP_SIZE;
    else
      extendedSizeX = ((map->mapsize.x / MAP_SIZE_STEP) + 1) * MAP_SIZE_STEP;
    
    if (map->mapsize.y < MINIMUM_MAP_SIZE)
      extendedSizeY = MINIMUM_MAP_SIZE;
    else
      extendedSizeY = ((map->mapsize.y / MAP_SIZE_STEP) + 1) * MAP_SIZE_STEP;
    
    top         = (extendedSizeY - map->mapsize.y) / 2;
    bottom      = extendedSizeY - top - map->mapsize.y;
    left        = (extendedSizeX - map->mapsize.x) / 2;
    right       = extendedSizeX - left - map->mapsize.x;
    
    globalSizeX = extendedSizeX * map->resolution;
    globalSizeY = extendedSizeY * map->resolution;
  
    fprintf( ofp, "robot_specifications->global_mapsize_x  %d\n", globalSizeY);
    fprintf( ofp, "robot_specifications->global_mapsize_y  %d\n", globalSizeX);
    fprintf( ofp, "robot_specifications->resolution %d\n", (int) map->resolution);
    
    fprintf( ofp, "global_map[0]: %d %d\n", extendedSizeY, extendedSizeX);
    
    for (x = 0; x < left; x++){
      printUnknown(ofp, extendedSizeY);
      fprintf( ofp, "\n");
    }
  } else {

    fprintf( ofp, "robot_specifications->global_mapsize_x  %d\n",
	     map->mapsize.y );
    fprintf( ofp, "robot_specifications->global_mapsize_y  %d\n",
	     map->mapsize.x) ;
    fprintf( ofp, "robot_specifications->resolution %d\n", (int) map->resolution);
    fprintf( ofp, "global_map[0]: %d %d\n", map->mapsize.y, map->mapsize.x );
  }

  for (x = 0; x < map->mapsize.x; x++){
    if (!clip)
      printUnknown( ofp, top);
    for (y = 0; y < map->mapsize.y; y++)
      if (map->mapsum[map->mapsize.x-x-1][map->mapsize.y-y-1] == 0)
	fprintf(ofp, "-1.000 ");
      else
	fprintf(ofp, "%.3f ",
		1.0-( map->maphit[map->mapsize.x-x-1][map->mapsize.y-y-1] /
		      (double) map->mapsum[map->mapsize.x-x-1][map->mapsize.y-y-1] ) );
    if (!clip)
      printUnknown( ofp, bottom);
    fprintf(ofp, "\n");
  }
  
  if (!clip) {
    for (x = 0; x < right; x++){
      printUnknown( ofp, extendedSizeY);
      fprintf( ofp, "\n");
    }
  }
  
  fclose(ofp);
} 
  
void
log2pic_write_carmen_map( logtools_grid_map2_t * map )
{
  char                creator[MAX_STRING_LENGTH];
  char                comment[MAX_STRING_LENGTH];
  carmen_FILE*   fp;

  fp = carmen_fopen( settings.outfilename, "w");
  if(fp == NULL) {
    fprintf( stderr, "# Error: Could not open file %s for writing.",
	     settings.outfilename);
    exit(1);
  }
  fprintf( stderr, "# INFO: write carmen map file %s\n",
	   settings.outfilename);
  snprintf( creator, MAX_STRING_LENGTH,
	    "CARMEN map file converted from %s",
	    settings.infilename );
  if (fabs(settings.rotation_angle)>MIN_ROTATION) {
    snprintf( comment, MAX_STRING_LENGTH,
	      "[offset={%.4f,%.4f};rotation={%.4f,%.4f,%.4f}]",
	      (map->offset.x - map->center.x * map->resolution)/100.0,
	      (map->offset.y - map->center.y * map->resolution)/100.0,
	      settings.rotation_angle,
	      settings.rotation_center.x/100.0,
	      settings.rotation_center.y/100.0 );
  } else {
    snprintf( comment, MAX_STRING_LENGTH,
	      "[offset={%.4f,%.4f}]",
	      (map->offset.x - map->center.x * map->resolution)/100.0,
	      (map->offset.y - map->center.y * map->resolution)/100.0 );
  }
  carmen_map_write_all( 
//			carmen_FILE* fp
                        fp, 
//			float **prob
			map->mapprob,
//			int size_x
			map->mapsize.x,
//			int size_x
			map->mapsize.y,
//			double resolution
			map->resolution/100.0, 
//			char *comment_origin, 
			"",
//                      char *comment_description,
			"",
//                      char *creator_origin
			creator, 
//                      char *creator_description
			comment,
//                      carmen_place_p places
			NULL,
//                      int num_places
			0,
//                      carmen_offlimits_p offlimits_list
			NULL,
//                      int offlimits_num_items
			0,
//                      carmen_laser_scan_p scan_list,
			NULL,
//                      int num_scans
			0 );
  carmen_fclose(fp);
}

void
log2pic_filetemplate_from_filename( char * filetemplate, char * filename )
{
  char    template[MAX_STRING_LENGTH];
  char  * ptr;

  strncpy( template, filename, MAX_STRING_LENGTH );
  ptr = rindex( template, '.' );
  if (ptr!=NULL) {
    *ptr = 0;
  }
  strncpy( filetemplate, template, MAX_STRING_LENGTH );
}

char *
log2pic_dump_filename( void )
{
  static char filename[MAX_STRING_LENGTH];
  static int ctr = 0;
  snprintf( filename, MAX_STRING_LENGTH, "%s-%s%s%s%s%d.png",
	    settings.filetemplate,
	    ctr<10000?"0":"",
	    ctr<1000?"0":"",
	    ctr<100?"0":"",
	    ctr<10?"0":"",
	    ctr );
  ctr++;
  return(filename);
}

void
print_usage( void )
{
  fprintf(stderr,
	  "\nusage: log2pic [options] <LOG-FILE> <PIC-FILE>\n"
	  "  -anim-step <STEP-SIZE>:    min distance between scans (in m)\n"
	  "  -anim-skip <NUM>:          skip NUM animation dumps (0: no skip\n"
	  "  -animation:                write sep. pics for animation\n"
	  "  -background <FILE>:        use background file\n"
	  "  -bee-map:                  save in bee-map format\n"
	  "  -bg-offset <X><Y>:         shift by <X>/<Y> pixels\n"
	  "  -bg-color <COLOR>:         background color\n"
	  "  -bg-map <FILE>:            read carmen-map as background image\n"
	  "  -border <BORDER>:          add a border (in m)\n"
 	  "  -carmen-map:               save in carmen-map format\n"
	  "  -convolve:                 convolve map with gaussian kerne;\n"
	  "  -crop <X><Y><X><Y>:        crop part of the map (min,max):\n"
	  "  -darken <FACTOR>:          darken the occ. cells\n"
	  "  -display-arrow:            display arrow in marking\n"
	  "  -endpoints:                use endpoints instead of beams\n"
	  "  -free-prob:                probability for free observation\n"
	  "  -from <NUM>:               start animation with scan NUM\n"
	  "  -gps-path:                 draw gps points\n"
	  "  -id <ID>:                  set laser number\n"
	  "  -kernel-size <NUM>:        size of the gaussian kernel (>0 and odd)\n"
	  "  -maxrange <MAX-RANGE>:     max range for building maps\n"
	  "  -no-scans:                 don't integrate the scans\n"
	  "  -odds-model:               use odds-model to compute probs\n"
	  "  -pathcolor <COLORNAME>:    color of the robot path\n"
	  "  -pathwidth <WIDTH>:        width of the robot path\n"
	  "  -start-pose <X><Y><O>:     start pose of the robot (in m, m, deg)\n"
	  "  -plot2d:                   save as 2d data file\n"
	  "  -plot3d:                   save map as 3d data file\n"
	  "  -pos-start <X><Y>:         pos of lower left bg-corner\n"
	  "  -rear-laser:               use rear laser instead of front laser\n"
	  "  -res   <RES> <RES>:        resolution of the map\n"
	  "  -res-x <RES>:              resolution in x direction\n"
	  "  -res-y <RES>:              resolution in y direction\n"
	  "  -rotate <ANGLE>:           rotate the map by ANGLE degree\n"
	  "  -showpath:                 show robot path\n"
	  "  -size:                     set the size of the output image\n"
	  "  -static-prob:              probability for static observation\n"
	  "  -to <NUM>:                 end animation with scan NUM\n"
	  "  -usablerange <MAX-RANGE>:  max range for detecting corrupted beams\n"
	  "  -utm-correct:              corrects gps positions for UTM tiles\n"
	  "  -zoom <ZOOM>:              scale factor for the map (must be >=1.0)\n" );
}

int
main( int argc, char** argv)
{
  logtools_grid_map2_t           map;
  char                           mapfilename[MAX_STRING_LENGTH];
  char                           bgfilename[MAX_STRING_LENGTH];
  logtools_log_data_t            rec;
  int                            i, j, idx;
  logtools_bounding_box_t        bbox;
  logtools_vector2_t             size;
  logtools_ivector2_t            isize = {0,0}, istart = {0,0};
  int                            readmap = FALSE;
  int                            readbg = FALSE;
  int                            numctr = 0;
  logtools_rpos2_t               npos = {0.0, 0.0, 0.0};
  logtools_rpos2_t               rpos = {0.0, 0.0, 0.0};
  logtools_rmove2_t              move;
  double                         res;
  
  if (argc<3) {
    print_usage();
    exit(0);
  }
  
  for (i=1; i<argc-2; i++) {
    if (!strcmp(argv[i],"-showpath")) {
      settings.showpath = TRUE;
    } else if (!strcmp(argv[i],"-carmen-map")) {
      settings.format = CARMEN_MAP; 
    } else if (!strcmp(argv[i],"-bee-map")) {
      settings.format = BEE_MAP; 
    } else if (!strcmp(argv[i],"-res") && (argc>i+2)) {
      settings.resolution_x = 100.0 * atof(argv[++i]); 
      settings.resolution_y = settings.resolution_x;
    } else if (!strcmp(argv[i],"-res-x") && (argc>i+2)) {
      settings.resolution_x = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-res-y") && (argc>i+2)) {
      settings.resolution_y = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-zoom") && (argc>i+2)) {
      settings.zoom = atof(argv[++i]);
    } else if (!strcmp(argv[i],"-pathcolor") && (argc>i+2)) {
      strncpy( settings.pathcolor, argv[++i], MAX_STRING_LENGTH );
    } else if (!strcmp(argv[i],"-pathwidth") && (argc>i+2)) {
      settings.pathwidth = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-border") && (argc>i+1)) {
      settings.border = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-bg-color") && (argc>i+1)) {
      strncpy( settings.bgcolor, argv[++i], MAX_STRING_LENGTH );
    } else if (!strcmp(argv[i],"-bg-map") && (argc>i+1)) {
      strncpy( mapfilename, argv[++i], MAX_STRING_LENGTH );
      readmap = TRUE;
    } else if (!strcmp(argv[i],"-id") && (argc>i+1)) {
      settings.laser_id = atoi(argv[++i]);
    } else if (!strcmp(argv[i],"-rear-laser")) {
      settings.flip = TRUE;
    } else if (!strcmp(argv[i],"-plot2d")) {
      settings.format = PLOT2D;  
    } else if (!strcmp(argv[i],"-plot3d")) {
      settings.format = PLOT3D;  
    } else if (!strcmp(argv[i],"-gps-path")) {
      settings.gpspath = TRUE;
    } else if (!strcmp(argv[i],"-utm-correct")) {
      settings.utm_correct = TRUE;
    } else if (!strcmp(argv[i],"-google-correct") && (argc>i+2)) {
      settings.google_correct = TRUE;  
      settings.google_zoom = atoi( argv[++i] );
    } else if (!strcmp(argv[i],"-background") && (argc>i+1)) {
      strncpy( bgfilename, argv[++i], MAX_STRING_LENGTH );
      readbg = TRUE;
    } else if (!strcmp(argv[i],"-bg-offset") && (argc>i+2)) {
      settings.bgoffset.x = atof( argv[++i] );
      settings.bgoffset.y = atof( argv[++i] );
    } else if (!strcmp(argv[i],"-pos-start") && (argc>i+2)) {
      settings.posstart.x = 100.0 * atof( argv[++i] );
      settings.posstart.y = 100.0 * atof( argv[++i] );
    } else if (!strcmp(argv[i],"-maxrange") && (argc>i+1)) {
      settings.max_range = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-rotate") && (argc>i+1)) {
      settings.rotation_angle = deg2rad(atof(argv[++i]));
    } else if (!strcmp(argv[i],"-display-arrow")) {
      settings.display_arrow = TRUE;
    } else if (!strcmp(argv[i],"-animation")) {
      settings.animation = TRUE;
    } else if (!strcmp(argv[i],"-anim-step") && (argc>i+1)) {
      settings.anim_step = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-anim-skip") && (argc>i+1)) {
      settings.anim_skip = atoi(argv[++i]);
    } else if (!strcmp(argv[i],"-usablerange") && (argc>i+1)) {
      settings.usable_range = 100.0 * atof(argv[++i]);
    } else if (!strcmp(argv[i],"-darken") && (argc>i+1)) {
      settings.darken = atof(argv[++i]);
    } else if (!strcmp(argv[i],"-endpoints")) {
      settings.endpoints = TRUE;
    } else if (!strcmp(argv[i],"-convolve")) {
      settings.convolve = TRUE;
    } else if (!strcmp(argv[i],"-kernel-size") && (argc>i+1)) {
      settings.kernel_size = atoi(argv[++i]);
      if (settings.kernel_size<1 || settings.kernel_size%2 != 1) {
	print_usage();
	exit(1);
      }
    } else if (!strcmp(argv[i],"-odds-model")) {
      settings.use_odds_model = TRUE;
    } else if (!strcmp(argv[i],"-static-prob") && (argc>i+1)) {
      settings.static_prob = atof(argv[++i]);
    } else if (!strcmp(argv[i],"-free-prob") && (argc>i+1)) {
      settings.dynamic_prob = atof(argv[++i]);
    } else if (!strcmp(argv[i],"-from") && (argc>i+1)) {
      settings.from = atoi(argv[++i]);
    } else if (!strcmp(argv[i],"-to") && (argc>i+1)) {
      settings.to = atoi(argv[++i]);
    } else if (!strcmp(argv[i],"-no-scans")) {
      settings.integrate_scans = FALSE;
    } else if (!strcmp(argv[i],"-crop") && (argc>i+4)) {
      settings.min_x = atof(argv[++i]);
      settings.min_y = atof(argv[++i]);
      settings.max_x = atof(argv[++i]);
      settings.max_y = atof(argv[++i]);
      settings.crop_size = TRUE;
    } else if (!strcmp(argv[i],"-size") && (argc>i+2)) {
      isize.x = atoi(argv[++i]);
      isize.y = atoi(argv[++i]);
      settings.set_size = TRUE;
    } else if (!strcmp(argv[i],"-pos") && (argc>i+3)) {
      settings.pos_x = 100.0 * atof(argv[++i]);
      settings.pos_y = 100.0 * atof(argv[++i]);
      settings.pos_o = deg2rad(atof(argv[++i]));
    } else {
      print_usage();
      exit(1);
    }
  }
  
  if (settings.zoom<1.0) {
    print_usage();
    exit(1);
  }

  fprintf( stderr, "\n");

  if (settings.use_odds_model) {
    if (settings.dynamic_prob>0.5) {
      fprintf( stderr, "# WARNING: dynamic-prob should not be > 0.5\n" );
    }
    if (settings.static_prob<0.5) {
      fprintf( stderr, "# WARNING: static-prob should not be < 0.5\n" );
    }
  }

  /** Warnings to avoid m/cm convertion errors  **********************************************/

  if (settings.max_range < 100.0)
    carmen_warn("# WARNUNG: max_range is comparably small (%f m). Is this value correct?\n",
		0.01 * settings.max_range);

  if (settings.usable_range < 100.0)
    carmen_warn("# WARNUNG: usable_range is comparably small (%f m). Is this value correct?\n",
		0.01 * settings.usable_range);

  if (settings.resolution_x < 1.0)
    carmen_warn("# WARNUNG: resolution_x is comparably small (%f m). Is this value correct?\n",
		0.01 * settings.resolution_x);

  if (settings.resolution_y < 1.0)
    carmen_warn("# WARNUNG: resolution_y is comparably small (%f m). Is this value correct?\n",
		0.01 * settings.resolution_y);

  if (settings.max_range >= 100000.0)
    carmen_warn("# WARNUNG: max_range is comparably big (%.2f m). Is this value correct?\n",
		0.01 * settings.max_range);

  if (settings.usable_range >= 100000.0)
    carmen_warn("# WARNUNG: usable_range is comparably big (%.2f m). Is this value correct?\n",
		0.01 * settings.usable_range);

  if (settings.resolution_x >= 1000.0)
    carmen_warn("# WARNUNG: resolution_x is comparably big (%.2f m). Is this value correct?\n",
		0.01 * settings.resolution_x);

  if (settings.resolution_y >= 1000.0)
    carmen_warn("# WARNUNG: resolution_y is comparably big (%.2f m). Is this value correct?\n",
		0.01 * settings.resolution_y);

  /*******************************************************************************************/
  
  strncpy( settings.infilename, argv[argc-2], MAX_STRING_LENGTH );
  strncpy( settings.outfilename, argv[argc-1], MAX_STRING_LENGTH );

  if (settings.format==CARMEN_MAP || settings.format==BEE_MAP) {
    settings.unknown_val = CARMEN_MAP_STD_VAL;
  }
  
  fprintf( stderr, "#####################################################################\n" );
  fprintf( stderr, "#              READ FILE\n" );
  fprintf( stderr, "#\n" );
  
  if (!logtools_read_logfile( &rec, settings.infilename ))
      exit(1);

  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );

  if (fabs(settings.rotation_angle)>MIN_ROTATION) {
    if (rec.info.system==CARMEN) {
      if (rec.numlaserscans>=1) {
	if (settings.set_pos) {
	  rpos.x = settings.pos_x;
	  rpos.y = settings.pos_y;
	  rpos.o = settings.pos_o;
	} else {
	  rpos = rec.lsens[0].estpos;
	}
	settings.rotation_center.x = rpos.x;
	settings.rotation_center.y = rpos.y;
        rec.lsens[0].estpos.o += settings.rotation_angle;
	for (i=1; i<rec.numlaserscans; i++) {
	  move = logtools_movement2_between_rpos2( rpos, rec.lsens[i].estpos );
	  rpos = rec.lsens[i].estpos;
	  rec.lsens[i].estpos = 
	    logtools_rpos2_with_movement2( rec.lsens[i-1].estpos, move );
	}
      }
    } else {
       if (rec.numpositions>=1) {
	 if (settings.set_pos) {
	   rpos.x = settings.pos_x;
	   rpos.y = settings.pos_y;
	   rpos.o = settings.pos_o;
	 } else {
	   rpos = rec.psens[0].rpos;
	 }
	 settings.rotation_center.x = rpos.x;
	 settings.rotation_center.y = rpos.y;
         rec.psens[0].rpos.o += settings.rotation_angle;
         npos = rec.psens[0].rpos;
	 for (i=0; i<rec.numentries; i++) {
	   idx = rec.entry[i].index;
	   if (rec.entry[i].type==POSITION) {
	     if (idx>0) {
	       move = logtools_movement2_between_rpos2( rpos,
						       rec.psens[idx].rpos );
	       rpos = rec.psens[idx].rpos;
	       rec.psens[idx].rpos = 
		 logtools_rpos2_with_movement2( rec.psens[idx-1].rpos, move );
	       npos = rec.psens[idx].rpos;
	     }
	   } else if (rec.entry[i].type==LASER_VALUES) {
	     rec.lsens[idx].estpos = npos;
	   }
	 }
      }
    }
  }

  if (settings.flip) {
    for (i=0; i<rec.numlaserscans; i++) {
      for (j=0;j<rec.lsens[i].laser.numvalues;j++) {
	rec.lsens[i].laser.angle[j] += M_PI;
      }
    }
  }

  /* compute abs values */
  logtools_compute_coordpts( &rec );

  if (settings.crop_size) {
    bbox.min.x = settings.min_x;
    bbox.min.y = settings.min_y;
    bbox.max.x = settings.max_x;
    bbox.max.y = settings.max_y;
  } else {
    bbox.min.x = MAXFLOAT;    bbox.min.y = MAXFLOAT;
    bbox.max.x = -MAXFLOAT;   bbox.max.y = -MAXFLOAT;
    /* compute bounding box */
    if (settings.gpspath) {
      for (j=0;j<rec.numgps;j++) {
	if (fabs(rec.gps[j].latitude) > 0.1 &&
	    fabs(rec.gps[j].longitude) > 0.1) {
	  if (rec.gps[j].latitude < bbox.min.x) {
	    bbox.min.x = rec.gps[j].latitude;
	  }
	  if (rec.gps[j].longitude < bbox.min.y) {
	    bbox.min.y = rec.gps[j].longitude;
	  }
	  if (rec.gps[j].latitude > bbox.max.x) {
	    bbox.max.x = rec.gps[j].latitude;
	  }
	  if (rec.gps[j].longitude > bbox.max.y) {
	    bbox.max.y = rec.gps[j].longitude;
	  }
	}
      }
      fprintf( stderr, "#              GPS DATA \n" );
      fprintf( stderr, "# INFO: min x / max x    = %.8f deg / %.8f deg\n",
	       bbox.min.x, bbox.max.x );
      fprintf( stderr, "# INFO: min y / max y    = %.8f deg / %.8f deg\n",
	       bbox.min.y, bbox.max.y );
      fprintf( stderr, "#####################################################################\n" );
    } else {
      for (i=0; i<rec.numlaserscans; i++) {
	if (rec.lsens[i].id==settings.laser_id) {
	  numctr++;
	  for (j=0;j<rec.lsens[i].laser.numvalues;j++) {
	    if (rec.lsens[i].laser.val[j]<settings.max_range) {
	      if (rec.lsens[i].coord[j].abspt.x < bbox.min.x) {
		bbox.min.x = rec.lsens[i].coord[j].abspt.x;
	      }
	      if (rec.lsens[i].coord[j].abspt.x > bbox.max.x) {
		bbox.max.x = rec.lsens[i].coord[j].abspt.x;
	      }
	      if (rec.lsens[i].coord[j].abspt.y < bbox.min.y) {
		bbox.min.y = rec.lsens[i].coord[j].abspt.y;
	      }
	      if (rec.lsens[i].coord[j].abspt.y > bbox.max.y) {
		bbox.max.y = rec.lsens[i].coord[j].abspt.y;
	      }
	    }
	  }
	}
      }
    }
  }

  if (readmap) {
    
    log2pic_read_carmen_map( mapfilename, &map, settings.zoom );
    if ( map.center.x < bbox.min.x ) {
      bbox.min.x = map.center.x;
    }
    if ( map.center.x + ( map.mapsize.x * map.resolution ) > bbox.max.x ) {
      bbox.max.x = map.center.x + ( map.mapsize.x * map.resolution );
    }
    if ( map.center.y < bbox.min.y ) {
      bbox.min.y = map.center.y;
    }
    if ( map.center.y + ( map.mapsize.y * map.resolution ) > bbox.max.y ) {
      bbox.max.y = map.center.y + ( map.mapsize.y * map.resolution );
    }

  } else if (readbg) {

    settings.bgfile = TRUE;
    fprintf( stderr, "#\n" );
    fprintf( stderr, "# INFO: read background image %s ... \n", bgfilename );
    log2pic_read_image_file( bgfilename, &(settings.background) );
    fprintf( stderr, "#\n" );
    fprintf( stderr, "# INFO: size of image %d x %d\n",
	     settings.background.width,
	     settings.background.height );
    fprintf( stderr, "# INFO: resolution %f x %f\n",
	     settings.resolution_x,
	     settings.resolution_y );
    bbox.min.x = settings.background.start.x =
      -settings.bgoffset.x * settings.resolution_x;
    bbox.max.x = settings.background.start.x +
      ( settings.background.width * settings.resolution_x );
    bbox.min.y = settings.background.start.y = 
      -settings.bgoffset.y * settings.resolution_y;
    bbox.max.y = settings.background.start.y +
      ( settings.background.height * settings.resolution_y );

  } else if (settings.set_size) {
    
    bbox.min.x = settings.background.start.x =
      -settings.bgoffset.x * settings.resolution_x;
    bbox.max.x = settings.background.start.x +
      ( settings.background.width * settings.resolution_x );
    bbox.min.y = settings.background.start.y = 
      -settings.bgoffset.y * settings.resolution_y;
    bbox.max.y = settings.background.start.y +
      ( settings.background.height * settings.resolution_y );

  } else {
    
    if (numctr==0 && !(settings.crop_size)) {
      /* no laser and no map read */
      if (!settings.showpath || settings.format!=GRAPHICS) {
	fprintf( stderr, "# ERROR: found %d laser scans!\n\n", numctr );
	exit(1);
      } else {
	if (rec.info.system==CARMEN) {
	  for (i=0; i<rec.numlaserscans; i++) {
	    if (rec.lsens[i].estpos.x < bbox.min.x) {
	      bbox.min.x = rec.lsens[i].estpos.x;
	    }
	    if (rec.lsens[i].estpos.y < bbox.min.y) {
	      bbox.min.y = rec.lsens[i].estpos.y;
	    }
	    if (rec.lsens[i].estpos.x > bbox.max.x) {
	      bbox.max.x = rec.lsens[i].estpos.x;
	    }
	    if (rec.lsens[i].estpos.y > bbox.max.y) {
	      bbox.max.y = rec.lsens[i].estpos.y;
	    }
	  }
	} else if (rec.info.system==REC) {
	  for (i=0; i<rec.numpositions; i++) {
	    if (rec.psens[i].rpos.x < bbox.min.x) {
	      bbox.min.x = rec.psens[i].rpos.x;
	    }
	    if (rec.psens[i].rpos.y < bbox.min.y) {
	      bbox.min.y = rec.psens[i].rpos.y;
	    }
	    if (rec.psens[i].rpos.x > bbox.max.x) {
	      bbox.max.x = rec.psens[i].rpos.x;
	    }
	    if (rec.psens[i].rpos.y > bbox.max.y) {
	      bbox.max.y = rec.psens[i].rpos.y;
	    }
	  }
	}
      }
    } else {
      fprintf( stderr, "# INFO: using %d laser scans\n", numctr );
    }
    
  }

  if (!settings.set_size) {
    size.x = bbox.max.x-bbox.min.x+2*settings.border;
    size.y = bbox.max.y-bbox.min.y+2*settings.border;
    isize.x = (int) (size.x/settings.resolution_x);
    isize.y = (int) (size.y/settings.resolution_y);
  } else {
    size.x = isize.x * settings.resolution_x;
    size.y = isize.y * settings.resolution_y;
    settings.background.width = isize.x;
    settings.background.height = isize.y;
  }
    
  

  istart.x = (int) ((bbox.min.x-settings.border)/settings.resolution_x);
  istart.y = (int) ((bbox.min.y-settings.border)/settings.resolution_y);
  
  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );
  fprintf( stderr, "#              DATA FILE\n" );
  fprintf( stderr, "#\n" );
  fprintf( stderr, "# INFO: min x / max x    = %.2f m / %.2f m\n",
	   bbox.min.x / 100.0, bbox.max.x / 100.0 );
  fprintf( stderr, "# INFO: min y / max y    = %.2f m / %.2f m\n",
	   bbox.min.y / 100.0, bbox.max.y / 100.0 );
  fprintf( stderr, "# INFO: size x / y       = %.2f m x %.2f m\n",
	   size.x / 100.0, size.y / 100.0 );
  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );
  fprintf( stderr, "#              MAP SIZE\n" );
  fprintf( stderr, "#\n" );
  fprintf( stderr, "# INFO: resolution-x       = %.1f cm per pixel\n",
	   settings.resolution_x );
  fprintf( stderr, "# INFO: resolution-y       = %.1f cm per pixel\n",
	   settings.resolution_y );
  fprintf( stderr, "# INFO: size x / y       = %d pixel x %d pixel\n",
	   (int) (settings.zoom * isize.x), (int) (settings.zoom * isize.y) );
  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );
  fprintf( stderr, "#\n" );

  res = (settings.resolution_x+settings.resolution_y)/2.0;
  map_initialize( &map, isize.x, isize.y, -istart.x, -istart.y, settings.zoom, 
		  res, settings.posstart );
  
  fprintf( stderr, "#\n" );
  fprintf( stderr, "#####################################################################\n" );
  
  switch (settings.format) {
  case PLOT2D:
    log2pic_write_plot2d_data( &rec );
    break;
  case PLOT3D:
    log2pic_compute_map( rec, &map );
    log2pic_write_plot3d_data( &map );
    break;
  case GRAPHICS:
    log2pic_write_image_magick_map( &map, &rec );
    break;
  case CARMEN_MAP:
    log2pic_compute_map( rec, &map );
    log2pic_write_carmen_map( &map );
    break;
  case BEE_MAP:
    log2pic_compute_map( rec, &map );
    log2pic_write_bee_map( &map, 1 );
    break;
  }
  
  fprintf( stderr, "#####################################################################\n" );
  exit(0);

}
  
