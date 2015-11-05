#define MAX_LINE_LENGTH          524288
#define MAX_CMD_LENGTH             2024
#define MAX_STRING_LENGTH           256
#define DEFAULT_MAX_RANGE           800.0
#define DEFAULT_MAX_USABLE_RANGE   8090.0
#define DEFAULT_ZOOM                  1.0
#define DEFAULT_RESOLUTION           10.0
#define DEFAULT_LASER_ID              0
#define DEFAULT_BORDER              100
#define MAP_STD_VAL                   0.1
#define CARMEN_MAP_STD_VAL           -1
#define MIN_ROTATION                  0.00872638    /* .5 degrees */
#define DEFAULT_ROTATION_ANGLE        0.0
#define EPSILON                       0.0001
#define ROBOT_SIZE                   50
#define ROBOT_FILL_COLOR        "yellow"
#define ROBOT_BORDER_COLOR       "black"
#define MINIMUM_MAP_SIZE             300
#define MAP_SIZE_STEP                50
#define DYNAMIC_PROB                 0.2
#define STATIC_PROB                  0.8

#ifndef MAXFLOAT
#define MAXFLOAT      3.40282347e+38F
#endif

typedef struct {
  int                  width;
  int                  height;
  logtools_vector2_t   start;
  RGB     ** pixel;
} log2pic_background_image_t;

typedef struct {
  double   * pixel;
} log2pic_image_t;

enum log2pic_format_t {
  GRAPHICS,
  PLOT2D,
  PLOT3D,
  CARMEN_MAP,
  BEE_MAP
};
  
typedef struct {
  enum log2pic_format_t         format;
  int                           display_arrow;
  RGB                           bg;
  char                          bgcolor[MAX_STRING_LENGTH];
  double                        darken;
  int                           showpath;
  char                          pathcolor[MAX_STRING_LENGTH];
  double                        pathwidth;
  double                        rotation_angle;
  logtools_vector2_t            rotation_center;
  char                          infilename[MAX_STRING_LENGTH];
  char                          outfilename[MAX_STRING_LENGTH];
  char                          filetemplate[MAX_STRING_LENGTH];
  double                        usable_range;
  double                        max_range;
  double                        zoom;
  double                        resolution_x;
  double                        resolution_y;
  int                           utm_correct;
  int                           google_correct;
  int                           google_zoom;
  double                        border;
  double                        unknown_val;
  int                           use_odds_model;
  double                        static_prob;
  double                        dynamic_prob;
  int                           flip;
  int                           animation;
  double                        anim_step;
  int                           anim_skip;
  int                           laser_id;
  int                           endpoints;
  int                           from;
  int                           to;
  int                           convolve;
  int                           kernel_size;
  int                           integrate_scans;
  int                           set_size;
  int                           crop_size;
  double                        min_x;
  double                        min_y;
  double                        max_x;
  double                        max_y;
  int                           set_pos;
  double                        pos_x;
  double                        pos_y;
  double                        pos_o;
  int                           bgfile;
  log2pic_background_image_t    background;
  int                           gpspath;
  logtools_vector2_t            bgoffset;
  logtools_rpos2_t                         posstart;
} log2pic_settings_t;

double round(double x);

extern log2pic_settings_t settings;

void
log2pic_map_integrate_scan( logtools_grid_map2_t *map, logtools_lasersens2_data_t data,
			    double max_range, double max_usable  );

int
log2pic_map_pos_from_vec2( logtools_vector2_t pos, logtools_grid_map2_t *map,
			   logtools_vector2_t *v );

void
log2pic_map_compute_probs( logtools_grid_map2_t * map, double unknown_val );

void
log2pic_filetemplate_from_filename( char * filetemplate, char * filename );

char *
log2pic_dump_filename( void );

void
log2pic_write_image_magick_map( logtools_grid_map2_t *map, logtools_log_data_t *rec );

void
log2pic_simple_convolve_map( logtools_grid_map2_t *map, logtools_gauss_kernel_t kernel );

void
log2pic_read_image_file( char * filename, log2pic_background_image_t * img );

