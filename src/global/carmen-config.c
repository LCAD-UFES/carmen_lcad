#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#define CARMEN_MAJOR_VERSION 0
#define CARMEN_MINOR_VERSION 7
#define CARMEN_REVISION 0

static char *home = NULL;

static void
usage(int argc __attribute__ ((unused)), char *argv[]) 
{
  fprintf(stderr, "Usage: %s [OPTIONS] [MODES]\n", argv[0]);
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "\t[--version]\n");
  fprintf(stderr, "\t[--libs]\n");
  fprintf(stderr, "\t[--cflags]\n");
  fprintf(stderr, "\t[--graphics]\n");
  fprintf(stderr, "Modes:\n");
  fprintf(stderr, "\tbasic\n");
  fprintf(stderr, "\tmotion\n");  
  fprintf(stderr, "\tnavigation\n");  
  
  exit(-1);
}

static void 
print_cflags(int mode __attribute__ ((unused)), int graphics)
{  
  printf("-I%s/include ", home);
  if (graphics)
    printf("-pthread -I/usr/include/gtk-2.0 -I/usr/lib/x86_64-linux-gnu/gtk-2.0/include -I/usr/include/atk-1.0 -I/usr/include/cairo -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/pango-1.0 -I/usr/include/gio-unix-2.0/ -I/usr/include/freetype2 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/pixman-1 -I/usr/include/libpng12 -I/usr/include/harfbuzz   ");

  printf("\n");
}

static void 
print_libs(int mode __attribute__ ((unused)), int graphics)
{
  printf("-L%s/lib ", home);

  if (mode & 1) {
    if (!graphics) 
      printf("-lglobal -lipc -lparam_interface ");
    else 
      printf("-lglobal -lipc -lglobal_graphics -lparam_interface ");
  }

  if (mode & 2) {
    printf("-lgeometry -lbase_interface -lrobot_interface ");
  }

  if (mode & 4) {
    printf("-lmap_interface -lmap_io -lmap_util -llocalize_interface "
	   "-lnavigator_interface ");
    printf("-lz ");
  }

  if (graphics) {
    printf("-lgtk-x11-2.0 -lgdk-x11-2.0 -latk-1.0 -lgio-2.0 -lpangoft2-1.0 -lpangocairo-1.0 -lgdk_pixbuf-2.0 -lcairo -lpango-1.0 -lfontconfig -lgobject-2.0 -lglib-2.0 -lfreetype   ");
    printf("-ljpeg ");
    if (mode & 4)
      printf("-lmap_graphics ");
  }
  printf("\n");
}

int main(int argc, char *argv[]) 
{
  int cflags = 0;
  int libs = 0;
  int graphics = 0;
  int index;
  int mode = 0;
  char path_buffer[1024], buffer[1024];
  char *dir;

  if (argc == 1)
    usage(argc, argv);

  if (getenv("CARMEN_HOME")) 
    home = getenv("CARMEN_HOME");
  else {
    sprintf(path_buffer, "/proc/%d/exe", getpid());
    memset(buffer, 0, 1024);
    readlink(path_buffer, buffer, 1024);
    home = buffer;

    dir = strrchr(buffer, '/');
    if (dir != NULL) {
      dir[0] = '\0';
      if (strrchr(buffer, '/'))
	dir = strrchr(buffer, '/');
      if (strlen(dir) >= 4 && strncmp(dir+1, "bin", 3) == 0)
	dir[0] = '\0';
      else if (dir - buffer >= 4 && strlen(dir) > 6 &&
	strncmp(dir-4, "/src/global", 10) == 0)
	dir[-4] = '\0';
      }
    }

  for (index = 1; index < argc; index++) {
    if (strcmp(argv[index], "--version") == 0) 
      printf("%d.%d.%d\n", CARMEN_MAJOR_VERSION, CARMEN_MINOR_VERSION,
	     CARMEN_REVISION);
    else if (strcmp(argv[index], "--cflags") == 0) 
      cflags = 1;
    else if (strcmp(argv[index], "--libs") == 0) 
      libs = 1;
    else if (strcmp(argv[index], "--graphics") == 0) {
      graphics = 1;
    }
    else if (strcmp(argv[index], "basic") == 0) 
      mode |= 1;  
    else if (strcmp(argv[index], "motion") == 0) 
      mode |= 3;  
    else if (strcmp(argv[index], "navigation") == 0) 
      mode |= 7;      
    else
      usage(argc, argv);
  }

  if (cflags)
    print_cflags(mode, graphics);
  if (libs)
    print_libs(mode, graphics);

  return 0;
}
