/*****************************************************************************
 * (c) Copyright 1996 Greg Whelan and Reid Simmons.  All rights reserved.
 *
 * FILE: main.c
 *
 * ABSTRACT: Message visualization tool
 *
 ****************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "Top.h"
#include "Standard.h"
#include "List.h"
#include "Array.h"
#include "MsgData.h"
#include "TaskTree.h"
#include "MyString.h"
#include "MsgData.h"
#include "Parser.h"
#include "main.h"
#include "comview.h"

/* These variables are used externally */
int display_all=1;
int show_prompt=0;
char zoom[255];
int zoom_found=0;
char set_filename[1024];
char filename[1024];
int file_selected=0;
int COMVIEW_DEBUG=0;
char comview_directory[1024];
int screen_size=2;

static void displayVersion(void)
{
  printf("comview %d.%d.%d\n",
	 COMVIEW_VERSION_MAJOR, COMVIEW_VERSION_MINOR, COMVIEW_VERSION_MICRO);
  printf(" Released : %s\n", COMVIEW_VERSION_DATE);
  printf(" Commited : %s\n", COMVIEW_COMMIT_DATE);
  printf(" Compiled : %s %s\n", __DATE__, __TIME__);
  fflush(stdout);
}

static void printHelp(void)
{
  printf("-h : Print this message\n");
  printf("-v : Print version information\n");
  printf("-l : Do not display all log file information (only messages)\n");
  printf("-f <logfile> : Use the logfile\n");
  printf("-z <value> : initial zoom value\n");
  printf("-g : Select 800x600 geometry layout\n");
}

static void parseCommandLineOptions (int argc, char **argv)
{
  int i;

  for (i=1; i<argc; i++) {
    if (!strcmp(argv[i], "-h")) {
      printHelp();
      exit(0);
    } else if (!strcmp(argv[i], "-v")) {
      displayVersion();
      exit(0);
    } else if (!strcmp(argv[i], "-l")) {
      display_all=0;
   } else if (!strcmp(argv[i], "-d")) {
     COMVIEW_DEBUG=1;
     show_prompt=1;
   } else if (!strcmp(argv[i], "-g")) {
     screen_size = 1;
   } else if (!strcmp(argv[i], "-p")) {
     show_prompt=1;
   } else if (!strcmp(argv[i], "-z")) {
     i++;
     strcpy(zoom,*(argv+i));
     zoom_found=1;
   } else if (!strcmp(argv[i], "-s")) {
     i++;
     sprintf(set_filename, "set settings_file %s", argv[i]);
     file_selected=1;
   } else if (!strcmp(argv[i], "-f")) {
      i++;
      if (!OpenFile(argv[i])) {
	fprintf(stderr, "File %s doesn't exist\n", argv[i]);
	exit(-1);
      } else {
	sprintf(filename, "set filename %s", argv[i]);
	file_selected=1;
      }
    } else {
      fprintf(stderr, "Unknown option %s\n", argv[i]);
    }
  }
}

int main (int argc, char **argv)
{
  char *comview_env_var;
  
  comview_env_var=getenv("COMVIEW_DIRECTORY");
  sprintf(set_filename, "set settings_file \"\"");

  if (!comview_env_var) {
    /* the constant COMVIEW_DIRECTORY is set as a define in the makefile
       for the comview application. if the user doesn't have an environment
       variable overriding that setting, use that definition. */
    strcpy(comview_directory, COMVIEW_DIRECTORY); 
  } else {
    strcpy(comview_directory, comview_env_var);
  }
  
  parseCommandLineOptions(argc, argv);
  
  Comview_Initialize();
  return 0;
}
