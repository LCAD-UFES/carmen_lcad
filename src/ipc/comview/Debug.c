/*
 * File: Debug.c
 * Author: Domingo Gallardo, CMU
 * Purpose: Utility to print debug messages
 * 
 *
 * REVISION HISTORY
 *
 * $Log: Debug.c,v $
 * Revision 1.2  1995/04/07 05:06:48  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:30:58  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.1  1994/05/31  03:25:48  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:34:32  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  02:09:08  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 * Dec 23 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */


#include "Debug.h"
#include <stdarg.h>

void Debug(BOOLEAN flag, const char *format, ...)
{
  va_list args;

  if (flag) {
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);
  }
}

void Error(const char *format, ...)
{
  va_list args;

  va_start(args, format);
  vfprintf(stderr, format, args);
  va_end(args);
  exit(-1);
}

void Warning(const char *format, ...)
{
  va_list args;

  va_start(args, format);
  vfprintf(stderr, format, args);
  va_end(args);
}
