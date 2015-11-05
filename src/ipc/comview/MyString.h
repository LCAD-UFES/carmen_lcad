
/*
 * File: MyString.h
 * Author: Domingo Gallardo, CMU
 * Purpose: String utilities
 * 
 *
 * REVISION HISTORY
 *                     
 * $Log: MyString.h,v $
 * Revision 1.4  1996/08/05 16:08:42  rich
 * Added comments to endifs.
 *
 * Revision 1.3  1995/12/15  01:25:53  rich
 * Fixed the includes.
 *
 * Revision 1.2  1995/04/07  05:07:00  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:18  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.2  1995/01/25  00:04:26  rich
 * Release of tca 7.9.  Mostly speed improvements.
 * The cvs binaries may now be located in /usr/local.
 * Formatting changes.
 *
 * Revision 1.1  1994/05/31  03:26:17  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.2  1993/08/13  01:43:08  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Dec 26 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Added function BeginsWith.
 *
 * Dec 25 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#ifndef _MYSTRING_H
#define _MYSTRING_H

#include <X11/Intrinsic.h>

BOOLEAN StringEqual(String str1, String str2);
String SkipSpaces(String str);
String SkipAlphaNum(String str);
String SkipUntilSubString(String str, String sub_string);
String DupUntil(String from_str, char until_char);
BOOLEAN BeginsWith(String str1, String str2);

#endif /* _MYSTRING_H */
