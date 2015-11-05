
/*
 * File: MyString.c
 * Author: Domingo Gallardo, CMU
 * Purpose: String utilities
 * 
 *
 * REVISION HISTORY
 *                     
 * $Log: MyString.c,v $
 * Revision 1.4  1996/02/10 23:36:22  rich
 * Ifdef out tests rather than comment them out.  Allows comments in the
 * test routines.
 *
 * Revision 1.3  1995/12/15  01:25:52  rich
 * Fixed the includes.
 *
 * Revision 1.2  1995/04/07  05:06:58  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:16  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.2  1995/01/25  00:04:24  rich
 * Release of tca 7.9.  Mostly speed improvements.
 * The cvs binaries may now be located in /usr/local.
 * Formatting changes.
 *
 * Revision 1.1  1994/05/31  03:26:16  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:34:43  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  02:09:22  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 * Jan 1 1993  - Domingo Gallardo at School of Computer Science, CMU
 * Added function DupUntil. Removed function CopyUntil
 *
 * Dec 26 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Added function BeginsWith.
 *
 * Dec 25 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */


#include "Standard.h"
#include "MyString.h"

BOOLEAN StringEqual(String str1, String str2)
{
  return (strcmp(str1, str2) == 0);
}


/*	Function Name: SkipSpaces
 *	Arguments:     str -- string
 *	Description:   Skips the spaces in the head of str
 *	Returns:       A pointer to the first non space character in the string
 */

String SkipSpaces(String str)
{
  while (*str++ == ' ');
  return(--str);
}

/*	Function Name: SkipAlphaNum
 *	Arguments:     str -- string
 *	Description:   Skips the alphanumeric characters in the head of str
 *	Returns:       A pointer to the first non alphanumeric character
 *                     in the string
 */

String SkipAlphaNum(String str)
{
  while (isalnum(*str++));
  return(--str);
}


/*	Function Name: SkipUntilSubString
 *	Arguments:     str        -- string
 *                     sub_string -- substring until to skip
 *	Description:   Find a substring in str and skip to the next character
 *	Returns:       A pointer to the first character after the substring, the
 *                     same string str if the substring is not found
 */

String SkipUntilSubString(String str, String sub_string)
{
  String initial_str, initial_sub_string;
  char current_char;
  
  initial_str = str;
  initial_sub_string = sub_string;
  current_char = *str++;
  while (current_char != '\0') {
    while(current_char == *sub_string++ &&
	  current_char != '\0'){
      if (*sub_string == '\0') 
	return(str);
      current_char = *str++;
    }
    sub_string = initial_sub_string;
    current_char = *str++;
  }
  return(initial_str);
}


/*	Function Name: DupUntil
 *	Arguments:     from_str   -- the string to duplicate
 *                     until_char -- char delimiter
 *	Description:   Returns the substring from the begining of from_str
 *                     to the char delimeter (without including it).
 *                     Returns the null string if until_char is not found. 
 *	Returns:       The duplicate string
 */

String DupUntil(String from_str, char until_char)
{
  String initial_string, return_string;
  
  initial_string = from_str;
  while(*from_str != until_char && *from_str != '\0') {
    from_str++;
  }
  if (*from_str == '\0') 
    return(strdup(""));
  *from_str = '\0';
  return_string = strdup(initial_string);
  *from_str = until_char;
  return(return_string);
}


/*	Function Name: BeginsWith
 *	Arguments:     str1 -- greater string
 *                     str2 -- smaller string
 *	Description:   Compares the beginnings of both string
 *	Returns:       True if str1 begins with str2
 */

BOOLEAN BeginsWith(String str1, String str2)
{
  if (*str2 == '\0')
    return(1);
  if (*str1 == '\0')
    return(0);
  while (*str1++ == *str2++) {
    if (*str2 == '\0')
      return(1);
    if (*str1 == '\0')
      return(0);
  }
  return(0);
}


/* ------------------------------------------------ */
/*                   MODULE TEST                    */
/* ------------------------------------------------ */

#ifdef TEST   
void main(int argc, char **argv)
{
  static char str1[] = "   Hola20, como estamos";
  static char str2[] = " Pepito*perez";
  String str;
  
  str = str1;
  printf("%s\n",str);
  str = SkipSpaces(str);
  printf("%s\n",str);
  str = SkipAlphaNum(str);
  printf("%s\n",str);
  str = SkipUntilSubString(str,"es");
  printf("%s\n",str);
  str = DupUntil(str2,'*');
  printf("%s\n",str);
  str = DupUntil(str2,'[');
  printf("%s\n",str);
  printf("%d %d %d %d\n", BeginsWith("pais", "pa"), BeginsWith("pais", ""),
	 BeginsWith("pais", "paisaje"), BeginsWith("pais", "ta"));
}
#endif
