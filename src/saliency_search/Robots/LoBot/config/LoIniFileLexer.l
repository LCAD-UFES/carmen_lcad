/* -*- mode: C;-*- */
%{
/*
   LoIniFileLexer.l -- lex source for Robolocust/Lobot config files

   This file implements the pattern-action pairs used by flex to spit out
   a C/C++ lexical analyzer that tokenizes the config files for the
   Robolocust/Lobot program. These config files are simplified INI files
   whose syntax rules are as follows:

       1. Blank lines and comments are ignored.

       2. Only single-line comments are supported.

       3. Comments are started with either a '#' or ';' character.

       4. Comments must appear on lines by themselves (i.e., config lines
          cannot have comments embedded in them).

       5. Config lines can be either section names or key-value
          assignments.

       6. Section names must start with a letter and may be followed by
          letters, numbers or the underscore character.

       7. Section names have to be enclosed in square brackets, i.e., '['
          and ']'.

       8. The key name in a key-value assignment follows the same rules as
          section names (except that they should not be enclosed in square
          brackets).

       9. The key name must be followed by an equals sign (which may be
          surrounded by optional whitespace).

      10. The value must follow the equals sign in a key-value assignment
          statement.

      11. All tokens following the equals sign (disregarding optional
          trailing whitespace after the equals sign) comprise the value
          portion of the key-value statement.

      12. Long key-value statements may be broken up across multiple
          lines and continued from one line to the next by ending each
          line with a '\' character. Note that the '\' must be the last
          character on the line, i.e., it should be immediately followed
          by a newline and no other trailing whitespace between the '\'
          and the newline.

          All initial whitespace on the following line will be discarded.
          Thus, to break up a really long string across multiple lines,
          don't have any whitespace between the '\' at the end of lines
          and the immediately preceding character. However, to ensure
          that tokens on the following lines are considered separate, put
          some intervening whitespace between the '\' and the previous
          character.

          The '\' line continuation characters themselves are not part of
          the value portion of key-value statements.

      13. The value portion of a key-value statement is taken verbatim,
          i.e., there is no notion of quoting strings and/or escape
          sequences; the whitespace following the equals sign is ignored
          and then all characters up to the end of the line are taken as
          the value corresponding to the specified key

   The following example illustrates what such an INI file looks like:

      # This is a comment and will be ignored as will all the blank lines
      # in this example.

      ; Comments can also be introduced by a semicolon

      # These entries do not belong to any section and are part of the
      # anonymous/unnamed "global" section:
      foo = 123
      bar = look Ma, no quotes!

      # Here is some random section
      [some_section]

      # It is okay to repeat key names in different sections/scope:
      foo = ping
      bar = pong

      # A section ends when another starts
      [another_section]
      some_key = some_value

      # It is okay to repeat sections. Key-value assignments will simply
      # "resume" under the previously defined section's scope.
      [some_section]
      quoted_string = "no such thing! quotes are part of the value"

      # Repeating keys in a section is also okay. The newly assigned
      # value will override the previous one. This feature should be
      # exercised with extreme care!
      foo = oops! already defined, but not a problem
      bar = # this is not a comment; it is the value for the bar key

      # Here is an example of a long statement continued across multiple
      # lines.
      baz = this is a very very very long key-value statement that \
            continues on and on across several consecutive lines simply \
            to illustrate how such long statements may be broken up for \
            the sake of readability by using the backslash at the end \
            of each line as a line continuation character

      boo = this is another long key-value statement conti\
            nued on the next line but broken in the middle of words to ill\
            ustrate how this strange feature works

   Straying from the above rules and the syntax illustrated above will
   probably lead to errors (usually of the worst kind, i.e., silent
   assumptions of weird intent). This code is mostly just a
   quick-and-dirty hack and has not been tested extensively to iron out
   all possible bugs. So try not to push the envelope here; the basic
   config file syntax rules are more than adequate for most purposes.
*/

// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/config/LoIniFileLexer.l $ */
// $Id: LoIniFileLexer.l 14083 2010-09-30 13:59:37Z mviswana $

// yacc headers
#include "Robots/LoBot/config/LoLexYaccDefs.H"   // for various declarations
#include "Robots/LoBot/config/LoIniFileLexer.H"  // dummy hdr for dep. calc.
#include "Robots/LoBot/config/LoIniFileParser.H" // for the parser tokens

// Standard C++ headers
#include <algorithm>
#include <string>

// In the code generated by flex, the yy_get_next_buffer() function
// performs a comparison between a signed and unsigned variable. This
// produces a compiler warning, which results in an error because, by
// default, INVT builds are setup to treat all warnings as errors.
// Since we really can't fix the code generated by flex, we resort to
// turning off this warning so that the .C file generated from this
// Lex source builds without any hassles.
//
// DEVNOTE: On Debian etch, which has GCC 4.1.2, this works fine
// because, apparently, the compiler does not care for the signed-
// unsigned match in yy_get_next_buffer().
//
// On Debian lenny, which comes with GCC 4.3.2, and on Ubuntu 10.04LTS
// (Lucid Lynx), which comes with GCC 4.4.3, the diagnostic pragma is
// supported. Thus, we test for at least version 4.3 of GCC.
//
// This may not work on other systems with other versions of GCC. An
// alternative workaround is not known for this particular problem.
#if __GNUC__ >= 4 && __GNUC_MINOR__ >= 3
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

%}

/*
   This state is used after the assignment operator is encountered on a
   key-value statement to be able to read everything up to the end of the
   line without stumbling on identifiers, comments, and so on.
*/
%x LOCD_IFL_READVAL

/*
   Instruct flex to suppress unnecessary functions so that we don't get
   any warnings (which can be show stoppers no thanks to INVT's use of
   -Werror in its build system).
*/
%option noyywrap nounput

/*----------------------------- LEX RULES -----------------------------*/

%%

^[ \t]*\n        {/* ignore blank lines */}
^[ \t]*[#;].*$   {/* ignore comments */}

   /*
      These two rules look for '[' and ']' respectively. These tokens
      indicate the start of a section in the config file. Whitespace
      surrounding the tokens are acceptable.
   */
[ \t]*\[[ \t]*   {return LOCD_IFP_OPENSEC  ;}
[ \t]*\][ \t]*   {return LOCD_IFP_CLOSESEC ;}

   /*
      This rule looks for identifiers. Identifiers are the names of keys
      that make up the key-value pairs of the Robolocust configuration
      database.
   */
[A-Za-z][A-Za-z0-9_]*   {
      locd_ifp_set_current_identifier(yytext) ;
      return LOCD_IFP_IDENTIFIER ;
   }

   /*
      This rule looks for equals signs. The config file consists of
      "[section]" definitions and "key = value" assignment statements.
      Therefore, this rule recognizes a part of the assignment
      statements. Whitespace around equals signs are acceptable.

      To recognize the value part of an assignment statement we need to
      enter a new state after the equals sign has been read. This is why
      this rule initiates the switch to the LOCD_IFL_READVAL state. This
      state allows us to read every- thing up to the end of the
      assignment statement without stumbling on identifiers, comments,
      etc.
   */
[ \t]*=[ \t]*   {
      BEGIN(LOCD_IFL_READVAL) ;
      return LOCD_IFP_ASSOP ;
   }

   /*
      This rule reads the value part of a key-value assignment statement.
      It recognizes '\' at the end of a line as a line continuation
      character. When it comes across a '\' at the end of a line, it
      simply continues reading the input file and adding the characters
      to the current token until it hits a line that does not end with a
      '\'.
   */
<LOCD_IFL_READVAL>.*\\\n   {yymore() ;}

   /*
      This rule is also for the value part of a key-value assignment
      statement. It deals with the case wherein the value is all on one
      line, i.e., not continued across multiple lines with the '\'
      character. This rule will also apply for the last line of a
      multi-line value specification.

      In the case of multi-line value specs, we want to clean-up the
      token by removing all the backslashes at the ends of preceding
      lines and swallowing any initial whitespace on following lines.
      That's what all the string manipulation code in this rule's action
      block is for.
   */
<LOCD_IFL_READVAL>.*$   {
      BEGIN(INITIAL) ;

         // Clean-up the value token
      char* token = new char[yyleng + 1] ;
      std::fill_n(token, yyleng + 1, '\0') ;

      char* dst = token ;
      char* src = yytext ;
      while (*src)
      {
         char* bs = strchr(src, '\\') ;
         if (! bs) {           // no more backslashes left in source string
            strcpy(dst, src) ; // copy remaining src string into destination
            break ;
         }
         std::copy(src, bs, dst) ; // copy from curr pt in src up to bkslash
         dst += bs - src ;
         src = bs + 1 ;    // skip over backslash in source string
         if (*src != '\n') // but only if it's immediately followed by newline
            *dst++ = '\\' ;// otherwise, leave backslash in token
         while (isspace(*src)) // eat initial whitespace on following line
            ++src ;
      }
      locd_ifp_set_current_value(token) ;
      delete[] token ;
      return LOCD_IFP_VALUE ;
   }

        /* Some fallback and boilerplate rules when above don't apply. */
.    {return yytext[0] ; /* unknown token */}
\n   {/* to prevent default ECHO action   */}
