/** @file rutz/timeformat.h c++-friendly wrapper around strftime() */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at klab dot caltech dot edu>
//
// created: Thu Jun 30 15:18:01 2005
// commit: $Id: timeformat.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/timeformat.h $
//
// --------------------------------------------------------------------
//
// This file is part of GroovX.
//   [http://www.klab.caltech.edu/rjpeters/groovx/]
//
// GroovX is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// GroovX is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GroovX; if not, write to the Free Software Foundation,
// Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
///////////////////////////////////////////////////////////////////////

#ifndef GROOVX_RUTZ_TIMEFORMAT_H_UTC20050630221801_DEFINED
#define GROOVX_RUTZ_TIMEFORMAT_H_UTC20050630221801_DEFINED

#include <sys/time.h> // for timeval

namespace rutz
{
  class fstring;

  /// Return a formatted string (a la strftime) for the given timeval.
  /** Formatting codes (see 'man strftime' for more details):

      %a     The abbreviated weekday name according to the current locale.

      %A     The full weekday name according to the current locale.

      %b     The abbreviated month name according to the current locale.

      %B     The full month name according to the current locale.

      %c     The preferred date and time representation for the current locale.

      %C     The century number (year/100) as a 2-digit integer. (SU)

      %d     The day of the month as a decimal number (range 01 to 31).

      %D     Equivalent to %m/%d/%y. (Yecch - for Americans only. Americans
             should note that in other countries %d/%m/%y is rather
             common. This means that in international context this format
             is ambiguous and should not be used.) (SU)

      %e     Like %d, the day of the month as a decimal number, but a leading
             zero is replaced by a space. (SU)

      %E     Modifier: use alternative format, see below. (SU)

      %F     Equivalent to %Y-%m-%d (the ISO 8601 date format). (C99)

      %G     The ISO 8601 year with century as a decimal number. The 4-digit
             year corresponding to the ISO week number (see %V). This
             has the same format and value as %y, except that if the ISO
             week number belongs to the previous or next year, that year
             is used instead. (TZ)

      %g     Like %G, but without century, i.e., with a 2-digit year
             (00-99). (TZ)

      %h     Equivalent to %b. (SU)

      %H     The hour as a decimal number using a 24-hour clock
             (range 00 to 23).

      %I     The hour as a decimal number using a 12-hour clock
             (range 01 to 12).

      %j     The day of the year as a decimal number (range 001 to 366).

      %k     The hour (24-hour clock) as a decimal number (range
             0 to 23); single digits are preceded by a blank.
             (See also %H.) (TZ)

      %l     The hour (12-hour clock) as a decimal number (range 1 to 12);
             single digits are preceded by a blank. (See also %I.) (TZ)

      %m     The month as a decimal number (range 01 to 12).

      %M     The minute as a decimal number (range 00 to 59).

      %n     A newline character. (SU)

      %O     Modifier: use alternative format, see below. (SU)

      %p     Either `AM' or `PM' according to the given time value, or the
             corresponding strings for the current locale. Noon is
             treated as `pm' and midnight as `am'.

      %P     Like %p but in lowercase: `am' or `pm' or a corresponding string
             for the current locale. (GNU)

      %r     The time in a.m. or p.m. notation. In the POSIX
             locale this is equivalent to `%I:%M:%S %p'. (SU)

      %R     The time in 24-hour notation (%H:%M). (SU) For a
             version including the seconds, see %T below.

      %s     The number of seconds since the Epoch, i.e., since
             1970-01-01 00:00:00 UTC. (TZ)

      %S     The second as a decimal number (range 00 to 61).

      %t     A tab character. (SU)

      %T     The time in 24-hour notation (%H:%M:%S). (SU)

      %u     The day of the week as a decimal, range 1 to 7,
             Monday being 1. See also %w. (SU)

      %U     The week number of the current year as a decimal
             number, range 00 to 53, starting with the first
             Sunday as the first day of week 01. See also %V and
             %W.

      %V     The ISO 8601:1988 week number of the current year as a decimal
             number, range 01 to 53, where week 1 is the first week that
             has at least 4 days in the current year, and with Monday as
             the first day of the week. See also %U and %W. (SU)

      %w     The day of the week as a decimal, range 0 to 6,
             Sunday being 0. See also %u.

      %W     The week number of the current year as a decimal
             number, range 00 to 53, starting with the first
             Monday as the first day of week 01.

      %x     The preferred date representation for the current
             locale without the time.

      %X     The preferred time representation for the current
             locale without the date.

      %y     The year as a decimal number without a century (range 00 to 99).

      %Y     The year as a decimal number including the century.

      %z     The time-zone as hour offset from GMT. Required to
             emit RFC822-conformant dates (using "%a, %d %b %Y
             %H:%M:%S %z"). (GNU)

      %Z     The time zone or name or abbreviation.

      %+     The date and time in date(1) format. (TZ)

      %%     A literal `%' character.

  */
  rutz::fstring format_time(const timeval& tval,
                            const char* formatstring
                            = "%a %b %d %H:%M:%S %Z %Y");
}

static const char __attribute__((used)) vcid_groovx_rutz_timeformat_h_utc20050630221801[] = "$Id: timeformat.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/timeformat.h $";
#endif // !GROOVX_RUTZ_TIMEFORMAT_H_UTC20050630221801DEFINED
