/*!@file Envision/env_stdio_interface.c */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_stdio_interface.c $
// $Id: env_stdio_interface.c 12962 2010-03-06 02:13:53Z irock $
//

#ifndef ENVISION_ENV_STDIO_INTERFACE_C_DEFINED
#define ENVISION_ENV_STDIO_INTERFACE_C_DEFINED

#include "Envision/env_stdio_interface.h"

#include "Envision/env_alloc.h"
#include "Envision/env_image.h"
#include "Envision/env_log.h"

#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h> // for fprintf()
#include <stdlib.h> // for abort()
#include <string.h> // for strerror()

// ######################################################################
static void lfatal(const char *msg, ...)
        __attribute__((format(__printf__,1,2)));

static void lfatal(const char *msg, ...)
{
        va_list args;
        va_start(args, msg);

        vfprintf(stderr, msg, args);

        va_end(args);

        fprintf(stderr, "\n");
        fflush(stderr);

        abort();
}

// ######################################################################
static void div2(const env_size_t numer, const env_size_t denom,
                 const env_size_t ndigits,
                 env_size_t* const whole, env_size_t* const fract)
{
  *whole = numer / denom;
  *fract = 0;
  env_size_t rem = numer - (*whole * denom);
  for (env_size_t i = 0; i < ndigits; ++i)
    {
      rem *= 10;
      const env_size_t newwhole = rem / denom;
      ENV_ASSERT(newwhole < 10);
      rem = rem - (newwhole * denom);
      *fract *= 10;
      *fract += newwhole;
    }
}

// ######################################################################
void env_stdio_assert_handler(const char* what, int custom_msg,
                                     const char* where, int line_no)
{
        if (custom_msg)
                fprintf(stderr, "Assertion failed (%s:%d):\n\t%s\n\n",
                        where, line_no, what);
        else
                fprintf(stderr, "Assertion failed (%s:%d):\n\texpected '%s'\n\n",
                        where, line_no, what);
        abort();
}

// ######################################################################
void env_stdio_print_alloc_stats(const struct env_alloc_stats* p,
                                 const env_size_t block_size)
{
        env_size_t kiB_block_size_whole, kiB_block_size_fract;
        div2(block_size, 1024, 2,
             &kiB_block_size_whole, &kiB_block_size_fract);

        ENV_ASSERT(block_size > 0);

        env_size_t n_cache_blocks = 0;

        for (env_size_t i = 0; i < p->ncache_used; ++i)
        {
                const env_size_t nb =
                        (p->cache[i].num_allocations
                         * p->cache[i].alloc_size);

                n_cache_blocks += p->cache[i].num_allocations;

                env_size_t kiB_alloc_whole, kiB_alloc_fract;
                div2(nb, 1024, 2,
                     &kiB_alloc_whole, &kiB_alloc_fract);

                env_size_t kiB_alloc_size_whole, kiB_alloc_size_fract;
                div2(p->cache[i].alloc_size, 1024, 4,
                     &kiB_alloc_size_whole, &kiB_alloc_size_fract);

                env_size_t alloc_percent_whole, alloc_percent_fract;
                div2(nb*100, p->bytes_allocated, 2,
                     &alloc_percent_whole, &alloc_percent_fract);

                env_size_t block_ratio_whole, block_ratio_fract;
                char symbol;

                if (p->cache[i].alloc_size - p->overhead >= block_size
                    || p->cache[i].alloc_size - p->overhead <= 1)
                {
                        div2(p->cache[i].alloc_size - p->overhead,
                             block_size, 1,
                             &block_ratio_whole, &block_ratio_fract);
                        symbol = '*';
                }
                else
                {
                        div2(block_size,
                             p->cache[i].alloc_size - p->overhead, 1,
                             &block_ratio_whole, &block_ratio_fract);
                        symbol = '/';
                }

                fprintf(stderr,
                        "memstats: cache[%02lu/%02lu]: "
                        "%5lu.%02lukiB (%3lu.%02lu%%) "
                        "in %4lu allocations "
                        "(%2lu active) of %5lu.%04lukiB"
                        " (%lu.%02lukiB %c %5lu.%01lu + %2luB)\n",
                        i, ENV_NCACHE,
                        kiB_alloc_whole, kiB_alloc_fract,
                        alloc_percent_whole, alloc_percent_fract,
                        p->cache[i].num_allocations,
                        p->cache[i].num_active,
                        kiB_alloc_size_whole, kiB_alloc_size_fract,
                        kiB_block_size_whole, kiB_block_size_fract,
                        (int) symbol,
                        block_ratio_whole, block_ratio_fract,
                        p->overhead);
        }

        env_size_t kiB_alloc_whole, kiB_alloc_fract;
        div2(p->bytes_allocated, 1024, 2,
             &kiB_alloc_whole, &kiB_alloc_fract);

        env_size_t n_blocks_whole, n_blocks_fract;
        div2(p->bytes_allocated, block_size, 1,
             &n_blocks_whole, &n_blocks_fract);

        fprintf(stderr,
                "memstats: =====[TOTAL]: "
                "%5lu.%02lukiB (100.00%%) "
                "in %4lu allocations "
                "(%2lu active) ================"
                " (%lu.%02lukiB * %5lu.%01lu      )\n",
                kiB_alloc_whole, kiB_alloc_fract,
                n_cache_blocks,
                p->nallocations_current,
                kiB_block_size_whole, kiB_block_size_fract,
                n_blocks_whole, n_blocks_fract);

        fprintf(stderr,
                "memstats: %lu/%lu cache table entries in use\n",
                p->ncache_used, ENV_NCACHE);

        fprintf(stderr,
                "memstats: block alignment: %lu bytes\n", p->nalign);

        fprintf(stderr,
                "memstats: all-time: %llukiB in %lu requested allocations\n",
                p->nbytes_alltime/1024, p->nallocations_alltime);

        fprintf(stderr,
                "memstats: current: %lukiB in %lu active allocations\n",
                (unsigned long)(p->nbytes_current/1024),
                p->nallocations_current);
}

// ######################################################################
struct env_rgb_pixel* env_stdio_parse_rgb(const char* fname,
                                          struct env_dims* outdims)
{
        FILE* f = fopen(fname, "rb");

        if (f == 0)
                lfatal("Couldn't open file '%s' for reading.", fname);

        int c = getc(f);
        if (c != 'P')
                lfatal("Missing magic number in pnm file '%s'"
                       "(got '%c' [%d], expected '%c' [%d]).",
                       fname, c, c, 'P', 'P');

        int mode = -1;
        int ret = fscanf(f, "%d", &mode);
        if (ret > 0 && mode != 6)
                lfatal("Wrong pnm mode (got 'P%d', expected 'P6')",
                       mode);

        while (1)
        {
                const int c = getc(f);
                if (!isspace(c))
                { ungetc(c, f); break; }
        }

        // copy and concatenate optional comment line(s) starting with '#'
        // into comments string

        while (1)
        {
                const int c = getc(f);
                if (c != '#')
                { ungetc(c, f); break; }
                else
                {
                        while (getc(f) != '\n')
                        { /* empty loop */ }
                }
        }

        int w = -1;
        int h = -1;
        int maxGrey = -1;
        ret = fscanf(f, "%d %d %d", &w, &h, &maxGrey);
        ENV_ASSERT(ret > 0);
        ENV_ASSERT(w > 0);
        ENV_ASSERT(h > 0);
        ENV_ASSERT(maxGrey > 0);

        // read one more character of whitespace from the stream after maxGrey
        c = getc(f);
        if ( !isspace(c) )
                lfatal("Missing whitespace after maxGrey in pbm file '%s'.", fname);

        struct env_rgb_pixel* result = (struct env_rgb_pixel*)
                env_allocate(w * h * sizeof(struct env_rgb_pixel));
        if (fread((char*) result, 3, w*h, f) != ((env_size_t)(w*h)))
                lfatal("%s: fread() failed", fname);
        outdims->w = w;
        outdims->h = h;
        return result;
}

// ######################################################################
void env_stdio_write_gray(const struct env_image* iimage,
                          const char* outstem, const char* name, int c)
{
        if (!env_img_initialized(iimage))
                return;

        // if outstem is the empty string, then the user wanted us to
        // suppress output:
        if (outstem[0] == '\0')
                return;

        char fname[256];
        snprintf(fname, sizeof(fname),
                 "%s-%s%06d.pnm", outstem, name, c);

        FILE* const f = fopen(fname, "wb");
        if (f == 0)
                lfatal("%s: couldn't open PNM file for writing (errno=%d, %s)",
                       fname, errno, strerror(errno));

        if (fprintf(f, "P5\n%d %d\n255\n",
                    (int) iimage->dims.w, (int) iimage->dims.h) < 0)
                lfatal("%s: fprintf() failed (errno=%d, %s)",
                       fname, errno, strerror(errno));

        const intg32* const src = env_img_pixels(iimage);
        const env_size_t sz = env_img_size(iimage);
        byte* bimage =
                (byte*) env_allocate(sz * sizeof(byte));

        for (env_size_t i = 0; i < sz; ++i)
        {
                // the caller is supposed to have already ensured that
                // the intg32 image has been downsampled to a [0,255]
                // range, so let's verify that:
                ENV_ASSERT(src[i] >= 0 && src[i] <= 255);
                bimage[i] = (byte) src[i];
        }

        if (fwrite(bimage, 1, sz, f) != sz)
        {
                env_deallocate(bimage);
                lfatal("%s: fwrite() failed (errno=%d, %s)",
                       fname, errno, strerror(errno));
        }

        env_deallocate(bimage);

        if (fclose(f) != 0)
                lfatal("%s: fclose() failed (errno=%d, %s)",
                       fname, errno, strerror(errno));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_STDIO_INTERFACE_C_DEFINED
