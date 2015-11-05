 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/


/** @addtogroup global libglobal **/
// @{

/** \file carmen_stdio.h
 * \brief stdio-functions for CARMEN in libglobal. This library supports gzipped files.
 *
 * stdio-functions for CARMEN. Support reading and writing gzipped files.
 **/

#ifndef DGC_MY_STDIO_H
#define DGC_MY_STDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NO_ZLIB
#include <zlib.h>
#endif

typedef struct {
  int compressed;
  FILE *fp;
#ifndef NO_ZLIB
  gzFile *comp_fp;
#endif
} carmen_FILE;

carmen_FILE *carmen_fopen(const char *filename, const char *mode);

int carmen_fgetc(carmen_FILE *fp);

int carmen_feof(carmen_FILE *fp);

int carmen_fseek(carmen_FILE *fp, off_t offset, int whence);

off_t carmen_ftell(carmen_FILE *fp);

int carmen_fclose(carmen_FILE *fp);

size_t carmen_fread(void *ptr, size_t size, size_t nmemb, carmen_FILE *fp);

size_t carmen_fwrite(const void *ptr, size_t size, size_t nmemb, 
		     carmen_FILE *fp);

char *carmen_fgets(char *s, int size, carmen_FILE *fp);

int carmen_fputc(int c, carmen_FILE *fp);

void carmen_fprintf(carmen_FILE *fp, const char *fmt, ...);

int carmen_fflush(carmen_FILE *fp);

int carmen_compress(unsigned char *pDest, unsigned long *pDest_len, unsigned char *pSource, unsigned long source_len, int level);

int carmen_uncompress(unsigned char *pDest, unsigned long *pDest_len, unsigned char *pSource, unsigned long source_len);

#ifdef __cplusplus
}
#endif

#endif
// @}
