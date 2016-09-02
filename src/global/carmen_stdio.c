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

#include "global.h"
#include "carmen_stdio.h"

carmen_FILE *carmen_fopen(const char *filename, const char *mode)
{
  carmen_FILE *fp;
  
  /* allocate a new file pointer */
  fp = (carmen_FILE *)calloc(1, sizeof(carmen_FILE));
  carmen_test_alloc(fp);

  /* look at filename extension to determine if file is compressed */
#ifndef NO_ZLIB
  if(strcmp(filename + strlen(filename) - 3, ".gz") == 0)
    fp->compressed = 1;
  else
    fp->compressed = 0;
#else
  if(strcmp(filename + strlen(filename) - 3, ".gz") == 0)
    carmen_die("Error: cannot write compressed file.\n"
	       "CARMEN was not compiled with ZLIB support.\n");
  fp->compressed = 0;
#endif

  if(!fp->compressed) {
    fp->fp = fopen(filename, mode);
    if(fp->fp == NULL) {
      free(fp);
      return NULL;
    }
  }
#ifndef NO_ZLIB
  else {
    fp->fp = fopen(filename, mode);
    if(fp->fp == NULL) {
      free(fp);
      return NULL;
    }
    fp->comp_fp = gzdopen(fileno(fp->fp), mode);
    if(fp->comp_fp == NULL) {
      fclose(fp->fp);
      free(fp);
      return NULL;
    }
  }
#endif
  return fp;
}

int carmen_fgetc(carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return fgetc(fp->fp);
  else
    return gzgetc((gzFile)fp->comp_fp);
#else
  return fgetc(fp->fp);
#endif
}

int carmen_feof(carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return feof(fp->fp);
  else
    return gzeof(fp->comp_fp);
#else
  return feof(fp->fp);
#endif
}

int carmen_fseek(carmen_FILE *fp, off_t offset, int whence)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return fseeko(fp->fp, offset, whence);
  else
    return gzseek(fp->comp_fp, offset, whence);
#else
  return fseeko(fp->fp, offset, whence);
#endif
}

off_t carmen_ftell(carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return ftello(fp->fp);
  else
    return gztell(fp->comp_fp);
#else
  return ftello(fp->fp);
#endif
}

int carmen_fclose(carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return fclose(fp->fp);
  else
    return gzclose(fp->comp_fp);
#else
  return fclose(fp->fp);
#endif
}

size_t carmen_fread(void *ptr, size_t size, size_t nmemb, carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return fread(ptr, size, nmemb, fp->fp);
  else
    return gzread(fp->comp_fp, ptr, size * nmemb) / size;
#else
  return fread(ptr, size, nmemb, fp->fp);
#endif
}

size_t carmen_fwrite(const void *ptr, size_t size, size_t nmemb, 
		     carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return fwrite(ptr, size, nmemb, fp->fp);
  else
    return gzwrite(fp->comp_fp, (void *)ptr, size * nmemb) / size;
#else
  return fwrite(ptr, size, nmemb, fp->fp);
#endif
}

char *carmen_fgets(char *s, int size, carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return fgets(s, size, fp->fp);
  else
    return gzgets(fp->comp_fp, s, size);
#else
  return fgets(s, size, fp->fp);
#endif
}

int carmen_fflush(carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return fflush(fp->fp);
  else
    return gzflush(fp->fp, Z_FINISH);
#else
  return fflush(fp->fp);
#endif
}

int carmen_fputc(int c, carmen_FILE *fp)
{
#ifndef NO_ZLIB
  if(!fp->compressed)
    return fputc(c, fp->fp);
  else
    return gzputc(fp->comp_fp, c);
#else
  return fputc(c, fp->fp);
#endif
}

void carmen_fprintf(carmen_FILE *fp, const char *fmt, ...)
{
  /* Guess we need no more than 100 bytes. */
  int n, size = 100;
  char *p;
  va_list ap;

  if((p = (char *)malloc(size)) == NULL)
    return;
  while(1) {
    /* Try to print in the allocated space. */
    va_start(ap, fmt);
    n = vsnprintf(p, size, fmt, ap);
    va_end(ap);
    /* If that worked, return the string. */
    if(n > -1 && n < size) {
      carmen_fwrite(p, strlen(p), 1, fp);
      free(p);
      return;
    }
    /* Else try again with more space. */
    if(n > -1)    /* glibc 2.1 */
      size = n + 1; /* precisely what is needed */
    else           /* glibc 2.0 */
      size *= 2;  /* twice the old size */
    if((p = (char *)realloc(p, size)) == NULL)
      return;
  }
}

#ifndef NO_ZLIB
int carmen_compress(unsigned char *pDest, unsigned long *pDest_len, unsigned char *pSource, unsigned long source_len, int level)
{
  int status;
  z_stream stream;
  memset(&stream, 0, sizeof(stream));

  stream.next_in = pSource;
  stream.avail_in = (uInt)source_len;
  stream.next_out = pDest;
  stream.avail_out = (uInt)*pDest_len;

  status = deflateInit(&stream, level);
  if (status != Z_OK) return status;

  status = deflate(&stream, Z_FINISH);
  if (status != Z_STREAM_END)
  {
    deflateEnd(&stream);
    return (status == Z_OK) ? Z_BUF_ERROR : status;
  }

  *pDest_len = stream.total_out;
  return deflateEnd(&stream);
}

int carmen_uncompress(unsigned char *pDest, unsigned long *pDest_len, unsigned char *pSource, unsigned long source_len)
{
  z_stream stream;
  int status;
  memset(&stream, 0, sizeof(stream));

  stream.next_in = pSource;
  stream.avail_in = (uInt)source_len;
  stream.next_out = pDest;
  stream.avail_out = (uInt)*pDest_len;

  status = inflateInit(&stream);
  if (status != Z_OK)
    return status;

  status = inflate(&stream, Z_FINISH);
  if (status != Z_STREAM_END)
  {
    inflateEnd(&stream);
    return ((status == Z_BUF_ERROR) && (!stream.avail_in)) ? Z_DATA_ERROR : status;
  }
  *pDest_len = stream.total_out;

  return inflateEnd(&stream);
}
#endif

