/*******************************************************************************#
#           guvcview              http://guvcview.berlios.de                    #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 2 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
#                                                                               #
# You should have received a copy of the GNU General Public License             #
# along with this program; if not, write to the Free Software                   #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA     #
#                                                                               #
********************************************************************************/
/*******************************************************************************#
#   Some utilities for writing and reading AVI files.                           # 
#   These are not intended to serve for a full blown                            #
#   AVI handling software (this would be much too complex)                      #
#   The only intention is to write out MJPEG encoded                            #
#   AVIs with sound and to be able to read them back again.                     #
#   These utilities should work with other types of codecs too, however.        #
#                                                                               #
#   Copyright (C) 1999 Rainer Johanni <Rainer@Johanni.de>                       #
********************************************************************************/


#ifndef AVILIB_H
#define AVILIB_H

#include "defs.h"
#include <inttypes.h>
#include <sys/types.h>
//#include <glib.h>

#define AVI_MAX_TRACKS 8
#define FRAME_RATE_SCALE 1000000

typedef struct _video_index_entry
{
	off_t key;
	off_t pos;
	off_t len;
} video_index_entry;

typedef struct _audio_index_entry
{
	off_t pos;
	off_t len;
	off_t tot;
} audio_index_entry;


typedef struct track_s
{

	long   a_fmt;             /* Audio format, see #defines below */
	long   a_chans;           /* Audio channels, 0 for no audio */
	long   a_rate;            /* Rate in Hz */
	long   a_bits;            /* bits per audio sample */
	long   mpgrate;           /* mpg bitrate kbs*/
	long   a_vbr;             /* 0 == no Variable BitRate */
	long   padrate;	      /* byte rate used for zero padding */

	long   audio_strn;        /* Audio stream number */
	off_t  audio_bytes;       /* Total number of bytes of audio data */
	long   audio_chunks;      /* Chunks of audio data in the file */

	char   audio_tag[4];      /* Tag of audio data */
	long   audio_posc;        /* Audio position: chunk */
	long   audio_posb;        /* Audio position: byte within chunk */
 
	off_t  a_codech_off;       /* absolut offset of audio codec information */ 
	off_t  a_codecf_off;       /* absolut offset of audio codec information */ 

	audio_index_entry *audio_index;

} __attribute__ ((packed)) track_t;

/*
typedef struct
{
  DWORD  bi_size;
  DWORD  bi_width;
  DWORD  bi_height;
  DWORD  bi_planes;
  DWORD  bi_bit_count;
  DWORD  bi_compression;
  DWORD  bi_size_image;
  DWORD  bi_x_pels_per_meter;
  DWORD  bi_y_pels_per_meter;
  DWORD  bi_clr_used;
  DWORD  bi_clr_important;
} __attribute__ ((packed)) alBITMAPINFOHEADER;

typedef struct
{
  WORD  w_format_tag;
  WORD  n_channels;
  DWORD  n_samples_per_sec;
  DWORD  n_avg_bytes_per_sec;
  WORD  n_block_align;
  WORD  w_bits_per_sample;
  WORD  cb_size;
} __attribute__ ((packed)) alWAVEFORMATEX;

typedef struct
{
  DWORD fcc_type; 
  DWORD fcc_handler; 
  DWORD dw_flags; 
  DWORD dw_caps; 
  WORD w_priority;
  WORD w_language;
  DWORD dw_scale;
  DWORD dw_rate;
  DWORD dw_start;
  DWORD dw_length;
  DWORD dw_initial_frames;
  DWORD dw_suggested_buffer_size;
  DWORD dw_quality;
  DWORD dw_sample_size;
  DWORD dw_left;
  DWORD dw_top;
  DWORD dw_right;
  DWORD dw_bottom;
  DWORD dw_edit_count;
  DWORD dw_format_change_count;
  char     sz_name[64];
} __attribute__ ((packed)) alAVISTREAMINFO;

*/

struct avi_t
{
	long   fdes;              /* File descriptor of AVI file */
	long   mode;              /* 0 for reading, 1 for writing */
	                          /* let's make it thead safe                        */
  
	long   width;             /* Width  of a video frame */
	long   height;            /* Height of a video frame */
	double fps;               /* Frames per second */
	char   compressor[8];     /* Type of compressor, 4 bytes + padding for 0 byte */
	char   compressor2[8];     /* Type of compressor, 4 bytes + padding for 0 byte */
	long   video_strn;        /* Video stream number */
	long   video_frames;      /* Number of video frames */
	char   video_tag[4];      /* Tag of video data */
	long   video_pos;         /* Number of next frame to be read
	                             (if index present) */
    
	DWORD max_len;    /* maximum video chunk present */
  
	track_t track[AVI_MAX_TRACKS];  // up to AVI_MAX_TRACKS audio tracks supported
  
	off_t  pos;               /* position in file */
	long   n_idx;             /* number of index entries actually filled */
	long   max_idx;           /* number of index entries actually allocated */
  
	off_t  v_codech_off;      /* absolut offset of video codec (strh) info */ 
	off_t  v_codecf_off;      /* absolut offset of video codec (strf) info */ 
  
	BYTE (*idx)[16]; /* index entries (AVI idx1 tag) */

	video_index_entry *video_index;
    
	//int is_opendml;           /* set to 1 if this is an odml file with multiple index chunks */
  
	off_t  last_pos;          /* Position of last frame written */
	DWORD last_len;   /* Length of last frame written */
	int must_use_index;       /* Flag if frames are duplicated */
	off_t  movi_start;
	int total_frames;         /* total number of frames if dmlh is present */
    
	int anum;            // total number of audio tracks 
	int aptr;            // current audio working track 
	// int comment_fd;      // Read avi header comments from this fd
	// char *index_file;    // read the avi index from this file
  
	//alBITMAPINFOHEADER *bitmap_info_header;
	//alWAVEFORMATEX *wave_format_ex[AVI_MAX_TRACKS];

	void*	extradata;
	ULONG	extradata_size;
	int closed; /* 0 - AVI is opened(recordind) 1 -AVI is closed (not recording)*/ 

} __attribute__ ((packed));

#define AVI_MODE_WRITE  0
#define AVI_MODE_READ   1

/* The error codes delivered by avi_open_input_file */

#define AVI_ERR_SIZELIM      1     /* The write of the data would exceed
                                      the maximum size of the AVI file.
                                      This is more a warning than an error
                                      since the file may be closed safely */

#define AVI_ERR_OPEN         2     /* Error opening the AVI file - wrong path
                                      name or file nor readable/writable */

#define AVI_ERR_READ         3     /* Error reading from AVI File */

#define AVI_ERR_WRITE        4     /* Error writing to AVI File,
                                      disk full ??? */

#define AVI_ERR_WRITE_INDEX  5     /* Could not write index to AVI file
                                      during close, file may still be
                                      usable */

#define AVI_ERR_CLOSE        6     /* Could not write header to AVI file
                                      or not truncate the file during close,
                                      file is most probably corrupted */

#define AVI_ERR_NOT_PERM     7     /* Operation not permitted:
                                      trying to read from a file open
                                      for writing or vice versa */

#define AVI_ERR_NO_MEM       8     /* malloc failed */

#define AVI_ERR_NO_AVI       9     /* Not an AVI file */

#define AVI_ERR_NO_HDRL     10     /* AVI file has no has no header list,
                                      corrupted ??? */

#define AVI_ERR_NO_MOVI     11     /* AVI file has no has no MOVI list,
                                      corrupted ??? */

#define AVI_ERR_NO_VIDS     12     /* AVI file contains no video data */

#define AVI_ERR_NO_IDX      13     /* The file has been opened with
                                      getIndex==0, but an operation has been
                                      performed that needs an index */

/* Possible Audio formats */

#define WAVE_FORMAT_UNKNOWN             (0x0000)
#define WAVE_FORMAT_PCM                 (0x0001)
#define WAVE_FORMAT_ADPCM               (0x0002)
#define WAVE_FORMAT_IBM_CVSD            (0x0005)
#define WAVE_FORMAT_ALAW                (0x0006)
#define WAVE_FORMAT_MULAW               (0x0007)
#define WAVE_FORMAT_OKI_ADPCM           (0x0010)
#define WAVE_FORMAT_DVI_ADPCM           (0x0011)
#define WAVE_FORMAT_DIGISTD             (0x0015)
#define WAVE_FORMAT_DIGIFIX             (0x0016)
#define WAVE_FORMAT_YAMAHA_ADPCM        (0x0020)
#define WAVE_FORMAT_DSP_TRUESPEECH      (0x0022)
#define WAVE_FORMAT_GSM610              (0x0031)
#define IBM_FORMAT_MULAW                (0x0101)
#define IBM_FORMAT_ALAW                 (0x0102)
#define IBM_FORMAT_ADPCM                (0x0103)
/*extra audio formats (codecs)*/
#define DOLBY_FORMAT_AC3 		(0x2000)
#define ANTEX_FORMAT_ADPCME		(0x0033)
#define AUDIO_FORMAT_APTX		(0x0025)
#define AUDIOFILE_FORMAT_AF10		(0x0026)
#define AUDIOFILE_FORMAT_AF36		(0x0024) 
#define BROOKTREE_FORMAT_BTVD		(0x0400)
#define CANOPUS_FORMAT_ATRAC		(0x0063)
#define CIRRUS_FORMAT_CIRRUS		(0x0060)
#define CONTROL_FORMAT_CR10		(0x0037)
#define CONTROL_FORMAT_VQLPC		(0x0034)
#define CREATIVE_FORMAT_ADPCM		(0x0200)
#define CREATIVE_FORMAT_FASTSPEECH10	(0x0203)
#define CREATIVE_FORMAT_FASTSPEECH8	(0x0202)
#define IMA_FORMAT_ADPCM		(0x0039)
#define CONSISTENT_FORMAT_CS2		(0x0260)
#define HP_FORMAT_CU			(0x0019)
#define DEC_FORMAT_G723			(0x0123)
#define DF_FORMAT_G726			(0x0085)
#define DSP_FORMAT_ADPCM		(0x0036)
#define DOLBY_FORMAT_AC2		(0x0030)
#define DOLBY_FORMAT_AC3_SPDIF		(0x0092)
#define ESS_FORMAT_ESPCM		(0x0061)
#define IEEE_FORMAT_FLOAT		(0x0003)
#define ISO_FORMAT_MP3			(0x0055)
#define ISO_FORMAT_MPEG12		(0x0050)
#define MS_FORMAT_MSAUDIO1_DIVX		(0x0160)
#define MS_FORMAT_MSAUDIO2_DIVX		(0x0161)
#define OGG_FORMAT_VORBIS1		(0x674f)
#define OGG_FORMAT_VORBIS1P		(0x676f)
#define OGG_FORMAT_VORBIS2		(0x6750)
#define OGG_FORMAT_VORBIS2P		(0x6770)
#define OGG_FORMAT_VORBIS3		(0x6751)
#define OGG_FORMAT_VORBIS3P		(0x6771)
#define MS_FORMAT_WMA9			(0x0163)
#define MS_FORMAT_WMA9_PRO		(0x0162)

int AVI_open_output_file(struct avi_t *AVI, const char * filename);
void AVI_set_video(struct avi_t *AVI, int width, int height, double fps, char *compressor);
void AVI_set_audio(struct avi_t *AVI, int channels, long rate, int mpgrate, int bits, int format);
int  AVI_write_frame(struct avi_t *AVI, BYTE *data, long bytes, int keyframe);
int  AVI_dup_frame(struct avi_t *AVI);
int  AVI_write_audio(struct avi_t *AVI, BYTE *data, long bytes);
int  AVI_append_audio(struct avi_t *AVI, BYTE *data, long bytes);
ULONG AVI_bytes_remain(struct avi_t *AVI);
int  AVI_close(struct avi_t *AVI);

//int avi_update_header(struct avi_t *AVI);
int AVI_set_audio_track(struct avi_t *AVI, int track);
void AVI_set_audio_vbr(struct avi_t *AVI, long is_vbr);


ULONG AVI_set_MAX_LEN(ULONG len);

int AVI_getErrno();

void AVI_print_error(char *str);
char *AVI_strerror();
char *AVI_syserror();

#endif
