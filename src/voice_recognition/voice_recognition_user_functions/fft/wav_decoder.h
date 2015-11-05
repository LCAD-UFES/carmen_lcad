
/**
 * @File: wav_decoder.h
 * @Author: _Filipe
 * @Date: 23/05/2012 18:56
 *
 * This file describes the structures of an WAVE sound file.
 * Each structure corresponds to a part (chunk, in technical language)
 * of the file.
 *
 */

#ifndef __WAV_DECODER_H_
#define __WAV_DECODER_H_

	/**
	 * WAVE structs description from:
	 * https://ccrma.stanford.edu/courses/422/projects/WaveFormat/
	 * http://www.sonicspot.com/guide/wavefiles.html#data (list chunk)
	 */

	typedef struct
	{
		unsigned char chunk_id[4]; // 4 bytes: (the letters of "RIFF", without quotations) -> (0x52494646 big-endian form).
		unsigned int  chunk_size;  // 4 bytes: (36 + SubChunk2Size, i.e., 36 plus the size of the chunk of data)
		unsigned char format[4];   // 4 bytes: (the letters of "WAVE", without quotations) -> (0x57415645 big-endian form)

	}wav_header_chunk;

	typedef struct
	{
		unsigned char chunk_id[4];       // 4 bytes: (the letters "fmt ") -> (0x666d7420 big-endian form).
		unsigned int  chunk_size;        // 4 byes: 16 for PCM.  This is the size of the rest of the Subchunk which follows this number.
		unsigned short audio_format;     // 2 bytes: PCM = 1 (i.e. Linear quantization). Values other than 1 indicate some form of compression.
		unsigned short num_channels;     // 2 bytes: Mono = 1, Stereo = 2, etc.
		unsigned int sample_rate;        // 4 bytes: 8000, 44100, etc.
		unsigned int byte_rate;          // 4 bytes: == SampleRate * NumChannels * BitsPerSample/8
		unsigned short block_align;      // 2 bytes: == NumChannels * BitsPerSample/8 -> The number of bytes for one sample including all channels. I wonder what happens when this number isn't an integer?
		unsigned short bits_per_sample;  // 2 bytes: 8 bits = 8, 16 bits = 16, etc.

		// 2 bytes to extra_param_size (if PCM, it doesn't exist)
		// X extra_params

	}wav_format_chunk;

	typedef struct
	{
		unsigned char chunk_id [4]; // 4 bytes: Contains the id of the chunk
		unsigned int chunk_size;    // 4 bytes: num bytes to the end of the chunk

	}wav_generic_chunk_header;

	typedef struct
	{
		wav_generic_chunk_header list_header;   // chunk_id = "list"
		char *data;                             // x bytes: list of text labels and names

	}wav_list_chunk;

	typedef struct
	{
		wav_generic_chunk_header data_header;   // chunk_id = "data"
		short *data;                            // x bytes: The actual sound data.

	}wav_data_chunk;

	void wav_decoder (char *wav_filename);
	void wav_release_data (void);
	unsigned int wav_get_num_mics (void);
	unsigned int wav_get_num_samples (void);
	unsigned int wav_get_sample_rate (void);
	short wav_get_sample (unsigned int sample_index, unsigned int mic_index);
	char* create_string_from_array (unsigned char byte_array[], int array_length);

#endif // __WAV_DECODER_H_
