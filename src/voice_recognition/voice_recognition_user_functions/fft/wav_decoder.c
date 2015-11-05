#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wav_decoder.h"
#include "wav_io.h"

wav_header_chunk file_header;
wav_format_chunk file_format;
wav_data_chunk file_data;
wav_list_chunk file_list;
wav_generic_chunk_header file_generic_header;

char* create_string_from_array (unsigned char byte_array[], int array_length)
{
	int i;
	char *str = (char *) malloc ((array_length + 1) * sizeof(char)); // + 1 for '\0'

	for (i = 0; i < array_length; i++)
		str [i] = byte_array [i];

	str[i] = '\0';

	return str;
}

int end_of_binary_file (FILE* wav_fileptr)
{
	long end_pos = 0;
	long current_pos = ftell (wav_fileptr);

	fseek (wav_fileptr, 0, SEEK_END);
	end_pos = ftell (wav_fileptr);
	fseek (wav_fileptr, current_pos, SEEK_SET);

	if ((end_pos - current_pos) == 0)
		return 1;
	else
		return 0;
}

void decode_wav_header (FILE *wav_fileptr)
{
	unsigned int num_bytes_read = 0;
	unsigned int header_chunk_length = sizeof (wav_header_chunk);

	num_bytes_read = fread (&file_header, 1, header_chunk_length, wav_fileptr);

	if (num_bytes_read != header_chunk_length)
	{
		printf("Error:: decode_wav_header\n");
		exit(-1);
	}

	printf("file offset (after decode_wav_header): %ld\n", ftell (wav_fileptr));
}

void decode_wav_format (FILE *wav_fileptr)
{
	unsigned int num_bytes_read = 0;
	unsigned int format_chunk_length = sizeof (wav_format_chunk);

	num_bytes_read = fread (&file_format, 1, format_chunk_length, wav_fileptr);

	if (num_bytes_read != format_chunk_length)
	{
		printf("Error:: decode_wav_format\n");
		exit(-1);
	}

	printf("file offset (after decode_wav_format): %ld\n", ftell (wav_fileptr));
}

void decode_generic_header (FILE* wav_fileptr)
{
	unsigned int num_bytes_read = 0;
	unsigned int chunk_header_length = sizeof (wav_generic_chunk_header);

	long pos = ftell (wav_fileptr);

	num_bytes_read = fread (&file_generic_header, 1, chunk_header_length, wav_fileptr);

	if (num_bytes_read != chunk_header_length)
	{
		printf("Error:: decode_wav_generic_header (header)\n");

		fseek(wav_fileptr, 0, SEEK_END);

		printf("Remaining bytes: %ld num bytes read: %d\n",
			ftell (wav_fileptr) - pos, num_bytes_read);

		exit(-1);
	}

	printf("file offset (after decode_generic_header): %ld\n", ftell (wav_fileptr));
}

void decode_wav_list (FILE *wav_fileptr)
{
	file_list.list_header =
		file_generic_header;

//	unsigned int num_bytes_read = 0;

//	file_list.data =
//		(char *) malloc (file_list.list_header.chunk_size * sizeof(char));

//	num_bytes_read =
//		fread (&file_list.data, 1, file_list.list_header.chunk_size, wav_fileptr);

//	if (num_bytes_read != file_list.list_header.chunk_size)
//	{
//		printf("Error:: decode_wav_list (content)\n");
//		exit(-1);
//	}

	// salta o numero de bytes equivalente aos dados do list_chunk
	fseek (wav_fileptr, file_list.list_header.chunk_size * sizeof(char), SEEK_CUR);

	printf("file offset (after decode_list_content): %ld\n", ftell (wav_fileptr));
}

void decode_wav_data_content (FILE* wav_fileptr)
{
	unsigned int num_samples_read = 0;

	file_data.data_header =
		file_generic_header;

	// if the chunk size is odd, there's
	// a padding bit
	if (file_data.data_header.chunk_size % 2 == 1)
		file_data.data_header.chunk_size --;

	unsigned int num_bytes_per_sample = (file_format.bits_per_sample / 8);
	unsigned int num_samples = file_data.data_header.chunk_size / num_bytes_per_sample;

	file_data.data = (short *) malloc (num_samples * sizeof(short));
	num_samples_read = fread (file_data.data, num_bytes_per_sample, num_samples, wav_fileptr);

	if (num_samples_read != num_samples)
	{
		printf("Error:: decode_wav_data_content\n");
		exit(-1);
	}

	printf("file offset (after decode_wav_data_content): %ld\n", ftell (wav_fileptr));

	unsigned int data_size_seconds =
		(num_samples * num_bytes_per_sample) / file_format.byte_rate;

	printf("num samples: %d - time: %ds\n", num_samples, data_size_seconds);
}

void wav_decoder (char *wav_filename)
{
	char *chunk_id;

	FILE *wav_fileptr = fopen (wav_filename, "rb+");

	decode_wav_header (wav_fileptr);
	decode_wav_format (wav_fileptr);

	// while (!feof(wav_fileptr))
	while (!end_of_binary_file(wav_fileptr))
	{
		decode_generic_header(wav_fileptr);
		chunk_id = create_string_from_array(file_generic_header.chunk_id, 4);

		// windows recorder format
		if (strcmp(chunk_id, "LIST") == 0) {
			free(chunk_id);
			decode_wav_list (wav_fileptr);
		}
		// unix recorder format
		else if (strcmp(chunk_id, "data") == 0) {
			free(chunk_id);
			decode_wav_data_content(wav_fileptr);
		}
		// unknown format
		else {
			free(chunk_id);
			exit(printf("Error:: chunk_id = \"%s\" while decoding\n", chunk_id));
		}
	}

	fclose (wav_fileptr);
}

void wav_release_data (void)
{
	free (file_data.data);
}

unsigned int wav_get_num_samples (void)
{
	unsigned int num_mics = file_format.num_channels;
	unsigned int bytes_per_sample = file_format.bits_per_sample / 8;
	unsigned int num_samples = file_data.data_header.chunk_size / (bytes_per_sample * num_mics);

	return num_samples;
}

unsigned int wav_get_num_mics (void)
{
	unsigned int num_mics = file_format.num_channels;

	return num_mics;
}

short wav_get_sample (unsigned int sample_index, unsigned int mic_index)
{
	unsigned int p = (sample_index * file_format.num_channels + mic_index);
	short sample = file_data.data[p];

	return sample;
}

unsigned int wav_get_sample_rate (void)
{
	return file_format.sample_rate;
}
