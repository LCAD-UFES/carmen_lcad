#include <cstdio>
#include <cstdlib>
#include "wav_decoder.h"
#include "wav_io.h"

extern wav_header_chunk file_header;
extern wav_format_chunk file_format;
extern wav_data_chunk file_data;
extern wav_list_chunk file_list;

void wav_show_header_and_format_information (void)
{
	char *header_file_format = create_string_from_array (file_header.format, 4);
	char *header_chunk_id = create_string_from_array (file_header.chunk_id, 4);
	char *format_chunk_id = create_string_from_array (file_format.chunk_id, 4);

	printf ("<HEADER>\n");
	printf ("\tHeader ID: %s\n", header_chunk_id);
	printf ("\tFile format: %4s\n", header_file_format);
	printf ("\tData length: %d\n", file_header.chunk_size - 36);
	printf ("</HEADER>\n");

	printf ("<FORMAT>\n");
	printf ("\tFormat ID: %s\n", format_chunk_id);
	printf ("\tAudio format: %d\n", file_format.audio_format);
	printf ("\tNum Channels: %d\n", file_format.num_channels);
	printf ("\tBits per sample: %d\n", file_format.bits_per_sample);
	printf ("\tBlock Align: %d\n", file_format.block_align);
	printf ("\tByte rate: %d\n", file_format.byte_rate);
	printf ("</FORMAT>\n");

	free (header_file_format);
	free (header_chunk_id);
	free (format_chunk_id);
}

void wav_show_list_information (void)
{
	char *list_chunk_id = create_string_from_array (file_list.list_header.chunk_id, 4);

	printf ("<LIST>\n");
	printf ("\tChunk ID: %s\n", list_chunk_id);
	printf ("\tText Length: %d\n", file_list.list_header.chunk_size);
	printf ("</LIST>\n");

	free(list_chunk_id);
}

void wav_show_data_information (void)
{
	char *data_chunk_id = create_string_from_array (file_data.data_header.chunk_id, 4);

	printf ("<DATA>\n");
	printf ("\tData ID: %s\n", data_chunk_id);
	printf ("\tData Length: %d\n", file_data.data_header.chunk_size);
	printf ("</DATA>\n");

	free(data_chunk_id);
}

void write_raw_data_to_file (char *output_filename)
{
	short frequency;
	unsigned int i, j, block_count;
	unsigned int num_mics = file_format.num_channels;
	unsigned int bytes_per_sample = file_format.bits_per_sample / 8;
	unsigned int num_samples = file_data.data_header.chunk_size / (bytes_per_sample * num_mics);

	FILE *output_fileptr = fopen (output_filename, "w");
	fprintf (output_fileptr, "%d %d\n", num_samples, num_mics);

	block_count = 0;

	for (i = 0; i < num_samples; i++)
	{
		fprintf (output_fileptr, "%d ", i);

		// o loop abaixo escreve os dados
		// relativos a cada microfone
		for(j = 0; j < num_mics; j++)
		{
			frequency = wav_get_sample(i, j);
			fprintf(output_fileptr, "%d ", frequency);
		}

		fprintf(output_fileptr, "\n");

		block_count ++;
	}

	fclose (output_fileptr);
}
