#include <stdio.h>
#include <stdlib.h>
#include "wav_decoder.h"
#include "wav_io.h"
#include "spectogram.h"

extern wav_data_chunk file_data;

extern unsigned int chunk_size;

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		printf("\n");
		printf("Use %s <filename.wav> <output-spectogram>\n", argv[0]);
		printf("\n");

		exit(-1);
	}

	wav_decoder (argv[1]);
	write_raw_data_to_file(argv[2]);

	spectogram_alloc ();

	unsigned int i = 0;
	unsigned int count = 0;

	printf ("num_colunas_max: %d\n", wav_get_num_samples() / chunk_size);

	while ((i + chunk_size) < wav_get_num_samples())
	{
		spectogram_create (i);

		i += chunk_size;
		count++;
	}

	spectogram_write_to_file((char *) argv[2]);
	spectogram_release ();
	wav_release_data ();

	printf ("Terminou!\n");
	return 0;
}
