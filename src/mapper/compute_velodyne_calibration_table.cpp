#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Type declarations
typedef struct
{
	int laser, ray_size, intensity, mx, my, cx, cy;
} LIMC;


typedef struct
{
	int mx, my, cx, cy;
	size_t file_offset;
} MC;


typedef struct
{
	long accumulated_intensity; // Tem que ser long para nao perder precisao durante a acumulacao
	long count;
} TEMP_TABLE;


// Global variables
FILE *calibration_file;
FILE *calibration_file_bin;

#define USE_RAY_SIZE

#ifdef USE_RAY_SIZE
TEMP_TABLE temp_table[32][10][256];
#else
TEMP_TABLE temp_table[32][256];
#endif

LIMC INVALID_LIMC = {-1, -1, -1, -1, -1, -1, -1};


bool
read_mc_file_line(MC *mc_line, FILE *mc_file)
{
	char line[1024];

	mc_line->file_offset = ftell(mc_file);
	if (fgets(line, 1023, mc_file))
	{
		if (sscanf(line, "%d %d %d %d", &(mc_line->mx), &(mc_line->my), &(mc_line->cx), &(mc_line->cy)) != 4)
		{
			printf("Could read the mc_line->mx, mc_line->my, mc_line->cx, mc_line->cy from line:\n%s\n", line);
			exit(1);
		}
		return (true);
	}
	else
		return (false);
}


bool
read_limc_file_line(LIMC *limc_line, FILE *limc_file)
{
	char line[1024];

	if (fgets(line, 1023, limc_file))
	{
		// calibration_file.txt line:
		// x_origin   y_origin   map_cell_x   map_cell_y   laser_id   laser_size_in_the_floor   intensity
		if (sscanf(line, "%d %d %d %d %d %d %d", &(limc_line->mx), &(limc_line->my), &(limc_line->cx), &(limc_line->cy),
				&(limc_line->laser), &(limc_line->ray_size), &(limc_line->intensity)) != 7)
		{
			printf("Could read the limc_line->mx, limc_line->my, limc_line->cx, limc_line->cy, limc_line->laser, limc_line->ray_size, limc_line->intensity from line:\n%s\n", line);
			exit(1);
		}
		return (true);
	}
	else
	{
		*limc_line = INVALID_LIMC;
		return (false);
	}
}


bool
different_mc(MC next_mc_line, MC mc_line)
{
	if (next_mc_line.mx == mc_line.mx && next_mc_line.my == mc_line.my && next_mc_line.cx == mc_line.cx && next_mc_line.cy == mc_line.cy)
		return (false);
	else
		return (true);
}


bool
different_mc(LIMC limc_line, MC mc_line)
{
	if (limc_line.mx == mc_line.mx && limc_line.my == mc_line.my && limc_line.cx == mc_line.cx && limc_line.cy == mc_line.cy)
		return (false);
	else
		return (true);
}

#ifdef USE_RAY_SIZE
bool
different_limc(LIMC limc_line, LIMC mc_line)
{
	if (limc_line.mx == mc_line.mx && limc_line.my == mc_line.my && limc_line.cx == mc_line.cx && limc_line.cy == mc_line.cy &&
		limc_line.laser == mc_line.laser && limc_line.ray_size == mc_line.ray_size && limc_line.intensity == mc_line.intensity)
		return (false);
	else
		return (true);
}


#else
bool
different_limc(LIMC limc_line, LIMC mc_line)
{
	if (limc_line.mx == mc_line.mx && limc_line.my == mc_line.my && limc_line.cx == mc_line.cx && limc_line.cy == mc_line.cy &&
		limc_line.laser == mc_line.laser && limc_line.intensity == mc_line.intensity)
		return (false);
	else
		return (true);
}
#endif



bool
different_map_cell(LIMC limc_line, LIMC mc_line)
{
	if (limc_line.mx == mc_line.mx && limc_line.my == mc_line.my && limc_line.cx == mc_line.cx && limc_line.cy == mc_line.cy)
		return (false);
	else
		return (true);
}


MC *
compute_mc_index(char *mc, size_t &mc_index_size)
{
	FILE *mc_file;

	if ((mc_file = fopen(mc, "r")) == NULL)
	{
		printf("Could not open input file %s!\n", mc);
		exit(1);
	}

	MC mc_line;
	if (!read_mc_file_line(&mc_line, mc_file))
	{
		printf("Input file %s is empty!\n", mc);
		exit(1);
	}

	MC *mc_file_index = (MC *) malloc(sizeof(MC));
	mc_index_size = 0;
	mc_file_index[mc_index_size] = mc_line;
	mc_index_size++;

	MC next_mc_line;
	while (read_mc_file_line(&next_mc_line, mc_file))
	{
		if (different_mc(next_mc_line, mc_line))
		{
			mc_line = next_mc_line;
			mc_file_index = (MC *) realloc((void *) mc_file_index, (mc_index_size + 1) * sizeof(MC));
			if (!mc_file_index)
			{
				printf("Could allocate memory in compute_mc_index()\n");
				exit(1);
			}
			mc_file_index[mc_index_size] = mc_line;
			mc_index_size++;
		}
	}
	fclose(mc_file);

	return (mc_file_index);
}

#ifdef USE_RAY_SIZE
void
average_non_filled_table_positions(FILE *calibration_file_bin)
{
	float table[32][10][256];

	for (int laser = 0; laser < 32; laser++)
	{
		for (int ray_size = 0; ray_size < 10; ray_size++)
		{
			for (int intensity = 0; intensity < 256; intensity++)
			{
				table[laser][ray_size][intensity] = (double) (temp_table[laser][ray_size][intensity].accumulated_intensity) / (double) (temp_table[laser][ray_size][intensity].count);
				if (temp_table[laser][ray_size][intensity].count != 0)
					fprintf(calibration_file_bin, "%02d %d %03d %2.2f %ld %ld\n", laser, ray_size, intensity,
							table[laser][ray_size][intensity],
							temp_table[laser][ray_size][intensity].accumulated_intensity, temp_table[laser][ray_size][intensity].count);
				else
					fprintf(calibration_file_bin, "%02d %d %03d 00.00 %ld %ld\n", laser, ray_size, intensity,
							temp_table[laser][ray_size][intensity].accumulated_intensity, temp_table[laser][ray_size][intensity].count);
			}
		}
	}
}


#else
void
average_non_filled_table_positions(FILE *calibration_file_bin)
{
	float table[32][256];

	for (int laser = 0; laser < 32; laser++)
	{
		for (int intensity = 0; intensity < 256; intensity++)
		{
			table[laser][intensity] = (double) (temp_table[laser][intensity].accumulated_intensity) / (double) (temp_table[laser][intensity].count);
			if (temp_table[laser][intensity].count != 0)
				fprintf(calibration_file_bin, "%02d %03d %2.2f %ld %ld\n", laser, intensity,
						table[laser][intensity],
						temp_table[laser][intensity].accumulated_intensity, temp_table[laser][intensity].count);
			else
				fprintf(calibration_file_bin, "%02d %03d 00.00 %ld %ld\n", laser, intensity,
						temp_table[laser][intensity].accumulated_intensity, temp_table[laser][intensity].count);
		}
	}
}
#endif


bool
mc_a_gt_mc_b(MC a, MC b)
{
	if (a.mx > b.mx)
	{
		return (true);
	}
	else if (a.mx == b.mx)
	{
		if (a.my > b.my)
		{
			return (true);
		}
		else if (a.my == b.my)
		{
			if (a.cx > b.cx)
			{
				return (true);
			}
			else if (a.cx == b.cx)
			{
				if (a.cy > b.cy)
					return (true);
				else
					return (false);
			}
			else
				return (false);
		}
		else
			return (false);
	}
	else
		return (false);
}


int
compare_function(const void *a, const void *b)
{
	MC mc_a = *((MC *) a);
	MC mc_b = *((MC *) b);

	if (mc_a_gt_mc_b(mc_a, mc_b)) // >
		return (1);
	else if (different_mc(mc_a, mc_b)) // !=
		return (-1);
	else
		return (0); // <
}


LIMC
find_first_same_mc(LIMC limc_line, MC *mc_index, size_t mc_index_size, FILE *mc_file)
{
	MC mc_line = {limc_line.mx, limc_line.my, limc_line.cx, limc_line.cy, 0};
	MC *same_mc = (MC *) bsearch((const void *) &mc_line, (const void *) mc_index, mc_index_size, sizeof(MC), compare_function);

	if (!same_mc)
	{
		printf("Error. Could not find samc_mc in find_first_same_mc()\n");
		exit(1);
	}

	fseek(mc_file, same_mc->file_offset, SEEK_SET);
	LIMC limc_line_from_mc;
	read_limc_file_line(&limc_line_from_mc, mc_file);

	return (limc_line_from_mc);
}


void
compute_calibration_table(char *limc, char *mc, FILE *calibration_file_bin)
{
	// L = laser, I = intensity, M = map, C = cell
	// limc    - Todas as celulas (M C) em que o laser L teve a intensidade I-> Arquivo ordenado por L I M C, nesta sequencia
	// mc      - Todas as celulas (M C) atingidas por laser -> Arquivo ordenado por M C, nesta sequencia
	//   (i)   - Para cada linha do arquivo limc, examinar cada linha do arquivo mc a partir da primeira ocorrencia de M C (estao em ordem de M C)
	//   (ii)  - Acumular em table[L][I].mean_intensity a intensidade, I, dos lasers no arquivo mc que NAO sao iguais ao laser L mas que recairam na mesma celula M C
	//   (iii) - Contar em table[L][I].count quantas acumulacoes ocorreram
	//   (iv)  - Ao fim de um L I, passar para o proximo usando o mesmo procedimento
	//
	// O arquivo mc tem que ter um indice para cada linha M C para facilitar o passo (i) acima. Para isso:
	//   (i)   - Ler mc e ir preenchendo um vetor de indice com a posicao inicial de cada linha com um novo M C
	//   (ii)  - Fazer buscas neste vetor usando uma combinação de M e C
	// Para (i), usar ftell para pegar a posicao corrente do arquivo e fseek para ir para uma posicao.
	// Para (ii), usar a busca binaria bsaerch() da biblioteca de C++ (C)

	printf("Computing MC file index...\n");
	size_t mc_index_size;
	MC *mc_index = compute_mc_index(mc, mc_index_size);
	printf("The MC file index size = %ld.\n", mc_index_size);

	FILE *limc_file;
	if ((limc_file = fopen(limc, "r")) == NULL)
	{
		printf("Could not open input file %s!\n", limc);
		exit(1);
	}

	// Read first limc line
	LIMC next_limc_line;
	if (!read_limc_file_line(&next_limc_line, limc_file))
	{
		printf("Input file %s is empty!\n", limc);
		exit(1);
	}

	FILE *mc_file = fopen(mc, "r");

	LIMC limc_line = INVALID_LIMC;
	int limc_lines_processed = 0;
	do
	{
		if (different_limc(limc_line, next_limc_line))
		{
			limc_line = next_limc_line;
			LIMC limc_line_from_mc = find_first_same_mc(limc_line, mc_index, mc_index_size, mc_file);
			while (!different_map_cell(limc_line, limc_line_from_mc))
			{
				if (limc_line.laser != limc_line_from_mc.laser)
				{
					#ifdef USE_RAY_SIZE
					temp_table[limc_line.laser][limc_line.ray_size][limc_line.intensity].accumulated_intensity += limc_line_from_mc.intensity;
					temp_table[limc_line.laser][limc_line.ray_size][limc_line.intensity].count += 1;
					#else
					temp_table[limc_line.laser][limc_line.intensity].accumulated_intensity += limc_line_from_mc.intensity;
					temp_table[limc_line.laser][limc_line.intensity].count += 1;
					#endif
				}
				read_limc_file_line(&limc_line_from_mc, mc_file);
			}
		}
		limc_lines_processed++;
		if ((limc_lines_processed % 100000) == 0)
			printf("limc_lines_processed = %d\n", limc_lines_processed);
	} while (read_limc_file_line(&next_limc_line, limc_file));

	free(mc_index);
	fclose(limc_file);
	fclose(mc_file);

	average_non_filled_table_positions(calibration_file_bin);
}


int
main(int argc, char **argv)
{
	if (argc != 3)
	{
		printf("Error. Wrong number of parameters.\n Usage: compute_velodyne_calibration_table calibration_file.txt output_file.txt\n");
		exit(1);
	}

	if ((calibration_file = fopen(argv[1], "r")) == NULL)
	{
		printf("Could not open input file %s!\n", argv[1]);
		exit(1);
	}
	fclose(calibration_file);

	if ((calibration_file_bin = fopen(argv[2], "w")) == NULL)
	{
		printf("Could not open output file %s!\n", argv[2]);
		exit(1);
	}

	// calibration_file.txt line:
	// x_origin   y_origin   map_cell_x   map_cell_y   laser_id   laser_size_in_the_floor   intensity

	char command[1024];
	printf("Computing LIMC.txt file...\n");
	sprintf(command, "sort -n -k 5 -k 6 -k 7 -k 1 -k 2 -k 3 -k 4 %s > LIMC.txt", argv[1]);
	system(command);

	printf("Computing MC.txt file...\n");
	sprintf(command, "sort -n -k 1 -k 2 -k 3 -k 4 %s > MC.txt", argv[1]);
	system(command);

	#ifdef USE_RAY_SIZE
	memset(&temp_table, 0, 32 * 10 * 256 * sizeof(TEMP_TABLE));
	#else
	memset(&temp_table, 0, 32 * 256 * sizeof(TEMP_TABLE));
	#endif
	compute_calibration_table((char *) "LIMC.txt", (char *) "MC.txt", calibration_file_bin);

	fclose(calibration_file_bin);
}
