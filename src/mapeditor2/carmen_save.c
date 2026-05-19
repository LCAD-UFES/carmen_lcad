#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <carmen/carmen.h>

void allocate_map(carmen_map_t *map, int x_size, int y_size, double res) {
    map->config.x_size = x_size;
    map->config.y_size = y_size;
    map->config.resolution = res;
    
    map->complete_map = (double *)malloc(x_size * y_size * sizeof(double));
    map->map = (double **)malloc(x_size * sizeof(double *));
    
    if (!map->map || !map->complete_map) exit(1);

    for (int i = 0; i < x_size; i++) {
        map->map[i] = &map->complete_map[i * y_size];
    }
}

int main(int argc, char **argv) {
    if (argc < 5) {
        fprintf(stderr, "Uso: ./carmen_save <arquivo> <cols> <rows> <res>\n");
        return 1;
    }

    char *filename = argv[1];
    int cols = atoi(argv[2]);
    int rows = atoi(argv[3]);
    double res = atof(argv[4]);

    carmen_map_t map;
    memset(&map, 0, sizeof(carmen_map_t));
    allocate_map(&map, cols, rows, res);

    // Lê dados do stdin
    int total = cols * rows;
    for (int i = 0; i < total; i++) {
        if (scanf("%lf", &map.complete_map[i]) != 1) break;
    }

    carmen_FILE *fp = carmen_fopen(filename, "w");
    if (fp == NULL) return 1;

    // --- CORREÇÃO: GRAVAR CABEÇALHOS IGUAL AO EDITOR ORIGINAL ---
    // Isso garante que o carmen_read consiga ler o arquivo depois
    
    // 1. Escreve info basica
    carmen_map_write_comment_chunk(fp, cols, rows, res, (char*) "map_editor_py", (char*) "user");
    
    // 2. Escreve ID do mapa (Essencial para validade)
    carmen_map_write_id(fp);
    
    // 3. Escreve quem criou
    carmen_map_write_creator_chunk(fp, (char*) "map_editor_py", (char*) "user");

    // 4. Escreve os dados do grid
    if (carmen_map_write_gridmap_chunk(fp, map.map, cols, rows, res) < 0) {
         carmen_fclose(fp);
         return 1;
    }

    carmen_fclose(fp);
    free(map.complete_map);
    free(map.map);
    
    printf("OK\n");
    return 0;
}