/* carmen_read.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <carmen/carmen.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Uso: ./carmen_read <arquivo.map>\n");
        return 1;
    }

    carmen_map_t map;
    memset(&map, 0, sizeof(carmen_map_t));

    if (carmen_map_read_gridmap_chunk(argv[1], &map) < 0) {
        fprintf(stderr, "ERRO: Falha ao ler %s\n", argv[1]);
        return 1;
    }

    printf("BRIDGE_HEADER_START\n");
    printf("cols %d\n", map.config.x_size);
    printf("rows %d\n", map.config.y_size);
    printf("res %f\n", map.config.resolution);
    printf("offset_x 0.0\n"); 
    printf("offset_y 0.0\n");
    printf("BRIDGE_DATA_START\n");

    int total = map.config.x_size * map.config.y_size;
    for (int i = 0; i < total; i++) {
        printf("%.4g ", map.complete_map[i]);
        if ((i + 1) % map.config.x_size == 0) printf("\n");
    }

    return 0;
}