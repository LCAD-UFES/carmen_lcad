#if 0
#include "Catch_provide_main.h"

#include "collision_detection.h"

TEST_CASE("OBB collision using SAT", "[COLLISION]")
{
    /* Center at (0,0), dimensions W = 1.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb1;
    obb1.orientation = 0.0;
    obb1.linear_velocity = 0.0;
    obb1.length = 1.0;
    obb1.width = 1.0;
    obb1.object_pose.x = 0.0;
    obb1.object_pose.y = 0.0;

    /* Center at (2,0), dimensions W = 1.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb2;
    obb2.orientation = 0.0;
    obb2.linear_velocity = 0.0;
    obb2.length = 1.0;
    obb2.width = 1.0;
    obb2.object_pose.x = 2.0;
    obb2.object_pose.y = 0.0;

    /* Center at (0,1), dimensions W = 2.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb3;
    obb3.orientation = 0.0;
    obb3.linear_velocity = 0.0;
    obb3.length = 1.0;
    obb3.width = 2.0;
    obb3.object_pose.x = 0.0;
    obb3.object_pose.y = 1.0;

    /* Center at (0.5,0.5), dimensions W = 1.0 and L = 1.0, theta = 45° */
    carmen_oriented_bounding_box obb4;
    obb4.orientation = carmen_degrees_to_radians(45);
    obb4.linear_velocity = 0.0;
    obb4.length = 1.0;
    obb4.width = 1.0;
    obb4.object_pose.x = 0.5;
    obb4.object_pose.y = 0.5;

    /* Center at (0.86,0.86), dimensions W = 1.0 and L = 1.0, theta = 45° */
    carmen_oriented_bounding_box obb5;
    obb5.orientation = carmen_degrees_to_radians(45);
    obb5.linear_velocity = 0.0;
    obb5.length = 1.0;
    obb5.width = 1.0;
    obb5.object_pose.x = 0.86;
    obb5.object_pose.y = 0.86;


    REQUIRE(compute_collision_obb_obb(obb1, obb2) == Approx(0.0));
    REQUIRE(compute_collision_obb_obb(obb1, obb3) == Approx(1.25));
    REQUIRE(compute_collision_obb_obb(obb1, obb4) == Approx(1.0));
    REQUIRE(compute_collision_obb_obb(obb1, obb5) == Approx(0.0));

    REQUIRE(compute_collision_obb_obb(obb2, obb1) == Approx(0.0));
    REQUIRE(compute_collision_obb_obb(obb3, obb1) == Approx(1.25));
    REQUIRE(compute_collision_obb_obb(obb4, obb1) == Approx(1.0 + sqrt(2.0)/2.0));
    REQUIRE(compute_collision_obb_obb(obb5, obb1) == Approx(0.0));

}

TEST_CASE("Collision between lines", "[COLLISION]")
{

    carmen_point_t line1;
    line1.x = 1.5;
    line1.y = 1.5;
    line1.theta = 0.5;

    carmen_point_t line2;
    line2.x = 7.0;
    line2.y = 7.0;
    line2.theta = 0.5;

    // CASO 1 - LINHAS PARALELAS COM ORIENTACAO ARBITRARIA (NAO COLIDEM)
    REQUIRE(has_collision_between_lines(line1, line2) == 0);

    // CASO 2 - LINHAS COLIDEM ARBITRARIAMENTE
    line1.theta = 0.5;
    line2.theta = 0.3;
    REQUIRE(has_collision_between_lines(line1, line2) != 0);

    // CASO 3 - LINHAS COLIDEM, UMA DELAS EH VERTICAL
    line1.theta = 0.5;
    line2.theta = M_PI / 2.0;
    REQUIRE(has_collision_between_lines(line1, line2) != 0);

    // CASO 4 - LINHAS NAO COLIDEM, AS DUAS SAO VERTICAIS
    line1.theta = M_PI / 2.0;
    line2.theta = M_PI / 2.0;
    REQUIRE(has_collision_between_lines(line1, line2) == 0);

    // CASO 5 - LINHAS COLIDEM, UMA HORIZONTAL
    line1.theta = 0.0;
    line2.theta = 0.3;
    REQUIRE(has_collision_between_lines(line1, line2) != 0);

    // CASO 6 - LINHAS NAO COLIDEM, AS DUAS HORIZONTAIS
    line1.theta = 0.0;
    line2.theta = 0.0;
    REQUIRE(has_collision_between_lines(line1, line2) == 0);

    // CASO 7 - LINHAS COLIDEM, UMA VERTICAL A OUTRA HORIZONTAL
    line1.theta = 0;
    line2.theta = M_PI / 2.0;
    REQUIRE(has_collision_between_lines(line1, line2) != 0);


}


#include <stdint.h>
#include <math.h>

// #define STRUCT_TEST

#ifdef STRUCT_TEST
typedef struct {
    int num_objects;
    double cell_width;
    int grid_width;
    int grid_height;
    int num_objects_div_32;
    int32_t **rowBitArray;
    int32_t **columnBitArray;
    carmen_oriented_bounding_box *objects;

} carmen_uniform_collision_grid;

int32_t
GetBitIndex(int32_t x)
{
    return (int32_t)log2((double)x);
}

void 
InsertObjectIntoGrid(carmen_oriented_bounding_box pObject, int object_index, carmen_uniform_collision_grid *grid)
{
    carmen_test_alloc(grid); //sanity check

    // Compute the extent of grid cells the bounding sphere of A overlaps.
    // Test assumes objects have been inserted in all rows/columns overlapped
    float ooCellWidth = 1.0f / grid->cell_width;
    double radius = pObject.length + pObject.width;
    int x1 = (int)floorf((pObject.object_pose.x - radius) * ooCellWidth);
    int x2 = (int)floorf((pObject.object_pose.x + radius) * ooCellWidth);
    int y1 = (int)floorf((pObject.object_pose.y - radius) * ooCellWidth);
    int y2 = (int)floorf((pObject.object_pose.y + radius) * ooCellWidth);
    x1 = MAX(x1, 0);
    x2 = MAX(x2, 0);
    y1 = MAX(y1, 0);
    y2 = MAX(y2, 0);

    int i = ((object_index + 31) / 32) ;
    // Compute the merged (bitwise-or) bit array of all overlapped grid rows.
    // Ditto for all overlapped grid columns
    for (int y = y1; y <= y2; y++)
        grid->rowBitArray[y][i] |= (1 << object_index);
    for (int x = x1; x <= x2; x++)
        grid->columnBitArray[x][i] |= (1 << object_index);
}


carmen_uniform_collision_grid
construct_uniform_collision_grid(int num_objects, carmen_oriented_bounding_box *objects, int grid_width, int grid_height, double cell_width)
{
    carmen_uniform_collision_grid grid;

    grid.num_objects = num_objects;
    grid.cell_width = cell_width;
    grid.grid_width = grid_width;
    grid.grid_height = grid_height;

    grid.num_objects_div_32 = (num_objects + 31) / 32;

    grid.rowBitArray = (int32_t **)malloc(grid_height * sizeof(int32_t *));
    for(int i = 0; i < grid_height; i++)
        grid.rowBitArray[i] = (int32_t *)calloc(grid.num_objects_div_32, sizeof(int32_t));

    grid.columnBitArray = (int32_t **)malloc(grid_width * sizeof(int32_t *));
    for (int i = 0; i < grid_width; i++)
        grid.columnBitArray[i] = (int32_t *)calloc(grid.num_objects_div_32, sizeof(int32_t));

    grid.objects = (carmen_oriented_bounding_box *)malloc(num_objects * sizeof(carmen_oriented_bounding_box));
    for(int i = 0; i < num_objects; i++)
    {
        grid.objects[i] = objects[i];
        InsertObjectIntoGrid(objects[i], i, &grid);  
    }

    return grid;

}

double
TestObjectAgainstGrid(carmen_oriented_bounding_box pObject, carmen_uniform_collision_grid *grid)
{

    // Allocate temporary bit arrays for all objects and clear them
    int32_t mergedRowArray[grid->num_objects_div_32];
    int32_t mergedColumnArray[grid->num_objects_div_32];
    memset(mergedRowArray, 0, grid->num_objects_div_32 * sizeof(int32_t));
    memset(mergedColumnArray, 0, grid->num_objects_div_32 * sizeof(int32_t));

    // Compute the extent of grid cells the bounding sphere of A overlaps.
    // Test assumes objects have been inserted in all rows/columns overlapped
    float ooCellWidth = 1.0f / grid->cell_width;
    double radius = pObject.length + pObject.width;
    int x1 = (int)floorf((pObject.object_pose.x - radius) * ooCellWidth);
    int x2 = (int)floorf((pObject.object_pose.x + radius) * ooCellWidth);
    int y1 = (int)floorf((pObject.object_pose.y - radius) * ooCellWidth);
    int y2 = (int)floorf((pObject.object_pose.y + radius) * ooCellWidth);
    // assert(x1 >= 0 && y1 >= 0 && x2 < GRID_WIDTH && y2 < GRID_HEIGHT);
    x1 = MAX(x1, 0);
    x2 = MAX(x2, 0);
    y1 = MAX(y1, 0);
    y2 = MAX(y2, 0);

    // Compute the merged (bitwise-or’ed) bit array of all overlapped grid rows.
    // Ditto for all overlapped grid columns
    for (int y = y1; y <= y2; y++)
        for (int i = 0; i < grid->num_objects_div_32; i++)
            mergedRowArray[i] |= grid->rowBitArray[y][i];
    for (int x = x1; x <= x2; x++)
        for (int i = 0; i < grid->num_objects_div_32; i++)
            mergedColumnArray[i] |= grid->columnBitArray[x][i];
    double sum = 0;
    // Now go through the intersection of the merged bit arrays and collision test
    // those objects having their corresponding bit set
    for (int i = 0; i < grid->num_objects_div_32; i++)
    {
        int32_t objectsMask = mergedRowArray[i] & mergedColumnArray[i];
        while (objectsMask)
        {
            // Clears all but lowest bit set (eg. 01101010 -> 00000010)
            int32_t less = objectsMask - 1;
            int32_t singleObjectMask = (less | objectsMask) ^ less;

            // Get index number of set bit, test against corresponding object
            // (GetBitIndex(v) returns log_2(v), i.e. n such that 2 ∧ n = v)
            int32_t objectIndex = GetBitIndex(singleObjectMask) + i * 32;

            sum += compute_collision_obb_obb(pObject, grid->objects[objectIndex]);
            // Mask out tested object, and continue with any remaining objects
            objectsMask ^= singleObjectMask;
        }
    }
    // int* x = NULL;
    // x[5] = 10;
    return sum;
}

TEST_CASE("Spatial Hashing", "[COLLISION]")
{

    REQUIRE(GetBitIndex(4) == 2);
    REQUIRE(GetBitIndex(8) == 3);
    REQUIRE(GetBitIndex(16) == 4);

    /* Center at (0,0), dimensions W = 1.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb1;
    obb1.orientation = 0.0;
    obb1.linear_velocity = 0.0;
    obb1.length = 1.0;
    obb1.width = 1.0;
    obb1.object_pose.x = 0.0;
    obb1.object_pose.y = 0.0;

    /* Center at (2,0), dimensions W = 1.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb2;
    obb2.orientation = 0.0;
    obb2.linear_velocity = 0.0;
    obb2.length = 1.0;
    obb2.width = 1.0;
    obb2.object_pose.x = 2.0;
    obb2.object_pose.y = 0.0;

    /* Center at (0,1), dimensions W = 2.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb3;
    obb3.orientation = 0.0;
    obb3.linear_velocity = 0.0;
    obb3.length = 1.0;
    obb3.width = 2.0;
    obb3.object_pose.x = 0.0;
    obb3.object_pose.y = 1.0;

    /* Center at (0.5,0.5), dimensions W = 1.0 and L = 1.0, theta = 45° */
    carmen_oriented_bounding_box obb4;
    obb4.orientation = carmen_degrees_to_radians(45);
    obb4.linear_velocity = 0.0;
    obb4.length = 1.0;
    obb4.width = 1.0;
    obb4.object_pose.x = 0.5;
    obb4.object_pose.y = 0.5;

    /* Center at (0.86,0.86), dimensions W = 1.0 and L = 1.0, theta = 45° */
    carmen_oriented_bounding_box obb5;
    obb5.orientation = carmen_degrees_to_radians(45);
    obb5.linear_velocity = 0.0;
    obb5.length = 1.0;
    obb5.width = 1.0;
    obb5.object_pose.x = 0.86;
    obb5.object_pose.y = 0.86;

    carmen_oriented_bounding_box array_of_objects[4];
    array_of_objects[0] = obb2;
    array_of_objects[1] = obb3;
    array_of_objects[2] = obb4;
    array_of_objects[3] = obb5;

    carmen_uniform_collision_grid grid = construct_uniform_collision_grid(4, array_of_objects, 30, 30, 0.2);

    REQUIRE(TestObjectAgainstGrid(obb1, &grid) == Approx(2.25));
}

#else

#define NUM_OBJECTS 4
#define CELL_WIDTH 0.2

#define GRID_WIDTH 30
#define GRID_HEIGHT 30

const int NUM_OBJECTS_DIV_32 = (NUM_OBJECTS + 31) / 32; // Round up
int32_t rowBitArray[GRID_HEIGHT][NUM_OBJECTS_DIV_32];
int32_t columnBitArray[GRID_WIDTH][NUM_OBJECTS_DIV_32];
carmen_oriented_bounding_box array_of_objects[NUM_OBJECTS];

void print_array2()
{
    int rows = GRID_HEIGHT;
    int cols = NUM_OBJECTS_DIV_32 + 1;
    printf("\nr %d c %d\n", rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            printf("%d ", rowBitArray[i][j]);
        }
        printf("\n");
    }
}

int32_t
GetBitIndex(int32_t x)
{
    return (int32_t)log2((double)x);
}

void
InsertObjectIntoGrid(carmen_oriented_bounding_box pObject, int object_index)
{
    // Allocate temporary bit arrays for all objects and clear them
    int32_t mergedRowArray[NUM_OBJECTS_DIV_32];
    int32_t mergedColumnArray[NUM_OBJECTS_DIV_32];
    memset(mergedRowArray, 0, NUM_OBJECTS_DIV_32 * sizeof(int32_t));
    memset(mergedColumnArray, 0, NUM_OBJECTS_DIV_32 * sizeof(int32_t));

    // Compute the extent of grid cells the bounding sphere of A overlaps.
    // Test assumes objects have been inserted in all rows/columns overlapped
    float ooCellWidth = 1.0f / CELL_WIDTH;
    double radius = pObject.length + pObject.width;
    int x1 = (int)floorf((pObject.object_pose.x - radius) * ooCellWidth);
    int x2 = (int)floorf((pObject.object_pose.x + radius) * ooCellWidth);
    int y1 = (int)floorf((pObject.object_pose.y - radius) * ooCellWidth);
    int y2 = (int)floorf((pObject.object_pose.y + radius) * ooCellWidth);
    // assert(x1 >= 0 && y1 >= 0 && x2 < GRID_WIDTH && y2 < GRID_HEIGHT);
    x1 = MAX(x1, 0);
    x2 = MAX(x2, 0);
    y1 = MAX(y1, 0);
    y2 = MAX(y2, 0);

    // printf("\n%d, %d, %d, %d, %d\n", object_index, x1, x2, y1, y2);
    int i = (object_index / 32);
    // Compute the merged (bitwise-or’ed) bit array of all overlapped grid rows.
    // Ditto for all overlapped grid columns
    for (int y = y1; y <= y2; y++)
        rowBitArray[y][i] |= (1 << object_index);
    for (int x = x1; x <= x2; x++)
        columnBitArray[x][i] |= (1 << object_index);
}

double 
TestObjectAgainstGrid(carmen_oriented_bounding_box pObject, carmen_oriented_bounding_box *grid)
{
    // Allocate temporary bit arrays for all objects and clear them
    int32_t mergedRowArray[NUM_OBJECTS_DIV_32];
    int32_t mergedColumnArray[NUM_OBJECTS_DIV_32];
    memset(mergedRowArray, 0, NUM_OBJECTS_DIV_32 * sizeof(int32_t));
    memset(mergedColumnArray, 0, NUM_OBJECTS_DIV_32 * sizeof(int32_t));
    
    // Compute the extent of grid cells the bounding sphere of A overlaps.
    // Test assumes objects have been inserted in all rows/columns overlapped
    float ooCellWidth = 1.0f / CELL_WIDTH;
    double radius = pObject.length + pObject.width;
    int x1 = (int)floorf((pObject.object_pose.x - radius) * ooCellWidth);
    int x2 = (int)floorf((pObject.object_pose.x + radius) * ooCellWidth);
    int y1 = (int)floorf((pObject.object_pose.y - radius) * ooCellWidth);
    int y2 = (int)floorf((pObject.object_pose.y + radius) * ooCellWidth);
    // assert(x1 >= 0 && y1 >= 0 && x2 < GRID_WIDTH && y2 < GRID_HEIGHT);
    x1 = MAX(x1, 0);
    x2 = MAX(x2, 0);
    y1 = MAX(y1, 0);
    y2 = MAX(y2, 0);

    // Compute the merged (bitwise-or’ed) bit array of all overlapped grid rows.
    // Ditto for all overlapped grid columns
    for (int y = y1; y <= y2; y++)
        for (int i = 0; i < NUM_OBJECTS_DIV_32; i++)
            mergedRowArray[i] |= rowBitArray[y][i];
    for (int x = x1; x <= x2; x++)
        for (int i = 0; i < NUM_OBJECTS_DIV_32; i++)
            mergedColumnArray[i] |= columnBitArray[x][i];
    double sum = 0;
    // Now go through the intersection of the merged bit arrays and collision test
    // those objects having their corresponding bit set
    for (int i = 0; i < NUM_OBJECTS_DIV_32; i++) {
        int32_t objectsMask = mergedRowArray[i] & mergedColumnArray[i];
        while (objectsMask) {
            // Clears all but lowest bit set (eg. 01101010 -> 00000010)
            int32_t less = objectsMask - 1;
            int32_t singleObjectMask = (less | objectsMask) ^ less;

            // Get index number of set bit, test against corresponding object
            // (GetBitIndex(v) returns log_2(v), i.e. n such that 2 ∧ n = v)   
            int32_t objectIndex = GetBitIndex(singleObjectMask) + i * 32;

            sum +=compute_collision_obb_obb(pObject, grid[objectIndex]);
            // Mask out tested object, and continue with any remaining objects
            objectsMask ^= singleObjectMask;
        }
    }

    return sum;
}



TEST_CASE("Spatial Hashing", "[COLLISION]")
{

    REQUIRE(GetBitIndex(4) == 2);
    REQUIRE(GetBitIndex(8) == 3);
    REQUIRE(GetBitIndex(16) == 4);

    /* Center at (0,0), dimensions W = 1.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb1;
    obb1.orientation = 0.0;
    obb1.linear_velocity = 0.0;
    obb1.length = 1.0;
    obb1.width = 1.0;
    obb1.object_pose.x = 0.0;
    obb1.object_pose.y = 0.0;

    /* Center at (2,0), dimensions W = 1.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb2;
    obb2.orientation = 0.0;
    obb2.linear_velocity = 0.0;
    obb2.length = 1.0;
    obb2.width = 1.0;
    obb2.object_pose.x = 5.0;
    obb2.object_pose.y = 0.0;

    /* Center at (0,1), dimensions W = 2.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb3;
    obb3.orientation = 0.0;
    obb3.linear_velocity = 0.0;
    obb3.length = 1.0;
    obb3.width = 2.0;
    obb3.object_pose.x = 0.0;
    obb3.object_pose.y = 1.0;

    /* Center at (0.5,0.5), dimensions W = 1.0 and L = 1.0, theta = 45° */
    carmen_oriented_bounding_box obb4;
    obb4.orientation = carmen_degrees_to_radians(45);
    obb4.linear_velocity = 0.0;
    obb4.length = 1.0;
    obb4.width = 1.0;
    obb4.object_pose.x = 0.5;
    obb4.object_pose.y = 0.5;

    /* Center at (0.86,0.86), dimensions W = 1.0 and L = 1.0, theta = 45° */
    carmen_oriented_bounding_box obb5;
    obb5.orientation = carmen_degrees_to_radians(45);
    obb5.linear_velocity = 0.0;
    obb5.length = 1.0;
    obb5.width = 1.0;
    obb5.object_pose.x = 0.86;
    obb5.object_pose.y = 0.86;

    array_of_objects[0] = obb2;
    array_of_objects[1] = obb3;
    array_of_objects[2] = obb4;
    array_of_objects[3] = obb5;

    print_array2();
    InsertObjectIntoGrid(obb2, 0);
    print_array2();
    InsertObjectIntoGrid(obb3, 1);
    print_array2();
    InsertObjectIntoGrid(obb4, 2);
    print_array2();
    InsertObjectIntoGrid(obb5, 3);
    print_array2();

    // printf("\n\n%f\n\n", TestObjectAgainstGrid(obb1, array_of_objects));

    REQUIRE(TestObjectAgainstGrid(obb1, array_of_objects) == Approx(2.25));
}

#endif

#endif

#if 1

#include "collision_detection.h"
#include <stdint.h>
#include <math.h>

typedef struct
{
    int num_objects;
    double cell_width;
    int grid_width;
    int grid_height;
    int num_objects_div_32;
    int32_t **rowBitArray;
    int32_t **columnBitArray;
    carmen_oriented_bounding_box *objects;

} carmen_uniform_collision_grid;

int32_t
GetBitIndex(int32_t x)
{
    return (int32_t)log2((double)x);
}

void
print_array(int32_t** array, int rows, int cols)
{
    printf("\nr %d c %d\n", rows, cols);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            printf("%d ", array[i][j]);
        }
        printf("\n");
    }
}

void InsertObjectIntoGrid(carmen_oriented_bounding_box pObject, int object_index, carmen_uniform_collision_grid *grid)
{
    carmen_test_alloc(grid); //sanity check

    // Compute the extent of grid cells the bounding sphere of A overlaps.
    // Test assumes objects have been inserted in all rows/columns overlapped
    float ooCellWidth = 1.0f / grid->cell_width;
    double radius = pObject.length + pObject.width;
    int x1 = (int)floorf((pObject.object_pose.x - radius) * ooCellWidth);
    int x2 = (int)floorf((pObject.object_pose.x + radius) * ooCellWidth);
    int y1 = (int)floorf((pObject.object_pose.y - radius) * ooCellWidth);
    int y2 = (int)floorf((pObject.object_pose.y + radius) * ooCellWidth);
    x1 = MAX(x1, 0);
    x2 = MAX(x2, 0);
    y1 = MAX(y1, 0);
    y2 = MAX(y2, 0);

    int i = object_index / 32;
    // Compute the merged (bitwise-or) bit array of all overlapped grid rows.
    // Ditto for all overlapped grid columns
    for (int y = y1; y <= y2; y++)
        grid->rowBitArray[y][i] |= (1 << object_index);
    for (int x = x1; x <= x2; x++)
        grid->columnBitArray[x][i] |= (1 << object_index);
}

carmen_uniform_collision_grid
construct_uniform_collision_grid(int num_objects, carmen_oriented_bounding_box *objects, int grid_width, int grid_height, double cell_width)
{
    carmen_uniform_collision_grid grid;

    grid.num_objects = num_objects;
    grid.cell_width = cell_width;
    grid.grid_width = grid_width;
    grid.grid_height = grid_height;

    grid.num_objects_div_32 = (num_objects + 31) / 32;

    grid.rowBitArray = (int32_t **)malloc(grid_height * sizeof(int32_t *));
    for (int i = 0; i < grid_height; i++)
        grid.rowBitArray[i] = (int32_t *)calloc(grid.num_objects_div_32, sizeof(int32_t));

    grid.columnBitArray = (int32_t **)malloc(grid_width * sizeof(int32_t *));
    for (int i = 0; i < grid_width; i++)
        grid.columnBitArray[i] = (int32_t *)calloc(grid.num_objects_div_32, sizeof(int32_t));
   
    print_array(grid.columnBitArray, grid_width, grid.num_objects_div_32);
   
    grid.objects = (carmen_oriented_bounding_box *)malloc(num_objects * sizeof(carmen_oriented_bounding_box));
    for (int i = 0; i < num_objects; i++)
    {
        grid.objects[i] = objects[i];
        InsertObjectIntoGrid(objects[i], i, &grid);
        print_array(grid.columnBitArray, grid_width, grid.num_objects_div_32);
    }

    return grid;
}

double
TestObjectAgainstGrid(carmen_oriented_bounding_box pObject, carmen_uniform_collision_grid *grid)
{

    // Allocate temporary bit arrays for all objects and clear them
    int32_t *mergedRowArray = (int32_t *)calloc(grid->num_objects_div_32, sizeof(int32_t));
    int32_t *mergedColumnArray = (int32_t *)calloc(grid->num_objects_div_32, sizeof(int32_t));
    // memset(mergedRowArray, 0, grid->num_objects_div_32 * sizeof(int32_t));
    // memset(mergedColumnArray, 0, grid->num_objects_div_32 * sizeof(int32_t));

    // Compute the extent of grid cells the bounding sphere of A overlaps.
    // Test assumes objects have been inserted in all rows/columns overlapped
    float ooCellWidth = 1.0f / grid->cell_width;
    double radius = pObject.length + pObject.width;
    int x1 = (int)floorf((pObject.object_pose.x - radius) * ooCellWidth);
    int x2 = (int)floorf((pObject.object_pose.x + radius) * ooCellWidth);
    int y1 = (int)floorf((pObject.object_pose.y - radius) * ooCellWidth);
    int y2 = (int)floorf((pObject.object_pose.y + radius) * ooCellWidth);
    // assert(x1 >= 0 && y1 >= 0 && x2 < GRID_WIDTH && y2 < GRID_HEIGHT);
    x1 = MAX(x1, 0);
    x2 = MAX(x2, 0);
    y1 = MAX(y1, 0);
    y2 = MAX(y2, 0);

    // Compute the merged (bitwise-or’ed) bit array of all overlapped grid rows.
    // Ditto for all overlapped grid columns
    for (int y = y1; y <= y2; y++)
        for (int i = 0; i < grid->num_objects_div_32; i++)
            mergedRowArray[i] |= grid->rowBitArray[y][i];
    for (int x = x1; x <= x2; x++)
        for (int i = 0; i < grid->num_objects_div_32; i++)
            mergedColumnArray[i] |= grid->columnBitArray[x][i];
    double sum = 0;
    // Now go through the intersection of the merged bit arrays and collision test
    // those objects having their corresponding bit set
    for (int i = 0; i < grid->num_objects_div_32; i++)
    {
        int32_t objectsMask = mergedRowArray[i] & mergedColumnArray[i];
        while (objectsMask)
        {
            // Clears all but lowest bit set (eg. 01101010 -> 00000010)
            int32_t less = objectsMask - 1;
            int32_t singleObjectMask = (less | objectsMask) ^ less;

            // Get index number of set bit, test against corresponding object
            // (GetBitIndex(v) returns log_2(v), i.e. n such that 2 ∧ n = v)
            int32_t objectIndex = GetBitIndex(singleObjectMask) + i * 32;

            sum += compute_collision_obb_obb(pObject, grid->objects[objectIndex]);
            // Mask out tested object, and continue with any remaining objects
            objectsMask ^= singleObjectMask;
        }
    }

    return sum;
}

int 
main()
{

    /* Center at (0,0), dimensions W = 1.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb1;
    obb1.orientation = 0.0;
    obb1.linear_velocity = 0.0;
    obb1.length = 1.0;
    obb1.width = 1.0;
    obb1.object_pose.x = 0.0;
    obb1.object_pose.y = 0.0;

    /* Center at (2,0), dimensions W = 1.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb2;
    obb2.orientation = 0.0;
    obb2.linear_velocity = 0.0;
    obb2.length = 1.0;
    obb2.width = 1.0;
    obb2.object_pose.x = 2.0;
    obb2.object_pose.y = 0.0;

    /* Center at (0,1), dimensions W = 2.0 and L = 1.0, theta = 0.0 */
    carmen_oriented_bounding_box obb3;
    obb3.orientation = 0.0;
    obb3.linear_velocity = 0.0;
    obb3.length = 1.0;
    obb3.width = 2.0;
    obb3.object_pose.x = 0.0;
    obb3.object_pose.y = 1.0;

    /* Center at (0.5,0.5), dimensions W = 1.0 and L = 1.0, theta = 45° */
    carmen_oriented_bounding_box obb4;
    obb4.orientation = carmen_degrees_to_radians(45);
    obb4.linear_velocity = 0.0;
    obb4.length = 1.0;
    obb4.width = 1.0;
    obb4.object_pose.x = 0.5;
    obb4.object_pose.y = 0.5;

    /* Center at (0.86,0.86), dimensions W = 1.0 and L = 1.0, theta = 45° */
    carmen_oriented_bounding_box obb5;
    obb5.orientation = carmen_degrees_to_radians(45);
    obb5.linear_velocity = 0.0;
    obb5.length = 1.0;
    obb5.width = 1.0;
    obb5.object_pose.x = 0.86;
    obb5.object_pose.y = 0.86;

    carmen_oriented_bounding_box array_of_objects[4];
    array_of_objects[0] = obb2;
    array_of_objects[1] = obb3;
    array_of_objects[2] = obb4;
    array_of_objects[3] = obb5;

    carmen_uniform_collision_grid grid = construct_uniform_collision_grid(4, array_of_objects, 30, 30, 0.2);

    double magic_special_value = TestObjectAgainstGrid(obb1, &grid);


    printf("\n\nMagic special value: %lf\n\n", magic_special_value);
}

#endif