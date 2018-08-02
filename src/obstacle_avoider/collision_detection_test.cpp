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


TEST_CASE("Index test", "[COLLISION]")
{
  for (uint32_t i = 0; i < 32; i++) {
    REQUIRE(GetBitIndex(pow(2, i)) == i);
  }
}


TEST_CASE("Spatial Hashing", "[COLLISION]")
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

    REQUIRE(TestObjectAgainstGrid(obb1, &grid) == Approx(2.25));
}

