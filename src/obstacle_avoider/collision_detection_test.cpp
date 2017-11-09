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