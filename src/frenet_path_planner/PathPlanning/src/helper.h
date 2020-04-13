#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");

  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }

  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Function which returns the closest way point within the given map
int ClosestWaypoint(double x,
                    double y,
                    vector<double> maps_x,
                    vector<double> maps_y)
{
  // Large number
	double closestLen = 100000;
	int closestWaypoint = 0;

  // Go through the way points and find the closest way point to the current
  // location of the car
	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

  // Return the closest way point index
	return closestWaypoint;
}

// Function which returns the next way point
int NextWaypoint(double x,
                 double y,
                 double theta,
                 vector<double> maps_x,
                 vector<double> maps_y,
                 vector<double> maps_dx,
                 vector<double> maps_dy)
{
  // Get the closest way point
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  // Get the corresponding co-ordinates
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	// Heading vector
  double hx = map_x-x;
  double hy = map_y-y;

  // Normal vector:
  double nx = maps_dx[closestWaypoint];
  double ny = maps_dy[closestWaypoint];

  // Vector into the direction of the road (perpendicular to the normal vector)
  double vx = -ny;
  double vy = nx;

  // If the inner product of v and h is positive then we are behind the waypoint
  // so we do not need to increment closestWaypoint.
  // Otherwise we are beyond the waypoint and we need to increment closestWaypoint.
  double inner = hx*vx+hy*vy;
  if (inner < 0.0)
  {
    closestWaypoint++;
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x,
                         double y,
                         double theta,
                         vector<double> maps_x,
                         vector<double> maps_y,
                         vector<double> maps_dx,
                         vector<double> maps_dy)
{
  // Get the next way points
	int next_wp = NextWaypoint(x,y, theta, maps_x, maps_y, maps_dx, maps_dy);

  // Previous way point
  int prev_wp;
  prev_wp = next_wp - 1;

  // If the next way point is 0(circular), get the last way point in the vector
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size() - 1;
  }

  // Distance in x, y
	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
  // Difference between current x,y and previous way point
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// Find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y)/(n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

  // Get frenet d
	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // See if d value is positive or negative by comparing it to a center point
	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  // Adjust 'd' based on center reference
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// Calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

  // Return the 's' and 'd' co-ordinates
	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  // Initialize previous way point to be -1
	int prev_wp = -1;

	while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

  // Way point based on the previous way point
	int wp2 = (prev_wp + 1) % maps_x.size();
	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));

	// The x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi()/2;
	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x,y};
}

// Function returns true if lane change is needed from the current lane
bool LaneChangePossible(const vector<vector<double>> sensor_fusion,
                        const int prev_size,
                        const int lane,
                        const double car_s,
                        double &ref_vel)
{
  // Find ref_v to use for the respective lane
  double closestDist_s = 100000;

  // Flag to indicate whether a lane change is required
  bool change_lanes = false;

  // Go through the sensor fusion data of the ego car
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    // Check if the car is in the same lane as the ego car
    float d = sensor_fusion[i][6];
    if(d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2))
    {
      // Get the (vx, vy) of that car
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];

      // Speed of the car
      double check_speed = sqrt(vx*vx + vy*vy);

      // 's' position of the car
      double check_car_s = sensor_fusion[i][5];

      // Car's 's' position based on the speed
      check_car_s += ((double)prev_size*.02 * check_speed);

      // Check if 's' value of the car is greater than the ego car
      // but less than 's' gap required(30)
      if((check_car_s > car_s) &&
         ((check_car_s - car_s) < 30) &&
         ((check_car_s - car_s) < closestDist_s))
      {
        // Closest car found
        closestDist_s = (check_car_s - car_s);

        // The gap between the car is greater than 20
        if( (check_car_s - car_s) > 20)
        {
          // Match that cars speed
          ref_vel = check_speed * 2.237;
        }
        else
        {
          // Go slightly slower than the cars speed
          ref_vel = check_speed*2.237 - 5;
        }

        // Change lanes since the ego car is getting too close
        change_lanes = true;
      }
    }
  }

  return change_lanes;
}

void ChangeLane(const vector<vector<double>> sensor_fusion,
                const vector<double> map_waypoints_x,
                const int prev_size,
                const bool change_lanes,
                int &lane,
                const double car_s,
                const int next_wp,
                int &lane_change_wp)
{
  // Try to change lanes ego car is too close to car in front(calculated above)
  // NOTE: Make sure lane change is triggered only if there is a
  //       difference of more than 2 way points
  if(change_lanes && ((next_wp - lane_change_wp) % map_waypoints_x.size() > 2))
  {
    // Flag to track changed lanes
    bool changed_lanes = false;

    // First try to change to left lane(if not in left lane already)
    if(lane != 0 && !changed_lanes)
    {
      // Default to true for safe lane change
      bool lane_safe = true;

      for(int i = 0; i < sensor_fusion.size(); i++)
      {
        // Check for cars in left lane
        float d = sensor_fusion[i][6];
        if(d < (2 + 4*(lane - 1) + 2) && d > (2 + 4*(lane - 1) - 2))
        {
          // Get the (vx, vy) of that car
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];

          // Speed of the car
          double check_speed = sqrt(vx*vx + vy*vy);

          // Current 's' position of the car
          double check_car_s = sensor_fusion[i][5];

          // Car's s position based on the speed
          check_car_s+=((double)prev_size * .02 * check_speed);

          // Distance between the cars
          double dist_s = check_car_s - car_s;

          // If the distance is less than the buffer of 20 then
          // lane change is not safe
          if(dist_s < 20 && dist_s > -20)
          {
            lane_safe = false;
          }
        }
      }
      // Check if the flag for safe lane change is true
      if(lane_safe)
      {
        // Safe to change to left lane!
        changed_lanes = true;
        // Change lane value
        lane -= 1;
        // For lane change detection in the next iteration
        lane_change_wp = next_wp;
      }
    }

    // Try to change to right lane(if not in right lane already)
    if(lane != 2 && !changed_lanes)
    {
      // Default to true for safe for lane change
      bool lane_safe = true;
      for(int i = 0; i < sensor_fusion.size(); i++)
      {
        // Check for car in right lane
        float d = sensor_fusion[i][6];
        if(d < (2 + 4*(lane + 1) + 2) && d > (2 + 4*(lane + 1) - 2))
        {
          // Get the (vx, vy) of that car
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];

          // Speed of the car
          double check_speed = sqrt(vx*vx + vy*vy);

          // Current 's' position of the car based on sensor data
          double check_car_s = sensor_fusion[i][5];

          // Car's s position based on the speed and size of the previous path
          check_car_s += ((double)prev_size * .02 * check_speed);

          // Distance between the cars
          double dist_s = check_car_s - car_s;

          // If the distance is less than the buffer of 20 then
          // lane change is not safe
          if(dist_s < 20 && dist_s > -20)
          {
            lane_safe = false;
          }
        }
      }
      // Check if the flag for safe lane change is true
      if(lane_safe)
      {
        // Safe to change to right lane!
        changed_lanes = true;
        // Change lane value
        lane += 1;
        // For lane change detection in the next iteration
        lane_change_wp = next_wp;
      }
    }
  }
}

void GenerateWayPoints(vector<double> &ptsx,
                       vector<double> &ptsy,
                       const int prev_size,
                       const double ref_x,
                       const double car_x,
                       const double ref_y,
                       const double car_y,
                       const double ref_yaw,
                       const double car_yaw,
                       const double car_s,
                       const int lane,
                       const vector<double> previous_path_x,
                       const vector<double> previous_path_y,
                       const vector<double> map_waypoints_x,
                       const vector<double> map_waypoints_y,
                       const vector<double> map_waypoints_s
                      )
{
  // This is the first time the path planning algorithm is running
  if(prev_size < 2)
  {
    // Get the car's previous 'x' & 'y' using the yaw
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    // Add the previous and the current car positions to the vectors
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  // Not the first time the car is running
  else
  {
    // Add the previous 2 car positions to the vectors
    ptsx.push_back(previous_path_x[prev_size - 2]);
    ptsx.push_back(previous_path_x[prev_size - 1]);
    ptsy.push_back(previous_path_y[prev_size - 2]);
    ptsy.push_back(previous_path_y[prev_size - 1]);
  }

  // Get next 3 way points using the car's current 's' and
  // current lane position('d' is at the middle, preferable & safe)
  vector<double> next_wp0 = getXY(car_s+30, (2 + 4*lane),
                                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, (2 + 4*lane),
                                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, (2 + 4*lane),
                                  map_waypoints_s, map_waypoints_x, map_waypoints_y);

  // Push 3 more way points
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Shift car reference angle to 0 degrees
  // NOTE: The heading angle is taken care of by the simulator based
  //       on the path points generated
  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = ((shift_x * cos(0 - ref_yaw)) - (shift_y * sin(0 - ref_yaw)));
    ptsy[i] = ((shift_x * sin(0 - ref_yaw)) + (shift_y * cos(0 - ref_yaw)));
  }
}

void CreateSpline(const vector<double> ptsx,
                  const vector<double> ptsy,
                  vector<double> &next_x_vals,
                  vector<double> &next_y_vals,
                  const vector<double> previous_path_x,
                  const vector<double> previous_path_y,
                  const double ref_x,
                  const double car_x,
                  const double ref_y,
                  const double car_y,
                  const double ref_yaw,
                  const double car_yaw,
                  double &car_speed,
                  const double ref_vel
                 )
{
  // Create spline for car to follow based on the points(5 of them)
  tk::spline s;
  s.set_points(ptsx, ptsy);

  // Start from the previous path to allow for a smoother transition
  for(int i = 0; i < previous_path_x.size(); i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Target 'x' position(the way points are seperated by approximately 30m)
  double target_x = 30.0;
  // Get 'y' using the spline equation
  double target_y = s(target_x);

  // Target distance to reach
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
  double x_add_on = 0;

  // Populate remaining 50 points for the car to follow using the spline
  for (int i = 1; i <= 50 - previous_path_x.size(); i++)
  {
    // The car speed if below reference velocity
    if(car_speed < ref_vel)
    {
      car_speed+=.224;
    }
    // Car speed is above reference velocity
    else if(car_speed > ref_vel)
    {
      car_speed-=.224;
    }

    // Calculate the increments for 'x' based on the current speed
    // and distance remaining
    // NOTE: The car moves to next point every 0.02 seconds, so the
    //       distances are calibrated for that time frame
    double N = (target_dist / (.02*car_speed / 2.24));
    // New 'x' point
    double x_point = x_add_on + (target_x)/N;
    // New 'y' point using the spline
    double y_point = s(x_point);

    // Store reference point for next iteration
    x_add_on = x_point;

    // For converting from car co-ordinates to global co-ordinates
    double x_ref = x_point;
    double y_ref = y_point;

    // Global co-ordinates after conversion
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    // New global 'x' & 'y' points with increments added
    x_point += ref_x;
    y_point += ref_y;

    // Add the points to the vector
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}
