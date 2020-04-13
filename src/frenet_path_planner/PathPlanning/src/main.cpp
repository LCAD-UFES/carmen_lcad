#include "helper.h"

int main()
{
  uWS::Hub h;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  string map_file_ = "../data/highway_map.csv";
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // Start in lane 1(Middle lane)
  int lane = 1;
  int lane_change_wp = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&lane_change_wp]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // Store the JSON data into local variables
          double car_x = j[1]["x"];
        	double car_y = j[1]["y"];
        	double car_s = j[1]["s"];
        	double car_d = j[1]["d"];
        	double car_yaw = j[1]["yaw"];
        	double car_speed = j[1]["speed"];
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];
        	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          // Reference velocity of 49.5 mph(below the 50mph speed limit)
        	double ref_vel = 49.5;

          // Get the previous path size
        	int prev_size = previous_path_x.size();

          // Initialize variables
        	int next_wp = -1;
        	double ref_x = car_x;
        	double ref_y = car_y;
        	double ref_yaw = deg2rad(car_yaw);

          // Check if the the size of the previous path is below 2
          // NOTE: This is the first time the path planner is going to run
        	if(prev_size < 2)
        	{
            // Get the next way point using the ego car's (x, y, yaw)
        		next_wp = NextWaypoint(ref_x, ref_y, ref_yaw,
                                   map_waypoints_x, map_waypoints_y,
                                   map_waypoints_dx, map_waypoints_dy);
        	}
          // Not the first time the path planner is running
        	else
        	{
            // Get the last 'x' from the previous path
    				ref_x = previous_path_x[prev_size - 1];
            // Get the previous to last 'x' from the previous path
    				double ref_x_prev = previous_path_x[prev_size - 2];
            // Get the last 'y' from the previous path
    				ref_y = previous_path_y[prev_size - 1];
            // Get the previous to last 'y' from the previous path
    				double ref_y_prev = previous_path_y[prev_size - 2];
            // Get the last heading direction of the previous path
    				ref_yaw = atan2((ref_y-ref_y_prev), (ref_x - ref_x_prev));

            // Get the next way point using the ego car's (x, y, yaw)
    				next_wp = NextWaypoint(ref_x, ref_y, ref_yaw,
                                   map_waypoints_x, map_waypoints_y,
                                   map_waypoints_dx, map_waypoints_dy);

            // Get the previous path end 's'
    				car_s = end_path_s;

            // Get the car speed from the previous path(1m/s = 2.23694mph)
    				car_speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) +
                              (ref_y - ref_y_prev) * (ref_y - ref_y_prev))/.02) * 2.237;
        	}

          // Call function to check if lane change is possible and required
          bool change_lanes = LaneChangePossible(sensor_fusion, prev_size, lane, car_s, ref_vel);

          // Based on the answer returned from the previous step change lanes
          ChangeLane(sensor_fusion, map_waypoints_x, prev_size, change_lanes,
                     lane, car_s, next_wp, lane_change_wp);

          // Vectors to store car's x and y
          vector<double> ptsx;
          vector<double> ptsy;

          // Generate the way points for creating the spline below
          GenerateWayPoints(ptsx, ptsy, prev_size,
                            ref_x, car_x, ref_y, car_y,
                            ref_yaw, car_yaw, car_s, lane,
                            previous_path_x, previous_path_y,
                            map_waypoints_x, map_waypoints_y,
                            map_waypoints_s);

          // Vector of co-ordinates for the car to follow
        	vector<double> next_x_vals;
        	vector<double> next_y_vals;

          // Call function to create a trajectory for the car to follow
          CreateSpline(ptsx, ptsy, next_x_vals, next_y_vals,
                       previous_path_x, previous_path_y,
                       ref_x, car_x, ref_y, car_y,
                       ref_yaw, car_yaw,
                       car_speed, ref_vel);

        	json msgJson;
        	msgJson["next_x"] = next_x_vals;
        	msgJson["next_y"] = next_y_vals;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";

        	// This_thread::sleep_for(chrono::milliseconds(1000));
        	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req,
                     char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  // Listen to port 4567 for communication
  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  // Run server
  h.run();
}
