#include "road_map_path_planning_utils.h"


t_route
load_route_from_file(string route_filename)
{
	t_route r;
	FILE *f = fopen(route_filename.c_str(), "r");
	fscanf(f, "%d\n", &r.route_size);
	r.route = (int*) malloc (r.route_size*sizeof(int));
	int cont = 0;
	while(cont < r.route_size)
	{
		fscanf(f, "%d\n", &r.route[cont]);
		cont++;
	}
	return r;
}


void
print_route_coordinates_in_carmen_coordinates(t_route r, t_graph graph)
{
	for (int i = 0; i < r.route_size-1; i++)
	{
		double theta = carmen_normalize_theta(atan2((graph.nodes[r.route[i+1]].point.y - graph.nodes[r.route[i]].point.y), (graph.nodes[r.route[i+1]].point.x - graph.nodes[r.route[i]].point.x)));
		//printf("%lf %lf %lf %lf\n", graph.nodes[r.route[i]].point.x, graph.nodes[r.route[i]].point.y, cos(theta)*3, sin(theta)*3); //mostra os vetores de direcao
		printf("%lf %lf %lf\n", graph.nodes[r.route[i]].point.x, graph.nodes[r.route[i]].point.y, theta);
	}
}


void
plot_state(t_forest forest, t_graph graph, vector<int> indices, int forest_index)
{
//	plot data Table - Last TCP - Optmizer tcp - Lane
	//Plot Optmizer step tcp and lane?


//	#define PAST_SIZE 300
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;


	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
//		fprintf(gnuplot_pipeMP, "set xrange [0:70]\n");
//		fprintf(gnuplot_pipeMP, "set yrange [-10:10]\n");
////		fprintf(gnuplot_pipe, "set y2range [-0.55:0.55]\n");
//		fprintf(gnuplot_pipeMP, "set xlabel 'senconds'\n");
//		fprintf(gnuplot_pipeMP, "set ylabel 'effort'\n");
////		fprintf(gnuplot_pipe, "set y2label 'phi (radians)'\n");
////		fprintf(gnuplot_pipe, "set ytics nomirror\n");
////		fprintf(gnuplot_pipe, "set y2tics\n");
//		fprintf(gnuplot_pipeMP, "set tics out\n");
	}

	FILE *gnuplot_data_graph = fopen("gnuplot_data_graph.txt", "w");
	FILE *gnuplot_data_rddf = fopen("gnuplot_data_rddf.txt", "w");
	FILE *gnuplot_data_closest = fopen("gnuplot_data_closest.txt", "w");

	for (int i = 0; i < graph.qtd_nodes; i++)
		fprintf(gnuplot_data_graph, "%lf %lf\n", graph.nodes[i].point.x, graph.nodes[i].point.y);

//	cout<<"Forest size: "<<forest.rddfs.size()<<endl;
	for (unsigned int i = 0; i < forest.rddfs.size(); i++)
		for (unsigned int j = 0; j < forest.rddfs[i].size(); j++)
			fprintf(gnuplot_data_rddf, "%lf %lf\n", forest.rddfs[i][j].pose.x, forest.rddfs[i][j].pose.y);

	for (unsigned int i = 0; i < indices.size(); i++)
		fprintf(gnuplot_data_closest, "%lf %lf\n", forest.rddfs[forest_index][indices[i]].pose.x, forest.rddfs[forest_index][indices[i]].pose.y);

	fclose(gnuplot_data_graph);
	fclose(gnuplot_data_rddf);
	fclose(gnuplot_data_closest);

//	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",0, -60.0, 0, 60.0);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_data_graph.txt' using 1:2 title 'OSM Graph',"
			"'./gnuplot_data_rddf.txt' using 1:2 w d title 'RDDF Points',"
			"'./gnuplot_data_closest.txt' using 1:2 title 'Closest Point'\n");

	fflush(gnuplot_pipeMP);
}


double
calc_theta_diff (double angle_x, double angle_y)
{
	return atan2(sin(angle_x-angle_y), cos(angle_x-angle_y));

}


void
knn (vector<int> &indices, vector<carmen_rddf_waypoint> rddf, double query_x, double query_y)
{
	vector<Point2f> cloud2d;
	Point2d p;
	vector<float> query;
	query.push_back(query_x);
	query.push_back(query_y);
	for (unsigned int i = 0; i < rddf.size(); i++)
	{
		p.x = rddf[i].pose.x;
		p.y = rddf[i].pose.y;
		cloud2d.push_back(p);
	}
	rddf.clear();

	flann::KDTreeIndexParams indexParams;
	flann::Index kdtree(Mat(cloud2d).reshape(1), indexParams);

	vector<int> ind;
	vector<float> dists;
	kdtree.knnSearch(query, ind, dists, 1);
	indices.push_back(ind[0]);
}


double
calc_theta (double x1, double y1, double x, double y)
{
	double theta = carmen_normalize_theta(atan2((y1 - y), (x1 - x)));
	return theta;
}


void
get_closest_points_from_osm_in_rddf (t_forest forest, t_graph graph, t_route r)
{
	//calcular o theta do primeiro ponto da rota  OK!
	//fazer knn do primeiro ponto da rota com todos os pontos do rddf 1 OK!
	//calcular a diferença angular do ponto da roda com ambos os rddfs: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
	//fazer knn do primeiro ponto da rota com todos os pontos do rddf n OK!
	//setar para o rddf com a menor diferença angular

//	vector<Point2f> cloud2d;
	vector<int> indices;
	int forest_index = 0;
	double minor_angle = 99999.9;
//	Point2d p;

	double theta_of_first_route_point = calc_theta(graph.nodes[r.route[1]].point.x, graph.nodes[r.route[1]].point.y, graph.nodes[r.route[0]].point.x,
			graph.nodes[r.route[0]].point.y);
//	cout<<"Theta "<<theta_of_first_route_point<<endl;

	for (unsigned int i = 0; i < forest.rddfs.size(); i++)
	{
		knn (indices, forest.rddfs[i], graph.nodes[r.route[0]].point.x, graph.nodes[r.route[0]].point.y);
//		cout<<"Theta Closest "<<i<<": "<<forest.rddfs[i][indices[0]].pose.theta<<endl;
		double theta_diff = calc_theta_diff(forest.rddfs[i][indices[0]].pose.theta, theta_of_first_route_point);
//		cout<<"if "<<abs(theta_diff)<<" < "<<minor_angle<<endl;
		if (abs(theta_diff) < minor_angle)
		{
//			cout<<"\ttroquei!"<<endl;
			minor_angle = abs(theta_diff);
			forest_index = i;
		}
//		cout<<indices[i]<<endl;
	}
//	cout<<"Chosen rddf in forest: "<<forest_index<<endl;

	indices.clear();
	for (int i = 0; i < r.route_size; i++)
	{
		knn (indices, forest.rddfs[forest_index], graph.nodes[r.route[i]].point.x, graph.nodes[r.route[i]].point.y);
	}


//	for (unsigned int i = 0; i < forest.rddfs.size(); i++)
//	{
//		for (unsigned int j = 0; j < forest.rddfs[1].size(); j++)
//		{
//			p.x = forest.rddfs[1][j].pose.x;
//			p.y = forest.rddfs[1][j].pose.y;
//			cloud2d.push_back(p);
//		}
//	}

//	flann::KDTreeIndexParams indexParams;
//	flann::Index kdtree(Mat(cloud2d).reshape(1), indexParams);

//	for (int i = 0; i < r.route_size; i++)
//	{
//		vector<float> query;
//		//query.push_back(graph.nodes[r.route[0]].point.x);
//		//query.push_back(graph.nodes[r.route[0]].point.y);
//		query.push_back(graph.nodes[r.route[i]].point.x);
//		query.push_back(graph.nodes[r.route[i]].point.y);
//		vector<int> ind;
//		vector<float> dists;
//		kdtree.knnSearch(query, ind, dists, 1);
//		indices.push_back(ind[0]);
//		//printf("Closest point of %lf x %lf is %lf x %lf\n", graph.nodes[0].point.x, graph.nodes[0].point.y, rddf_points[ind[0]].pose.x, rddf_points[ind[0]].pose.y);
//	}

//cout<<"Closest point of "<<graph.nodes[0].point.x<<" , "<<graph.nodes[0].point.y<<" is "<<rddf_points[indices[0]].pose.x<<" , "<<rddf_points[indices[0]].pose.y<<endl;
	plot_state (forest, graph, indices, forest_index);
	getchar();
}


t_graph
convert_from_lon_lat_nodes_to_utm_nodes(t_graph graph)
{
	for(int i = 0; i < graph.qtd_nodes; i++)
	{
																			  //gps_gpgga->sea_level
		Gdc_Coord_3d gdc = Gdc_Coord_3d(graph.nodes[i].lat, graph.nodes[i].lon, 0.0);
		Utm_Coord_3d utm;
		Gdc_To_Utm_Converter::Init();
		Gdc_To_Utm_Converter::Convert(gdc,utm);
		graph.nodes[i].point.x = utm.y;
		graph.nodes[i].point.y = -utm.x;
		//printf("%lf %lf\n", graph.nodes[i].point.x, graph.nodes[i].point.y);
	}
	return graph;
}


t_forest
load_rddf_files (t_forest rddf_points, vector <string> rddf_filename)
{
	FILE *fr;

	for (unsigned int i = 0; i < rddf_filename.size(); i++){
		fr = fopen(rddf_filename[i].c_str(), "r");
		if (fr == NULL)
		{
			cout<<"Error to open file: "<<rddf_filename[i]<<endl;
			exit(0);
		}
		else
		{
			vector <carmen_rddf_waypoint> rddf;
			carmen_rddf_waypoint w;
//			cout<<"Abri "<<rddf_filename[i];
			while (!feof(fr))
			{
				fscanf (fr, "%lf %lf %lf %lf %lf %lf\n", &w.pose.x, &w.pose.y, &w.pose.theta, &w.driver_velocity, &w.phi, &w.timestamp);
				rddf.push_back(w);
			}
			fclose (fr);
			rddf_points.rddfs.push_back(rddf);
//			cout<<" com linhas = "<<rddf_points.rddfs[i].size()<<endl;
		}

	}

	return rddf_points;

}


t_graph
read_graph_file(string graph_filename)
{
	t_graph graph;
	t_node node;
	t_edge edge;
	FILE *graph_file;
	int cont = 0;

	graph_file = fopen(graph_filename.c_str(), "r");
	fscanf(graph_file, "%d %d\n", &graph.qtd_nodes, &graph.qtd_edges);

	graph.nodes = (t_node*) malloc (graph.qtd_nodes * sizeof(t_node));
	graph.edges = (t_edge*) malloc (graph.qtd_edges * sizeof(t_edge));

	while (cont < graph.qtd_nodes)
	{
		fscanf(graph_file, "%d %lf %lf\n", &node.node_id, &node.lon, &node.lat);
		graph.nodes[node.node_id] = node;
		cont++;
	}
	cont = 0;

	while (cont < graph.qtd_edges)
	{
		fscanf(graph_file, "%d %d\n", &edge.u, &edge.v);
		graph.edges[cont] = edge;
		cont++;
	}
	return graph;
}


void
process_graph (string python_command)
{
//	+ " " + to_string(current_gps_info.latitude)
//									+ " " + to_string(current_gps_info.longitude)
	string s;
	t_graph graph;
	t_route route;
	t_forest rddf_points;
	s = "/home/pedro/carmen_lcad/data/rndf/rddf_log_volta_da_ufes-201903025.txt";
	vector <string> rddf_filename;
	rddf_filename.push_back(s);
	s.clear();
	s = "/home/pedro/carmen_lcad/data/rndf/rddf-log_volta_da_ufes-20190915-contrario.txt";
	rddf_filename.push_back(s);
	cout<<rddf_filename[0]<<endl<<rddf_filename[1]<<endl;


	system(python_command.c_str());
	system("clear");

	graph = read_graph_file("graph.txt");
	route = load_route_from_file("route.txt");
	rddf_points = load_rddf_files (rddf_points, rddf_filename);


//	printf("%d %d\n", graph.qtd_nodes, graph.qtd_edges);
//
//	for (int i = 0; i < graph.qtd_nodes; i++)
//	{
//		printf("%d %lf %lf\n", graph.nodes[i].node_id, graph.nodes[i].lon, graph.nodes[i].lat);
//	}
//
//	for (int i = 0; i < graph.qtd_edges; i++)
//	{
//		printf("%d %d\n", graph.edges[i].u, graph.edges[i].v);
//	}

	graph = convert_from_lon_lat_nodes_to_utm_nodes(graph);

	get_closest_points_from_osm_in_rddf (rddf_points, graph, route);

}
