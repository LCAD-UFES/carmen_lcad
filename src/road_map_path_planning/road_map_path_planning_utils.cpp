#include "road_map_path_planning_utils.h"


t_route
load_route_from_file(char *route_filename)
{
	t_route r;
	FILE *f = fopen(route_filename, "r");
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
plot_state(vector<carmen_rddf_waypoint> rddf_points, t_graph graph, vector<int> indices)
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

	for (unsigned int i = 0; i < graph.qtd_nodes; i++)
		fprintf(gnuplot_data_graph, "%lf %lf\n", graph.nodes[i].point.x, graph.nodes[i].point.y);

	for (unsigned int i = 0; i < rddf_points.size(); i++)
		fprintf(gnuplot_data_rddf, "%lf %lf\n", rddf_points[i].pose.x, rddf_points[i].pose.y);

	for (unsigned int i = 0; i < indices.size(); i++)
		fprintf(gnuplot_data_closest, "%lf %lf\n", rddf_points[indices[i]].pose.x, rddf_points[indices[i]].pose.y);

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


void
get_closest_points_from_osm_in_rddf (vector<carmen_rddf_waypoint> rddf_points, t_graph graph, t_route r)
{
	vector<Point2f> cloud2d;
	vector<int> indices;
	Point2d p;
	for (int i = 0; i < rddf_points.size(); i++)
	{
		p.x = rddf_points[i].pose.x;
		p.y = rddf_points[i].pose.y;
		cloud2d.push_back(p);
	}

	flann::KDTreeIndexParams indexParams;
	flann::Index kdtree(Mat(cloud2d).reshape(1), indexParams);

	for (int i = 0; i < r.route_size; i++)
	{
		vector<float> query;
		//query.push_back(graph.nodes[r.route[0]].point.x);
		//query.push_back(graph.nodes[r.route[0]].point.y);
		query.push_back(graph.nodes[r.route[i]].point.x);
		query.push_back(graph.nodes[r.route[i]].point.y);
		vector<int> ind;
		vector<float> dists;
		kdtree.knnSearch(query, ind, dists, 1);
		indices.push_back(ind[0]);
		printf("Closest point of %lf x %lf is %lf x %lf\n", graph.nodes[0].point.x, graph.nodes[0].point.y, rddf_points[ind[0]].pose.x, rddf_points[ind[0]].pose.y);
	}

//cout<<"Closest point of "<<graph.nodes[0].point.x<<" , "<<graph.nodes[0].point.y<<" is "<<rddf_points[indices[0]].pose.x<<" , "<<rddf_points[indices[0]].pose.y<<endl;
	plot_state (rddf_points, graph, indices);
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


void
load_rddf_file (vector<carmen_rddf_waypoint> &rddf_points, string rddf_filename)
{
	FILE *fr;
	carmen_rddf_waypoint w;
	fr = fopen(rddf_filename.c_str(), "r");
	if (fr == NULL)
	{
		cout<<"Error to open file: "<<rddf_filename<<endl;
		exit(0);
	}
	else
	{
		while (!feof(fr))
		{
			fscanf (fr, "%lf %lf %lf %lf %lf %lf\n", &w.pose.x, &w.pose.y, &w.pose.theta, &w.driver_velocity, &w.phi, &w.timestamp);
			rddf_points.push_back(w);
		}
		fclose (fr);
	}

}


t_graph
read_graph_file(char *graph_filename)
{
	t_graph graph;
	t_node node;
	t_edge edge;
	FILE *graph_file;
	int cont = 0;

	graph_file = fopen(graph_filename, "r");
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
	t_graph graph;
	t_route route;
	vector<carmen_rddf_waypoint> rddf_points;
	string rddf_filename = "/home/pedro/carmen_lcad/data/rndf/rddf_log_volta_da_ufes-201903025.txt";


	system(python_command.c_str());
	system("clear");

	graph = read_graph_file("graph.txt");
	route = load_route_from_file("route.txt");
	load_rddf_file (rddf_points, rddf_filename);

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
