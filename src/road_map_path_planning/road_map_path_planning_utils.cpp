#include "road_map_path_planning_utils.h"


t_graph
convert_from_lon_lat_nodes_to_utm_nodes(t_graph graph)
{
	int route_size;
	int *route;
	FILE *r = fopen("route.txt", "r");
	fscanf(r, "%d\n", &route_size);
	route = (int*) malloc (route_size*sizeof(int));
	int cont = 0;
	while(cont<route_size)
	{
		fscanf(r, "%d\n", &route[cont]);
		cont++;
	}


	for(int i = 0; i < graph.qtd_nodes; i++)
	{
																			  //gps_gpgga->sea_level
		Gdc_Coord_3d gdc = Gdc_Coord_3d(graph.nodes[i].lat, graph.nodes[i].lon, 0.0);
		Utm_Coord_3d utm;
		Gdc_To_Utm_Converter::Init();
		Gdc_To_Utm_Converter::Convert(gdc,utm);
		graph.nodes[i].point.x = utm.y;
		graph.nodes[i].point.y = -utm.x;
	}

	for (int i = 0; i < route_size-1; i++)
	{
		double theta = carmen_normalize_theta(atan2((graph.nodes[route[i+1]].point.y - graph.nodes[route[i]].point.y), (graph.nodes[route[i+1]].point.x - graph.nodes[route[i]].point.x)));
		//printf("%lf %lf %lf %lf\n", graph.nodes[route[i]].point.x, graph.nodes[route[i]].point.y, cos(theta)*3, sin(theta)*3); //mostra os vetor de direcao
		printf("%lf %lf %lf\n", graph.nodes[route[i]].point.x, graph.nodes[route[i]].point.y, theta);
	}


	return graph;
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

	system(python_command.c_str());

	graph = read_graph_file("graph.txt");

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


}
