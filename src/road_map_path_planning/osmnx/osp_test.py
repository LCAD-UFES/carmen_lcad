#https://github.com/gboeing/osmnx-examples

import sys
import networkx as nx
import osmnx as ox
import requests
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors
import numpy as np
from sklearn.neighbors import KDTree
ox.config(use_cache=True, log_console=True)
ox.__version__

qtd_clicks = 0
click_lon = 0
click_lat = 0

clicks = []

def save_graph_file (nodes, edges):
    #matrix = [[0 for i in range(len(nodes))] for j in range(2)] 
    
    #for i in range(len(nodes)):
    #    matrix[i][0] = 
    
    dict = {}    
    f = open("graph.txt", 'w')
    texto = str(len(nodes)) + " " + str(len(edges)) + "\n"
    f.write(texto)
    i = 0
    for osmid, lon, lat in zip(nodes['osmid'],nodes['lon'], nodes['lat']):
        dict.update({osmid:i}) 
        i+=1
        texto = str(dict[osmid]) + " " + str(lon) + " " + str(lat) + "\n" 
        f.write (texto)
        
    for u, v in zip(edges['u'], edges['v']):
        texto = str(dict[u]) + " " + str(dict[v]) + "\n"
        f.write (texto)
        
    f.close()
    
    
def onclick(event):
    global qtd_clicks
    global clicks
    
    print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          ('double' if event.dblclick else 'single', event.button,
           event.x, event.y, event.xdata, event.ydata))
    
    if int(sys.argv[4]) == 1 and (qtd_clicks < 2):
        qtd_clicks += 1
        clicks += [(event.ydata, event.xdata)]
    
    if(event.button is "single"):
        print("cliquei")


def plot_graph(G):
    
    global qtd_clicks
    nc = ['r' if (osmid==ox.get_nearest_node(G, (-20.273804, -40.306124))) else 'b' for osmid in G.nodes()]
    nz = [36 if (osmid==ox.get_nearest_node(G, (-20.273804, -40.306124))) else 13 for osmid in G.nodes()]
#fig, ax = ox.plot_graph(G, node_color='w', node_edgecolor='k', node_size=30, node_zorder=3, edge_color=ec, edge_linewidth=3)
    fig, ax = ox.plot_graph(G, node_color=nc, node_size = nz, show=False, close=False)
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    #print(type(fig))
    #plt.imshow(fig)
    plt.show(block = False)
    plt.pause(5)
    plt.close(fig)


def get_graph ():
    # get a graph for some city
    #G = ox.graph_from_place('UFES, Vitoria, BR', network_type='drive', simplify=False)
    print (sys.argv[1], sys.argv[2], sys.argv[3])
    G = ox.graph_from_place(sys.argv[1], network_type='drive', simplify=int(sys.argv[2]))
   # G = ox.graph_from_point((-20.273804, -40.306124), distance = 500, network_type='drive', simplify=int(sys.argv[2]))
      
    #print(*dir(G), sep = "\n")
    #print(G.adjacency)
    
    G_proj = ox.project_graph(G) #Project a graph from lat-long to the UTM zone appropriate for its geographic location.
    nodes_proj, edges_proj = ox.graph_to_gdfs(G_proj) #Convert a graph into node and/or edge GeoDataFrames
    print(edges_proj)
    save_graph_file(nodes_proj, edges_proj)
    
    if int(sys.argv[3]) == 1:
        plot_graph(G)       
    
    return G, G_proj, nodes_proj, edges_proj
    

def get_route (G, G_proj, nodes_proj, edges_proj):
    plot_graph(G)
    #print(clicks[0])
    #print('\n')
    #print(clicks[1])
    
    
    dict = {}
    i = 0
    for osmid, lon, lat in zip(nodes_proj['osmid'],nodes_proj['lon'], nodes_proj['lat']):
        dict.update({osmid:i}) 
        i+=1
        #texto = str(dict[osmid]) + " " + str(osmid) + " " + str(lon) + " " + str(lat) + "\n" 
        #print(texto)

    orig_node = ox.get_nearest_node(G, (-20.273804, -40.306124))
    dest_node = ox.get_nearest_node(G, clicks[0])
    
    #find the route between these nodes then plot it
    route = nx.shortest_path(G, orig_node, dest_node, weight='length')
    #print(route)
    f = open("route.txt", 'w')
    texto = str(len(route)) + "\n"
    f.write(texto)
    for i in range(len(route)):
        texto = str(dict[route[i]]) + "\n"
        #print(dict[route[i]])
        f.write(texto)
       
       
    fig, ax = ox.plot_graph_route(G, route, node_size=0)
    
    


G, G_proj, nodes_proj, edges_proj = get_graph();

if int(sys.argv[4]) == 1:
    get_route (G, G_proj, nodes_proj, edges_proj)

#print(*dir(nodes_proj), sep = "\n")
#print(nodes_proj.head())
#print(*list(nodes_proj['lon']), sep="\n")

#for key in nodes_proj
#    print (key)
#UTM_pts = list(nodes_proj['geometry'])
#print (dir(UTM_pts))

#graph_area_m = nodes_proj.unary_union.convex_hull.area
#print (graph_area_m) # what sized area does our network cover in square meters?

# show some basic stats about the network
#print ("show some basic stats about the network \n")
#print(ox.basic_stats(G_proj, area=graph_area_m, clean_intersects=True, circuity_dist='euclidean'))
#more_stats = ox.basic_stats(G_proj, area=graph_area_m, clean_intersects=True, circuity_dist='euclidean')
#print (more_stats['n'])
#for key in (more_stats.keys()):
#    print(key)

#print ("\n")

# see more stats (mostly topological stuff) with extended_stats
#print ("see more stats (mostly topological stuff) with extended_stats\n")
#more_stats = ox.extended_stats(G, ecc=True, bc=True, cc=True) #use arguments to turn other toplogical analyses on/off
#for key in sorted(more_stats.keys()):
#    print(key)
    
# pull up some stat's value
#print (more_stats['radius'])

# save graph to disk as shapefile (for GIS) or graphml file (for gephi etc)
#ox.save_graph_shapefile(G, filename='mynetwork_shapefile')
#ox.save_graphml(G, filename='mynetwork.graphml')

# edge closeness centrality: convert graph to line graph so edges become nodes and vice versa
#edge_centrality = nx.closeness_centrality(nx.line_graph(G))


# list of edge values for the orginal graph
#ev = [edge_centrality[edge + (0,)] for edge in G.edges()]

# color scale converted to list of colors for graph edges
#norm = colors.Normalize(vmin=min(ev)*0.8, vmax=max(ev))
#cmap = cm.ScalarMappable(norm=norm, cmap=cm.inferno)
#ec = [cmap.to_rgba(cl) for cl in ev]

# color the edges in the original graph with closeness centralities in the line graph
#fig, ax = ox.plot_graph(G, bgcolor='k', axis_off=True, node_size=0,
                        #edge_color=ec, edge_linewidth=1.5, edge_alpha=1)

# get the nearest network node to each point

