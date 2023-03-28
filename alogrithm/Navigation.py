import networkx as nx
import math

# network takes the form of [start point,end point, weight]
def weighted_graph(network):
    G = nx.Graph()
    for link in network:
        G.add_edge(link[0],link[1], weight = link [2])
 
    # plot the network
    nx.draw(G, with_labels=True, font_weight='bold')
    return G

def dijkstra(G,origin,destination):        
    path = nx.disjktra_path(G,origin,destination)
    path_length = nx.dijkstra_path_length(G,origin,destination)
    return  path, path_length

def a_star(G,origin,destination):
    # first find the straight line distance between two points
    Distance = math.sqrt((destination[0]-origin[0])**2+(destination[1]-origin[1])**2)

    path = nx.astar_path(G,origin,destination,heuristic = Distance)
    
    path_length = 0
    for link in range(len(path)-1):
        weight_edge = sum(G.edges[path[link],path[link+1]]['weight'])
        path_length += weight_edge

    return path, path_length







