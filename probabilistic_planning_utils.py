import networkx as nx
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue
from sklearn.neighbors import KDTree

def extract_polygons(data):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        corners = [(north-d_north,east-d_east), (north-d_north,east+d_east), (north+d_north,east+d_east), (north+d_north,east-d_east)]
        
        # TODO: Compute the height of the polygon
        height = alt+d_alt

        # TODO: Once you've defined corners, define polygons
        p = Polygon(corners)
        #print(p.area)
        polygons.append((p, height,(north,east)))
    return polygons

def collides(polygons, poly_index, point, poly_tree):   
    # TODO: Determine whether the point collides
    # with any obstacles.
    
    for i in poly_index:
        (p,height,center) = polygons[i]
        if p.contains(Point(point)) and height >= point[2]:
            return True
    return False


def get_random_samples(data, num_samples, polygons, poly_tree ):
    xmin = np.min(data[:, 0] - data[:, 3])
    xmax = np.max(data[:, 0] + data[:, 3])

    ymin = np.min(data[:, 1] - data[:, 4])
    ymax = np.max(data[:, 1] + data[:, 4])

    zmin = 15
    # Limit the z axis for the visualization
    zmax = 16

    xvals = np.random.randint(xmin, np.ceil(xmax), num_samples)
    yvals = np.random.randint(ymin, np.ceil(ymax), num_samples)
    zvals = np.random.randint(zmin, zmax, num_samples)
    
    samples = zip(xvals, yvals, zvals)
    max_search_radius_kdtree = 2*np.max((data[:, 3], data[:, 4])) 
    to_keep = []
    for point in samples:
        #print(point)
        poly_index = poly_tree.query_radius([(point[0],point[1])],r=max_search_radius_kdtree)[0]
        #print(poly_index)
        
        if not collides(polygons, poly_index, point, poly_tree):
             to_keep.append(point)
    return to_keep

def can_connect(n1, n2, polygons):
    line = LineString([n1, n2])
    for (p,height,center)  in polygons:
        if p.crosses(line) and height >= min(n1[2], n2[2]):
            return False
    return True

def create_graph(nodes, polygons):
	g = nx.Graph()
	node_tree = KDTree(nodes)
	for n in nodes:
		num_neighbours = 5
		something_connected = False
		#while(num_neighbours < len(nodes)/4 and something_connected==False):
		n_index = node_tree.query([n], num_neighbours, return_distance=False)[0]
		num_neighbours = 2*num_neighbours
		#print(n_index)
		for ni in n_index:
			n_to_connect = nodes[ni]
			if n == n_to_connect:
				something_connected = True
				continue
			if can_connect(n,n_to_connect,polygons):
				g.add_edge(n,n_to_connect)
	something_connected = True
	return g

def heuristic(n1, n2):
    # TODO: complete
    return np.linalg.norm(np.array(n2) - np.array(n1))

def a_star_graph(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph.neighbors(current_node):
                
                # TODO: calculate branch cost (action.cost + g)
                # TODO: calculate queue cost (action.cost + g + h)
                action_cost = np.linalg.norm(np.array(next_node) - np.array(current_node))
                branch_cost = current_cost + 1
                queue_cost = current_cost + action_cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))    
                    
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
        
    return path[::-1], path_cost

    	




