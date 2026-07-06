

import os
import heapq
import time
import osmnx as ox
from flask import Flask, render_template, request, jsonify


app = Flask(__name__, template_folder='.')

#defaults
DEFAULT_GRAPH = "bengaluru_cached_map.graphml"
#Arekere coords
DEFAULT_LAT = 12.8877
DEFAULT_LON = 77.5996
print("In the next prompt, please enter the name of the file that is to be loaded.\n If you want to download and save a new location file, enter the name that the file has to be saved as.\n")
print("Press [Enter] to accept the default values(bengaluru map centered at arekere).\n")
user_graph = input(f"Enter graph filename [{DEFAULT_GRAPH}]: ").strip()
GRAPH_FILENAME = user_graph if user_graph else DEFAULT_GRAPH
user_lat = input(f"Enter origin latitude [{DEFAULT_LAT}]: ").strip()
lat = float(user_lat) if user_lat else DEFAULT_LAT
user_lon = input(f"Enter origin longitude [{DEFAULT_LON}]: ").strip()
lon = float(user_lon) if user_lon else DEFAULT_LON
origin_point = (lat, lon)

#Download and load up the file if it doesnt exist.
if os.path.exists(GRAPH_FILENAME):
    print("found file!loading it.")
    t0 = time.time()
    graph = ox.load_graphml(GRAPH_FILENAME)
    print(f"Map loaded in {round(time.time() - t0, 2)} seconds!")
else:
    print(f"Cache file not found.")
    print("Downloading and building graph model...")
    t0 = time.time()
    graph = ox.graph_from_point(origin_point, dist=50000, network_type='drive')#creates a graph object 50 km radius frm orini_point , only drivable roads   
    print("Download complete!Saving file...")
    ox.save_graphml(graph, GRAPH_FILENAME)#save file
    print(f"Map saved locally in {round(time.time() - t0, 2)} seconds!")



def run_dijkstra(G, start, end):
    """
    This function is the direct application of dijkstra's algorithm for shortest path.
    It takes all the vertices and loads it up in a distance dict.
    Then it performs a BFS traversal on all the unexplored neighbors and updates their distance.
    Then it does the same thing with the vertex with the least distance and which is unexplored.
    """
    distances = {node: float('inf') for node in G.nodes}
    distances[start] = 0
    came_from = {}
    pq = [(0, start)]
    
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        
        if current_node == end:
            break
        if current_distance > distances[current_node]:
            continue
            
        for neighbor in G.successors(current_node):#.successors gives all neighbouring nodes
            edge_data = G.get_edge_data(current_node, neighbor)
            if not edge_data: continue
            edge_length = edge_data[0].get('length', float('inf'))
            g_score = current_distance + edge_length 
            #Updates the distance
            if g_score < distances[neighbor]:
                distances[neighbor] = g_score
                came_from[neighbor] = current_node
                heapq.heappush(pq, (g_score, neighbor))
    path = []
    current = end
    #Back-tracking the entire path
    while current in came_from:
        path.append(current)
        current = came_from[current]
    if path: path.append(start)
    path.reverse()
    return path


def run_standard_a_star(G, start, end):
    """
    This function is the direct application of A* algorithm for shortest path.
    It is exactly the same as dijkstra's algorithm except it is directional alorithm.
    It has a heuristic which punishes the paths which are going in the wrong direction as the destination.
    This makes it so that A* has to visit significantly lesser nodes than dijkstra.
    """
    
    end_lat, end_lon = G.nodes[end]['y'], G.nodes[end]['x']
    
    def heuristic(node_id):
        n_lat, n_lon = G.nodes[node_id]['y'], G.nodes[node_id]['x']
        return ox.distance.great_circle(n_lat, n_lon, end_lat, end_lon)
    distances = {node: float('inf') for node in G.nodes}
    distances[start] = 0
    came_from = {}
    pq = [(heuristic(start), 0, start)]
    
    while pq:
        _, current_distance, current_node = heapq.heappop(pq)
        
        if current_node == end:
            break
        if current_distance > distances[current_node]:
            continue
            
        for neighbor in G.successors(current_node):
            edge_data = G.get_edge_data(current_node, neighbor)
            if not edge_data: continue
            edge_length = edge_data[0].get('length', float('inf'))
            g_score = current_distance + edge_length
            
            if g_score < distances[neighbor]:
                distances[neighbor] = g_score
                came_from[neighbor] = current_node
                #punishing the nodes moving in the wrong direction
                f_score = g_score + heuristic(neighbor)
                heapq.heappush(pq, (f_score, g_score, neighbor))
                
    path = []
    current = end
    while current in came_from:
        path.append(current)
        current = came_from[current]
    if path: path.append(start)
    path.reverse()
    return path

def run_double_a_star(G, start, end):
    """
    This function is the direct application of double A* algorithm for shortest path.
    It is exactly the same as A* algorithm except it runs the algorithm from the start as well as the end, so that it meets somewhere in the middle.
    This makes it so that double A* has to visit significantly lesser nodes than dijkstra and A*.
    """
    start_lat, start_lon = G.nodes[start]['y'], G.nodes[start]['x']
    end_lat, end_lon = G.nodes[end]['y'], G.nodes[end]['x']
    
    #define the heuristic for forward and backward
    def heuristic_f(node_id):
        n_lat, n_lon = G.nodes[node_id]['y'], G.nodes[node_id]['x']
        return ox.distance.great_circle(n_lat, n_lon, end_lat, end_lon)

    def heuristic_b(node_id):
        n_lat, n_lon = G.nodes[node_id]['y'], G.nodes[node_id]['x']
        return ox.distance.great_circle(n_lat, n_lon, start_lat, start_lon)

    #setting up 2 completely different set of copies, one for front and one for back
    pq_f, dist_f, parent_f = [(heuristic_f(start), 0, start)], {n: float('inf') for n in G.nodes}, {}
    pq_b, dist_b, parent_b = [(heuristic_b(end), 0, end)], {n: float('inf') for n in G.nodes}, {}
    dist_f[start], dist_b[end] = 0, 0
    
    intersect_node, min_total_path = None, float('inf')

    while pq_f and pq_b:
        # Forward Step
        _, d_curr_f, curr_f = heapq.heappop(pq_f)
        if d_curr_f <= dist_f[curr_f]:
            #if back node has already visited this node, then check if we have a shorter path then current found path
            if dist_b[curr_f] != float('inf'):
                if dist_f[curr_f] + dist_b[curr_f] < min_total_path:
                    min_total_path = dist_f[curr_f] + dist_b[curr_f]
                    intersect_node = curr_f
                    break
            for neighbor in G.successors(curr_f):
                edge_data = G.get_edge_data(curr_f, neighbor)
                if not edge_data: continue
                #update closest dist from start
                weight = edge_data[0].get('length', float('inf'))
                if dist_f[curr_f] + weight < dist_f[neighbor]:
                    dist_f[neighbor] = dist_f[curr_f] + weight
                    parent_f[neighbor] = curr_f
                    heapq.heappush(pq_f, (dist_f[neighbor] + heuristic_f(neighbor), dist_f[neighbor], neighbor))

        # Backward Step
        _, d_curr_b, curr_b = heapq.heappop(pq_b)
        if d_curr_b <= dist_b[curr_b]:
            if dist_f[curr_b] != float('inf'):
                if dist_f[curr_b] + dist_b[curr_b] < min_total_path:
                    min_total_path = dist_f[curr_b] + dist_b[curr_b]
                    intersect_node = curr_b
                    break
            for neighbor in G.predecessors(curr_b):
                edge_data = G.get_edge_data(neighbor, curr_b)
                if not edge_data: continue
                weight = edge_data[0].get('length', float('inf'))
                if dist_b[curr_b] + weight < dist_b[neighbor]:
                    dist_b[neighbor] = dist_b[curr_b] + weight
                    parent_b[neighbor] = curr_b
                    heapq.heappush(pq_b, (dist_b[neighbor] + heuristic_b(neighbor), dist_b[neighbor], neighbor))
        #this is the stopping condition , basically we are checking that the most promising nodes that we have so far.....the sum of those nodes would basically be more than the sum of shortest distance found so far+ straight line distance of start and end , then we break 
        if pq_f and pq_b:
            if pq_f[0][0] + pq_b[0][0] >= min_total_path + heuristic_f(start):
                break
    if intersect_node is None: return []
    path_f, curr = [], intersect_node
    while curr in parent_f:
        path_f.append(curr); curr = parent_f[curr]
    if curr: path_f.append(curr)
    path_f.reverse()
    path_b, curr = [], parent_b.get(intersect_node)
    while curr in parent_b:
        path_b.append(curr); curr = parent_b[curr]
    if curr: path_b.append(curr)
    return path_f + path_b

@app.route('/')
def home():
    return open('index.html').read()

@app.route('/get_route')
def get_route():
    try:
        start_lat = float(request.args.get('start_lat'))
        start_lon = float(request.args.get('start_lon'))
        end_lat = float(request.args.get('end_lat'))
        end_lon = float(request.args.get('end_lon'))
        selected_algo = request.args.get('algo', 'double_a_star')
        
        #get respective nodes
        start_node = ox.distance.nearest_nodes(graph, X=start_lon, Y=start_lat)
        end_node = ox.distance.nearest_nodes(graph, X=end_lon, Y=end_lat)
        
        # run all 3 algorithms and find the time taken and the paths found
        t0 = time.perf_counter()
        dijkstra_path = run_dijkstra(graph, start_node, end_node)
        t_dijkstra = (time.perf_counter() - t0) * 1000 

        t0 = time.perf_counter()
        a_star_path = run_standard_a_star(graph, start_node, end_node)
        t_astar = (time.perf_counter() - t0) * 1000

        t0 = time.perf_counter()
        double_astar_path = run_double_a_star(graph, start_node, end_node)
        t_double = (time.perf_counter() - t0) * 1000

        # choose which path to show based on user choice
        chosen_node_path = double_astar_path
        if selected_algo == 'dijkstra': chosen_node_path = dijkstra_path
        elif selected_algo == 'a_star': chosen_node_path = a_star_path

        if not chosen_node_path:#send error message
            return jsonify({"status": "error", "message": "No valid route path found."}), 400
            
        geometry_path = [[graph.nodes[node]['y'], graph.nodes[node]['x']] for node in chosen_node_path]
        
        #return final values
        return jsonify({
            "status": "success", 
            "route": geometry_path,
            "benchmarks": {
                "dijkstra": round(t_dijkstra, 2),
                "a_star": round(t_astar, 2),
                "double_a_star": round(t_double, 2)
            }
        })
        
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

if __name__ == '__main__':
    app.run(debug=True, port=5000, use_reloader=False)
