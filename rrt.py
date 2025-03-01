import csv
import math
import random
import sys
import os
import heapq # for priority queue (to reduce time complexity)

C_SPACE_MIN = -0.5
C_SPACE_MAX = 0.5
START = (-0.5, -0.5)
GOAL = (0.5, 0.5)
NUM_NODES = 300
K_NEAREST = 15

def read_obstacles(filename):
    obstacles = []
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if len(row) < 3:
                continue
            if row[0][0] == '#':
                continue
            cx = float(row[0])
            cy = float(row[1])
            r = float(row[2])
            obstacles.append((cx, cy, r))
    return obstacles

def write_csv(filename, rows):
    filename = os.path.join("rrt_results", filename)
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for row in rows:
            writer.writerow(row)

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def collision_line_circle(p1, p2, circle):
    # Check if the line segment from p1 to p2 collides with a circle.
    # circle is defined as (center_x, center_y, radius).
    cx, cy, r = circle
    if euclidean_distance(p1, (cx, cy)) <= r:
        return True
    if euclidean_distance(p2, (cx, cy)) <= r:
        return True
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return euclidean_distance(p1, (cx, cy)) <= r
    t = ((cx - x1) * dx + (cy - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return euclidean_distance((closest_x, closest_y), (cx, cy)) <= r

def is_collision_free(p1, p2, obstacles):
    for circle in obstacles:
        if collision_line_circle(p1, p2, circle):
            return False
    return True

def sample_nodes(num_nodes):
    nodes = []
    nodes.append(START)
    nodes.append(GOAL)
    for _ in range(num_nodes - 2):
        x = random.uniform(C_SPACE_MIN, C_SPACE_MAX)
        y = random.uniform(C_SPACE_MIN, C_SPACE_MAX)
        nodes.append((x, y))
    return nodes

def build_roadmap(nodes, obstacles, k=K_NEAREST):
    graph = {i: [] for i in range(len(nodes))}
    for i, node in enumerate(nodes):
        dists = []
        for j, other in enumerate(nodes):
            if i == j:
                continue
            d = euclidean_distance(node, other)
            dists.append((j, d))
        dists.sort(key=lambda x: x[1])
        for j, d in dists[:k]:
            if is_collision_free(node, nodes[j], obstacles):
                graph[i].append((j, d))
                graph[j].append((i, d))
    return graph

def a_star(graph, nodes, start_index, goal_index):
    # Perform A* search on the roadmap graph from start_index to goal_index
    open_set = {start_index}
    came_from = {}
    g_score = {i: float('inf') for i in range(len(nodes))}
    f_score = {i: float('inf') for i in range(len(nodes))}
    g_score[start_index] = 0
    f_score[start_index] = euclidean_distance(nodes[start_index], nodes[goal_index])
    open_heap = []
    heapq.heappush(open_heap, (f_score[start_index], start_index))
    
    while open_heap:
        current_f, current = heapq.heappop(open_heap)
        if current == goal_index:
            return reconstruct_path(came_from, current)
        open_set.discard(current)
        for neighbor, weight in graph[current]:
            tentative_g = g_score[current] + weight
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + euclidean_distance(nodes[neighbor], nodes[goal_index])
                if neighbor not in open_set:
                    open_set.add(neighbor)
                    heapq.heappush(open_heap, (f_score[neighbor], neighbor))
    return None

def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return total_path

def main():
    random.seed(42)
    obstacles = read_obstacles(os.path.join("Scene5_example", "obstacles.csv"))
    if not obstacles:
        print("No obstacles found in obstacles.csv. Please check the file.")
        sys.exit(1)
    nodes = sample_nodes(NUM_NODES)
    graph = build_roadmap(nodes, obstacles, k=K_NEAREST)
    start_index = 0
    goal_index = 1
    path_indices = a_star(graph, nodes, start_index, goal_index)
    if path_indices is None:
        print("No path found!")
        sys.exit(1)
    else:
        print("Path found with {} nodes.".format(len(path_indices)))
    nodes_rows = []
    for i, (x, y) in enumerate(nodes):
        nodes_rows.append([i, x, y])
    write_csv("nodes.csv", nodes_rows)
    edges_rows = []
    added_edges = set()
    for i, neighbors in graph.items():
        for j, weight in neighbors:
            if (j, i) not in added_edges:
                edges_rows.append([i, j, weight])
                added_edges.add((i, j))
    write_csv("edges.csv", edges_rows)
    path_rows = []
    for index in path_indices:
        x, y = nodes[index]
        path_rows.append([index, x, y])
    write_csv("path.csv", path_rows)
    # print("CSV files written: nodes.csv, edges.csv, and path.csv.")

if __name__ == "__main__":
    main()
