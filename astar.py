import numpy as np
import matplotlib.pyplot as plt
import os
import csv

BASE_PATH = os.path.join("..", "V-REP_scenes", "V-REP_scenes", "Scene5_example")

class tree_node(object):
    def __init__(self, id, x, y, cost_to_go):
        self.id = id
        self.x = x
        self.y = y
        self.cost_to_go = cost_to_go
        # self.next = list()
        self.parent = None

    def update_parent(self, parent):
        self.parent = parent
    
def read_nodes() -> list[tree_node]:
    node_list = list()
    with open(os.path.join(BASE_PATH, "nodes.csv"), 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if (row[0][0] == '#'):
                continue
            # print(type(row), row)
            # print(", ".join(row))
            id, x, y, cost_to_go = row
            id = int(id)
            x = float(x)
            y = float(y)
            cost_to_go = float(cost_to_go)
            node = tree_node(id, x, y, cost_to_go)
            node_list.append(node)
        # global NODECOUNT
        # NODECOUNT = len(node_list)
        # print(NODECOUNT)
    return node_list
        
def read_edges():
    cost = np.zeros((NODECOUNT, NODECOUNT))
    with open(os.path.join(BASE_PATH, "edges.csv"), 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if (row[0][0] == '#'):
                continue
            start, end, weight = row
            start = int(start)
            end = int(end)
            weight = float(weight)
            cost[start-1, end-1] = weight
            cost[end-1, start-1] = weight
        
    # print(cost)
    return cost
    
OPEN = list()
CLOSED = list()
past_cost = list()
NODECOUNT: int

node_list = read_nodes()
NODECOUNT = len(node_list)
cost = read_edges()
OPEN.append(0)
past_cost = np.zeros(NODECOUNT) + np.inf
start = int(input("start node: "))
finish = int(input("finish node: "))
past_cost[start] = 0
# print(past_cost)
while len(OPEN) > 0:
    # past_cost.sort(order = lambda x: node_list[x]+)
    OPEN.sort(key = lambda x: past_cost[x] + node_list[x].cost_to_go)
    current = OPEN.pop(0)
    CLOSED.append(current)
    if current == finish:
        print("found it!")
        parent_path = list()
        while current is not None:
            # print()
            parent_path.insert(0, current)
            current = node_list[current].parent
        # parent_path.insert(0, current)
        print(np.asarray(parent_path) + 1)
        break
    for i in range(NODECOUNT):
        if i in CLOSED:
            continue
        elif i == current:
            continue
        elif np.isclose(cost[current, i], 0, atol=1e-6):
            continue
        else:
            tentative_past_cost = past_cost[current] + cost[current, i]
            if (tentative_past_cost < past_cost[i]):
                
                past_cost[i] = tentative_past_cost
                node_list[i].update_parent(current)
                OPEN.insert(0, i)
                print(OPEN)
    