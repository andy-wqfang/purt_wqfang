# Path Planning (MR Chapter 10)

## A* Algorithm

### Overview
A* is a graph search algorithm that finds the shortest path between nodes using heuristics to guide its search. It is widely used in robotics, game development, and navigation.

### Key Components
- Cost Function 
  - Uses a combined cost function: `f(n) = g(n) + h(n)`  
  - g(n): The exact cost from the start node to the current node.  
  - h(n): A heuristic estimate of the cost from the current node to the goal.
  
- Heuristic  
  - The choice of heuristic greatly influences performance. Common heuristics include:  
  - **Euclidean Distance:** For continuous spaces.  
  - **Manhattan Distance:** For grid-based maps.  
  An admissible heuristic (one that never overestimates) guarantees optimality.

- Optimality & Completeness:
  - **Optimal:** If h(n) is admissible and consistent, A* will find the optimal path.  
  - **Complete:** It will find a solution if one exists, provided the search space is finite.

- Algorithm Steps:
  1. Initialize an open list (priority queue) with the start node.
  2. Repeat until the goal is reached or the open list is empty:
     - Remove the node with the lowest f(n) from the open list.
     - If it’s the goal, reconstruct the path and finish.
     - Otherwise, expand the node’s neighbors, update their costs, and add them to the open list.
     
- Performance Considerations:
  Efficiency depends on the heuristic’s accuracy; a more informed heuristic reduces the search space.

## RRT (Rapidly-exploring Random Tree)

### Overview 
RRT is a sampling-based planning algorithm designed to efficiently explore high-dimensional and complex continuous spaces. It is popular in robotics for motion planning.

### Key Components
- Random Sampling: 
  Instead of exhaustively searching a grid, RRT randomly samples the configuration space. This allows the algorithm to quickly explore large or complex spaces.

- Tree Expansion:
  - Nearest Node Selection: For each randomly sampled point, the algorithm finds the nearest node already in the tree.
  - Extension: It then extends the tree from this nearest node towards the sample by a fixed step size (or up to the sample if within range).
  - Incorporation:If the extension is valid (e.g., collision-free), the new point is added as a node in the tree.

- Probabilistic Completeness:
  RRT is **probabilistically complete**—if a path exists, the probability that RRT will find it approaches 1 as the number of samples increases. However, it does not guarantee optimality.

- Variants:
  - **RRT\*:** An improved version that incrementally optimizes the path, providing asymptotic optimality—the path cost converges to the optimal cost with infinite samples.
  
- Algorithm Steps:
  1. Initialize the tree with the start configuration.
  2. For a set number of iterations (or until a goal is reached):
     - Randomly sample a point in the configuration space.
     - Find the nearest node in the tree to this sample.
     - Extend the tree toward the sample by a fixed step.
     - Add the new node if the path is collision-free.
  3. Optionally, connect the tree to the goal or continue sampling until a satisfactory path is found.

- Advantages & Disadvantages
  - Advantages:
    - Excellent for high-dimensional spaces.  
    - No need for explicit discretization of the space.
  - Disadvantages:
    - Paths may be suboptimal or jagged, often requiring post-processing (e.g., smoothing).
    - The random nature means results can vary between runs.

## Probabilistic Roadmap Method (PRM)

PRM is a sampling-based motion planning algorithm designed primarily for high-dimensional configuration spaces. It is especially useful in multi-query scenarios where the same environment is used repeatedly for planning different paths.

### Key Phases

1. **Construction Phase:**
   - Sampling  
     Randomly generate a set of configurations (nodes) within the free (collision-free) space.
   - Validation
     Each sample is checked for feasibility—only collision-free configurations are kept.
   - Roadmap Creation 
     For each valid sample, the algorithm attempts to connect it with its neighbors (using a local planner) if the connecting path is collision-free. This builds a graph (roadmap) of nodes and edges representing the free space.

2. **Query Phase:**
   - Connecting Start and Goal
     When a planning query is made, the start and goal configurations are connected to the nearest nodes on the roadmap.
   - Path Search
     A graph search algorithm (e.g., Dijkstra’s or A*) is used to find a path through the roadmap from start to goal.
   - Path Retrieval
     If a valid path exists on the roadmap, it is returned; otherwise, additional sampling may be required.

### Properties & Considerations

- **Probabilistic Completeness:**  
  PRM is probabilistically complete, meaning that if a path exists, the likelihood of finding it approaches 1 as the number of samples increases.
  
- **Efficiency for Multi-query Scenarios:**  
  Since the roadmap is built once and can be reused for multiple planning queries, PRM is efficient in environments where many paths need to be planned over the same configuration space.

- **No Optimality Guarantee (Standard PRM):**  
  The basic PRM does not guarantee that the shortest or most optimal path will be found. For optimality, variants like **PRM\*** are used, which provide asymptotic optimality given sufficient samples.

- **Trade-offs:**  
  - Density of Sampling: More samples generally improve the connectivity and quality of the roadmap but increase computation time.
  - Local Planner Quality: The success of connecting nodes depends on the effectiveness of the local planner and collision detection.

### Variants

- PRM\*
  A variant of PRM that, under certain conditions and with sufficient samples, guarantees asymptotic optimality. This means that as the number of samples increases, the cost of the found path converges to the optimal cost.

### Applications
PRM is widely used in robotics and autonomous systems for motion planning in environments with complex, high-dimensional spaces, such as robotic manipulators or mobile robots navigating cluttered spaces.

