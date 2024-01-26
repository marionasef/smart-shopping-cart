import heapq
import matplotlib.pyplot as plt
import numpy as np

class AStar:
    def __init__(self, costs):
        self.costs = costs

    def astar_with_obstacles(self, start, goal):
       
        g_costs = {start: 0}
        f_costs = {start: self.heuristic(start, goal)}
        open_list = [(f_costs[start], start)]
        came_from = {}

        while open_list:
            _, current_node = heapq.heappop(open_list)

            if current_node == goal:
                break

            for neighbor in self.costs[current_node]:
                tentative_g_cost = g_costs[current_node] + self.costs[current_node][neighbor]

                if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g_cost
                    f_costs[neighbor] = tentative_g_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_costs[neighbor], neighbor))
                    came_from[neighbor] = current_node

        path = self.reconstruct_path(came_from, start, goal)
        return path

    def heuristic(self, node, goal):
        # Manhattan distance heuristic
        x1, y1 = node
        x2, y2 = goal
        return abs(x1 - x2) + abs(y1 - y2)

    def reconstruct_path(self, came_from, start, goal):
        current_node = goal
        path = [current_node]

        while current_node != start:
            current_node = came_from[current_node]
            path.append(current_node)

        path.reverse()
        return path

# Define the supermarket map with costs
costs_with_obstacles = {
    (0, 0): {(1, 0): 1},  # Entrance
    (1, 0): {(0, 0): 1, (3, 0):1 ,(1,2):1},  # Section 1
    (3,0): {(1, 0): 1, (3, 2): 1},  # Section 2
    (3,2): {(3,0): 1,(1,2):1,(3,3):1},  # Section 3
    (1,2): {(1, 0): 1, (3, 2): 1,(1,3):1},  # Section 4
    (1, 3): {(1, 2): 1, (3, 3): 1},  # Section 5
    (3, 3): {(1, 3): 1},  # Checkout
}

section_names = {
    (0, 0): 'Entrance',
    (1,1):'aisle',
    (2,0):'aisle',
    (3,1):'aisle',
    (2,2):'aisle',
    (2,3):'aisle',
    (1, 0): 'Section 1',
    (3,0): 'Section 2',
    (3, 2): 'Section 3',
    (1, 2): 'Section 4',
    (1,3): 'Section 5',
    (3, 3): 'Checkout',
}


# Create an instance of AStar
astar = AStar(costs_with_obstacles)

# Find the shortest path from the entrance to the checkout
start_node = (0,0)
goal_node = (3,3)
path = astar.astar_with_obstacles(start_node, goal_node)

# Visualize the map
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xticks(np.arange(0, 6, 1))
ax.set_yticks(np.arange(0, 6, 1))
ax.grid(color="black", linewidth=1)
ax.tick_params(axis="both", which="both", length=0)
ax.imshow(np.zeros((5, 5)), cmap="viridis")

# Plot the obstacles
for node in costs_with_obstacles:
    if node != goal_node and node != start_node:
        ax.add_patch(plt.Rectangle(node, 1, 1, facecolor="gray"))
        
# Plot the section names
for section, name in section_names.items():
    ax.text(section[0] + 0.5, section[1] + 0.5, name, ha="center", va="center", color="white")

# Plot the path
for i in range(len(path) - 1):
    current_node = path[i]
    next_node = path[i + 1]
    ax.plot([current_node[0] + 0.5, next_node[0] + 0.5], [current_node[1] + 0.5, next_node[1] + 0.5], color="blue")

# Plot the start and goal nodes
ax.add_patch(plt.Rectangle(start_node, 1, 1, facecolor="green"))
ax.add_patch(plt.Rectangle(goal_node, 1, 1, facecolor="red"))

plt.xlim(0, 5)
plt.ylim(0, 4)
plt.title("Supermarket Map")
plt.show()