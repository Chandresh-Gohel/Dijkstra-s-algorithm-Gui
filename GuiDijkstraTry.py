import heapq
import random
import time
import tkinter as tk

class Graph:
    def __init__(self, nodes):
        self.nodes = nodes
        self.graph = {i: {} for i in range(nodes)}  # Empty graph (adjacency list)
        self.locked_junctions = set()  # Set to track locked junctions
        self.am1 = None
        self.am2 = None


    def add_edge(self, u, v, cost):
        #Add an edge between u and v with a given cost.
        self.graph[u][v] = cost
        self.graph[v][u] = cost  # Since it's undirected, also reverse the edge

    def create_graph(self):
        """
        Create a graph where each node has exactly two edges connecting it to other nodes
        and no self loops.
        """
        edges = set()  # To store unique edges

        # Step 1: Create a connected graph (spanning tree)
        visited = [False] * self.nodes
        def dfs(node):
            visited[node] = True
            neighbors = [n for n in range(self.nodes) if not visited[n]]
            random.shuffle(neighbors)  # Shuffle neighbors to randomize the tree edges
            for neighbor in neighbors:
                if not visited[neighbor]:
                    cost = random.randint(1, 10)
                    self.add_edge(node, neighbor, cost)
                    dfs(neighbor)

        # Start DFS from node 0
        dfs(0)

        # Step 2: Ensure each node has exactly two edges
        for node in range(self.nodes):
            # Ensure each node has exactly two edges (by adding extra edges if necessary)
            while len(self.graph[node]) < 2:
                # Find a node that is not already connected to 'node' and add an edge
                potential_nodes = [n for n in range(self.nodes) if n != node and len(self.graph[n]) < 2]
                if not potential_nodes:
                    print("Could not find a valid node to connect to.")
                    return
                neighbor = random.choice(potential_nodes)
                if neighbor != node and (neighbor not in self.graph[node]):
                    cost = random.randint(1, 10)
                    self.add_edge(node, neighbor, cost)

        # Ensure the graph is connected (all nodes should be visited)
        if not all(visited):
            print("The graph is disconnected. Something went wrong.")
            return
        print("Graph created successfully with each node having exactly two edges.")

    def dijkstra(self, start, end):
        dist = {i: float('inf') for i in range(self.nodes)}
        dist[start] = 0
        pq = [(0, start)]  # Priority queue: (distance, node)
        prev = {i: None for i in range(self.nodes)}

        while pq:
            current_dist, current_node = heapq.heappop(pq)
            if current_node == end:
                break

            for neighbor, cost in self.graph[current_node].items():
                alt = current_dist + cost
                if alt < dist[neighbor]:
                    dist[neighbor] = alt
                    prev[neighbor] = current_node
                    heapq.heappush(pq, (alt, neighbor))

        # Reconstruct the path
        path = []
        node = end
        while prev[node] is not None:
            path.append(node)
            node = prev[node]
        path.append(start)

        # If no path exists
        if dist[end] == float('inf'):
            print(f"No path found from {start} to {end}")
            return [], float('inf')  # Return empty path if no path exists
        return path[::-1], dist[end]

    def lock_junction(self, u, v):
        # Lock the junction temporarily when an ambulance is passing
        self.locked_junctions.add((u, v))
        self.locked_junctions.add((v, u))

    def unlock_junction(self, u, v):
        # Unlock the junction when the ambulance passes through
        if (u, v) in self.locked_junctions:
            self.locked_junctions.remove((u, v))
        if (v, u) in self.locked_junctions:
            self.locked_junctions.remove((v, u))


class Ambulance:
    def __init__(self, start, destination, graph):
        self.start = start
        self.destination = destination
        self.graph = graph
        self.current_location = start
        self.path = []
        self.time_taken = 0

    def move(self):
        if self.path:
            print(self.graph.nodes)
            if self.start<0 or self.start>self.graph.nodes or self.destination < 0 or self.destination >self.graph.nodes:
                print(f"Destination {self.destination} is too far to reach")
                return
            next_node = self.path.pop(0)
            # Skip if current location is the same as next node
            if self.current_location == next_node:
                return  # Skip this iteration

            if next_node in self.graph.graph[self.current_location]:
                junction = (self.current_location, next_node)

                # Lock the junction temporarily while the ambulance moves
                self.graph.lock_junction(self.current_location, next_node)

                # Add the cost to the total time taken
                self.time_taken += self.graph.graph[self.current_location][next_node]
                self.current_location = next_node

                # Unlock the junction if not at the destination
                if self.current_location != self.destination:
                    self.graph.unlock_junction(self.current_location, next_node)


    def plan_route(self):
        self.path, _ = self.graph.dijkstra(self.start, self.destination)
        if not self.path:
            print(f"Error: No path found for Ambulance from {self.start} to {self.destination}")
        else:
            print(f"Ambulance path: {self.path}")


class GraphGUI:
    def __init__(self, master, graph):
        self.master = master
        self.graph = graph
        self.canvas = tk.Canvas(master, width=800, height=800, bg="white")
        self.canvas.pack()

        # Initialize and draw graph visualization
        self.nodes = {}
        self.edges = []  # Store edge information here
        self.create_graph_visualization()

        # Ambulances' positions (representing their starting nodes)
        self.am1_pos = self.graph.am1.current_location
        self.am2_pos = self.graph.am2.current_location

        # Create circles (representing the ambulances) at their starting positions
        self.am1_circle = self.canvas.create_oval(
            self.node_positions[self.am1_pos][0] - 10, 
            self.node_positions[self.am1_pos][1] - 10,
            self.node_positions[self.am1_pos][0] + 10, 
            self.node_positions[self.am1_pos][1] + 10, 
            fill="red"
        )
        self.am2_circle = self.canvas.create_oval(
            self.node_positions[self.am2_pos][0] - 10, 
            self.node_positions[self.am2_pos][1] - 10,
            self.node_positions[self.am2_pos][0] + 10, 
            self.node_positions[self.am2_pos][1] + 10, 
            fill="blue"
        )

        # Text area to display time
        self.time_text = self.canvas.create_text(10, 780, anchor="w", text="Time: 0s", font=('Helvetica', 12))

    def create_graph_visualization(self):
        """Draws nodes and edges on the canvas."""
        # Generate random node positions for 50 nodes
        self.node_positions = {i: (random.randint(50, 750), random.randint(50, 750)) for i in range(self.graph.nodes)}

        # Draw the nodes as circles
        for node, pos in self.node_positions.items():
            self.nodes[node] = self.canvas.create_oval(
                pos[0] - 10, pos[1] - 10, pos[0] + 10, pos[1] + 10,
                fill="black", outline="black"
            )

        # Draw edges (connecting nodes with two edges)
        for u in self.graph.graph:
            for v, cost in self.graph.graph[u].items():
                if u < v:  # Prevent duplicate edges
                    edge_line = self.canvas.create_line(
                        self.node_positions[u][0], self.node_positions[u][1],
                        self.node_positions[v][0], self.node_positions[v][1],
                        width=2, fill="black"  # Initially black
                    )
                    self.edges.append((u, v, edge_line))  # Store the edge info here

    def update_edge_color(self, u, v, color):
        """Change the color of a specific edge."""
        for edge in self.edges:
            if edge[0] == u and edge[1] == v:
                self.canvas.itemconfig(edge[2], fill=color)
                break

    def move_ambulance(self, am, next_node, am_num):
        """Move the ambulance smoothly from one node to the next."""
        u = am.current_location
        v = next_node

        # Update edge color to green while the ambulance is moving
        self.update_edge_color(u, v, "green")

        # Move the ambulance (animate)
        target_x, target_y = self.node_positions[v]
        step_size = 5
        current_x, current_y = self.node_positions[u]

        while (current_x, current_y) != (target_x, target_y):
            dx, dy = target_x - current_x, target_y - current_y
            current_x += step_size * (dx / max(abs(dx), abs(dy)))
            current_y += step_size * (dy / max(abs(dx), abs(dy)))

            self.canvas.coords(
                self.am1_circle if am_num == 1 else self.am2_circle,
                current_x - 10, current_y - 10, current_x + 10, current_y + 10
            )
            self.master.update()
            time.sleep(0.05)  # Slow down the animation

        # Final position
        self.canvas.coords(
            self.am1_circle if am_num == 1 else self.am2_circle,
            target_x - 10, target_y - 10, target_x + 10, target_y + 10
        )

        # Unlock the junction (edge)
        self.update_edge_color(u, v, "black")  # Reset edge color to black after moving
        am.move()

    def run_simulation(self):
        """Runs the simulation of ambulance movement."""
        start_time = time.time()
        while (self.graph.am1.current_location != self.graph.am1.destination or 
               self.graph.am2.current_location != self.graph.am2.destination):

            # Move Ambulance 1 if it has not reached its destination
            if self.graph.am1.path:
                next_node_am1 = self.graph.am1.path.pop(0)
                self.move_ambulance(self.graph.am1, next_node_am1, 1)

            # Move Ambulance 2 if it has not reached its destination
            if self.graph.am2.path:
                next_node_am2 = self.graph.am2.path.pop(0)
                self.move_ambulance(self.graph.am2, next_node_am2, 2)

            # Update time on canvas
            elapsed_time = int(time.time() - start_time)
            self.canvas.itemconfig(self.time_text, text=f"Time: {elapsed_time}s")
            self.master.update()


# Main execution
def main():
    root = tk.Tk()
    graph = Graph(15)  # Create graph with 50 nodes
    graph.create_graph()  # Randomly create graph edges

    # Initialize the ambulances
    am1 = Ambulance(start=2, destination=7, graph=graph)  # Ambulance 1 from node 2 to node 7
    am2 = Ambulance(start=8, destination=11, graph=graph)  # Ambulance 2 from node 8 to node 10

    # Plan the routes for both ambulances
    am1.plan_route()
    am2.plan_route()

    graph.am1 = am1
    graph.am2 = am2

    # Create the GUI to visualize the graph and simulate ambulance movement
    gui = GraphGUI(root, graph)
    gui.run_simulation()

    root.mainloop()

if __name__ == "__main__":
    main()
