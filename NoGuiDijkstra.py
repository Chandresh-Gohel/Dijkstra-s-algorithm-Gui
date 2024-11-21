import heapq
import random
import time

class Graph:
    def __init__(self, nodes):
        self.nodes = nodes
        self.graph = {i: {} for i in range(nodes)}  # Empty graph (adjacency list)
        self.locked_junctions = set()  # Set to track locked junctions

    def add_edge(self, u, v, cost):
        # Add an edge with a given cost
        self.graph[u][v] = cost
        self.graph[v][u] = cost  # Since it's undirected, also reverse the edge

    def update_edge_costs(self):
        # Simulate dynamic edge costs (changing every 5 seconds)
        for u in range(self.nodes):
            for v in self.graph[u]:
                new_cost = self._calculate_dynamic_cost(u, v)
                self.graph[u][v] = new_cost
                self.graph[v][u] = new_cost  # Update reverse edge too

    def _calculate_dynamic_cost(self, u, v):
        # Simulate some changing cost based on the time
        return abs(u - v) + (time.time() % 10)  # Example cost based on node difference

    def dijkstra(self, start, end):
        # Dijkstra's algorithm to find the shortest path
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

    def print_graph(self):
        # Debugging helper to print the graph structure
        print("Graph structure:")
        for u in range(self.nodes):
            print(f"Node {u}: {self.graph[u]}")

    def create_graph(self):
        """
        Create a graph where each node has exactly two edges connecting it to other nodes
        and no self loops.
        """
        # edges = set()  # To store unique edges

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


# Ambulance class to simulate the movement
class Ambulance:
    def __init__(self, start, destination, graph):
        self.start = start
        self.destination = destination
        self.graph = graph
        self.current_location = start
        self.path = []
        self.time_taken = 0

    def move(self):
        # Move ambulance along the path
        if self.start<0 or self.start>self.graph.nodes or self.destination < 0 or self.destination >self.graph.nodes:
            print(f"Destination {self.destination} is too far to reach")
            return
        if self.path:
            next_node = self.path.pop(0)

            # Skip if current location is the same as next node
            if self.current_location == next_node:
                return  # Skip this iteration

            # Ensure the edge exists before trying to access it
            if next_node in self.graph.graph[self.current_location]:
                junction = (self.current_location, next_node)

                # Lock the junction temporarily while the ambulance moves
                self.graph.lock_junction(self.current_location, next_node)

                # Add the cost to the total time taken
                self.time_taken += self.graph.graph[self.current_location][next_node]
                self.current_location = next_node

                # Only unlock the junction if it's not the destination
                if self.current_location != self.destination:
                    self.graph.unlock_junction(self.current_location, next_node)
                else:
                    print(f"Ambulance has reached the destination {self.destination}, no unlock needed.")

    def plan_route(self):
        # Plan the route using Dijkstra's algorithm
        self.path, _ = self.graph.dijkstra(self.start, self.destination)
        if not self.path:
            print(f"Error: No path found for Ambulance from {self.start} to {self.destination}")
        else:
            print(f"Ambulance path: {self.path}")


# Main function to simulate the ambulances' movements
def main():
    # Initialize the graph with 50 nodes (index 0 to 49)
    graph = Graph(15)

    # Create the graph where each node has exactly two edges
    graph.create_graph()

    # Print the graph structure to verify the edges
    # graph.print_graph()

    # Initialize the ambulances
    am1 = Ambulance(start=2, destination=7, graph=graph)  # Ambulance 1 from node 2 to node 47
    am2 = Ambulance(start=14, destination=7, graph=graph)  # Ambulance 2 from node 39 to node 7

    # Plan the routes for both ambulances
    am1.plan_route()
    am2.plan_route()

    # If no valid path exists for either ambulance, skip the simulation
    if not am1.path or not am2.path:
        print("One or both ambulances have no valid path. Ending simulation.")
        return

    # Simulate the movement of the ambulances with time passing
    start_time = time.time()
    while am1.current_location != am1.destination or am2.current_location != am2.destination:
        current_time = time.time()
        if current_time - start_time >= 5:
            start_time = current_time
            graph.update_edge_costs()  # Update edge costs every 5 seconds
        
        # Move ambulances if they have not yet reached their destination
        if am1.current_location != am1.destination:
            am1.move()
        
        if am2.current_location != am2.destination:
            am2.move()

        # Print the status of both ambulances
        print(f"Ambulance 1: {am1.current_location} -> {am1.destination}, Time Taken: {am1.time_taken}s")
        print(f"Ambulance 2: {am2.current_location} -> {am2.destination}, Time Taken: {am2.time_taken}s")

        # Small delay to simulate time passing (for demonstration purposes)
        time.sleep(1)

    print(f"Ambulance 1 arrived at destination {am1.destination} with total time {am1.time_taken}s")
    print(f"Ambulance 2 arrived at destination {am2.destination} with total time {am2.time_taken}s")

if __name__ == "__main__":
    main()
