import heapq

# College Square: A, Lewis Science Center: B, Speech Language Hearing: C
# Prince Center: D, Computer Science: E, Burdick F, Torreyson Library: G
# Maintenance College: H, Old Main: I, Police Dept: J, Fine Art: K,
# McAlister Hall: L, Student Health Center: M, Student Center: N,
# Wingo: O, New Business Building: P, Oak Tree Apts: Q,
# Brewer Hegema: R, Bear Village Apt: S
graph = {
    'A': {'B': 200, 'D': 300},
    'B': {'A': 200, 'E': 150, 'C': 250},
    'C': {'B': 250, 'F': 100, 'H': 120},
    'D': {'A': 300, 'E': 80, 'G': 30, 'J': 100},
    'E': {'B': 150, 'F': 30, 'G': 40, 'D': 80},
    'F': {'C': 100, 'L': 200, 'G': 80, 'E': 30},
    'G': {'E': 40, 'F': 80, 'I': 30, 'D': 30},
    'H': {'C': 120, 'Q': 160, 'P': 150, 'O': 100, 'L': 150},
    'I': {'G': 30, 'L': 100, 'K': 90, 'J': 200},
    'J': {'D': 100, 'I': 200, 'K': 50, 'M': 100},
    'K': {'I': 90, 'L': 180, 'N': 80, 'J': 50},
    'L': {'F': 200, 'H': 150, 'O': 50, 'N': 100, 'K': 180, 'I': 100},
    'M': {'J': 100, 'N': 50, 'R': 200},
    'N': {'K': 80, 'L': 100, 'O': 100, 'P': 110, 'M': 50},
    'O': {'L': 50, 'H': 100, 'P': 50, 'N': 100},
    'P': {'O': 50, 'H': 150, 'Q': 30, 'R': 20, 'N': 110},
    'Q': {'H': 150, 'R': 40, 'P': 30},
    'R': {'P': 20, 'Q': 40, 'S': 350, 'M': 200},
    'S': {'R': 350}
}


def dijkstra(adj_matrix, start):
    # Initialize distance dictionary with start node as 0 and all others as infinity
    distances = {node: float('inf') for node in range(len(adj_matrix))}
    distances[start] = 0

    # Initialize heap with start node and its distance as tuple
    heap = [(0, start)]

    # Initialize visited set
    visited = set()

    while heap:
        # Pop smallest distance node from heap
        curr_dist, curr_node = heapq.heappop(heap)

        # Check if node has already been visited
        if curr_node in visited:
            continue

        # Add node to visited set
        visited.add(curr_node)

        # Update distances for neighbors of current node
        for neighbor, weight in enumerate(adj_matrix[curr_node]):
            if weight > 0 and neighbor not in visited:
                new_dist = curr_dist + weight
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    heapq.heappush(heap, (new_dist, neighbor))

    return distances


def bellman_ford(adj_matrix, start):
    # Initialize distance dictionary with start node as 0 and all others as infinity
    distances = {node: float('inf') for node in range(len(adj_matrix))}
    distances[start] = 0

    # Iterate over all nodes n-1 times to update distances
    for i in range(len(adj_matrix) - 1):
        for node in range(len(adj_matrix)):
            for neighbor, weight in enumerate(adj_matrix[node]):
                if weight > 0:
                    new_dist = distances[node] + weight
                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist

    # Check for negative weight cycles
    for node in range(len(adj_matrix)):
        for neighbor, weight in enumerate(adj_matrix[node]):
            if weight > 0:
                if distances[node] + weight < distances[neighbor]:
                    raise ValueError("Graph contains negative weight cycle")

    return distances


def main():
    # Convert graph to adjacency matrix
    adj_matrix = [[0] * len(graph) for _ in range(len(graph))]
    node_to_index = {node: index for index, node in enumerate(graph)}
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors.items():
            adj_matrix[node_to_index[node]][node_to_index[neighbor]] = weight

    # Run Dijkstra's Algorithm on the graph
    start_node = 'A'
    start_index = node_to_index[start_node]
    dijkstra_distances = dijkstra(adj_matrix, start_index)

    # Run Bellman-Ford Algorithm on the graph
    try:
        bellman_ford(adj_matrix, start_index)
    except ValueError as e:
        print(e)
        return

    # Print shortest path distances
    for node, index in node_to_index.items():
        print(f"Shortest path distance from {start_node} to {node}:")
        print(f"Dijkstra's Algorithm: {dijkstra_distances[index]}")

    # Run bellman ford and print distances
    bellman_ford_distances = bellman_ford(adj_matrix, 0)
    print("\nBellman-Ford Algorithm: ")
    for node, dist in bellman_ford_distances.items():
        print(f"Node {node}: Distance from start node - {dist}")


if __name__ == '__main__':
    main()
