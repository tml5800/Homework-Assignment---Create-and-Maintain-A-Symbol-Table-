from collections import deque
import heapq

# Graph examples
graph = {
    "a": ["b", "c"],
    "b": ["c", "d"],
    "c": ["d"],
    "d": ["c"],
    "e": ["f"],
    "f": []
}

graph1 = {
    "a": ["d", "f"],
    "b": ["c"],
    "c": ["b", "c", "d", "e"],
    "d": ["a", "c"],
    "e": ["c"],
    "f": ["a"]
}

# Generate all edges
def generate_edges(graph):
    edges = []
    for node in graph:
        for neighbour in graph[node]:
            edges.append((node, neighbour))
    return edges

# Find isolated nodes
def find_isolated_nodes(graph):
    isolated = []
    for node in graph:
        if not graph[node]:
            isolated.append(node)
    return isolated

# Find one path
def find_path(graph, start, end, path=None):
    if path is None:
        path = []
    path = path + [start]
    if start == end:
        return path
    if start not in graph:
        return None
    for node in graph[start]:
        if node not in path:
            new_path = find_path(graph, node, end, path)
            if new_path:
                return new_path
    return None

# Find all paths
def find_all_paths(graph, start, end, path=[]):
    path = path + [start]
    if start == end:
        return [path]
    if start not in graph:
        return []
    paths = []
    for node in graph[start]:
        if node not in path:
            newpaths = find_all_paths(graph, node, end, path)
            for p in newpaths:
                paths.append(p)
    return paths

# Check if connected
def is_connected(graph, visited=None, node=None):
    if visited is None:
        visited = set()
    if node is None:
        node = list(graph.keys())[0]
    visited.add(node)
    for neighbor in graph[node]:
        if neighbor not in visited:
            is_connected(graph, visited, neighbor)
    return len(visited) == len(graph)

# BFS
def bfs(graph, start):
    visited = []
    queue = deque([start])
    while queue:
        vertex = queue.popleft()
        if vertex not in visited:
            visited.append(vertex)
            queue.extend([n for n in graph[vertex] if n not in visited])
    return visited

# DFS
def dfs(graph, start, visited=None):
    if visited is None:
        visited = []
    if start not in visited:
        visited.append(start)
        for neighbor in graph[start]:
            dfs(graph, neighbor, visited)
    return visited

# Kruskal
def kruskal(graph_edges):
    parent = {}
    def find(v):
        while parent[v] != v:
            parent[v] = parent[parent[v]]
            v = parent[v]
        return v

    def union(u, v):
        root1 = find(u)
        root2 = find(v)
        parent[root1] = root2

    mst = []
    for u, v, weight in sorted(graph_edges, key=lambda x: x[2]):
        if u not in parent:
            parent[u] = u
        if v not in parent:
            parent[v] = v
        if find(u) != find(v):
            union(u, v)
            mst.append((u, v, weight))
    return mst

# Prim
def prim(graph):
    start = next(iter(graph))
    visited = set([start])
    edges = [(0, start, to) for to in graph[start]]
    heapq.heapify(edges)
    mst = []
    while edges:
        weight, frm, to = heapq.heappop(edges)
        if to not in visited:
            visited.add(to)
            mst.append((frm, to, weight))
            for to_next in graph[to]:
                heapq.heappush(edges, (1, to, to_next))
    return mst

# Dijkstra
def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    heap = [(0, start)]
    while heap:
        current_dist, current_node = heapq.heappop(heap)
        if current_dist > distances[current_node]:
            continue
        for neighbor in graph[current_node]:
            distance = current_dist + 1
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(heap, (distance, neighbor))
    return distances

# Test it all
if __name__ == "__main__":
    print("Edges:", generate_edges(graph))
    print("Isolated Nodes:", find_isolated_nodes(graph))
    print("Path a to d:", find_path(graph, "a", "d"))
    print("All Paths a to d:", find_all_paths(graph, "a", "d"))
    print("Is Connected:", is_connected(graph))
    print("BFS:", bfs(graph, "a"))
    print("DFS:", dfs(graph, "a"))
    print("Dijkstra:", dijkstra(graph, "a"))
    print("Prim’s MST:", prim(graph))
    weighted_edges = [('a', 'b', 1), ('a', 'c', 2), ('b', 'd', 3), ('c', 'd', 1), ('e', 'f', 5)]
    print("Kruskal’s MST:", kruskal(weighted_edges))
