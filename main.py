import collections
import heapq

# BFS - Breadth-First Search (Parcours en largeur) 
def bfs(graph, start):
    visited = set()
    queue = collections.deque([start])
    visited.add(start)
    
    while queue:
        node = queue.popleft()
        print(node)
        for neighbor in graph[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)

# DFS - Depth-First Search (Parcours en profondeur)
def dfs(graph, start, visited=None):
    if visited is None:
        visited = set()
    visited.add(start)
    print(start)
    for neighbor in graph[start]:
        if neighbor not in visited:
            dfs(graph, neighbor, visited)

# Dijkstra - Shorter path (calcule du chemin le plus court)
def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    pq = [(0, start)]
    
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        
        if current_distance > distances[current_node]:
            continue
        
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))
    
    return distances

# kruskal
def find(parent, i):
    if parent[i] == i:
        return i
    return find(parent, parent[i])

def union(parent, rank, x, y):
    xroot = find(parent, x)
    yroot = find(parent, y)
    if rank[xroot] < rank[yroot]:
        parent[xroot] = yroot
    elif rank[xroot] > rank[yroot]:
        parent[yroot] = xroot
    else:
        parent[yroot] = xroot
        rank[xroot] += 1

def kruskal(graph):
    result = []
    parent = {}
    rank = {}

    edges = sorted(graph, key=lambda x: graph[x])
    
    for node in graph:
        parent[node] = node
        rank[node] = 0

    for edge in edges:
        weight, u, v = edge
        x = find(parent, u)
        y = find(parent, v)
        if x != y:
            result.append((u, v, weight))
            union(parent, rank, x, y)

    return result

# SCC - Strongly Connected Components search (algorithme de Tarjan pour la recherche de composantes fortement connexes)
def tarjan_scc(graph):
    index = {}
    lowlink = {}
    on_stack = set()
    stack = []
    result = []

    def strongconnect(node):
        index[node] = len(index)
        lowlink[node] = len(lowlink)
        stack.append(node)
        on_stack.add(node)
        for neighbor in graph[node]:
            if neighbor not in index:
                strongconnect(neighbor)
                lowlink[node] = min(lowlink[node], lowlink[neighbor])
            elif neighbor in on_stack:
                lowlink[node] = min(lowlink[node], index[neighbor])
        if index[node] == lowlink[node]:
            component = []
            while True:
                neighbor = stack.pop()
                on_stack.remove(neighbor)
                component.append(neighbor)
                if neighbor == node:
                    break
            result.append(component)

    for node in graph:
        if node not in index:
            strongconnect(node)

    return result

# Graphe coloriation
def welsh_powell(graph):
    color_map = {}
    nodes = sorted(graph, key=lambda x: len(graph[x]), reverse=True)
    color = 0
    for node in nodes:
        if node not in color_map:
            color_map[node] = color
            for neighbor in graph[node]:
                if neighbor not in color_map:
                    color_map[neighbor] = color
            color += 1
    return color_map

# Ford-Fulkerson algorithm (Algorithme de Flot maximal) 
def ford_fulkerson(graph, source, sink):
    max_flow = 0
    visited = set()
    dfs(graph, source, visited)
    while sink not in visited:
        path = None
        for node in visited:
            for neighbor in graph[node]:
                if neighbor not in visited and graph[node][neighbor] > 0:
                    path = (node, neighbor)
                    break
            if path:
                break
        if not path:
            break
        node, neighbor = path
        flow = graph[node][neighbor]
        while neighbor != source:
            flow = min(flow, graph[node][neighbor])
            neighbor = node
            node = None
            for next_node in visited:
                if next_node in graph[neighbor] and graph[neighbor][next_node] > 0:
                    node = next_node
                    break
            if node is None:
                break
        max_flow += flow
        neighbor = path[1]
        while neighbor != source:
            node = path[0]
            graph[node][neighbor] -= flow
            graph[neighbor][node] += flow
            neighbor = node
            for next_node in visited:
                if next_node in graph[neighbor] and graph[neighbor][next_node] > 0:
                    node = next_node
                    break
        visited = set()
        dfs(graph, source, visited)
    return max_flow

