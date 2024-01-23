from collections import deque

class Graph:
    def __init__(self):
        self.graph = {
            'A1': ['N1'],
            'A2': ['N2'],
            'A3': ['N3'],
            'A4': ['N4'],
            'B1': ['N15'],
            'B2': ['N16'],
            'B3': ['N17'],
            'B4': ['N18'],
            'N1': ['A1', 'N2', 'N7'],
            'N2': ['A2', 'N3', 'N1'],
            'N3': ['A3', 'N2', 'N4'],
            'N4': ['A4', 'N3', 'N5'],
            'N5': ['N4', 'N8', 'N6'],
            'N6': ['N5', 'N9'],
            'N7': ['N1', 'N8', 'N10'],
            'N8': ['N7', 'N5', 'N9', 'N11'],
            'N9': ['N6', 'N8', 'N12'],
            'N10': ['N7', 'N11', 'N13'],
            'N11': ['N8', 'N10', 'N12', 'N14'],
            'N12': ['N9', 'N11', 'N18'],
            'N13': ['N10', 'N14'],
            'N14': ['N11', 'N13', 'N15'],
            'N15': ['B1', 'N14', 'N16'],
            'N16': ['B2', 'N15', 'N17'],
            'N17': ['B3', 'N16', 'N18'],
            'N18': ['B4', 'N17', 'N12']
        }
    
    def path_BFS(self, start, goal):
        # Check if both start and goal nodes exist in the graph
        if start not in self.graph or goal not in self.graph:
            return None

        # Use BFS to find the shortest path
        queue = deque([[start]])  # Queue of paths
        visited = set()

        while queue:
            path = queue.popleft()
            node = path[-1]

            if node == goal:
                return path  # Return the path if the goal is reached

            if node not in visited:
                visited.add(node)
                for adjacent in self.graph.get(node, []):
                    new_path = list(path)
                    new_path.append(adjacent)
                    queue.append(new_path)
        
        return None  # If no path is found

    
    def __str__(self):
        return str(self.graph)

graph_instance = Graph()
print(graph_instance)

path = graph_instance.path_BFS('B4', 'A4')
print("Path:", path)
