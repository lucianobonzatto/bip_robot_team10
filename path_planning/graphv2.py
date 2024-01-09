from collections import deque

class Graph:
    def __init__(self):
        self.graph = {
            'A1': {'N1': 'down'},
            'A2': {'N2': 'down'},
            'A3': {'N3': 'down'},
            'A4': {'N4': 'down'},
            'B1': {'N15': 'up'},
            'B2': {'N16': 'up'},
            'B3': {'N17': 'up'},
            'B4': {'N18': 'up'},
            'N1': {'A1': 'up', 'N2': 'right', 'N7': 'down'},
            'N2': {'A2': 'up', 'N3': 'right', 'N1': 'left'},
            'N3': {'A3': 'up', 'N4': 'right', 'N2': 'left'},
            'N4': {'A4': 'up', 'N5': 'right', 'N3': 'left'},
            'N5': {'N4': 'left', 'N6': 'right', 'N8': 'down'},
            'N6': {'N5': 'left', 'N9': 'down'},
            'N7': {'N1': 'up', 'N8': 'right', 'N10': 'down'},
            'N8': {'N7': 'left', 'N5': 'up', 'N9': 'right', 'N11': 'down'},
            'N9': {'N6': 'up', 'N8': 'left', 'N12': 'down'},
            'N10': {'N7': 'up', 'N11': 'right', 'N13': 'down'},
            'N11': {'N8': 'up', 'N10': 'left', 'N12': 'right', 'N14': 'down'},
            'N12': {'N9': 'up', 'N11': 'left', 'N18': 'down'},
            'N13': {'N10': 'up', 'N14': 'right'},
            'N14': {'N11': 'up', 'N13': 'left', 'N15': 'right'},
            'N15': {'N14': 'left', 'B1': 'down', 'N16': 'right'},
            'N16': {'N15': 'left', 'B2': 'down', 'N17': 'right'},
            'N17': {'N16': 'left', 'B3': 'down', 'N18': 'right'},
            'N18': {'N17': 'left', 'B4': 'down', 'N12': 'up'},
        }

    def path_BFS(self, start, goal):
        # Check if both start and goal nodes exist in the graph
        if start not in self.graph or goal not in self.graph:
            return None

        queue = deque([[start]])
        visited = set()

        while queue:
            path = queue.popleft()
            node = path[-1]

            if node == goal:
                return path  # Return the path if the goal is reached

            if node not in visited:
                visited.add(node)

                # Iterate through adjacent nodes
                for adjacent in self.graph[node]:
                    if adjacent not in visited:
                        new_path = list(path)
                        new_path.append(adjacent)
                        queue.append(new_path)
        
        return None
    
    def get_relative_position(self, start, end):
        if start in self.graph and end in self.graph[start]:
            return self.graph[start][end]
        else:
            return None


    def __str__(self):
        return str(self.graph)

graph_instance = Graph()
print(graph_instance)

path = graph_instance.path_BFS('B4', 'A3')
print("Path:", path)


for i in range(len(path) - 1):
    position = graph_instance.get_relative_position(path[i], path[i + 1])
    print(f"'{path[i]}' to '{path[i + 1]}': {position}")