from collections import deque

class Graph:
    def __init__(self):
        self.graph = {
            'A1': {'N1': 'baixo'},
            'A2': {'N2': 'baixo'},
            'A3': {'N3': 'baixo'},
            'A4': {'N4': 'baixo'},
            'B1': {'N15': 'cima'},
            'B2': {'N16': 'cima'},
            'B3': {'N17': 'cima'},
            'B4': {'N18': 'cima'},
            'N1': {'A1': 'cima', 'N2': 'direita', 'N7': 'baixo'},
            'N2': {'A2': 'cima', 'N3': 'direita', 'N1': 'esquerda'},
            'N3': {'A3': 'cima', 'N4': 'direita', 'N2': 'esquerda'},
            'N4': {'A4': 'cima', 'N5': 'direita', 'N3': 'esquerda'},
            'N5': {'N4': 'esquerda', 'N6': 'direita', 'N8': 'baixo'},
            'N6': {'N5': 'esquerda', 'N9': 'baixo'},
            'N7': {'N1': 'cima', 'N8': 'direita', 'N10': 'baixo'},
            'N8': {'N7': 'esquerda', 'N5': 'cima', 'N9': 'direita', 'N11': 'baixo'},
            'N9': {'N6': 'cima', 'N8': 'esquerda', 'N12': 'baixo'},
            'N10': {'N7': 'cima', 'N11': 'direita', 'N13': 'baixo'},
            'N11': {'N8': 'cima', 'N10': 'esquerda', 'N12': 'direita', 'N14': 'baixo'},
            'N12': {'N9': 'cima', 'N11': 'esquerda', 'N18': 'baixo'},
            'N13': {'N10': 'cima', 'N14': 'direita'},
            'N14': {'N11': 'cima', 'N13': 'esquerda', 'N15': 'direita'},
            'N15': {'N14': 'esquerda', 'B1': 'baixo', 'N16': 'direita'},
            'N16': {'N15': 'esquerda', 'B2': 'baixo', 'N17': 'direita'},
            'N17': {'N16': 'esquerda', 'B3': 'baixo', 'N18': 'direita'},
            'N18': {'N17': 'esquerda', 'B4': 'baixo', 'N12': 'cima'},
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

path = graph_instance.path_BFS('B4', 'A4')
print("Path:", path)


for i in range(len(path) - 1):
    position = graph_instance.get_relative_position(path[i], path[i + 1])
    print(f"De '{path[i]}' para '{path[i + 1]}': {position}")