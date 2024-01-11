from collections import deque

class Graph:
    def __init__(self):
        self.graph = {
            'A1': {'N1': 'down'},
            'A2': {'N2': 'down'},
            'A3': {'N3': 'down'},
            'A4': {'N4': 'down'},
            'B1': {'N18': 'up'},
            'B2': {'N19': 'up'},
            'B3': {'N20': 'up'},
            'B4': {'N21': 'up'},
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
            'N12': {'N9': 'up', 'N11': 'left', 'N15': 'down'},
            'N13': {'N10': 'up', 'N14': 'right', 'N16': 'down'},
            'N14': {'N11': 'up', 'N13': 'left', 'N15': 'right', 'N17': 'down'},
            'N15': {'N14': 'left', 'N21': 'down', 'N12': 'up'},
            'N16': {'N13': 'up', 'N17': 'right'},
            'N17': {'N14': 'up', 'N16': 'left', 'N18': 'right'},
            'N18': {'N17': 'left', 'B1': 'down', 'N19': 'right'},
            'N19': {'N18': 'left', 'B2': 'down', 'N20': 'right'},
            'N20': {'N19': 'left', 'B3': 'down', 'N21': 'right'},
            'N21': {'N15': 'up'  , 'B4': 'down', 'N20': 'left'},
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

    def get_commands(self, start, end, initial_orientation):
        path = self.path_BFS(start, end)
        if not path:
            return ["Caminho não encontrado"]
        
        direction_to_command = {
            ('up', 'right'): 'goRight',
            ('up', 'left'): 'goLeft',
            ('up', 'down'): 'goLeft',

            ('right', 'up'): 'goLeft',
            ('right', 'down'): 'goRight',
            ('right', 'left'): 'goLeft',

            ('down', 'left'): 'goRight',
            ('down', 'right'): 'goLeft',
            ('down', 'up'): 'goLeft',

            ('left', 'down'): 'goLeft',
            ('left', 'up'): 'goRight',
            ('left', 'right'): 'goLeft',
        }
        
        commands = []
        current_position = start
        current_orientation = initial_orientation

        for step in path:
            if(step == current_position):
                continue
            
            step_orientation = self.get_relative_position(current_position, step)
            
            if(step_orientation == current_orientation):
                commands.append('followLine')
                current_position = step
                current_orientation = step_orientation
            else:
                rotation_action = direction_to_command.get((current_orientation, step_orientation))
                new = self.get_new_orientation(current_position, current_orientation, rotation_action)
                
                if(new == step_orientation):
                    commands.append('goFront')
                    commands.append(rotation_action)
                    commands.append('followLine')
                else:
                    commands.append('goFront')
                    commands.append(rotation_action)
                    commands.append(rotation_action)
                    commands.append('followLine')
                
                current_position = step
                current_orientation = step_orientation

        return commands
    
    def get_new_orientation(self, node, current_orientation, rotation_action):
        rotation_map = {
            'up': {'goRight': ['right', 'down', 'left', 'up'],
                'goLeft': ['left', 'down', 'right', 'up']},
            'down': {'goRight': ['left', 'up', 'right', 'down'],
                    'goLeft': ['right', 'up', 'left', 'down']},
            'right': {'goRight': ['down', 'left', 'up', 'right'],
                    'goLeft': ['up', 'left', 'down', 'right']},
            'left': {'goRight': ['up', 'right', 'down', 'left'],
                    'goLeft': ['down', 'right', 'up', 'left']},
        }
        
        if node in self.graph and current_orientation in rotation_map:
            relative_position = []
            for orient in self.graph[node]:
                relative_position.append(self.graph[node][orient])
            
            for vl in rotation_map[current_orientation][rotation_action]:
                if vl in relative_position:
                    return vl

        return None
    
    def __str__(self):
        return str(self.graph)
