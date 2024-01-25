import matplotlib.pyplot as plt
import networkx as nx

# Define the graph structure from the provided dictionary
graph_structure = {
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
    'N21': {'N15': 'up', 'B4': 'down', 'N20': 'left'},
}

# Create a directed graph
G = nx.DiGraph()

# Add edges to the graph
for node, edges in graph_structure.items():
    for connected_node, direction in edges.items():
        G.add_edge(node, connected_node)

# Definir posições dos nós manualmente
pos = {
    'A1': (0, 4), 'A2': (1, 4), 'A3': (2, 4), 'A4': (3, 4),
    'N1': (0, 3), 'N2': (1, 3), 'N3': (2, 3), 'N4': (3, 3), 'N5': (4, 3),                                                   'N6': (8, 3),
    'N7': (0, 2),                                           'N8': (4, 2),                                                   'N9': (8, 2),
    'N10': (0, 1),                                          'N11': (4, 1),                                                  'N12': (8, 1),
    'N13': (0, 0),                                          'N14': (4, 0),                                                  'N15': (8, 0),
    'N16': (0, -1),                                         'N17': (4, -1), 'N18': (5, -1), 'N19': (6, -1), 'N20': (7, -1), 'N21': (8, -1),
                                                                            'B1': (5, -2),  'B2': (6, -2),  'B3': (7, -2),  'B4': (8, -2)
}

# Desenhar o gráfico
plt.figure(figsize=(12, 8))
nx.draw_networkx_nodes(G, pos, node_size=2700)
# nx.draw_networkx_edges(G, pos, edgelist=G.edges(), edge_color='black')
# nx.draw_networkx_edges(G, pos, edgelist=G.edges(), style='solid')
nx.draw_networkx_edges(G, pos, edgelist=G.edges(), arrows=False)

nx.draw_networkx_labels(G, pos, font_size=25, font_family="sans-serif")

plt.title("Map Representation",  fontsize=30)
plt.axis("off")
plt.show()