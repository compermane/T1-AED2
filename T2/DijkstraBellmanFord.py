import networkx as nx

"""
    Os algoritmos implementam os algoritmos de Dijkstra e Bellman-Ford para grafos
    direcionados. Os grafos seguem a seguinte implementação em dicionários:

    vertices = {
        v1: {v2: distancia_de_v1_ate_v2, v3: distancia_de_v1_ate_v3},
        v2: {v3: distancia_de_v2_ate_v3},
        ...
    }
"""

# Cria um grafo direcionado
def criaGrafo(vertices: dict) -> nx.DiGraph:
    G = nx.DiGraph()
    for node, vizinhos in vertices.items():
        for vizinho, peso in vizinhos.items():
            G.add_edge(node, vizinho, weight = peso)
    return G

def dijkstra(G: nx.DiGraph, origin) -> None:
    # Primeiramente, seta as distancias como infinito e os predecessores de cada 
    # vertice como None, com excecao da origem, onde a distancia eh 0
    distancia = {node: float('inf') for node in G.nodes}
    predecessores = {node: None for node in G.nodes}
    distancia[origin] = 0

    # Cria um conjunto de nohs nao visitados
    naoVisitados = set(G.nodes)
    
    while naoVisitados:
        atual = min(naoVisitados, key = lambda node: distancia[node])
        naoVisitados.remove(atual)

        for vizinho in G.neighbors(atual):
            peso = G[atual][vizinho]['weight']
            pesoPotencial = distancia[atual] + peso

            # Relaxamento
            if pesoPotencial < distancia[vizinho]:
                distancia[vizinho] = pesoPotencial
                predecessores[vizinho] = atual

    print(f"Distancias minimas de {origin} até:")
    for node in distancia:
        if distancia[node] == float('inf'):
            print(f"{node}: inalcançável\n")
        else:
            print(f"{node}: {distancia[node]}")
            print("Caminho: ", end = "")
            caminho = list()
            passo = node
            caminho.append(node)

            while predecessores[passo] != None:
                caminho.append(predecessores[passo])
                passo = predecessores[passo]

            print(f"{caminho[::-1]}\n")

def bellmanFord(G: nx.DiGraph, origin) -> None:
    # Comeca setando as distancias como infinito e todo predecessor como none
    distancia = {node: float('inf') for node in G.nodes}
    predecessores = {node: None for node in G.nodes}
    distancia[origin] = 0

    for _ in range(len(G.nodes) - 1):
        for edge in G.edges:
            source, target = edge
            peso = G[source][target]['weight']
            pesoPotencial = distancia[source] + peso

            if pesoPotencial < distancia[target]:
                distancia[target] = pesoPotencial
                predecessores[target] = source

    # Verificar para ciclos de peso negativo
    for edge in G.edges:
        source, target = edge
        peso = G[source][target]['weight']

        if distancia[source] + peso < distancia[target]:
            print("O grafo contem um ciclo de peso negativo")
            return
        
    print(f"Distancias minimas de {origin} até:")
    for node in distancia:
        if distancia[node] == float('inf'):
            print(f"{node}: inalcançável\n")

        else:
            print(f"{node}: {distancia[node]}")
            print("Caminho: ", end = "")
            caminho = list()
            passo = node
            caminho.append(node)

            while predecessores[passo] != None:
                caminho.append(predecessores[passo])
                passo = predecessores[passo]

            print(f"{caminho[::-1]}\n")

if __name__ == "__main__":
    vertices = {
        'A': {'B': 10, 'D': 5},          
        'B': {'C': 1, 'D': 2},          
        'C': {'E': 4},                   
        'D': {'B': 3, 'C': 9, 'E': 2},  
        'E': {'A': 7, 'C': 6}
    }

    G = criaGrafo(vertices)

    vertices2 = {
        'A': {'B': -1, 'C': 4},
        'B': {'C': 3, 'D': 2, 'E': 2},
        'C': {},
        'D': {'B': 1, 'C': 5},
        'E': {'D': -3}
    }

    F = criaGrafo(vertices2)

    # Grafo com loop de peso negativo
    vertices3 = {
        'A': {'B': 2},
        'B': {'C': 3},
        'C': {'D': 1, 'E': 90},
        'E': {'F': -120},
        'F': {'B': 23}
    }

    H = criaGrafo(vertices3)

    print("Dijkstra: ")
    for node in G:
        dijkstra(G, node)

    print("\n")

    print("Bellman-Ford")
    for node in F:
        bellmanFord(F, node)
    
    print("\nGrafo H:")
    bellmanFord(H, 'A')
