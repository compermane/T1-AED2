from sys import maxsize

"""
    Um grafo G segue a seguinte implementação:
    G = {
        'A': {'B': 1, 'C': 1}
        'B': {'A': 1}
        'C': {'A': 1}
    }
"""

class Grafo:
    def __init__(self, vertices: dict):
        self.vertices = vertices

    def Dijkstra(self, origem: str):
        distancias = {v: maxsize for v in self.vertices}
        predecessores = {v: None for v in self.vertices}

        distancias[origem] = 0
        visitados = set()

        while visitados != set(distancias):
            atual = None
            menorDistancia = maxsize

            for v in self.vertices:
                if v not in visitados and distancias[v] < menorDistancia:
                    atual = v
                    menorDistancia = distancias[v]

            visitados.add(atual)

            for vizinho, peso in self.vertices[atual].items():
                if distancias[atual] + peso < distancias[vizinho]:
                    distancias[vizinho] = distancias[atual] + peso

        return distancias
    

if __name__ == "__main__":
    grafo = {
        'A': {'B': 5, 'C': 3, 'D': 2},
        'B': {'A': 5, 'C': 2, 'E': 4},
        'C': {'A': 3, 'B': 2, 'D': 1},
        'D': {'A': 2, 'C': 1, 'E': 7},
        'E': {'B': 4, 'D': 7}
    }
    origem = 'A'

    G = Grafo(grafo)
    curto = G.Dijkstra(origem)

    for destino, distancia in curto.items():
        print(f"Caminho mais curto de {origem} para {destino}: {distancia}")
