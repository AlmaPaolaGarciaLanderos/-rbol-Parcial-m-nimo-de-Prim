#Alma Paola Garcia Landeros
#21110038
#6E1
#Inteligencia Artificial
#Centro de Enseñanza Tecnica Industrial

import sys

def prim_mst(graph):
    """
    Función para encontrar el Árbol Parcial Mínimo de Prim en un grafo dado.

    Args:
    - graph: Grafo representado como una matriz de adyacencia.

    Returns:
    - mst: Lista de aristas que forman el Árbol Parcial Mínimo.
    """
    n = len(graph)
    mst = []  # Lista para almacenar el Árbol Parcial Mínimo
    min_weight = [float('inf')] * n  # Inicializa todas las distancias como infinito
    parent = [-1] * n  # Lista para rastrear el nodo padre en el Árbol Parcial Mínimo
    visited = [False] * n  # Lista para rastrear nodos visitados
    
    # Comienza desde el primer nodo
    min_weight[0] = 0
    parent[0] = -1
    
    for _ in range(n):
        # Encuentra el nodo no visitado con la mínima distancia actual
        u = min_distance_node(min_weight, visited)
        visited[u] = True
        
        # Si no es el primer nodo, agrega la arista al MST
        if parent[u] != -1:
            mst.append((parent[u], u))
        
        # Actualiza las distancias de los nodos adyacentes no visitados
        for v in range(n):
            if not visited[v] and graph[u][v] < min_weight[v]:
                min_weight[v] = graph[u][v]
                parent[v] = u
        
        # Muestra paso a paso el progreso del MST
        print_step_by_step(graph, mst, min_weight, visited, u)
    
    return mst

def min_distance_node(min_weight, visited):
    """
    Encuentra el nodo no visitado con la mínima distancia actual.

    Args:
    - min_weight: Lista de mínimas distancias a cada nodo.
    - visited: Lista que indica si cada nodo ha sido visitado.

    Returns:
    - El nodo no visitado con la mínima distancia.
    """
    min_dist = float('inf')
    min_node = -1

    for i in range(len(min_weight)):
        if not visited[i] and min_weight[i] < min_dist:
            min_dist = min_weight[i]
            min_node = i

    return min_node

def print_step_by_step(graph, mst, min_weight, visited, u):
    """
    Muestra paso a paso la construcción del MST.

    Args:
    - graph: Grafo representado como una matriz de adyacencia.
    - mst: Lista de aristas que forman el Árbol Parcial Mínimo.
    - min_weight: Lista de mínimas distancias a cada nodo.
    - visited: Lista que indica si cada nodo ha sido visitado.
    - u: Nodo actual en el paso actual.
    """
    print(f"\nExplorando nodo {u}. Aristas en el Árbol Parcial Mínimo hasta el momento:")
    for edge in mst:
        print(f"Arista: {edge[0]} - {edge[1]}")
    
    print("\nDistancias mínimas actuales:")
    for i in range(len(min_weight)):
        print(f"Nodo {i}: {min_weight[i]}")
    
    print("\nNodos visitados:")
    for i in range(len(visited)):
        if visited[i]:
            print(f"Nodo {i}: Visitado")
        else:
            print(f"Nodo {i}: No visitado")

# Ejemplo de uso:
graph = [
    [0, 2, 0, 6, 0],
    [2, 0, 3, 8, 5],
    [0, 3, 0, 0, 7],
    [6, 8, 0, 0, 9],
    [0, 5, 7, 9, 0]
]

print("Simulador de Árbol Parcial Mínimo de Prim")
print("Grafo de ejemplo:")
for row in graph:
    print(row)

print("\nCalculando Árbol Parcial Mínimo de Prim...")
mst = prim_mst(graph)

print("\nÁrbol Parcial Mínimo de Prim:")
for edge in mst:
    print(f"Arista: {edge[0]} - {edge[1]}")
