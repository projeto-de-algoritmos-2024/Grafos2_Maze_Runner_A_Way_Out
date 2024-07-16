import pygame
import random
import heapq

# Configurações
RESOLUTION = 720
DIM = 15
LENGTH = RESOLUTION // DIM
BUTTON_WIDTH = 200
BUTTON_HEIGHT = 50
BUTTON_X = (RESOLUTION - BUTTON_WIDTH) // 3
BUTTON_Y = RESOLUTION + 10
BUTTON_X_NEW_MAZE = (BUTTON_X + BUTTON_WIDTH + 10)

# Cores
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)

class PriorityQueue:
    """Implementação de fila de prioridade usando heapq."""
    
    def __init__(self):
        """Inicializa a fila de prioridade."""
        self.elements = []

    def empty(self):
        """Verifica se a fila de prioridade está vazia."""
        return len(self.elements) == 0

    def put(self, item, priority):
        """Coloca um item na fila de prioridade com uma prioridade dada."""
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        """Remove e retorna o item com a menor prioridade."""
        return heapq.heappop(self.elements)[1]

class Vertex:
    """Representa um vértice em um grafo."""

    def __init__(self, id):
        """Inicializa um vértice."""
        self.id = id
        self.edges = []
        self.visited = False  # Adicionado para controlar o estado de visita
        self.distance = float('inf')
        self.prev = None

    def add_edge(self, edge):
        """Adiciona uma aresta ao vértice."""
        self.edges.append(edge)

    def get_neighbors(self):
        """Obtém os vizinhos do vértice."""
        return [e.end_vertex for e in self.edges if not e.is_wall]

    def get_edges(self):
        """Obtém as arestas do vértice."""
        return self.edges

    def __lt__(self, other):
        """Compara as distâncias entre vértices."""
        return self.distance < other.distance

class Edge:
    """Representa uma aresta em um grafo."""
    
    def __init__(self, start_vertex, end_vertex):
        """Inicializa uma aresta."""
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.is_wall = True

class Graph:
    """Representa um grafo."""
    
    def __init__(self):
        """Inicializa um grafo."""
        self.vertices = {}
        self.exit_vertex = None

    def add_vertex(self, id):
        """Adiciona um vértice ao grafo."""
        self.vertices[id] = Vertex(id)

    def get_vertex(self, id):
        """Obtém um vértice do grafo."""
        return self.vertices[id]

    def add_edge(self, start_vertex_id, end_vertex_id):
        """Adiciona uma aresta entre dois vértices."""
        start_vertex = self.get_vertex(start_vertex_id)
        end_vertex = self.get_vertex(end_vertex_id)
        edge = Edge(start_vertex, end_vertex)
        start_vertex.add_edge(edge)
        reverse_edge = Edge(end_vertex, start_vertex)
        end_vertex.add_edge(reverse_edge)

    def remove_wall(self, start_vertex_id, end_vertex_id):
        """Remove a parede entre dois vértices."""
        start_vertex = self.get_vertex(start_vertex_id)
        end_vertex = self.get_vertex(end_vertex_id)
        for edge in start_vertex.edges:
            if edge.end_vertex == end_vertex:
                edge.is_wall = False
        for edge in end_vertex.edges:
            if edge.end_vertex == start_vertex:
                edge.is_wall = False

    def reset(self):
        """Reseta o estado dos vértices para não visitado."""
        for vertex in self.vertices.values():
            vertex.visited = False
        self.exit_vertex = self.get_vertex(get_index(LENGTH - 1, LENGTH - 1))

class Dijkstra:
    """Implementação do algoritmo de Dijkstra para encontrar o caminho mais curto em um grafo."""

    def __init__(self, graph, target):
        """Inicializa o algoritmo de Dijkstra."""
        self.graph = graph
        self.visited = []  # Lista para armazenar os vértices visitados durante o caminho
        self.target = target

    def shortest_path(self, start_id):
        """Calcula o caminho mais curto do vértice inicial para todos os outros vértices."""
        pq = PriorityQueue()
        start = self.graph.get_vertex(start_id)
        start.distance = 0
        pq.put(start, start.distance)

        while not pq.empty():
            vertex = pq.get()
            vertex.visited = True  # Marca o vértice como visitado
            self.visited.append(vertex)  # Adiciona o vértice à lista de visitados
            if vertex == self.target:
                break

            for neighbor in vertex.get_neighbors():
                if not neighbor.visited:
                    cost = vertex.distance + 1
                    if cost < neighbor.distance:
                        neighbor.distance = cost
                        pq.put(neighbor, cost)
                        neighbor.prev = vertex

    def path_to_target(self):
        """Obtém o caminho do vértice inicial ao vértice alvo."""
        path = []
        current = self.graph.get_vertex(self.target.id)
        while current:
            path.append(current)
            current = current.prev
        return path[::-1]

def get_neighbors_coordinates(x, y):
    """Obtém as coordenadas dos vizinhos de uma célula na grade."""
    coordinates = [
        (x, y - 1),
        (x + 1, y),
        (x, y + 1),
        (x - 1, y)
    ]
    return [(x, y) for (x, y) in coordinates if 0 <= x < LENGTH and 0 <= y < LENGTH]

def get_index(x, y):
    """Obtém o índice de uma célula na grade a partir de suas coordenadas."""
    return x + y * LENGTH

def coordinates(id):
    """Obtém as coordenadas de uma célula na grade a partir do seu índice."""
    x = id % LENGTH
    y = id // LENGTH
    return (x, y)

def draw_cell(screen, x, y, walls):
    """Desenha uma célula na grade."""
    cell_size = RESOLUTION // LENGTH
    cell_x = x * cell_size
    cell_y = y * cell_size

    if len(walls) > 0 and walls[0]:  # Parede superior
        pygame.draw.line(screen, WHITE, (cell_x, cell_y), (cell_x + cell_size, cell_y), 2)
    if len(walls) > 1 and walls[1]:  # Parede direita
        pygame.draw.line(screen, WHITE, (cell_x + cell_size, cell_y), (cell_x + cell_size, cell_y + cell_size), 2)
    if len(walls) > 2 and walls[2]:  # Parede inferior
        pygame.draw.line(screen, WHITE, (cell_x + cell_size, cell_y + cell_size), (cell_x, cell_y + cell_size), 2)
    if len(walls) > 3 and walls[3]:  # Parede esquerda
        pygame.draw.line(screen, WHITE, (cell_x, cell_y + cell_size), (cell_x, cell_y), 2)

def draw_buttons(screen, font):
    """Desenha os botões na parte inferior da tela."""
    button_new_maze = pygame.Rect(BUTTON_X, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT)
    button_find_path = pygame.Rect(BUTTON_X_NEW_MAZE, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT)

    pygame.draw.rect(screen, WHITE, button_new_maze)
    pygame.draw.rect(screen, WHITE, button_find_path)

    text_surface_new_maze = font.render("New Maze", True, BLACK)
    text_surface_find_path = font.render("Find Path", True, BLACK)

    screen.blit(text_surface_new_maze, (button_new_maze.centerx - text_surface_new_maze.get_width() // 2,
                                        button_new_maze.centery - text_surface_new_maze.get_height() // 2))
    screen.blit(text_surface_find_path, (button_find_path.centerx - text_surface_find_path.get_width() // 2,
                                         button_find_path.centery - text_surface_find_path.get_height() // 2))

def draw_path(screen, path):
    """Desenha o caminho encontrado pelo algoritmo de Dijkstra."""
    cell_size = RESOLUTION // LENGTH
    for vertex in path:
        x, y = coordinates(vertex.id)
        pygame.draw.rect(screen, GREEN, (x * cell_size + cell_size // 8, y * cell_size + cell_size // 8,
                                         cell_size // 1, cell_size // 1))

def create_maze(graph):
    """Gera um labirinto usando o algoritmo de Prim."""
    stack = []
    start_vertex = graph.get_vertex(0)  # Ponto inicial no canto superior esquerdo
    start_vertex.visited = True
    stack.append(start_vertex)

    while stack:
        current = stack.pop()
        neighbors = [
            (nx, ny) for nx, ny in get_neighbors_coordinates(*coordinates(current.id))
            if not graph.get_vertex(get_index(nx, ny)).visited
        ]
        if neighbors:
            stack.append(current)
            nx, ny = random.choice(neighbors)
            neighbor = graph.get_vertex(get_index(nx, ny))
            graph.remove_wall(current.id, neighbor.id)  # Remove parede entre atual e vizinho
            neighbor.visited = True
            stack.append(neighbor)

    for vertex in graph.vertices.values():
        vertex.visited = False

    # Marcar a saída do labirinto
    graph.exit_vertex = graph.get_vertex(get_index(LENGTH - 1, LENGTH - 1))

def main():
    pygame.init()
    screen = pygame.display.set_mode((RESOLUTION, RESOLUTION + BUTTON_HEIGHT))
    pygame.display.set_caption("Maze Generator and Solver")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 50)

    graph = Graph()
    for i in range(LENGTH * LENGTH):
        graph.add_vertex(i)
    for y in range(LENGTH):
        for x in range(LENGTH):
            current_id = get_index(x, y)
            neighbors = get_neighbors_coordinates(x, y)
            for nx, ny in neighbors:
                neighbor_id = get_index(nx, ny)
                graph.add_edge(current_id, neighbor_id)

    create_maze(graph)

    solve_maze = False
    find_path = False
    dijkstra = Dijkstra(graph, graph.exit_vertex)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    if event.pos[0] >= BUTTON_X and event.pos[0] <= BUTTON_X + BUTTON_WIDTH and event.pos[1] >= BUTTON_Y and event.pos[1] <= BUTTON_Y + BUTTON_HEIGHT:
                        graph = Graph()
                        for i in range(LENGTH * LENGTH):
                            graph.add_vertex(i)
                        for y in range(LENGTH):
                            for x in range(LENGTH):
                                current_id = get_index(x, y)
                                neighbors = get_neighbors_coordinates(x, y)
                                for nx, ny in neighbors:
                                    neighbor_id = get_index(nx, ny)
                                    graph.add_edge(current_id, neighbor_id)

                        create_maze(graph)
                        solve_maze = False
                        find_path = False

                    elif event.pos[0] >= BUTTON_X_NEW_MAZE and event.pos[0] <= BUTTON_X_NEW_MAZE + BUTTON_WIDTH and event.pos[1] >= BUTTON_Y and event.pos[1] <= BUTTON_Y + BUTTON_HEIGHT:
                        dijkstra = Dijkstra(graph, graph.exit_vertex)
                        solve_maze = True
                        find_path = True
                        visited_vertices = []
                        dijkstra.shortest_path(0)
                        visited_vertices = dijkstra.visited[:]  # Create a copy for iteration

        screen.fill(BLACK)

        for vertex in graph.vertices.values():
            x, y = coordinates(vertex.id)
            walls = []
            for nx, ny in get_neighbors_coordinates(x, y):
                neighbor = graph.get_vertex(get_index(nx, ny))
                walls.append(any(e.is_wall for e in vertex.edges if e.end_vertex == neighbor))
            draw_cell(screen, x, y, walls)

        draw_buttons(screen, font)

        if solve_maze and find_path:
            if visited_vertices:
                draw_path(screen, [visited_vertices.pop(0)])

        pygame.display.flip()
        clock.tick(100)  # Adjust the speed of visualization by changing the tick value

    pygame.quit()

if __name__ == "__main__":
    main()
