import pygame
import random
import heapq

# Configurações
RESOLUTION = 720
DIM = 10
LENGTH = (RESOLUTION // DIM)-2 # Ajuste para adicionar a borda
BUTTON_WIDTH = 200
BUTTON_HEIGHT = 50
BUTTON_X = (RESOLUTION - BUTTON_WIDTH) // 3
BUTTON_Y = RESOLUTION + 10

# Posições dos botões
BUTTON_X_NEW_MAZE = (BUTTON_X + BUTTON_WIDTH + 10)

# Cores
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PURPLE = (170, 125, 197)
DARK_BLUE = (0, 0, 139)

class PriorityQueue:
    """Implementação de uma fila de prioridade usando heapq."""

    def __init__(self):
        """Inicializa a fila de prioridade."""
        self.elements = []

    def empty(self):
        """Verifica se a fila de prioridade está vazia.
        
        Returns:
            bool: True se a fila estiver vazia, False caso contrário.
        """
        return len(self.elements) == 0

    def put(self, item, priority):
        """Adiciona um item à fila com uma determinada prioridade.
        
        Args:
            item: O item a ser adicionado.
            priority (float): A prioridade do item.
        """
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        """Remove e retorna o item com a maior prioridade.
        
        Returns:
            O item com a maior prioridade.
        """
        return heapq.heappop(self.elements)[1]

class Vertex:
    """Representa um vértice em um grafo."""

    def __init__(self, id):
        """Inicializa um vértice.
        
        Args:
            id (int): O identificador do vértice.
        """
        self.id = id
        self.edges = []
        self.visited = False
        self.distance = float('inf')
        self.prev = None

    def add_edge(self, edge):
        """Adiciona uma aresta ao vértice.
        
        Args:
            edge (Edge): A aresta a ser adicionada.
        """
        self.edges.append(edge)

    def get_neighbors(self):
        """Obtém os vizinhos do vértice que não são paredes.
        
        Returns:
            list: Uma lista de vértices vizinhos.
        """
        return [e.end_vertex for e in self.edges if not e.is_wall]

    def get_edges(self):
        """Obtém as arestas do vértice.
        
        Returns:
            list: Uma lista de arestas do vértice.
        """
        return self.edges

    def __lt__(self, other):
        """Compara a distância deste vértice com a de outro vértice.
        
        Args:
            other (Vertex): O outro vértice a ser comparado.
        
        Returns:
            bool: True se a distância deste vértice for menor que a do outro vértice.
        """
        return self.distance < other.distance

class Edge:
    """Representa uma aresta em um grafo."""

    def __init__(self, start_vertex, end_vertex):
        """Inicializa uma aresta.
        
        Args:
            start_vertex (Vertex): O vértice de início da aresta.
            end_vertex (Vertex): O vértice de fim da aresta.
        """
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.is_wall = True

class Graph:
    """Representa um grafo."""

    def __init__(self):
        """Inicializa um grafo."""
        self.vertices = {}

    def add_vertex(self, id):
        """Adiciona um vértice ao grafo.
        
        Args:
            id (int): O identificador do vértice.
        """
        self.vertices[id] = Vertex(id)

    def get_vertex(self, id):
        """Obtém um vértice do grafo.
        
        Args:
            id (int): O identificador do vértice.
        
        Returns:
            Vertex: O vértice correspondente ao id.
        """
        return self.vertices[id]

    def add_edge(self, start_vertex_id, end_vertex_id):
        """Adiciona uma aresta entre dois vértices.
        
        Args:
            start_vertex_id (int): O identificador do vértice de início.
            end_vertex_id (int): O identificador do vértice de fim.
        """
        start_vertex = self.get_vertex(start_vertex_id)
        end_vertex = self.get_vertex(end_vertex_id)
        edge = Edge(start_vertex, end_vertex)
        start_vertex.add_edge(edge)
        reverse_edge = Edge(end_vertex, start_vertex)
        end_vertex.add_edge(reverse_edge)

    def remove_wall(self, start_vertex_id, end_vertex_id):
        """Remove a parede entre dois vértices.
        
        Args:
            start_vertex_id (int): O identificador do vértice de início.
            end_vertex_id (int): O identificador do vértice de fim.
        """
        start_vertex = self.get_vertex(start_vertex_id)
        end_vertex = self.get_vertex(end_vertex_id)
        for edge in start_vertex.get_edges():
            if edge.end_vertex == end_vertex:
                edge.is_wall = False
                #print(f"removido a parede entre {start_vertex_id} e {end_vertex_id}")
        for edge in end_vertex.get_edges():
            if edge.end_vertex == start_vertex:
                edge.is_wall = False

class Dijkstra:
    """Implementação do algoritmo de Dijkstra para encontrar o caminho mais curto em um grafo."""

    def __init__(self, graph, target):
        """Inicializa o algoritmo de Dijkstra.
        
        Args:
            graph (Graph): O grafo no qual o algoritmo será executado.
            target (Vertex): O vértice alvo.
        """
        self.graph = graph
        self.visited = []
        self.target = target

    def distances(self, start_id):
        """Calcula as distâncias mínimas do vértice inicial a todos os outros vértices.
        
        Args:
            start_id (int): O identificador do vértice inicial.
        """
        pq = PriorityQueue()
        start = self.graph.get_vertex(start_id)
        start.distance = 0
        pq.put(start, start.distance)

        while not pq.empty():
            vertex = pq.get()
            if vertex.visited:
                continue
            vertex.visited = True
            self.visited.append(vertex)
            if vertex == self.target:
                break

            for neighbor in vertex.get_neighbors():
                if not neighbor.visited:
                    cost = vertex.distance + 1
                    if cost < neighbor.distance:
                        neighbor.distance = cost
                        pq.put(neighbor, cost)
                        neighbor.prev = vertex

    def path(self, target_id):
        """Obtém o caminho do vértice inicial ao vértice alvo.
        
        Args:
            target_id (int): O identificador do vértice alvo.
        
        Returns:
            list: Uma lista de vértices que representam o caminho do inicial ao alvo.
        """
        path = []
        current = self.graph.get_vertex(target_id)
        while current:
            path.append(current)
            current = current.prev
        return path[::-1]

def get_neighbors_coordinates(x, y):
    """Obtém as coordenadas dos vizinhos de uma célula na grade.
    
    Args:
        x (int): A coordenada x da célula.
        y (int): A coordenada y da célula.
    
    Returns:
        list: Uma lista de coordenadas dos vizinhos.
    """
    coordinates = [
        (x, y - 1),
        (x + 1, y),
        (x, y + 1),
        (x - 1, y)
    ]
    return [(x, y) for (x, y) in coordinates if 0 <= x < LENGTH and 0 <= y < LENGTH]

def get_index(x, y):
    """Obtém o índice de uma célula na grade a partir das coordenadas.
    
    Args:
        x (int): A coordenada x da célula.
        y (int): A coordenada y da célula.
    
    Returns:
        int: O índice da célula.
    """
    return x + y * LENGTH

def coordinates(id):
    """Obtém as coordenadas de uma célula na grade a partir do índice.
    
    Args:
        id (int): O índice da célula.
    
    Returns:
        tuple: Uma tupla (x, y) com as coordenadas da célula.
    """
    x = id % LENGTH
    y = (id-x) // LENGTH
    return (x, y)

def draw_cell(screen, x, y, walls, maze_x, maze_y):
    """Desenha uma célula na grade.
    
    Args:
        screen (pygame.Surface): A superfície da tela onde a célula será desenhada.
        x (int): A coordenada x da célula.
        y (int): A coordenada y da célula.
        walls (list): Uma lista de booleans indicando se há paredes nas direções [topo, direita, baixo, esquerda].
        maze_x (int): A coordenada x do canto superior esquerdo do labirinto.
        maze_y (int): A coordenada y do canto superior esquerdo do labirinto.
    """
    x = (x * DIM) + maze_x
    y = (y * DIM) + maze_y
    if walls[0]:  # Topo
        pygame.draw.line(screen, BLACK, (x, y), (x + DIM, y))
    if walls[1]:  # Direita
        pygame.draw.line(screen, RED, (x + DIM, y), (x + DIM, y + DIM))
    if walls[2]:  # Baixo
        pygame.draw.line(screen, BLACK, (x, y + DIM), (x + DIM, y + DIM))
    if walls[3]:  # Esquerda
        pygame.draw.line(screen, RED, (x, y), (x, y + DIM))
    
    #print(f"Cell ({x//DIM}, {y//DIM}): Walls - {walls}")

def draw_grid(screen, graph, maze_x, maze_y):
    """Desenha a grade na tela.
    
    Args:
        screen (pygame.Surface): A superfície da tela onde a grade será desenhada.
        graph (Graph): O grafo que representa a grade.
        maze_x (int): A coordenada x do canto superior esquerdo do labirinto.
        maze_y (int): A coordenada y do canto superior esquerdo do labirinto.
    """
    for y in range(LENGTH):
        for x in range(LENGTH):
            current = graph.get_vertex(get_index(x, y))
            neighbors = get_neighbors_coordinates(x, y)
            walls = [True, True, True, True]

            for i, (nx, ny) in enumerate(neighbors):
                neighbor_id = get_index(nx, ny)
                for edge in current.get_edges():
                    if edge.end_vertex.id == neighbor_id and not edge.is_wall:
                        walls[i] = False
            
            draw_cell(screen, x, y, walls, maze_x, maze_y)
            #print(f"Cell ({x}, {y}) - Neighbors: {neighbors}, Walls: {walls}")

def draw_path(screen, path, color, maze_x, maze_y):
    """Desenha um caminho na grade.
    
    Args:
        screen (pygame.Surface): A superfície da tela onde o caminho será desenhado.
        path (list): Uma lista de vértices que representam o caminho.
        color (tuple): A cor do caminho.
        maze_x (int): A coordenada x do canto superior esquerdo do labirinto.
        maze_y (int): A coordenada y do canto superior esquerdo do labirinto.
    """
    for vertex in path:
        x, y = coordinates(vertex.id)
        pygame.draw.rect(screen, color, ((x * DIM) + maze_x, (y * DIM) + maze_y, DIM, DIM))

def draw_start_and_end(screen, maze_x, maze_y):
    """Desenha os pontos de início e fim na grade.
    
    Args:
        screen (pygame.Surface): A superfície da tela onde os pontos serão desenhados.
        maze_x (int): A coordenada x do canto superior esquerdo do labirinto.
        maze_y (int): A coordenada y do canto superior esquerdo do labirinto.
    """
    start_x, start_y = coordinates(0)
    end_x, end_y = coordinates(LENGTH * LENGTH - 1)
    pygame.draw.rect(screen, PURPLE, ((start_x * DIM) + maze_x, (start_y * DIM) + maze_y, DIM, DIM))
    pygame.draw.rect(screen, RED, ((end_x * DIM) + maze_x, (end_y * DIM) + maze_y, DIM, DIM))

def draw_button(screen, text, rect, color, text_color):
    """Desenha um botão na tela.
    
    Args:
        screen (pygame.Surface): A superfície da tela onde o botão será desenhado.
        text (str): O texto do botão.
        rect (pygame.Rect): O retângulo que define a posição e tamanho do botão.
        color (tuple): A cor do botão.
        text_color (tuple): A cor do texto do botão.
    """
    pygame.draw.rect(screen, color, rect)
    font = pygame.font.Font(None, 36)
    text_surf = font.render(text, True, text_color)
    text_rect = text_surf.get_rect(center=rect.center)
    screen.blit(text_surf, text_rect)

def create_maze(graph):
    """Gera um labirinto usando o algoritmo de Prim.
    
    Args:
        graph (Graph): O grafo onde o labirinto será gerado.
    """
    stack = []
    start_vertex = graph.get_vertex(0)  # Ponto de início no canto superior esquerdo
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
            graph.remove_wall(current.id, neighbor.id)
            neighbor.visited = True
            stack.append(neighbor)

    for vertex in graph.vertices.values():
        vertex.visited = False

def draw_borders(screen, maze_x, maze_y):
    """Desenha as bordas ao redor do labirinto.
    
    Args:
        screen (pygame.Surface): A superfície da tela onde as bordas serão desenhadas.
        maze_x (int): A coordenada x do canto superior esquerdo do labirinto.
        maze_y (int): A coordenada y do canto superior esquerdo do labirinto.
    """
    # Desenha as bordas
    pygame.draw.line(screen, BLACK, (maze_x, maze_y), (maze_x + LENGTH * DIM, maze_y))  # Topo
    pygame.draw.line(screen, BLACK, (maze_x, maze_y), (maze_x, maze_y + LENGTH * DIM))  # Esquerda
    pygame.draw.line(screen, BLACK, (maze_x + LENGTH * DIM, maze_y), (maze_x + LENGTH * DIM, maze_y + LENGTH * DIM))  # Direita
    pygame.draw.line(screen, BLACK, (maze_x, maze_y + LENGTH * DIM), (maze_x + LENGTH * DIM, maze_y + LENGTH * DIM))  # Baixo

def main():
    """Função principal que inicializa o Pygame, cria o grafo e executa o loop principal do jogo."""
    pygame.init()
    screen = pygame.display.set_mode((RESOLUTION, RESOLUTION + BUTTON_HEIGHT + 20))
    pygame.display.set_caption("Maze Generator and Solver")
    clock = pygame.time.Clock()

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

    end_vertex_id = get_index(LENGTH - 1, LENGTH - 1)  # Ponto de término no canto inferior direito
    solver = Dijkstra(graph, graph.get_vertex(end_vertex_id))

    button_rect_find_path = pygame.Rect(BUTTON_X, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT)
    button_rect_new_maze = pygame.Rect(BUTTON_X_NEW_MAZE, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT)
    solve_maze = False

    path_index = 0
    running = True

    # Posição do labirinto
    maze_x = 0.13 * LENGTH
    maze_y = 0.010 * RESOLUTION

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if button_rect_find_path.collidepoint(event.pos):
                    solver.distances(0)
                    solution = solver.path(end_vertex_id)
                    visited = solver.visited
                    solve_maze = True
                    path_index = 0
                elif button_rect_new_maze.collidepoint(event.pos):
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
                    solver = Dijkstra(graph, graph.get_vertex(end_vertex_id))
                    solve_maze = False
                    path_index = 0

        screen.fill(WHITE)
        draw_grid(screen, graph, maze_x, maze_y)
        draw_start_and_end(screen, maze_x, maze_y)
        draw_borders(screen, maze_x, maze_y)
        draw_button(screen, "Find the Path", button_rect_find_path, PURPLE, BLACK)
        draw_button(screen, "New Maze", button_rect_new_maze, DARK_BLUE, WHITE)

        if solve_maze:
            draw_path(screen, visited[:path_index], BLUE, maze_x, maze_y)
            draw_path(screen, solution[:path_index], GREEN, maze_x, maze_y)

            if path_index < len(solution):
                path_index += 1

        pygame.display.flip()
        clock.tick(300)

    pygame.quit()

if __name__ == "__main__":
    main()
