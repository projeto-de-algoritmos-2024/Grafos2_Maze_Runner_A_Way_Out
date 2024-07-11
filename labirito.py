import pygame
import random
import heapq

# Configurações
RESOLUTION = 600
DIM = 20
LENGTH = RESOLUTION // DIM

# Cores
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class Vertex:
    def __init__(self, id):
        self.id = id
        self.edges = []
        self.visited = False
        self.distance = float('inf')
        self.prev = None

    def add_edge(self, edge):
        self.edges.append(edge)

    def get_neighbors(self):
        return [e.end_vertex for e in self.edges if not e.is_wall]

    def get_edges(self):
        return self.edges

    def __lt__(self, other):
        return self.distance < other.distance

class Edge:
    def __init__(self, start_vertex, end_vertex):
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.is_wall = True

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, id):
        self.vertices[id] = Vertex(id)

    def get_vertex(self, id):
        return self.vertices[id]

    def add_edge(self, start_vertex_id, end_vertex_id):
        start_vertex = self.get_vertex(start_vertex_id)
        end_vertex = self.get_vertex(end_vertex_id)
        edge = Edge(start_vertex, end_vertex)
        start_vertex.add_edge(edge)
        reverse_edge = Edge(end_vertex, start_vertex)
        end_vertex.add_edge(reverse_edge)

    def remove_wall(self, start_vertex_id, end_vertex_id):
        start_vertex = self.get_vertex(start_vertex_id)
        end_vertex = self.get_vertex(end_vertex_id)
        for edge in start_vertex.get_edges():
            if edge.end_vertex == end_vertex:
                edge.is_wall = False
        for edge in end_vertex.get_edges():
            if edge.end_vertex == start_vertex:
                edge.is_wall = False

class Dijkstra:
    def __init__(self, graph, target):
        self.graph = graph
        self.visited = []
        self.target = target

    def distances(self, start_id):
        pq = PriorityQueue()
        start = self.graph.get_vertex(start_id)
        start.distance = 0
        pq.put(start, start.distance)

        while not pq.empty():
            vertex = pq.get()
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
        path = []
        current = self.graph.get_vertex(target_id)
        while current:
            path.append(current)
            current = current.prev
        return path[::-1]

def get_neighbors_coordinates(x, y):
    coordinates = [
        (x, y - 1),
        (x + 1, y),
        (x, y + 1),
        (x - 1, y)
    ]
    return [(x, y) for (x, y) in coordinates if 0 <= x < LENGTH and 0 <= y < LENGTH]

def get_index(x, y):
    return x + y * LENGTH

def coordinates(id):
    x = id % LENGTH
    y = id // LENGTH
    return (x, y)

def draw_grid(screen, graph):
    for y in range(LENGTH):
        for x in range(LENGTH):
            current = graph.get_vertex(get_index(x, y))
            walls = [True, True, True, True]
            neighbors = get_neighbors_coordinates(x, y)
            for i, (nx, ny) in enumerate(neighbors):
                neighbor_id = get_index(nx, ny)
                for edge in current.get_edges():
                    if edge.end_vertex.id == neighbor_id and not edge.is_wall:
                        walls[i] = False

            draw_cell(screen, x, y, walls)

def draw_cell(screen, x, y, walls):
    x = x * DIM
    y = y * DIM
    if walls[0]:  # Top
        pygame.draw.line(screen, BLACK, (x, y), (x + DIM, y))
    if walls[1]:  # Right
        pygame.draw.line(screen, BLACK, (x + DIM, y), (x + DIM, y + DIM))
    if walls[2]:  # Bottom
        pygame.draw.line(screen, BLACK, (x, y + DIM), (x + DIM, y + DIM))
    if walls[3]:  # Left
        pygame.draw.line(screen, BLACK, (x, y), (x, y + DIM))

def main():
    pygame.init()
    screen = pygame.display.set_mode((RESOLUTION, RESOLUTION))
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

    # Maze generation using Prim's algorithm
    stack = []
    start_vertex = graph.get_vertex(0)
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

    end_vertex_id = get_index(LENGTH - 1, LENGTH - 1)
    solver = Dijkstra(graph, graph.get_vertex(end_vertex_id))
    solver.distances(0)
    solution = solver.path(end_vertex_id)
    visited = solver.visited

    path_index = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(WHITE)
        draw_grid(screen, graph)
        for vertex in visited[:path_index]:
            x, y = coordinates(vertex.id)
            pygame.draw.rect(screen, BLUE, (x * DIM, y * DIM, DIM, DIM))

        for vertex in solution[:path_index]:
            x, y = coordinates(vertex.id)
            pygame.draw.rect(screen, GREEN, (x * DIM, y * DIM, DIM, DIM))

        if path_index < len(solution):
            path_index += 1

        pygame.display.flip()
        clock.tick(50)

    pygame.quit()

if __name__ == "__main__":
    main()
