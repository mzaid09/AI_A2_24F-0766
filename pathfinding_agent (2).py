import pygame
import heapq
import random
import math
import time

# ─────────────────────────────────────────────
#   CONFIGURATION
# ─────────────────────────────────────────────
ROWS          = 25
COLS          = 25
CELL_SIZE     = 28
FPS           = 60

OBSTACLE_DENSITY = 0.30
AGENT_SPEED      = 100

# Dynamic difficulty spawn probabilities
DIFFICULTY_SETTINGS = {
    "EASY"   : 0.04,   # 4%  chance per step
    "MEDIUM" : 0.10,   # 10% chance per step
    "HARD"   : 0.22,   # 22% chance per step
}

# ─────────────────────────────────────────────
#   COLORS
# ─────────────────────────────────────────────
BLACK  = (15,  15,  30)
WHITE  = (240, 240, 240)

COLOR_EMPTY      = (20,  25,  50)
COLOR_WALL       = (45,  45,  80)
COLOR_START      = (78,  204, 163)
COLOR_GOAL       = (233, 69,  96)
COLOR_VISITED    = (74,  158, 255)
COLOR_FRONTIER   = (255, 215, 0)
COLOR_PATH       = (255, 107, 53)
COLOR_AGENT      = (204, 68,  255)
COLOR_NEWWALL    = (255, 68,  68)
COLOR_FINALPATH  = (0,   230, 100)   # ✅ GREEN — final path after goal
COLOR_GRID       = (25,  30,  60)

UI_PANEL     = (20,  25,  55)
UI_BORDER    = (40,  50,  100)
UI_TEXT      = (220, 220, 240)
UI_HIGHLIGHT = (78,  204, 163)

# ─────────────────────────────────────────────
#   CELL TYPES
# ─────────────────────────────────────────────
EMPTY     = 0
WALL      = 1
START     = 2
GOAL      = 3
VISITED   = 4
FRONTIER  = 5
PATH      = 6
AGENT     = 7
NEW_WALL  = 8
FINALPATH = 9    # green path shown after goal reached

CELL_COLORS = {
    EMPTY    : COLOR_EMPTY,
    WALL     : COLOR_WALL,
    START    : COLOR_START,
    GOAL     : COLOR_GOAL,
    VISITED  : COLOR_VISITED,
    FRONTIER : COLOR_FRONTIER,
    PATH     : COLOR_PATH,
    AGENT    : COLOR_AGENT,
    NEW_WALL : COLOR_NEWWALL,
    FINALPATH: COLOR_FINALPATH,
}

# ─────────────────────────────────────────────
#   WINDOW SIZE
# ─────────────────────────────────────────────
GRID_WIDTH  = COLS * CELL_SIZE
GRID_HEIGHT = ROWS * CELL_SIZE
PANEL_WIDTH = 265
WIN_WIDTH   = GRID_WIDTH + PANEL_WIDTH
WIN_HEIGHT  = GRID_HEIGHT + 60
# ═════════════════════════════════════════════
#   HEURISTICS
# ═════════════════════════════════════════════

def manhattan(r1, c1, r2, c2):
    return abs(r1 - r2) + abs(c1 - c2)

def euclidean(r1, c1, r2, c2):
    return math.sqrt((r1 - r2)**2 + (c1 - c2)**2)


# ═════════════════════════════════════════════
#   GET NEIGHBORS
# ═════════════════════════════════════════════

def get_neighbors(grid, row, col):
    directions = [(-1,0),(1,0),(0,-1),(0,1)]
    neighbors  = []
    for dr, dc in directions:
        nr, nc = row+dr, col+dc
        if 0 <= nr < ROWS and 0 <= nc < COLS:
            if grid[nr][nc] not in (WALL, NEW_WALL):
                neighbors.append((nr, nc))
    return neighbors


# ═════════════════════════════════════════════
#   A* SEARCH
# ═════════════════════════════════════════════

def astar_search(grid, start, goal, h_func):
    sr, sc = start
    gr, gc = goal
    open_list = []
    heapq.heappush(open_list, (h_func(sr,sc,gr,gc), 0, sr, sc))
    parent = {start: None}
    g_cost = {start: 0}
    expanded = set()
    visited_order = []
    nodes_expanded = 0

    while open_list:
        f, g, r, c = heapq.heappop(open_list)
        current = (r, c)
        if current in expanded:
            continue
        expanded.add(current)
        visited_order.append(current)
        nodes_expanded += 1

        if current == goal:
            return reconstruct_path(parent, goal), visited_order, nodes_expanded

        for neighbor in get_neighbors(grid, r, c):
            if neighbor in expanded:
                continue
            nr, nc = neighbor
            new_g = g + 1
            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                parent[neighbor] = current
                h = h_func(nr, nc, gr, gc)
                heapq.heappush(open_list, (new_g+h, new_g, nr, nc))

    return None, visited_order, nodes_expanded


# ═════════════════════════════════════════════
#   GREEDY BEST-FIRST SEARCH
# ═════════════════════════════════════════════

def greedy_search(grid, start, goal, h_func):
    sr, sc = start
    gr, gc = goal
    open_list = []
    heapq.heappush(open_list, (h_func(sr,sc,gr,gc), sr, sc))
    parent  = {start: None}
    visited = set([start])
    visited_order  = []
    nodes_expanded = 0

    while open_list:
        h, r, c = heapq.heappop(open_list)
        current = (r, c)
        visited_order.append(current)
        nodes_expanded += 1

        if current == goal:
            return reconstruct_path(parent, goal), visited_order, nodes_expanded

        for neighbor in get_neighbors(grid, r, c):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                nr, nc = neighbor
                heapq.heappush(open_list, (h_func(nr,nc,gr,gc), nr, nc))

    return None, visited_order, nodes_expanded

# ═════════════════════════════════════════════
#   RECONSTRUCT PATH
# ═════════════════════════════════════════════

def reconstruct_path(parent, goal):
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path


# ═════════════════════════════════════════════
#   GRID CLASS
# ═════════════════════════════════════════════

class Grid:
    def __init__(self):
        self.start = (0, 0)
        self.goal  = (ROWS-1, COLS-1)
        self.reset()

    def reset(self):
        self.cells = [[EMPTY]*COLS for _ in range(ROWS)]
        self.cells[self.start[0]][self.start[1]] = START
        self.cells[self.goal[0]][self.goal[1]]   = GOAL

    def generate_maze(self):
        self.reset()
        for r in range(ROWS):
            for c in range(COLS):
                if self.cells[r][c] == EMPTY:
                    if random.random() < OBSTACLE_DENSITY:
                        self.cells[r][c] = WALL
        self.cells[self.start[0]][self.start[1]] = START
        self.cells[self.goal[0]][self.goal[1]]   = GOAL

    def clear_search(self):
        for r in range(ROWS):
            for c in range(COLS):
                if self.cells[r][c] in (VISITED, FRONTIER, PATH, AGENT, FINALPATH, NEW_WALL):
                    self.cells[r][c] = EMPTY

    def set_start(self, row, col):
        # Clear old start
        old_r, old_c = self.start
        if self.cells[old_r][old_c] == START:
            self.cells[old_r][old_c] = EMPTY
        self.start = (row, col)
        self.cells[row][col] = START

    def set_goal(self, row, col):
        # Clear old goal
        old_r, old_c = self.goal
        if self.cells[old_r][old_c] == GOAL:
            self.cells[old_r][old_c] = EMPTY
        self.goal = (row, col)
        self.cells[row][col] = GOAL

    def toggle_wall(self, row, col):
        if self.cells[row][col] == EMPTY:
            self.cells[row][col] = WALL
        elif self.cells[row][col] == WALL:
            self.cells[row][col] = EMPTY

    def place_dynamic_obstacle(self, row, col):
        if self.cells[row][col] in (EMPTY, VISITED):
            self.cells[row][col] = NEW_WALL
            return True
        return False

    def show_final_path(self, path):
        """
        After goal reached:
        1. Clear everything (blue visited, orange path)
        2. Highlight ONLY the path agent followed in GREEN
        """
        # Step 1 — clear all search markings
        for r in range(ROWS):
            for c in range(COLS):
                if self.cells[r][c] in (VISITED, PATH, AGENT, FRONTIER, NEW_WALL):
                    self.cells[r][c] = EMPTY

        # Step 2 — draw final path in GREEN
        for (r, c) in path:
            if self.cells[r][c] not in (START, GOAL):
                self.cells[r][c] = FINALPATH


# ═════════════════════════════════════════════
#   BUTTON CLASS
# ═════════════════════════════════════════════

class Button:
    def __init__(self, x, y, w, h, text, color, text_color=BLACK):
        self.rect       = pygame.Rect(x, y, w, h)
        self.text       = text
        self.color      = color
        self.base_color = color
        self.hover_color= tuple(min(255,v+30) for v in color)
        self.text_color = text_color
        self.hovered    = False

    def draw(self, surface, font):
        color = self.hover_color if self.hovered else self.color
        pygame.draw.rect(surface, color,     self.rect, border_radius=8)
        pygame.draw.rect(surface, UI_BORDER, self.rect, 1, border_radius=8)
        surf = font.render(self.text, True, self.text_color)
        rect = surf.get_rect(center=self.rect.center)
        surface.blit(surf, rect)

    def check_hover(self, pos):
        self.hovered = self.rect.collidepoint(pos)

    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)


