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

# ═════════════════════════════════════════════
#   MAIN APPLICATION
# ═════════════════════════════════════════════

class PathfindingApp:

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_WIDTH, WIN_HEIGHT))
        pygame.display.set_caption("Dynamic Pathfinding Agent - AI 2002")
        self.clock  = pygame.time.Clock()

        self.font_large  = pygame.font.SysFont("segoeui", 16, bold=True)
        self.font_medium = pygame.font.SysFont("segoeui", 13)
        self.font_small  = pygame.font.SysFont("segoeui", 11)

        self.grid = Grid()

        # ── Draw mode ──
        # "wall"  = click to draw wall
        # "start" = click to place start
        # "goal"  = click to place goal
        self.draw_mode    = "wall"
        self.mouse_down   = False

        # ── Algorithm & Heuristic ──
        self.algorithm = "astar"
        self.heuristic = "manhattan"

        # ── Dynamic mode ──
        self.dynamic_mode  = False
        self.difficulty    = "EASY"
        self.spawn_prob    = DIFFICULTY_SETTINGS["EASY"]

        # ── Search state ──
        self.agent_moving   = False
        self.agent_path     = []
        self.agent_index    = 0
        self.agent_pos      = None
        self.last_move_time = 0
        self.walked_path    = []   # tracks cells agent actually walked

        # ── Metrics ──
        self.nodes_expanded = 0
        self.path_cost      = 0
        self.exec_time      = 0
        self.replan_count   = 0

        # ── Status ──
        self.status = "Welcome! Generate a maze or draw walls, then Run Search!"

        self._build_buttons()

    # ─────────────────────────────────────────
    #   BUILD BUTTONS
    # ─────────────────────────────────────────
    def _build_buttons(self):
        px = GRID_WIDTH + 12
        bw = PANEL_WIDTH - 24
        bh = 30

        # Main controls
        self.btn_generate = Button(px, 75,  bw, bh, "Generate Maze",      (60,100,200),  WHITE)
        self.btn_run      = Button(px, 112, bw, bh, "Run Search",          (78,204,163),  BLACK)
        self.btn_clear    = Button(px, 149, bw, bh, "Clear Path",          (255,215,0),   BLACK)
        self.btn_reset    = Button(px, 186, bw, bh, "Reset Grid",          (233,69,96),   WHITE)

        # Draw mode buttons
        self.btn_draw_wall  = Button(px, 243, bw//3-2,   bh, "Wall",  (45,45,80),    WHITE)
        self.btn_draw_start = Button(px+bw//3+1, 243, bw//3-2, bh, "Start", (78,204,163),  BLACK)
        self.btn_draw_goal  = Button(px+2*(bw//3)+2, 243, bw//3-2, bh, "Goal",  (233,69,96),   WHITE)

        # Dynamic toggle
        self.btn_dynamic  = Button(px, 300, bw, bh, "Dynamic: OFF",        (80,40,40),    WHITE)

        # Difficulty buttons
        self.btn_easy   = Button(px,           337, bw//3-2,   bh, "EASY",   (40,120,80),  WHITE)
        self.btn_medium = Button(px+bw//3+1,   337, bw//3-2,   bh, "MED",    (180,120,0),  BLACK)
        self.btn_hard   = Button(px+2*(bw//3)+2,337, bw//3-2,  bh, "HARD",   (150,30,30),  WHITE)

        self.all_buttons = [
            self.btn_generate, self.btn_run, self.btn_clear, self.btn_reset,
            self.btn_draw_wall, self.btn_draw_start, self.btn_draw_goal,
            self.btn_dynamic,
            self.btn_easy, self.btn_medium, self.btn_hard,
        ]

        # Highlight active difficulty
        self._update_difficulty_buttons()

    # ─────────────────────────────────────────
    #   UPDATE DIFFICULTY BUTTON HIGHLIGHTS
    # ─────────────────────────────────────────
    def _update_difficulty_buttons(self):
        self.btn_easy.color   = (0,200,100)   if self.difficulty=="EASY"   else (40,80,50)
        self.btn_medium.color = (230,160,0)   if self.difficulty=="MEDIUM" else (80,60,0)
        self.btn_hard.color   = (220,40,40)   if self.difficulty=="HARD"   else (80,20,20)

    # ─────────────────────────────────────────
    #   UPDATE DRAW MODE BUTTON HIGHLIGHTS
    # ─────────────────────────────────────────
    def _update_draw_mode_buttons(self):
        self.btn_draw_wall.color  = (90,90,150) if self.draw_mode=="wall"  else (45,45,80)
        self.btn_draw_start.color = (78,204,163)if self.draw_mode=="start" else (30,80,60)
        self.btn_draw_goal.color  = (233,69,96) if self.draw_mode=="goal"  else (90,30,40)

    # ─────────────────────────────────────────
    #   HEURISTIC FUNCTION
    # ─────────────────────────────────────────
    def get_heuristic(self):
        return manhattan if self.heuristic == "manhattan" else euclidean

    # ─────────────────────────────────────────
    #   RUN SEARCH
    # ─────────────────────────────────────────
    def run_search(self, start=None, goal=None):
        if start is None: start = self.grid.start
        if goal  is None: goal  = self.grid.goal
        h_func  = self.get_heuristic()
        t_start = time.time()

        if self.algorithm == "astar":
            path, visited, nodes = astar_search(self.grid.cells, start, goal, h_func)
        else:
            path, visited, nodes = greedy_search(self.grid.cells, start, goal, h_func)

        self.exec_time      = round((time.time()-t_start)*1000, 2)
        self.nodes_expanded = nodes
        return path, visited

    # ─────────────────────────────────────────
    #   START SEARCH
    # ─────────────────────────────────────────
    def start_search(self):
        if self.agent_moving:
            return

        self.grid.clear_search()
        self.replan_count = 0
        self.walked_path  = []
        self.status = f"Running {self.algorithm.upper()} with {self.heuristic.capitalize()}..."

        path, visited = self.run_search()

        if path is None:
            self.status = "No path found! Try a different maze."
            return

        # Color visited BLUE
        for (r, c) in visited:
            if self.grid.cells[r][c] == EMPTY:
                self.grid.cells[r][c] = VISITED

        # Color path ORANGE
        for (r, c) in path:
            if self.grid.cells[r][c] not in (START, GOAL):
                self.grid.cells[r][c] = PATH

        self.path_cost = len(path) - 1
        self.status    = f"Path found! Cost={self.path_cost} | Nodes={self.nodes_expanded} | Time={self.exec_time}ms"

        # Start agent
        self.agent_path     = path[:]
        self.agent_index    = 0
        self.agent_pos      = path[0]
        self.agent_moving   = True
        self.last_move_time = pygame.time.get_ticks()

    # ─────────────────────────────────────────
    #   UPDATE AGENT (called every frame)
    # ─────────────────────────────────────────
    def update_agent(self):
        if not self.agent_moving:
            return

        now = pygame.time.get_ticks()
        if now - self.last_move_time < AGENT_SPEED:
            return

        self.last_move_time = now

        # ── GOAL REACHED ──
        if self.agent_index >= len(self.agent_path) - 1:
            self.agent_moving = False
            self._on_goal_reached()
            return

        # Move one step
        prev = self.agent_path[self.agent_index]
        self.agent_index += 1
        curr = self.agent_path[self.agent_index]

        # Track walked path
        self.walked_path.append(prev)

        # Previous cell → BLUE
        pr, pc = prev
        if self.grid.cells[pr][pc] not in (START, GOAL):
            self.grid.cells[pr][pc] = VISITED

        # Current cell → PURPLE (agent)
        nr, nc = curr
        if self.grid.cells[nr][nc] not in (START, GOAL):
            self.grid.cells[nr][nc] = AGENT

        # Keep remaining path ORANGE ahead of agent
        for (rr, rc) in self.agent_path[self.agent_index+1:]:
            if self.grid.cells[rr][rc] not in (START, GOAL, WALL, NEW_WALL, AGENT):
                self.grid.cells[rr][rc] = PATH

        self.agent_pos = curr

        # Dynamic obstacles
        if self.dynamic_mode:
            self._try_spawn_obstacle()

    # ─────────────────────────────────────────
    #   ON GOAL REACHED
    # ─────────────────────────────────────────
    def _on_goal_reached(self):
        """
        When agent reaches goal:
        1. Add final cell to walked path
        2. Wait briefly
        3. Clear grid
        4. Show only the path agent walked in GREEN
        """
        # Add last cell
        self.walked_path.append(self.agent_path[-1])

        self.status = "🎉 Goal Reached! Showing path agent followed in GREEN..."

        # Small delay so user sees goal reached message
        pygame.display.flip()
        pygame.time.wait(800)

        # Clear grid and show final GREEN path
        self.grid.show_final_path(self.walked_path)

        self.status = (
            f"✅ Done! GREEN = path agent followed | "
            f"Cost={self.path_cost} | Nodes={self.nodes_expanded} | "
            f"Re-plans={self.replan_count}"
        )

    # ─────────────────────────────────────────
    #   SPAWN DYNAMIC OBSTACLE
    # ─────────────────────────────────────────
    def _try_spawn_obstacle(self):
        if random.random() > self.spawn_prob:
            return

        # For HARD difficulty — spawn near the path!
        if self.difficulty == "HARD" and self.agent_path:
            remaining = self.agent_path[self.agent_index:]
            if remaining and random.random() < 0.6:
                # Pick a cell near the remaining path
                target = random.choice(remaining)
                tr, tc = target
                # Spawn near that cell
                dr = random.randint(-2, 2)
                dc = random.randint(-2, 2)
                r  = max(0, min(ROWS-1, tr+dr))
                c  = max(0, min(COLS-1, tc+dc))
            else:
                r = random.randint(0, ROWS-1)
                c = random.randint(0, COLS-1)
        else:
            r = random.randint(0, ROWS-1)
            c = random.randint(0, COLS-1)

        placed = self.grid.place_dynamic_obstacle(r, c)
        if not placed:
            return

        # Check if it blocks remaining path
        remaining = self.agent_path[self.agent_index:]
        if (r, c) in remaining:
            self.replan_count += 1
            self.status = f"⚠️ Path blocked! Re-planning... (#{self.replan_count})"

            current  = self.agent_path[self.agent_index]
            new_path, new_visited = self.run_search(start=current)

            if new_path:
                # Clear old remaining path
                for (pr, pc) in remaining:
                    if self.grid.cells[pr][pc] == PATH:
                        self.grid.cells[pr][pc] = EMPTY

                # Color new path orange
                for (pr, pc) in new_path:
                    if self.grid.cells[pr][pc] not in (START, GOAL, AGENT, NEW_WALL, WALL):
                        self.grid.cells[pr][pc] = PATH

                # Update agent path
                self.agent_path = self.agent_path[:self.agent_index] + new_path
                self.status = f"✅ Re-planned! (#{self.replan_count})"
            else:
                self.agent_moving = False
                self.status = "❌ No path found! Agent is stuck."

    # ─────────────────────────────────────────
    #   HANDLE GRID CLICK
    # ─────────────────────────────────────────
    def handle_grid_click(self, pos):
        if self.agent_moving:
            return
        x, y = pos
        if x >= GRID_WIDTH or y >= GRID_HEIGHT:
            return
        col = x // CELL_SIZE
        row = y // CELL_SIZE
        if not (0 <= row < ROWS and 0 <= col < COLS):
            return

        if self.draw_mode == "wall":
            if self.grid.cells[row][col] not in (START, GOAL):
                self.grid.toggle_wall(row, col)

        elif self.draw_mode == "start":
            if self.grid.cells[row][col] not in (GOAL, WALL):
                self.grid.set_start(row, col)
                self.status = f"Start set to ({row}, {col})"
                self.draw_mode = "wall"
                self._update_draw_mode_buttons()

        elif self.draw_mode == "goal":
            if self.grid.cells[row][col] not in (START, WALL):
                self.grid.set_goal(row, col)
                self.status = f"Goal set to ({row}, {col})"
                self.draw_mode = "wall"
                self._update_draw_mode_buttons()

    # ─────────────────────────────────────────
    #   HANDLE PANEL CLICK
    # ─────────────────────────────────────────
    def handle_panel_click(self, pos):
        # Main buttons
        if self.btn_generate.is_clicked(pos):
            self.grid.generate_maze()
            self.agent_moving = False
            self.walked_path  = []
            self.replan_count = 0
            self.status = "Maze generated! Select algorithm and click Run Search."

        elif self.btn_run.is_clicked(pos):
            self.start_search()

        elif self.btn_clear.is_clicked(pos):
            self.grid.clear_search()
            self.agent_moving = False
            self.walked_path  = []
            self.status = "Path cleared."

        elif self.btn_reset.is_clicked(pos):
            self.grid.reset()
            self.agent_moving = False
            self.walked_path  = []
            self.replan_count = 0
            self.nodes_expanded = 0
            self.path_cost = 0
            self.status = "Grid reset."

        # Draw mode
        elif self.btn_draw_wall.is_clicked(pos):
            self.draw_mode = "wall"
            self._update_draw_mode_buttons()
            self.status = "Draw mode: WALL — click cells to add/remove walls"

        elif self.btn_draw_start.is_clicked(pos):
            self.draw_mode = "start"
            self._update_draw_mode_buttons()
            self.status = "Click anywhere on grid to SET START position!"

        elif self.btn_draw_goal.is_clicked(pos):
            self.draw_mode = "goal"
            self._update_draw_mode_buttons()
            self.status = "Click anywhere on grid to SET GOAL position!"

        # Dynamic toggle
        elif self.btn_dynamic.is_clicked(pos):
            self.dynamic_mode = not self.dynamic_mode
            if self.dynamic_mode:
                self.btn_dynamic.text  = "Dynamic: ON"
                self.btn_dynamic.color = (0, 180, 80)
                self.status = f"Dynamic Mode ON! Difficulty: {self.difficulty}"
            else:
                self.btn_dynamic.text  = "Dynamic: OFF"
                self.btn_dynamic.color = (80, 40, 40)
                self.status = "Dynamic Mode OFF."

        # Difficulty
        elif self.btn_easy.is_clicked(pos):
            self.difficulty  = "EASY"
            self.spawn_prob  = DIFFICULTY_SETTINGS["EASY"]
            self._update_difficulty_buttons()
            self.status = "Difficulty: EASY — few obstacles spawn"

        elif self.btn_medium.is_clicked(pos):
            self.difficulty  = "MEDIUM"
            self.spawn_prob  = DIFFICULTY_SETTINGS["MEDIUM"]
            self._update_difficulty_buttons()
            self.status = "Difficulty: MEDIUM — moderate obstacles spawn"

        elif self.btn_hard.is_clicked(pos):
            self.difficulty  = "HARD"
            self.spawn_prob  = DIFFICULTY_SETTINGS["HARD"]
            self._update_difficulty_buttons()
            self.status = "Difficulty: HARD — many obstacles spawn near path!"

        # Algorithm radio
        px = GRID_WIDTH + 12
        if pygame.Rect(px, 398, 200, 18).collidepoint(pos):
            self.algorithm = "astar"
            self.status = "Algorithm: A* Search"
        elif pygame.Rect(px, 418, 200, 18).collidepoint(pos):
            self.algorithm = "greedy"
            self.status = "Algorithm: Greedy Best-First Search"

        # Heuristic radio
        if pygame.Rect(px, 450, 200, 18).collidepoint(pos):
            self.heuristic = "manhattan"
            self.status = "Heuristic: Manhattan Distance"
        elif pygame.Rect(px, 470, 200, 18).collidepoint(pos):
            self.heuristic = "euclidean"
            self.status = "Heuristic: Euclidean Distance"

    # ─────────────────────────────────────────
    #   DRAW GRID
    # ─────────────────────────────────────────
    def draw_grid(self):
        for r in range(ROWS):
            for c in range(COLS):
                val   = self.grid.cells[r][c]
                color = CELL_COLORS.get(val, COLOR_EMPTY)
                x, y  = c*CELL_SIZE, r*CELL_SIZE
                pygame.draw.rect(self.screen, color,      (x+1, y+1, CELL_SIZE-2, CELL_SIZE-2))
                pygame.draw.rect(self.screen, COLOR_GRID, (x, y, CELL_SIZE, CELL_SIZE), 1)

                # Labels
                if val == START:
                    self._cell_label(x, y, "S", BLACK)
                elif val == GOAL:
                    self._cell_label(x, y, "G", WHITE)
                elif val == AGENT:
                    self._cell_label(x, y, "@", WHITE)

        # Draw mode indicator on grid border
        if self.draw_mode == "start":
            pygame.draw.rect(self.screen, COLOR_START, (0,0,GRID_WIDTH,GRID_HEIGHT), 4)
        elif self.draw_mode == "goal":
            pygame.draw.rect(self.screen, COLOR_GOAL,  (0,0,GRID_WIDTH,GRID_HEIGHT), 4)

    def _cell_label(self, x, y, text, color):
        surf = self.font_medium.render(text, True, color)
        rect = surf.get_rect(center=(x+CELL_SIZE//2, y+CELL_SIZE//2))
        self.screen.blit(surf, rect)

    # ─────────────────────────────────────────
    #   DRAW PANEL
    # ─────────────────────────────────────────
    def draw_panel(self):
        px = GRID_WIDTH
        pygame.draw.rect(self.screen, UI_PANEL,  (px, 0, PANEL_WIDTH, WIN_HEIGHT))
        pygame.draw.line(self.screen, UI_BORDER, (px,0),(px,WIN_HEIGHT), 2)

        mx = px + 12
        mouse_pos = pygame.mouse.get_pos()

        # Title
        self._text("PATHFINDING AGENT", mx, 10, self.font_large,  (78,204,163))
        self._text("AI 2002 - Assignment 2", mx, 30, self.font_small, (150,150,180))
        self._text("NUCES Chiniot-Faisalabad", mx, 44, self.font_small, (100,100,140))

        # Buttons
        for btn in self.all_buttons:
            btn.check_hover(mouse_pos)
            btn.draw(self.screen, self.font_small)

        # Section labels
        self._section("DRAW MODE", mx, 228)
        self._section("DYNAMIC OBSTACLES", mx, 288)
        self._section("DIFFICULTY", mx, 325)

        # Algorithm
        self._section("ALGORITHM", mx, 385)
        self._radio("A* Search",        mx, 398, self.algorithm=="astar")
        self._radio("Greedy BFS",        mx, 418, self.algorithm=="greedy")

        # Heuristic
        self._section("HEURISTIC", mx, 438)
        self._radio("Manhattan Distance", mx, 450, self.heuristic=="manhattan")
        self._radio("Euclidean Distance", mx, 470, self.heuristic=="euclidean")

        # Metrics
        self._section("LIVE METRICS", mx, 490)
        self._metric("Nodes Visited:", str(self.nodes_expanded), mx, 508)
        self._metric("Path Cost:",     str(self.path_cost),      mx, 524)
        self._metric("Exec Time:",     f"{self.exec_time} ms",   mx, 540)
        self._metric("Re-plans:",      str(self.replan_count),   mx, 556)

        # Legend
        self._section("LEGEND", mx, 575)
        legend = [
            (COLOR_START,     "Start (S)"),
            (COLOR_GOAL,      "Goal (G)"),
            (COLOR_WALL,      "Wall"),
            (COLOR_FRONTIER,  "Frontier"),
            (COLOR_VISITED,   "Visited"),
            (COLOR_PATH,      "Path (orange)"),
            (COLOR_AGENT,     "Agent (@)"),
            (COLOR_NEWWALL,   "Dynamic Obstacle"),
            (COLOR_FINALPATH, "Final Path (green)"),
        ]
        ly = 593
        for color, label in legend:
            pygame.draw.rect(self.screen, color, (mx, ly, 13, 13), border_radius=2)
            self._text(label, mx+18, ly, self.font_small, UI_TEXT)
            ly += 17

        # Difficulty indicator
        diff_colors = {"EASY":(0,200,100),"MEDIUM":(230,160,0),"HARD":(220,40,40)}
        diff_color  = diff_colors[self.difficulty]
        diff_text   = f"Difficulty: {self.difficulty}"
        if self.dynamic_mode:
            self._text(diff_text, mx, ly+5, self.font_small, diff_color)

        # Controls help
        self._text("G=Maze  R=Reset  SPACE=Run", mx, WIN_HEIGHT-50, self.font_small, (100,100,140))
        self._text("ESC=Quit  Click=Draw Wall",  mx, WIN_HEIGHT-35, self.font_small, (100,100,140))

    def _text(self, text, x, y, font, color):
        surf = font.render(text, True, color)
        self.screen.blit(surf, (x, y))

    def _section(self, text, x, y):
        self._text(text, x, y, self.font_small, (100,120,180))
        pygame.draw.line(self.screen, UI_BORDER, (x, y+13), (x+PANEL_WIDTH-24, y+13), 1)

    def _metric(self, label, val, x, y):
        self._text(label, x,     y, self.font_small, UI_TEXT)
        self._text(val,   x+120, y, self.font_small, (78,204,163))

    def _radio(self, text, x, y, selected):
        color = (78,204,163) if selected else (100,100,130)
        pygame.draw.circle(self.screen, color, (x+5, y+6), 5, 0 if selected else 2)
        self._text(text, x+15, y, self.font_small, (78,204,163) if selected else UI_TEXT)

    # ─────────────────────────────────────────
    #   DRAW STATUS BAR
    # ─────────────────────────────────────────
    def draw_status_bar(self):
        pygame.draw.rect(self.screen, UI_PANEL,  (0, GRID_HEIGHT, GRID_WIDTH, 60))
        pygame.draw.line(self.screen, UI_BORDER, (0, GRID_HEIGHT),(GRID_WIDTH, GRID_HEIGHT), 2)
        self._text(self.status, 10, GRID_HEIGHT+8,  self.font_medium, (78,204,163))

        mode_text = (
            f"Algorithm: {self.algorithm.upper()}  |  "
            f"Heuristic: {self.heuristic.capitalize()}  |  "
            f"Dynamic: {'ON ('+self.difficulty+')' if self.dynamic_mode else 'OFF'}  |  "
            f"Draw Mode: {self.draw_mode.upper()}"
        )
        self._text(mode_text, 10, GRID_HEIGHT+30, self.font_small, (130,130,160))

    # ─────────────────────────────────────────
    #   MAIN LOOP
    # ─────────────────────────────────────────
    def run(self):
        running = True
        while running:
            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_g:
                        self.grid.generate_maze()
                        self.agent_moving = False
                        self.status = "Maze generated!"
                    elif event.key == pygame.K_r:
                        self.grid.reset()
                        self.agent_moving = False
                        self.status = "Grid reset."
                    elif event.key == pygame.K_SPACE:
                        self.start_search()

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    pos = event.pos
                    if pos[0] >= GRID_WIDTH:
                        self.handle_panel_click(pos)
                    else:
                        self.mouse_down = True
                        self.handle_grid_click(pos)

                elif event.type == pygame.MOUSEBUTTONUP:
                    self.mouse_down = False

                elif event.type == pygame.MOUSEMOTION:
                    if self.mouse_down and self.draw_mode == "wall":
                        self.handle_grid_click(event.pos)

            self.update_agent()

            self.screen.fill(BLACK)
            self.draw_grid()
            self.draw_panel()
            self.draw_status_bar()
            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()


# ═════════════════════════════════════════════
#   ENTRY POINT
# ═════════════════════════════════════════════
if __name__ == "__main__":
    app = PathfindingApp()
    app.run()

