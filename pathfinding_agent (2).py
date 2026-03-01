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

