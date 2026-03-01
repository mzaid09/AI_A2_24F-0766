# 🤖 Dynamic Pathfinding Agent
### AI 2002  Artificial Intelligence | Assignment 2 | Spring 2026
### National University of Computer & Emerging Sciences Chiniot-Faisalabad Campus

---

## 📋 Table of Contents
- [Project Overview](#project-overview)
- [Features](#features)
- [Algorithms Implemented](#algorithms-implemented)
- [Heuristic Functions](#heuristic-functions)
- [Project Structure](#project-structure)
- [Requirements](#requirements)
- [How to Install](#how-to-install)
- [How to Run](#how-to-run)
- [How to Use](#how-to-use)
- [Keyboard Shortcuts](#keyboard-shortcuts)
- [Color Legend](#color-legend)
- [Live Metrics](#live-metrics)
- [Dynamic Mode](#dynamic-mode)
- [Algorithm Comparison](#algorithm-comparison)
- [Screenshots](#screenshots)
- [Concepts Explained](#concepts-explained)
- [Developer](#developer)

---

## 📌 Project Overview

This project implements a **Dynamic Pathfinding Agent** capable of navigating a grid-based environment using **Informed Search Algorithms**. The agent finds the optimal path from a **Start node (S)** to a **Goal node (G)** while avoiding obstacles (walls).

The most advanced feature is **Dynamic Mode** — where new obstacles randomly appear while the agent is already moving, forcing it to **detect the blockage and re-plan a new path in real time**.

---

## ✨ Features

| Feature | Description |
|---|---|
| **Dynamic Grid Sizing** | Grid is 25×25 by default (easily configurable) |
| **Random Maze Generation** | Auto-generates maze with 30% obstacle density |
| **Interactive Map Editor** | Click on any cell to add or remove walls manually |
| **A* Search** | Finds the optimal (shortest) path |
| **Greedy Best-First Search** | Finds a fast path (not always optimal) |
| **Manhattan Heuristic** | Best for 4-directional grid movement |
| **Euclidean Heuristic** | Straight-line distance estimate |
| **Dynamic Obstacles** | New walls spawn randomly while agent moves |
| **Auto Re-planning** | Agent detects blockage and re-plans instantly |
| **Live Metrics Dashboard** | Shows nodes visited, path cost, time, re-plans |
| **Full GUI Visualization** | Color-coded cells showing every step of search |
| **Keyboard Shortcuts** | Quick controls for fast testing |

---

## 🧠 Algorithms Implemented

### 1. A\* Search
```
f(n) = g(n) + h(n)

g(n) = actual cost from Start to current node
h(n) = estimated cost from current node to Goal
f(n) = total estimated cost of path through node n
```

**Properties:**
- ✅ Complete — always finds a solution if one exists
- ✅ Optimal — always finds the SHORTEST path
- ✅ Uses both actual cost and heuristic estimate
- ⚠️ Explores more nodes than Greedy BFS

---

### 2. Greedy Best-First Search
```
f(n) = h(n) only

h(n) = estimated cost from current node to Goal
```

**Properties:**
- ✅ Fast — explores fewer nodes
- ✅ Uses visited list to avoid loops
- ❌ Not optimal — may find longer path
- ❌ Only considers heuristic, ignores actual cost

---

## 📐 Heuristic Functions

### Manhattan Distance
```
h(n) = |row1 - row2| + |col1 - col2|
```
- Best for grids with 4-directional movement
- Like counting city blocks 🏙️
- Admissible — never overestimates

### Euclidean Distance
```
h(n) = √( (row1-row2)² + (col1-col2)² )
```
- Straight line distance between two points
- Like a bird flying directly to destination 🐦
- Admissible — never overestimates

---

## 📁 Project Structure

```

│
├── pathfinding_agent.py    ← Main project file (run this)
├── screenshots/            ← Folder containing all screenshots
│   ├── 01_main_interface.png
│   ├── 02_generated_maze.png
│   ├── 03_astar_running.png
│   ├── 04_astar_path.png
│   ├── 05_greedy_path.png


---

## 💻 Requirements

| Requirement | Version |
|---|---|
| Python | 3.11.x (recommended) |
| Pygame | 2.6.1 or higher |

> ⚠️ Python 3.13+ may have compatibility issues with Pygame. Use Python 3.11 for best results.

---

## 📦 How to Install

### Step 1 — Install Python 3.11
Download from:
```
https://www.python.org/downloads/release/python-3119/
```
> ⚠️ Check **"Add Python to PATH"** during installation!

### Step 2 — Install Pygame
```bash
py -3.11 -m pip install pygame
```

Wait for:
```
Successfully installed pygame ✅
```

---

## ▶️ How to Run

### Step 1 — Open Command Prompt
```
Press Win + R → type cmd → press Enter
```

### Step 2 — Navigate to Project Folder
```bash
cd C:\Users\YourName\OneDrive\Desktop
```

### Step 3 — Run the Project
```bash
py -3.11 pathfinding_agent.py
```

### Project window will open! 🎉

---

## 🎮 How to Use

```
Step 1 → Click "Generate Maze"     generates random grid with walls
Step 2 → Select Algorithm          A* Search OR Greedy Best-First
Step 3 → Select Heuristic          Manhattan OR Euclidean
Step 4 → Click "Run Search"        watch yellow/blue cells appear
Step 5 → Watch agent move          orange path, purple agent (@)
Step 6 → Enable "Dynamic Mode"     obstacles spawn while moving
Step 7 → Click "Run Search" again  agent auto re-plans if blocked!
```

### Button Functions

| Button | Function |
|---|---|
| **Generate Maze** | Randomly fills grid with 30% walls |
| **Run Search** | Runs selected algorithm and animates agent |
| **Clear Path** | Removes visited/path colors, keeps walls |
| **Reset Grid** | Completely clears the entire grid |
| **Dynamic: OFF/ON** | Toggles dynamic obstacle mode |

---

## ⌨️ Keyboard Shortcuts

| Key | Action |
|---|---|
| `G` | Generate new maze |
| `SPACE` | Run search algorithm |
| `R` | Reset entire grid |
| `ESC` | Quit application |
| `Click on cell` | Toggle wall on/off |
| `Click + Drag` | Draw multiple walls |

---

## 🎨 Color Legend

| Color | Cell Type | Meaning |
|---|---|---|
| 🟢 Green | Start (S) | Where agent begins |
| 🔴 Pink/Red | Goal (G) | Where agent must reach |
| ⬛ Dark Blue | Empty | Agent can walk here |
| ⬜ Dark Gray | Wall | Blocked, cannot pass |
| 🟡 Yellow | Frontier | Currently in priority queue |
| 🔵 Blue | Visited | Already explored |
| 🟠 Orange | Final Path | The best route found |
| 🟣 Purple | Agent (@) | Current agent position |
| 🔴 Bright Red | Dynamic Obstacle | Newly spawned wall |

---

## 📊 Live Metrics

| Metric | Description |
|---|---|
| **Nodes Visited** | Total cells expanded by algorithm |
| **Path Cost** | Number of steps in final path |
| **Exec Time** | Time taken to compute solution (ms) |
| **Re-plans** | How many times agent re-planned |

---

## ⚡ Dynamic Mode

When **Dynamic Mode** is ON:
```
1. Agent starts moving along planned path
2. Every step → 8% chance of new obstacle spawning
3. If obstacle appears ON the path:
      → Agent detects blockage immediately
      → Calls A* again from current position
      → New path calculated instantly
      → Agent continues on new route
      → Re-plan counter increases by 1
4. If obstacle NOT on path:
      → Agent ignores it and continues
```

---

## 🆚 Algorithm Comparison

| Property | A\* Search | Greedy BFS |
|---|---|---|
| **Formula** | f = g + h | f = h only |
| **Optimal?** | ✅ Yes | ❌ No |
| **Complete?** | ✅ Yes | ✅ Yes |
| **Speed** | Slower | Faster |
| **Nodes Expanded** | More | Less |
| **Best For** | Optimal path | Fast path |

---



---

### 1. 🖥️ Main Interface

---![alt text](image-4.png)

### 2. 🗺️ Generated Maze
>![alt text](image-7.png)



---

### 3. 🔍 A* BY Manhattan
> ![alt text](image-5.png)


---

### 4. ✅ A* By Euclidean
>
![alt text](image-6.png)

---

### 5. ⚡ Greedy Best-First Search By Manhattan
>

![alt text](image-8.png)
---
### 5. ⚡ Greedy Best-First Search By Euclidean

> ![alt text](image-9.png)
---

## 💡 Concepts Explained

### What is g(n)?
> The **actual cost** from Start node to current node. Increases by 1 for each step taken.

### What is h(n)?
> The **heuristic estimate** of cost from current node to Goal. An educated guess — not exact.

### What is f(n)?
> **f(n) = g(n) + h(n)** — Total estimated cost of cheapest path through node n.

### What is a Priority Queue?
> A data structure that always returns element with **lowest value first**. Used by both algorithms to always explore most promising node next.

### What is Backtracking?
> After Goal is found, trace back from Goal to Start using **parent dictionary**. Each node remembers who discovered it. This gives us the final orange path.

### What is Re-planning?
> When new obstacle blocks current path, agent runs A\* again from its **current position** to find new route to Goal.

### What is Admissible Heuristic?
> A heuristic that **never overestimates** the true cost. Both Manhattan and Euclidean are admissible. This guarantees A\* finds optimal path.

---

## ⚙️ Configuration

Change these values at top of `pathfinding_agent.py`:

```python
ROWS             = 25      # grid rows
COLS             = 25      # grid columns
CELL_SIZE        = 28      # cell size in pixels
OBSTACLE_DENSITY = 0.35    # obstacle density (35%)
SPAWN_PROB       = 0.08    # dynamic obstacle spawn chance
AGENT_SPEED      = 100     # agent movement speed (ms)
```

---

## 👨‍💻 Developer

```
Name        :  [Your Full Name Here]
Roll Number :  [Your Roll Number Here]
Course      :  AI 2002 – Artificial Intelligence
Campus      :  NUCES Chiniot-Faisalabad
Semester    :  Spring 2026
Submitted   :  March 01, 2026
```

---

## 📜 Academic Integrity

> This project was developed individually as per the assignment guidelines.
> Code logic and implementation reflects personal understanding of
> informed search algorithms covered in AI 2002.

---

*© 2026 National University of Computer & Emerging Sciences*
