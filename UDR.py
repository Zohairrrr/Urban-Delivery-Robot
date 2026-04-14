import pygame
import pygame.font
import sys
import random
import heapq
import math
import time
from collections import deque

# CONSTANTS
GRID_SIZE   = 15
CELL        = 44       
PANEL_W     = 420         # side panel
WIN_W       = GRID_SIZE * CELL + PANEL_W
WIN_H       = GRID_SIZE * CELL + 80   
GRID_OFFSET_Y = 60     
GRID_OFFSET_X = 0

FPS         = 100 # keeping this 100 kiyunkay speeding up of traversal is fluid and faster
SEED        = 42

# Cell types 
ROAD     = 0
BUILDING = 1
TRAFFIC  = 2
DELIVERY = 3
BASE     = 4

# COLOUR PALETTE 
C = {
    "bg":          ( 12,  12,  16),
    "panel":       ( 38,  38,  44),
    "border":      (255, 255, 255),
    "road":        ( 42,  42,  50),
    "building":    ( 62,  62,  72),
    "building2":   ( 85,  85,  98),
    "traffic":     ( 95,  48,  15),
    "delivery":    ( 22,  80,  52),
    "base":        ( 48,  48,  62),
    "explored":    ( 72,  75,  88),
    "path":        (230, 230, 235),
    "robot":       (248, 248, 252),
    "accent":      (240, 240, 245),
    "green":       (108, 215, 148),
    "orange":      (218, 125,  58),
    "yellow":      (228, 208, 122),
    "red":         (205,  78,  88),
    "purple":      (168, 150, 215),
    "text":        (218, 218, 225),
    "muted":       (115, 115, 128),
    "white":       (250, 250, 255),
    "grid_line":   ( 55,  55,  65),
}

ALGO_COLORS = {
    "BFS":               (185, 198, 218),
    "DFS":               (205,  90, 108),
    "UCS":               (108, 208, 158),
    "Greedy-Manhattan":  (218, 198, 118),
    "Greedy-Euclidean":  (212, 150,  80),
    "A*-Manhattan":      (160, 148, 208),
    "A*-Euclidean":      (208, 198, 242),
}
ALGO_KEYS = list(ALGO_COLORS.keys())

# ENVIRONMENT SETUP
# is function mein pehle buildings place karni hain env mein, then road costs assign karain ge
# phir apna traffic zone implement karna hai
# end mei deliverable locations ayengi and us se pehle just put a starting position
def create_environment(seed=SEED):
    random.seed(seed)
    grid  = [[ROAD] * GRID_SIZE for _ in range(GRID_SIZE)]
    costs = [[0]    * GRID_SIZE for _ in range(GRID_SIZE)]
    placed = 0
    attempts = 0
    while placed < 38 and attempts < 800:
        r, c = random.randint(1, GRID_SIZE-2), random.randint(1, GRID_SIZE-2)
        if (r, c) not in [(1,1),(1,2),(2,1)]:
            grid[r][c] = BUILDING
            placed += 1
        attempts += 1
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if grid[r][c] != BUILDING:
                costs[r][c] = random.randint(1, 5)
    road_cells = [(r,c) for r in range(GRID_SIZE) for c in range(GRID_SIZE)
                  if grid[r][c] == ROAD]
    n_traffic = max(1, len(road_cells)//6)
    for r, c in random.sample(road_cells, n_traffic):
        grid[r][c]  = TRAFFIC
        costs[r][c] = random.randint(10, 20)
    grid[1][1]  = BASE
    costs[1][1] = 1
    avail = [(r,c) for r in range(GRID_SIZE) for c in range(GRID_SIZE)
             if grid[r][c] in (ROAD, TRAFFIC) and (r,c) != (1,1)]
    deliveries = random.sample(avail, 5)
    for r, c in deliveries:
        grid[r][c]  = DELIVERY
        costs[r][c] = random.randint(1, 5)
    return grid, costs, (1,1), deliveries

# yeh function dekhta hai ke robot kahan freely move kar sakta hai aur kon sa neighbour choose karna hai
# buildings waghaira ko avoid karte hue
def get_neighbors(pos, grid):
    r, c = pos
    result = []
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE and grid[nr][nc] != BUILDING:
            result.append((nr, nc))
    return result


# SEARCH ALGORITHMS (returns path, cost, nodes_explored, explored_order)

# simple bfs
def bfs(start, goal, grid, costs):
    queue = deque([(start, [start], 0)])
    visited = {start}
    explored_order = []
    while queue:
        pos, path, cost = queue.popleft()
        explored_order.append(pos)
        if pos == goal:
            return path, cost, len(explored_order), explored_order
        for nb in get_neighbors(pos, grid):
            if nb not in visited:
                visited.add(nb)
                queue.append((nb, path+[nb], cost+costs[nb[0]][nb[1]]))
    return None, float('inf'), len(explored_order), explored_order

# normal dfs
def dfs(start, goal, grid, costs):
    stack = [(start, [start], 0)]
    visited = set()
    explored_order = []
    while stack:
        pos, path, cost = stack.pop()
        if pos in visited:
            continue
        visited.add(pos)
        explored_order.append(pos)
        if pos == goal:
            return path, cost, len(explored_order), explored_order
        for nb in get_neighbors(pos, grid):
            if nb not in visited:
                stack.append((nb, path+[nb], cost+costs[nb[0]][nb[1]]))
    return None, float('inf'), len(explored_order), explored_order

# basic ucs
def ucs(start, goal, grid, costs):
    heap = [(0, id(start), start, [start])]
    visited = {}
    explored_order = []
    while heap:
        cost, _, pos, path = heapq.heappop(heap)
        if pos in visited:
            continue
        visited[pos] = cost
        explored_order.append(pos)
        if pos == goal:
            return path, cost, len(explored_order), explored_order
        for nb in get_neighbors(pos, grid):
            if nb not in visited:
                nc = cost + costs[nb[0]][nb[1]]
                heapq.heappush(heap, (nc, id(nb), nb, path+[nb]))
    return None, float('inf'), len(explored_order), explored_order

# greedy approach
def greedy(start, goal, grid, costs, heuristic):
    heap = [(heuristic(start,goal), id(start), 0, start, [start])]
    visited = set()
    explored_order = []
    while heap:
        h, _, cost, pos, path = heapq.heappop(heap)
        if pos in visited:
            continue
        visited.add(pos)
        explored_order.append(pos)
        if pos == goal:
            return path, cost, len(explored_order), explored_order
        for nb in get_neighbors(pos, grid):
            if nb not in visited:
                nc = cost + costs[nb[0]][nb[1]]
                heapq.heappush(heap, (heuristic(nb,goal), id(nb), nc, nb, path+[nb]))
    return None, float('inf'), len(explored_order), explored_order

# normal A* approach 
def astar(start, goal, grid, costs, heuristic):
    heap = [(heuristic(start,goal), id(start), 0, start, [start])]
    visited = {}
    explored_order = []
    while heap:
        f, _, g, pos, path = heapq.heappop(heap)
        if pos in visited:
            continue
        visited[pos] = g
        explored_order.append(pos)
        if pos == goal:
            return path, g, len(explored_order), explored_order
        for nb in get_neighbors(pos, grid):
            if nb not in visited:
                ng = g + costs[nb[0]][nb[1]]
                heapq.heappush(heap, (ng+heuristic(nb,goal), id(nb), ng, nb, path+[nb]))
    return None, float('inf'), len(explored_order), explored_order

# Manhattan distance nikalne ke liye
def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])
# Euclidean distance ki calc
def euclidean(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

# jo algo select kiya hoga yeh function usko run karega
# aur traversal vaghaira ka poora result la ke dega
def run_algorithm(name, start, goal, grid, costs):
    t0 = time.perf_counter()
    if   name == "BFS":              result = bfs(start, goal, grid, costs)
    elif name == "DFS":              result = dfs(start, goal, grid, costs)
    elif name == "UCS":              result = ucs(start, goal, grid, costs)
    elif name == "Greedy-Manhattan": result = greedy(start, goal, grid, costs, manhattan)
    elif name == "Greedy-Euclidean": result = greedy(start, goal, grid, costs, euclidean)
    elif name == "A*-Manhattan":     result = astar(start, goal, grid, costs, manhattan)
    elif name == "A*-Euclidean":     result = astar(start, goal, grid, costs, euclidean)
    else:                            result = (None, float('inf'), 0, [])
    elapsed = (time.perf_counter() - t0) * 1000
    path, cost, nodes, explored = result
    return path, cost, nodes, elapsed, explored

# yeh class apni main cheez hai jo game window banati hai and everything holds here
# fonts, grid, panel, robot sab idhar hi hai
class UrbanDeliveryRobot:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        pygame.display.set_caption("DELIVERY ROBOT")
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.clock  = pygame.time.Clock()
        def make_font(size, bold=False):
            for name in ["Courier New", "DejaVu Sans Mono", "Liberation Mono",
                         "Monospace", "FreeMono", ""]:
                try:
                    f = pygame.font.SysFont(name, size, bold=bold)
                    if f:
                        return f
                except Exception:
                    pass
            return pygame.font.Font(None, size + 4)

        self.font_tiny   = make_font(10)
        self.font_small  = make_font(12)
        self.font_med    = make_font(14, bold=True)
        self.font_large  = make_font(18, bold=True)
        self.font_huge   = make_font(26, bold=True)
        self.font_title  = make_font(15, bold=True)

        self._init_state()


    # sab kuch wapas start pe reset karne ke liye like a new game
    # grid, robot ki pos, logs wagaira sub goes zero
    def _init_state(self):
        self.grid, self.costs, self.base, self.deliveries = create_environment()
        self.algo_idx        = 2         
        self.speed           = 8         
        self.explore_speed   = 30        


        self.running         = False
        self.paused          = False
        self.delivery_idx    = 0        
        self.phase           = "idle"    
        self.robot_pos       = self.base


        self.current_path      = []
        self.current_explored  = []
        self.path_index        = 0      
        self.explore_index     = 0        
        self.robot_draw_pos    = None    
        self.robot_target_px   = None
        self.move_progress     = 0.0
        self.move_speed_frac   = 0.06    


        self.explored_set    = set()
        self.drawn_path      = []

        self.delivery_results = []       


        self.total_cost      = 0
        self.total_nodes     = 0
        self.total_time_ms   = 0.0
        self.completed       = 0

        self.log_lines       = ["Press SPACE to start."]


        self.done_deliveries = set()

        br, bc = self.base
        self.robot_px = (bc*CELL + CELL//2 + GRID_OFFSET_X,
                         br*CELL + CELL//2 + GRID_OFFSET_Y)

        self.comparison     = {}

    # screen par cell ka just rectangle return karta hai
    # row and col se humain pixel pos mil jati hai
    def cell_rect(self, r, c):
        x = c*CELL + GRID_OFFSET_X
        y = r*CELL + GRID_OFFSET_Y
        return pygame.Rect(x, y, CELL, CELL)
    
    # cell ka center pixel ta ke apna robot bilkul middle mein draw ho
    def cell_center(self, r, c):
        return (c*CELL + CELL//2 + GRID_OFFSET_X,
                r*CELL + CELL//2 + GRID_OFFSET_Y)
    
    # aik delivery run start karta hai, run all algos for comparison table
    # result store kar ke explore phase shuru ho jata hai
    def start_delivery(self):
        if self.delivery_idx >= 5:
            self.phase = "done"
            self.running = False
            self.log("** ALL DELIVERIES COMPLETE **")
            return

        dest = self.deliveries[self.delivery_idx]
        algo = ALGO_KEYS[self.algo_idx]
        path, cost, nodes, elapsed, explored = run_algorithm(
            algo, self.robot_pos, dest, self.grid, self.costs)
        self.comparison = {}
        for a in ALGO_KEYS:
            p, co, n, e, _ = run_algorithm(a, self.robot_pos, dest, self.grid, self.costs)
            self.comparison[a] = {"cost": co if p else "N/A", "nodes": n, "time": e,
                                   "path_len": len(p) if p else 0}
        if path is None:
            self.log(f"[!] Delivery {self.delivery_idx+1}: No path found!")
            self.delivery_idx += 1
            return

        self.current_path     = path
        self.current_explored = explored
        self.explore_index    = 0
        self.path_index       = 0
        self.explored_set     = set()
        self.drawn_path       = [path[0]]
        self.phase            = "exploring"

        self.delivery_results.append({
            "n": self.delivery_idx+1,
            "algo": algo,
            "from": self.robot_pos,
            "to": dest,
            "cost": cost,
            "nodes": nodes,
            "time": elapsed,
            "path_len": len(path),
        })

        self.log(f"[>] D{self.delivery_idx+1} [{algo}] -> {dest} | cost={cost} nodes={nodes}")

    # log list mein message add karne ke liye, purane remove kar do agar too many
    def log(self, msg):
        self.log_lines.append(msg)
        if len(self.log_lines) > 12:
            self.log_lines.pop(0)

    # apna main update loop for every frame, check which phase innit
    # wot task itlll do
    def update(self):
        if not self.running or self.paused:
            return

        if self.phase == "idle":
            self.start_delivery()

        elif self.phase == "exploring":
            batch = self.explore_speed
            while batch > 0 and self.explore_index < len(self.current_explored):
                self.explored_set.add(self.current_explored[self.explore_index])
                self.explore_index += 1
                batch -= 1
            if self.explore_index >= len(self.current_explored):
                self.phase = "moving"
                self.path_index  = 0
                r, c = self.current_path[0]
                self.robot_px    = self.cell_center(r, c)
                self.robot_draw_pos = self.robot_px
                if len(self.current_path) > 1:
                    r2, c2 = self.current_path[1]
                    self.robot_target_px = self.cell_center(r2, c2)
                    self.move_progress = 0.0
                else:
                    self.phase = "done_delivery"

        elif self.phase == "moving":
            self.move_progress += self.move_speed_frac * (self.speed / 8)
            if self.move_progress >= 1.0:
                self.move_progress = 0.0
                self.path_index += 1
                if self.path_index < len(self.current_path):
                    r, c = self.current_path[self.path_index]
                    self.robot_draw_pos = self.cell_center(r, c)
                    self.drawn_path = self.current_path[:self.path_index+1]
                if self.path_index + 1 < len(self.current_path):
                    r2, c2 = self.current_path[self.path_index+1]
                    self.robot_target_px = self.cell_center(r2, c2)
                else:
                    self.phase = "done_delivery"
            else:
                sx, sy = self.robot_draw_pos
                tx, ty = self.robot_target_px
                t = self.move_progress
                self.robot_px = (int(sx + (tx-sx)*t), int(sy + (ty-sy)*t))

        elif self.phase == "done_delivery":
            dest = self.deliveries[self.delivery_idx]
            self.robot_pos = dest
            r, c = dest
            self.robot_px = self.cell_center(r, c)
            self.done_deliveries.add(self.delivery_idx)

            # stats update hote hue
            if self.delivery_results:
                last = self.delivery_results[-1]
                self.total_cost   += last["cost"]
                self.total_nodes  += last["nodes"]
                self.total_time_ms += last["time"]
                self.completed    += 1

            self.log(f"[OK] Delivery {self.delivery_idx+1} complete! cost={self.delivery_results[-1]['cost']}")
            self.delivery_idx += 1

            pygame.time.delay(400)
            self.explored_set = set()
            self.drawn_path   = []

            if self.delivery_idx >= 5:
                self.phase   = "done"
                self.running = False
                self.log("** ALL DELIVERIES COMPLETE! **")
            else:
                self.phase = "idle"

    # sare draw functions ko call karta hai screen par dikhane ke liye
    def draw(self):
        self.screen.fill(C["bg"])
        self._draw_topbar()
        self._draw_grid()
        self._draw_panel()
        pygame.display.flip()

    # thori mazeed formatting for title, algo keys aur hints waghaira
    def _draw_topbar(self):
        bar = pygame.Rect(0, 0, WIN_W, GRID_OFFSET_Y)
        pygame.draw.rect(self.screen, C["panel"], bar)
        pygame.draw.line(self.screen, C["border"], (0, GRID_OFFSET_Y-1), (WIN_W, GRID_OFFSET_Y-1))

        # Title
        t = self.font_title.render("[ DELIVERY ROBOT ] PATH PLANNING SIMULATION", True, C["accent"])
        self.screen.blit(t, (14, 10))

        # Algorithm keys hint
        hint = "  [1-7] Algorithm   [SPACE] Start/Pause   [R] Reset   [UP/DN] Speed   [ESC] Quit"
        h = self.font_tiny.render(hint, True, C["muted"])
        self.screen.blit(h, (14, 34))

        # Current algo badge
        algo = ALGO_KEYS[self.algo_idx]
        col  = ALGO_COLORS[algo]
        badge_text = f" {self.algo_idx+1}:{algo} "
        bt = self.font_med.render(badge_text, True, C["bg"])
        bw = bt.get_width() + 8
        bx = WIN_W - PANEL_W - bw - 10
        pygame.draw.rect(self.screen, col, (bx, 12, bw, 20), border_radius=3)
        self.screen.blit(bt, (bx+4, 14))

        # Speed
        sp = self.font_small.render(f"SPD:{self.speed}", True, C["yellow"])
        self.screen.blit(sp, (bx - 70, 16))

        # Status
        if self.phase == "done":
            status, sc = "COMPLETE", C["green"]
        elif self.running and not self.paused:
            status, sc = "RUNNING", C["yellow"]
        elif self.paused:
            status, sc = "PAUSED", C["orange"]
        else:
            status, sc = "READY", C["muted"]
        pygame.draw.circle(self.screen, sc, (WIN_W - PANEL_W - 100, 24), 5)
        st = self.font_small.render(status, True, sc)
        self.screen.blit(st, (WIN_W - PANEL_W - 90, 17))

    # yeh sare 15x15 cells ko unke colors aur details k sath draw karega
    # aur path, ghost path aur top par apna robot bhi draw hoga
    def _draw_grid(self):
        grid_surf_w = GRID_SIZE * CELL
        grid_surf_h = GRID_SIZE * CELL

        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                rect  = self.cell_rect(r, c)
                ctype = self.grid[r][c]
                cost  = self.costs[r][c]

                # Base colour
                if ctype == BUILDING:
                    col = C["building"]
                elif ctype == TRAFFIC:
                    col = C["traffic"]
                elif ctype == DELIVERY:
                    col = C["delivery"]
                elif ctype == BASE:
                    col = C["base"]
                else:
                    col = C["road"]
                pygame.draw.rect(self.screen, col, rect)

                # Explored overlay
                if (r,c) in self.explored_set and ctype != BUILDING:
                    s = pygame.Surface((CELL, CELL), pygame.SRCALPHA)
                    s.fill((0, 180, 220, 50))
                    self.screen.blit(s, rect.topleft)

                # Grid lines
                pygame.draw.rect(self.screen, C["grid_line"], rect, 1)

                # Building — hatching
                if ctype == BUILDING:
                    inner = rect.inflate(-8, -8)
                    pygame.draw.rect(self.screen, C["building2"], inner, border_radius=2)
                    for wr in range(2):
                        for wc in range(2):
                            wx = rect.left + 10 + wc*16
                            wy = rect.top  + 10 + wr*16
                            pygame.draw.rect(self.screen, (50,60,100), (wx,wy,6,5))

                # Traffic warning triangle
                elif ctype == TRAFFIC:
                    cx, cy = rect.centerx, rect.centery
                    pts = [(cx, cy-10),(cx-9,cy+7),(cx+9,cy+7)]
                    pygame.draw.polygon(self.screen, C["orange"], pts)
                    pygame.draw.polygon(self.screen, C["traffic"], pts, 1)
                    wt = self.font_tiny.render("!", True, C["orange"])
                    self.screen.blit(wt, (cx-3, cy-4))
                    ct = self.font_tiny.render(str(cost), True, (180,100,50))
                    self.screen.blit(ct, (rect.left+2, rect.bottom-12))

                # Delivery target
                elif ctype == DELIVERY:
                    idx = self.deliveries.index((r,c)) if (r,c) in self.deliveries else -1
                    done = idx in self.done_deliveries
                    col2 = C["green"] if done else (0,180,90)
                    pygame.draw.circle(self.screen, col2, rect.center, CELL//2-5, 2)
                    lbl = "OK" if done else str(idx+1)
                    lt  = self.font_med.render(lbl, True, col2)
                    self.screen.blit(lt, lt.get_rect(center=rect.center))
                    pkg = self.font_tiny.render("PKG", True, (0,120,60))
                    self.screen.blit(pkg, (rect.left+2, rect.bottom-12))

                # Base station
                elif ctype == BASE:
                    inner = rect.inflate(-6,-6)
                    pygame.draw.rect(self.screen, C["accent"], inner, 2, border_radius=3)
                    bt2 = self.font_med.render("B", True, C["accent"])
                    self.screen.blit(bt2, bt2.get_rect(center=rect.center))
                    pkg = self.font_tiny.render("BASE", True, C["muted"])
                    self.screen.blit(pkg, (rect.left+2, rect.bottom-12))

                # Road cost
                else:
                    ct = self.font_tiny.render(str(cost), True, C["muted"])
                    self.screen.blit(ct, (rect.left+2, rect.bottom-12))

        if len(self.drawn_path) > 1:
            algo_col = ALGO_COLORS[ALGO_KEYS[self.algo_idx]]
            pts = [self.cell_center(r,c) for r,c in self.drawn_path]
            pygame.draw.lines(self.screen, algo_col, False, pts, 3)
            for p in pts:
                pygame.draw.circle(self.screen, algo_col, p, 4)
                pygame.draw.circle(self.screen, C["bg"], p, 2)

        if self.phase in ("moving","exploring") and len(self.current_path) > 1:
            algo_col = ALGO_COLORS[ALGO_KEYS[self.algo_idx]]
            pts = [self.cell_center(r,c) for r,c in self.current_path]
            s = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
            if len(pts) > 1:
                pygame.draw.lines(s, (*algo_col, 40), False, pts, 2)
            self.screen.blit(s, (0,0))

        rx, ry = self.robot_px
        glow = pygame.Surface((CELL, CELL), pygame.SRCALPHA)
        for rad in range(CELL//2, 4, -4):
            alpha = max(0, 60 - rad*3)
            pygame.draw.circle(glow, (*C["robot"], alpha), (CELL//2, CELL//2), rad)
        self.screen.blit(glow, (rx-CELL//2, ry-CELL//2))
        pygame.draw.circle(self.screen, C["robot"], (rx,ry), 13)
        pygame.draw.circle(self.screen, C["bg"],    (rx,ry), 10)
        pygame.draw.circle(self.screen, C["robot"], (rx,ry), 5)
        pygame.draw.circle(self.screen, C["robot"], (rx-4, ry-3), 2)
        pygame.draw.circle(self.screen, C["robot"], (rx+4, ry-3), 2)


    # right side wala panel draw hota hai is se jismein legend, algo list
    # comparison table, stats, delivery progress aur log show hongay
    def _draw_panel(self):
        px = GRID_SIZE * CELL + GRID_OFFSET_X
        panel = pygame.Rect(px, 0, PANEL_W, WIN_H)
        pygame.draw.rect(self.screen, C["panel"], panel)
        pygame.draw.line(self.screen, C["border"], (px, 0), (px, WIN_H), 1)

        y = GRID_OFFSET_Y + 10
        x = px + 12

        # ── Section: Legend ───────────────────────────────────────────────
        y = self._section_title("LEGEND", x, y, px)
        legend_items = [
            (C["base"],      "Base Station"),
            (C["road"],      "Road  (cost 1-5)"),
            (C["traffic"],   "Traffic Zone (10-20)"),
            (C["building"],  "Building - obstacle"),
            (C["delivery"],  "Delivery target"),
            (C["explored"],  "Explored node"),
        ]
        for col, label in legend_items:
            pygame.draw.rect(self.screen, col, (x, y, 14, 14), border_radius=2)
            pygame.draw.rect(self.screen, C["border"], (x, y, 14, 14), 1, border_radius=2)
            lt = self.font_small.render(label, True, C["text"])
            self.screen.blit(lt, (x+20, y))
            y += 18
        y += 4

        # algo selection
        y = self._section_title("SELECT ALGORITHM  [1-7]", x, y, px)
        for i, name in enumerate(ALGO_KEYS):
            col = ALGO_COLORS[name]
            active = (i == self.algo_idx)
            bg = col if active else C["bg"]
            pygame.draw.rect(self.screen, bg, (x, y, PANEL_W-24, 18), border_radius=2)
            if active:
                pygame.draw.rect(self.screen, col, (x, y, PANEL_W-24, 18), 1, border_radius=2)
            num = self.font_small.render(f"{i+1}:", True, col if not active else C["bg"])
            self.screen.blit(num, (x+4, y+2))
            nt = self.font_small.render(name, True, C["bg"] if active else C["text"])
            self.screen.blit(nt, (x+26, y+2))
            y += 22
        y += 6

        # compare table
        y = self._section_title("ALGORITHM COMPARISON", x, y, px)
        if self.comparison:
            hdr_cols = [("ALGO", 100), ("COST", 48), ("NODES", 52), ("ms", 44)]
            hx = x
            for hname, hw in hdr_cols:
                ht = self.font_tiny.render(hname, True, C["muted"])
                self.screen.blit(ht, (hx, y))
                hx += hw
            y += 14
            pygame.draw.line(self.screen, C["border"], (x, y), (x+PANEL_W-24, y))
            y += 4
            valid_costs = [v["cost"] for v in self.comparison.values() if isinstance(v["cost"],int)]
            best_cost = min(valid_costs) if valid_costs else None

            for i, (name, res) in enumerate(self.comparison.items()):
                active = (i == self.algo_idx)
                if active:
                    pygame.draw.rect(self.screen, (20,35,50), (x-4, y-1, PANEL_W-16, 16))
                col = ALGO_COLORS[name]
                is_best = (res["cost"] == best_cost)
                hx = x
                # Algo
                pygame.draw.circle(self.screen, col, (hx+4, y+6), 4)
                short = name[:11]
                nt = self.font_tiny.render(short, True, col if active else C["text"])
                self.screen.blit(nt, (hx+12, y+1))
                hx += 100
                # Cost
                cost_col = C["green"] if is_best else C["text"]
                ct = self.font_tiny.render(str(res["cost"]), True, cost_col)
                self.screen.blit(ct, (hx, y+1))
                hx += 48
                # Nodes
                nt2 = self.font_tiny.render(str(res["nodes"]), True, C["yellow"])
                self.screen.blit(nt2, (hx, y+1))
                hx += 52
                # Time
                tt = self.font_tiny.render(f"{res['time']:.3f}", True, C["muted"])
                self.screen.blit(tt, (hx, y+1))
                y += 16
        else:
            nt = self.font_small.render("Run simulation to populate", True, C["muted"])
            self.screen.blit(nt, (x, y))
            y += 18
        y += 6

        # stats or output you can say
        y = self._section_title("SESSION STATS", x, y, px)
        stats = [
            ("Total Cost",       str(self.total_cost),          C["accent"]),
            ("Nodes Explored",   str(self.total_nodes),         C["yellow"]),
            ("Deliveries Done",  f"{self.completed}/5",         C["green"]),
            ("Compute Time",     f"{self.total_time_ms:.2f}ms", C["purple"]),
            ("Algorithm",        ALGO_KEYS[self.algo_idx],      ALGO_COLORS[ALGO_KEYS[self.algo_idx]]),
        ]
        for label, val, col in stats:
            lt = self.font_small.render(label+":", True, C["muted"])
            vt = self.font_small.render(val, True, col)
            self.screen.blit(lt, (x, y))
            self.screen.blit(vt, (x + PANEL_W//2 - 20, y))
            y += 17
        y += 6

        # delivery process
        y = self._section_title("DELIVERIES", x, y, px)
        for i in range(5):
            done   = i in self.done_deliveries
            active = (i == self.delivery_idx and self.running)
            col    = C["green"] if done else (C["accent"] if active else C["muted"])
            r2,c2  = self.deliveries[i]
            lbl    = f"D{i+1}: ({r2:2d},{c2:2d})"
            # dot
            pygame.draw.circle(self.screen, col, (x+6, y+8), 5)
            lt = self.font_small.render(lbl, True, col)
            self.screen.blit(lt, (x+18, y+1))
            if done and self.delivery_results:
                for dr in self.delivery_results:
                    if dr["n"] == i+1:
                        rt = self.font_tiny.render(f"cost={dr['cost']} n={dr['nodes']}", True, C["muted"])
                        self.screen.blit(rt, (x+120, y+3))
                        break
            y += 18
        y += 6

        # logs
        y = self._section_title("ACTIVITY LOG", x, y, px)
        remaining = WIN_H - y - 10
        lines_fit = remaining // 14
        for line in self.log_lines[-lines_fit:]:
            col = C["green"] if line.startswith("[OK]") or line.startswith("**") else \
                  C["accent"] if line.startswith("[>]") else \
                  C["orange"] if line.startswith("[!]") else C["muted"]
            lt = self.font_tiny.render(line[:52], True, col)
            self.screen.blit(lt, (x, y))
            y += 14

    def _section_title(self, title, x, y, px):
        t = self.font_tiny.render(f"-- {title} {'- '*10}", True, C["muted"])
        self.screen.blit(t, (x, y))
        return y + 16

    # keyboard inputs handle karne ke liye, space se pause/start, r for reset
    # number keys 1-7 se algo pick karo, arrows for speed kam ziada
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False

                elif event.key == pygame.K_SPACE:
                    if self.phase == "done":
                        pass
                    elif not self.running:
                        self.running = True
                        self.paused  = False
                        self.log(f"[>] Simulation started [{ALGO_KEYS[self.algo_idx]}]")
                    else:
                        self.paused = not self.paused
                        self.log("[||] Paused" if self.paused else "[>] Resumed")

                elif event.key == pygame.K_r:
                    self._init_state()
                    self.log("[R] Reset.")

                elif event.key == pygame.K_UP:
                    self.speed = min(20, self.speed + 1)
                elif event.key == pygame.K_DOWN:
                    self.speed = max(1, self.speed - 1)

                # Algorithm select 1-7
                elif pygame.K_1 <= event.key <= pygame.K_7:
                    idx = event.key - pygame.K_1
                    if idx < len(ALGO_KEYS):
                        self.algo_idx = idx
                        self.log(f"[>] Algorithm: {ALGO_KEYS[idx]}")

        return True

    # main loop jo bas chalta rehta hai jab tak k window close na ki jaye
    def run(self):
        running = True
        while running:
            running = self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(FPS)
        pygame.quit()
        sys.exit()


# Main
if __name__ == "__main__":
    app = UrbanDeliveryRobot()
    app.run()