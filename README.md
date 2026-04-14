# 🤖 Urban Delivery Robot — Path Planning Simulation

A real-time, interactive path planning visualizer built with Python and Pygame. Watch a delivery robot navigate a procedurally generated city grid using 7 classical AI search algorithms — side by side, with live stats and full algorithm comparison.

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=flat-square&logo=python)
![Pygame](https://img.shields.io/badge/Pygame-2.x-green?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)

---

## ✨ Features

- **7 Search Algorithms** — BFS, DFS, UCS, Greedy (Manhattan & Euclidean), A\* (Manhattan & Euclidean)
- **Live Traversal Animation** — watch the exploration frontier expand before the robot moves
- **Algorithm Comparison Table** — cost, nodes explored, and compute time for all 7 algorithms on every delivery
- **Procedural City Grid** — randomized buildings, traffic zones, roads, and delivery targets each run
- **Weighted Costs** — roads (1–5), traffic jams (10–20), smooth base station (1)
- **Session Stats** — cumulative cost, nodes, deliveries completed, and total compute time
- **Speed Control** — adjust movement speed on the fly
- **Activity Log** — timestamped event feed in the UI panel

---

## 🗺️ The Grid

| Cell Type | Color | Description |
|-----------|-------|-------------|
| **Base** | Dark blue | Robot starting point `(1,1)` |
| **Road** | Dark grey | Traversable, cost 1–5 |
| **Traffic Zone** | Amber | High-cost traversal, cost 10–20 |
| **Building** | Grey block | Impassable obstacle |
| **Delivery Target** | Green circle | 5 delivery stops per run |
| **Explored Node** | Cyan tint | Cells visited during search |

---

## 🧠 Algorithms

| # | Algorithm | Optimality | Notes |
|---|-----------|-----------|-------|
| 1 | **BFS** | Unweighted shortest path | Ignores edge costs |
| 2 | **DFS** | No | Fast but suboptimal |
| 3 | **UCS** | ✅ Yes | Optimal with weighted edges |
| 4 | **Greedy-Manhattan** | No | Fast, heuristic-only |
| 5 | **Greedy-Euclidean** | No | Fast, heuristic-only |
| 6 | **A\*-Manhattan** | ✅ Yes | Optimal + efficient |
| 7 | **A\*-Euclidean** | ✅ Yes | Optimal + efficient |

---

## 🚀 Getting Started

### Prerequisites

- Python 3.8 or higher
- pip

### Installation

```bash
# 1. Clone the repository
git clone https://github.com/YOUR_USERNAME/urban-delivery-robot.git
cd urban-delivery-robot

# 2. Install dependencies
pip install -r requirements.txt

# 3. Run the simulation
python main.py
```

### requirements.txt

```
pygame>=2.1.0
```

---

## 🎮 Controls

| Key | Action |
|-----|--------|
| `SPACE` | Start / Pause the simulation |
| `1` – `7` | Switch between the 7 algorithms |
| `↑` / `↓` | Increase / Decrease robot speed |
| `R` | Reset — regenerates a brand-new city grid |
| `ESC` | Quit |

---

## 📁 Project Structure

```
urban-delivery-robot/
├── main.py              # Entry point — run this
├── requirements.txt     # Python dependencies
└── README.md
```

All simulation logic lives in `main.py`:

- `create_environment()` — procedural grid generation
- `get_neighbors()` — movement rules (buildings blocked)
- `bfs / dfs / ucs / greedy / astar` — search algorithm implementations
- `run_algorithm()` — unified algorithm runner with timing
- `UrbanDeliveryRobot` — main Pygame app class (rendering, input, game loop)

---

## 🖥️ How It Works

1. **Grid Generation** — a 15×15 city is procedurally built with ~38 buildings, traffic zones, and 5 delivery targets, seeded for reproducibility.
2. **Algorithm Selection** — press `1`–`7` to pick an algorithm before or between deliveries.
3. **Exploration Phase** — the selected algorithm searches for a path; explored cells are highlighted in real time.
4. **Movement Phase** — the robot smoothly follows the computed path to the delivery target.
5. **Comparison** — after each delivery, all 7 algorithms are benchmarked on that same trip and displayed in the panel.

---

## 📊 Sample Output

After each delivery the right-side panel shows a comparison table like:

```
ALGO            COST   NODES   ms
─────────────────────────────────
BFS              34      89   0.412
DFS              61      43   0.198
UCS              29      97   0.531   ← best cost
Greedy-Manh      31      22   0.095
Greedy-Eucl      30      19   0.088
A*-Manh          29      41   0.213   ← best cost
A*-Eucl          29      38   0.201   ← best cost
```

---

## 🔧 Configuration

Key constants at the top of `main.py` you can tweak:

| Constant | Default | Description |
|----------|---------|-------------|
| `GRID_SIZE` | `15` | Grid dimensions (NxN) |
| `CELL` | `44` | Pixel size of each cell |
| `FPS` | `100` | Frames per second cap |
| `SEED` | `42` | Random seed for grid generation |

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.

---

*Built as an AI/search algorithms visualizer. Great for studying uninformed vs informed search, heuristic design, and the cost-vs-speed tradeoffs between pathfinding strategies.*
