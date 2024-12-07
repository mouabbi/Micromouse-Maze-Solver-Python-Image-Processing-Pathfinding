
# 🧩 Maze Solver and Micromouse Pathfinding

This project includes:

- 🗺️ **MazeSolver**: A Python class for solving mazes using A* and BFS algorithms, with visualization capabilities.
- 📓 **micromouse.ipynb**: A notebook for generating mazes from images and displaying them as grids.

The tools help simulate and solve maze navigation problems, useful for applications like robotics and micromouse competitions.

---

## 🌟 Features

- 🗺️ **MazeSolver**:
  - 🔍 **Pathfinding Algorithms**: Implements A* and BFS algorithms to find the shortest path.
  - 🎨 **Visualization**: Displays the maze and highlights the path.
  - ⚙️ **Custom Heuristic**: Utilizes Manhattan distance for A* optimization.

- 🛠️ **Micromouse Notebook**:
  - 🖼️ **Maze Generation**: Creates maze grids from input images.
  - 🔧 **Grid Customization**: Supports resolution control for detailed or abstract representations.
  - 📊 **Visualization**: Displays mazes with clear walls and paths.

---

## ⚙️ Installation

1. 🛠️ **Clone the repository**:
   ```bash
   git clone https://github.com/mouabbi/Micromouse-Maze-Solver-Python-Image-Processing-Pathfinding.git
   cd Micromouse-Maze-Solver-Python-Image-Processing-Pathfinding
   ```

2. 🛠️ **Install Dependencies**:  
   Ensure you have Python 3.7+ installed. Install the required packages:  
   ```bash
   pip install numpy matplotlib pillow heapq deque
   ```

3. 🚀 **Setup Notebook Environment**:
   If Jupyter Notebook is not already installed, install it using:
   ```bash
   pip install notebook
   ```

---

## 🧭 Usage

### 🗺️ MazeSolver

#### 🏁 Import and Initialize:
```python
from mazeSolver import MazeSolver
import numpy as np

# Example maze: generate_and_show_maze() function
maze = generate_and_show_maze(image_path="cc.jpg", resolution=400, threshold=200)
```

![maze example](/images/maze_exp.png)


#### Initialize solver & Start and End:
```python
start = (0, 0)
end = (3, 3)
solver = MazeSolver(maze, start, end)
```

#### Display Maze with Start and End Positions:
```python 
solver.display_maze(resolution=32)
```

 ![maze with start and end positions](/images/maze_with_start_ans_end.png)


#### 🔍 Find a Path:
```python
solver.findpath()
```

#### 🎨 Visualize the Maze and Path:
```python
solver.display_path()
```

![maze with solved path](/images/maze_solved.png)


---

## 📦 Dependencies

- 🐍 **Python 3.7+**
- 📚 **Libraries**:
  - 🔢 `numpy`: Matrix manipulation
  - 📊 `matplotlib`: Visualization
  - 🖼️ `Pillow`: Image processing
  - ⚙️ `heapq` and `deque`: Python standard library
