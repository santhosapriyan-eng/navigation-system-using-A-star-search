## 📄 Complete Code

### a_star_navigation.py
```python
"""
A* Navigation System
Implementation of A* pathfinding algorithm with visualization
"""

import heapq
import math
import time
from typing import List, Tuple, Dict, Optional

class AStarNavigation:
    def __init__(self, grid: List[List[int]], width: int, height: int):
        """
        Initialize A* Navigation System
        
        Args:
            grid: 2D grid where 0 = free, 1 = obstacle
            width: Grid width
            height: Grid height
        """
        self.grid = grid
        self.width = width
        self.height = height
        
    def manhattan_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """Manhattan distance heuristic (for 4-direction movement)"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def euclidean_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic (for free movement)"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def chebyshev_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """Chebyshev distance (for 8-direction movement)"""
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))
    
    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring nodes (4-directional)"""
        x, y = node
        neighbors = []
        
        # Four directions: up, down, left, right
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny][nx] == 0:  # Check if not obstacle
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int], 
                   heuristic: str = "manhattan") -> Tuple[Optional[List[Tuple[int, int]]], Dict]:
        """
        Find shortest path using A* algorithm
        
        Args:
            start: Starting coordinates (x, y)
            goal: Goal coordinates (x, y)
            heuristic: Type of heuristic ('manhattan', 'euclidean', 'chebyshev')
        
        Returns:
            Tuple of (path, metrics)
        """
        start_time = time.time()
        
        # Select heuristic function
        if heuristic == "manhattan":
            h_func = self.manhattan_distance
        elif heuristic == "euclidean":
            h_func = self.euclidean_distance
        else:
            h_func = self.chebyshev_distance
        
        # Priority queue: (f_score, node)
        open_set = [(0 + h_func(start, goal), start)]
        heapq.heapify(open_set)
        
        # Dictionaries for tracking
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score = {start: 0}
        f_score = {start: h_func(start, goal)}
        
        # Tracking metrics
        nodes_explored = 0
        open_set_history = []
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            nodes_explored += 1
            open_set_history.append(len(open_set))
            
            # Goal reached
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                execution_time = time.time() - start_time
                
                metrics = {
                    'path_length': len(path),
                    'nodes_explored': nodes_explored,
                    'execution_time_ms': execution_time * 1000,
                    'max_open_set_size': max(open_set_history),
                    'path': path
                }
                
                return path, metrics
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1  # Cost between adjacent nodes
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + h_func(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        execution_time = time.time() - start_time
        metrics = {
            'path_length': 0,
            'nodes_explored': nodes_explored,
            'execution_time_ms': execution_time * 1000,
            'max_open_set_size': max(open_set_history) if open_set_history else 0,
            'path': None
        }
        
        return None, metrics
    
    def print_grid(self, path: Optional[List[Tuple[int, int]]] = None, 
                   start: Optional[Tuple[int, int]] = None, 
                   goal: Optional[Tuple[int, int]] = None):
        """Print grid with path visualization"""
        path_set = set(path) if path else set()
        
        print("\n" + "="*50)
        print("GRID MAP VISUALIZATION")
        print("="*50)
        print("Legend: # = Obstacle | . = Free | S = Start | G = Goal | * = Path")
        print("-"*50)
        
        for y in range(self.height):
            row = ""
            for x in range(self.width):
                if (x, y) == start:
                    row += "S "
                elif (x, y) == goal:
                    row += "G "
                elif (x, y) in path_set:
                    row += "* "
                elif self.grid[y][x] == 1:
                    row += "# "
                else:
                    row += ". "
            print(row)
        print("="*50)


class NavigationDemo:
    """Demo class to showcase A* navigation system"""
    
    @staticmethod
    def create_sample_grid():
        """Create a sample grid with obstacles"""
        grid = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0, 1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0, 1, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0, 1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]
        return grid, len(grid[0]), len(grid)
    
    @staticmethod
    def create_maze_grid():
        """Create a maze-like grid"""
        grid = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ]
        return grid, len(grid[0]), len(grid)
    
    def run_demo(self):
        """Run complete demo with different scenarios"""
        print("\n" + "="*60)
        print("A* NAVIGATION SYSTEM DEMO")
        print("="*60)
        
        # Demo 1: Basic pathfinding
        print("\n📍 DEMO 1: Basic Pathfinding")
        print("-"*40)
        grid, width, height = self.create_sample_grid()
        nav = AStarNavigation(grid, width, height)
        
        start = (0, 0)
        goal = (9, 9)
        
        print(f"Start: {start}")
        print(f"Goal: {goal}")
        
        path, metrics = nav.find_path(start, goal, heuristic="manhattan")
        
        if path:
            print(f"\n✅ Path found!")
            print(f"📏 Path length: {metrics['path_length']} steps")
            print(f"🔍 Nodes explored: {metrics['nodes_explored']}")
            print(f"⏱️ Execution time: {metrics['execution_time_ms']:.2f} ms")
            print(f"📊 Max open set size: {metrics['max_open_set_size']}")
            print(f"🗺️ Path: {path[:10]}..." if len(path) > 10 else f"🗺️ Path: {path}")
        else:
            print("❌ No path found!")
        
        nav.print_grid(path, start, goal)
        
        # Demo 2: Compare heuristics
        print("\n📊 DEMO 2: Heuristic Comparison")
        print("-"*40)
        
        heuristics = ["manhattan", "euclidean", "chebyshev"]
        results = {}
        
        for h in heuristics:
            path, metrics = nav.find_path(start, goal, heuristic=h)
            results[h] = metrics
            print(f"\n{h.upper()} Heuristic:")
            print(f"  Path length: {metrics['path_length']}")
            print(f"  Nodes explored: {metrics['nodes_explored']}")
            print(f"  Time: {metrics['execution_time_ms']:.2f} ms")
        
        # Demo 3: Maze navigation
        print("\n🏰 DEMO 3: Maze Navigation")
        print("-"*40)
        maze_grid, maze_width, maze_height = self.create_maze_grid()
        maze_nav = AStarNavigation(maze_grid, maze_width, maze_height)
        
        start_maze = (0, 0)
        goal_maze = (14, 6)
        
        print(f"Navigating through complex maze...")
        path_maze, metrics_maze = maze_nav.find_path(start_maze, goal_maze, heuristic="manhattan")
        
        if path_maze:
            print(f"✅ Path found through maze!")
            print(f"📏 Path length: {metrics_maze['path_length']} steps")
            print(f"🔍 Nodes explored: {metrics_maze['nodes_explored']}")
        else:
            print("❌ No path through maze!")
        
        maze_nav.print_grid(path_maze, start_maze, goal_maze)


class InteractiveNavigation:
    """Interactive navigation system with user input"""
    
    def __init__(self):
        self.grid = None
        self.nav = None
        self.width = 20
        self.height = 15
    
    def create_empty_grid(self):
        """Create empty grid"""
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.nav = AStarNavigation(self.grid, self.width, self.height)
    
    def add_obstacles(self):
        """Add obstacles interactively"""
        print("\nAdd obstacles to the grid")
        print("Enter coordinates in format: x,y (0-19,0-14)")
        print("Type 'done' when finished")
        
        while True:
            user_input = input("Obstacle: ")
            if user_input.lower() == 'done':
                break
            try:
                x, y = map(int, user_input.split(','))
                if 0 <= x < self.width and 0 <= y < self.height:
                    self.grid[y][x] = 1
                    print(f"✓ Obstacle added at ({x}, {y})")
                else:
                    print("❌ Coordinates out of range!")
            except:
                print("❌ Invalid format! Use: x,y")
        
        self.nav = AStarNavigation(self.grid, self.width, self.height)
    
    def find_path_interactive(self):
        """Interactive path finding"""
        print("\n🎯 Find Path")
        print("-"*30)
        
        try:
            start_x = int(input("Start X (0-19): "))
            start_y = int(input("Start Y (0-14): "))
            goal_x = int(input("Goal X (0-19): "))
            goal_y = int(input("Goal Y (0-14): "))
            
            start = (start_x, start_y)
            goal = (goal_x, goal_y)
            
            print("\nSelect heuristic:")
            print("1. Manhattan (4-direction)")
            print("2. Euclidean (straight line)")
            print("3. Chebyshev (8-direction)")
            
            h_choice = input("Choice (1/2/3): ")
            heuristic_map = {'1': 'manhattan', '2': 'euclidean', '3': 'chebyshev'}
            heuristic = heuristic_map.get(h_choice, 'manhattan')
            
            path, metrics = self.nav.find_path(start, goal, heuristic)
            
            if path:
                print(f"\n✅ Path found!")
                print(f"Path length: {metrics['path_length']}")
                print(f"Nodes explored: {metrics['nodes_explored']}")
                print(f"Time: {metrics['execution_time_ms']:.2f} ms")
                self.nav.print_grid(path, start, goal)
            else:
                print("❌ No path exists between these points!")
                
        except ValueError:
            print("❌ Invalid input!")
    
    def run(self):
        """Run interactive session"""
        print("\n" + "="*50)
        print("INTERACTIVE NAVIGATION SYSTEM")
        print("="*50)
        
        self.create_empty_grid()
        
        while True:
            print("\n" + "-"*40)
            print("MAIN MENU")
            print("-"*40)
            print("1. Add obstacles")
            print("2. Find path")
            print("3. Reset grid")
            print("4. Print current grid")
            print("5. Exit")
            
            choice = input("\nSelect option: ")
            
            if choice == '1':
                self.add_obstacles()
            elif choice == '2':
                self.find_path_interactive()
            elif choice == '3':
                self.create_empty_grid()
                print("✓ Grid reset!")
            elif choice == '4':
                self.nav.print_grid()
            elif choice == '5':
                print("Goodbye!")
                break
            else:
                print("Invalid option!")


def main():
    """Main function"""
    print("\n" + "="*60)
    print("A* PATHFINDING ALGORITHM - NAVIGATION SYSTEM")
    print("="*60)
    
    print("\nSelect mode:")
    print("1. Run Demo (automatic tests)")
    print("2. Interactive Mode (manual navigation)")
    
    mode = input("\nChoice (1/2): ")
    
    if mode == '1':
        demo = NavigationDemo()
        demo.run_demo()
    elif mode == '2':
        interactive = InteractiveNavigation()
        interactive.run()
    else:
        print("Invalid choice!")


if __name__ == "__main__":
    main()
