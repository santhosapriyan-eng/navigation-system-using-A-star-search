# 🗺️ Navigation System Using A* Search Algorithm

An intelligent navigation system that finds the shortest path between two points using the A* (A-Star) search algorithm, with both console and GUI implementations.

## 🎯 Project Overview

This project implements the A* pathfinding algorithm, one of the most efficient and widely used algorithms for finding the shortest path in grid-based environments. The system can be used for GPS navigation, robotics path planning, and game AI.

## 📐 Algorithm Explanation

### A* Cost Function
f(n) = g(n) + h(n)

text
- **g(n)**: Actual cost from start node to current node
- **h(n)**: Heuristic estimated cost from current node to goal
- **f(n)**: Total estimated cost

### Heuristic Functions Used
- **Manhattan Distance**: For grid-based 4-direction movement
- **Euclidean Distance**: For free movement in continuous space

## ✨ Features

- ✅ **Optimal Path Finding** - Guarantees shortest path when heuristic is admissible
- ✅ **Multiple Heuristics** - Manhattan, Euclidean, Chebyshev distance
- ✅ **Obstacle Avoidance** - Dynamic obstacle handling
- ✅ **Visualization** - Grid visualization with path highlighting
- ✅ **GUI Interface** - Interactive map with click-to-set start/goal
- ✅ **Real-time Performance** - Efficient node exploration
- ✅ **Custom Maps** - Load custom grid maps from JSON
- ✅ **Performance Metrics** - Nodes explored, path length, execution time

## 🛠️ Technologies Used

- **Python 3.8+** - Core implementation
- **Pygame** - GUI visualization
- **NumPy** - Grid operations
- **Heapq** - Priority queue for open list
