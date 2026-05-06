"""
GUI Navigation System with Pygame Visualization
"""

import pygame
import sys
from a_star_navigation import AStarNavigation

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GRAY = (128, 128, 128)
LIGHT_BLUE = (173, 216, 230)

class GUINavigation:
    def __init__(self, width=30, height=20, cell_size=30):
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.screen_width = width * cell_size
        self.screen_height = height * cell_size + 100
        
        pygame.init()
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("A* Navigation System - Pathfinding Visualizer")
        self.clock = pygame.time.Clock()
        
        # Initialize grid
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        self.nav = AStarNavigation(self.grid, width, height)
        
        # User interface states
        self.start = None
        self.goal = None
        self.path = None
        self.mode = "start"  # start, goal, obstacle
        self.running = True
        
        self.font = pygame.font.Font(None, 24)
        self.big_font = pygame.font.Font(None, 36)
    
    def draw_grid(self):
        """Draw the grid and all elements"""
        for y in range(self.height):
            for x in range(self.width):
                rect = pygame.Rect(x * self.cell_size, y * self.cell_size, 
                                   self.cell_size, self.cell_size)
                
                # Fill cell color based on type
                if self.grid[y][x] == 1:
                    color = BLACK  # Obstacle
                elif (x, y) == self.start:
                    color = GREEN  # Start
                elif (x, y) == self.goal:
                    color = RED  # Goal
                elif self.path and (x, y) in self.path:
                    color = YELLOW  # Path
                else:
                    color = WHITE  # Free
                
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, GRAY, rect, 1)  # Grid lines
    
    def draw_ui(self):
        """Draw UI elements"""
        # UI Panel
        panel_rect = pygame.Rect(0, self.height * self.cell_size, 
                                  self.screen_width, 100)
        pygame.draw.rect(self.screen, LIGHT_BLUE, panel_rect)
        
        # Mode indicator
        mode_text = self.font.render(f"Mode: {self.mode.upper()}", True, BLACK)
        self.screen.blit(mode_text, (10, self.height * self.cell_size + 10))
        
        # Instructions
        instructions = [
            "Click: Set point | S: Start mode | G: Goal mode | O: Obstacle mode",
            "C: Clear path | R: Reset all | Space: Find path | ESC: Exit"
        ]
        
        for i, instruction in enumerate(instructions):
            text = self.font.render(instruction, True, BLACK)
            self.screen.blit(text, (10, self.height * self.cell_size + 40 + i * 25))
        
        # Start and Goal info
        if self.start:
            start_text = self.font.render(f"Start: {self.start}", True, GREEN)
            self.screen.blit(start_text, (400, self.height * self.cell_size + 10))
        
        if self.goal:
            goal_text = self.font.render(f"Goal: {self.goal}", True, RED)
            self.screen.blit(goal_text, (400, self.height * self.cell_size + 40))
        
        # Path info
        if self.path:
            path_text = self.font.render(f"Path length: {len(self.path)} steps", True, BLUE)
            self.screen.blit(path_text, (600, self.height * self.cell_size + 10))
    
    def find_path(self):
        """Find path using A* algorithm"""
        if self.start and self.goal:
            self.path, metrics = self.nav.find_path(self.start, self.goal)
            if self.path:
                print(f"Path found! Length: {len(self.path)}, Nodes explored: {metrics['nodes_explored']}")
            else:
                print("No path found!")
    
    def reset(self):
        """Reset the grid"""
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.nav = AStarNavigation(self.grid, self.width, self.height)
        self.start = None
        self.goal = None
        self.path = None
    
    def clear_path(self):
        """Clear only the path, keep obstacles"""
        self.path = None
    
    def handle_click(self, pos):
        """Handle mouse clicks"""
        x, y = pos[0] // self.cell_size, pos[1] // self.cell_size
        
        if 0 <= x < self.width and 0 <= y < self.height:
            if self.mode == "start":
                self.start = (x, y)
                self.path = None
            elif self.mode == "goal":
                self.goal = (x, y)
                self.path = None
            elif self.mode == "obstacle":
                if (x, y) != self.start and (x, y) != self.goal:
                    self.grid[y][x] = 1 if self.grid[y][x] == 0 else 0
                    self.nav = AStarNavigation(self.grid, self.width, self.height)
                    self.path = None
    
    def run(self):
        """Main game loop"""
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    elif event.key == pygame.K_s:
                        self.mode = "start"
                    elif event.key == pygame.K_g:
                        self.mode = "goal"
                    elif event.key == pygame.K_o:
                        self.mode = "obstacle"
                    elif event.key == pygame.K_SPACE:
                        self.find_path()
                    elif event.key == pygame.K_c:
                        self.clear_path()
                    elif event.key == pygame.K_r:
                        self.reset()
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left click
                        self.handle_click(pygame.mouse.get_pos())
            
            self.screen.fill(WHITE)
            self.draw_grid()
            self.draw_ui()
            pygame.display.flip()
            self.clock.tick(60)
        
        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    gui = GUINavigation(width=30, height=20)
    gui.run()
