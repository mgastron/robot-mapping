import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import numpy as np

class OccupancyMap:
    def __init__(self, map_size=10, resolution=0.1):
        self.map_size = map_size
        self.resolution = resolution
        self.grid = np.zeros((int(map_size / resolution), int(map_size / resolution)))
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-map_size / 2, map_size / 2)
        self.ax.set_ylim(-map_size / 2, map_size / 2)
        self.ax.set_aspect('equal')
        plt.xlabel("X [meters]")
        plt.ylabel("Y [meters]")
        plt.title("Occupancy Map (Grid-Based)")
        plt.ion()
        plt.show()

    def world_to_grid(self, x, y):
        grid_x = int((x + self.map_size / 2) / self.resolution)
        grid_y = int((y + self.map_size / 2) / self.resolution)
        return grid_x, grid_y

    def update(self, robot_pos):
        robot_x, robot_y, robot_theta = robot_pos
        grid_x, grid_y = self.world_to_grid(robot_x, robot_y)
        if 0 <= grid_x < self.grid.shape[0] and 0 <= grid_y < self.grid.shape[1]:
            self.grid[grid_x, grid_y] = 1  # Marcar como ocupado
            self.ax.plot(robot_x, robot_y, 'bo', markersize=1, alpha=0.5)

    def display(self):
        plt.pause(0.1)

    def save_to_file(self, filename):
        plt.savefig(f"{filename}.png", bbox_inches='tight', dpi=300)  # Resolución mejorada
        print(f"Mapa guardado como imagen en {filename}.png")
