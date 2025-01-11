'''
slam.py

Implementación de SLAM (Simultaneous Localization and Mapping) usando LIDAR.
'''

import numpy as np
import matplotlib.pyplot as plt

class SLAM:
    def __init__(self, map_size=20, resolution=0.05):
        """
        Inicializa el sistema SLAM.
        
        Args:
            map_size: Tamaño del mapa en metros
            resolution: Resolución del mapa en metros/celda
        """
        self.resolution = resolution
        self.map_size = map_size
        self.grid_size = int(map_size / resolution)
        
        # Inicializar mapa de ocupación
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size))
        self.points = []  # Lista de puntos detectados
        self.robot_path = []  # Lista de posiciones del robot
        self.current_pose = None  # Pose actual del robot
        
        # Configuración de visualización
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(-map_size/2, map_size/2)
        self.ax.set_ylim(-map_size/2, map_size/2)
        self.ax.grid(True)
        
    def world_to_grid(self, x, y):
        """Convierte coordenadas del mundo a índices de la grilla."""
        gx = int((x + self.map_size/2) / self.resolution)
        gy = int((y + self.map_size/2) / self.resolution)
        return gx, gy
        
    def update_map(self, lidar_data, robot_pose):
        """
        Actualiza el mapa con nuevas lecturas del LIDAR.
        """
        self.current_pose = robot_pose  # Guardar la pose actual
        x, y, theta = robot_pose
        self.robot_path.append((x, y))
        
        # Convertir lecturas del LIDAR a puntos en el espacio
        new_points = []
        for i, distance in enumerate(lidar_data):
            if 0.1 < distance < 5.0:  # Filtrar lecturas inválidas y muy lejanas
                # Calcular ángulo para cada lectura
                angle = theta - np.pi/2 + (i * np.pi/len(lidar_data))
                # Convertir a coordenadas cartesianas
                point_x = x + distance * np.cos(angle)
                point_y = y + distance * np.sin(angle)
                new_points.append((point_x, point_y))
                
                # Actualizar grid de ocupación
                gx, gy = self.world_to_grid(point_x, point_y)
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    self.occupancy_grid[gx, gy] = 1
        
        # Añadir nuevos puntos a la lista
        self.points.extend(new_points)
        
        # Mantener un número máximo de puntos para evitar sobrecarga
        max_points = 10000
        if len(self.points) > max_points:
            self.points = self.points[-max_points:]
    
    def display(self):
        """Muestra el mapa actual."""
        self.ax.clear()
        
        # Configurar límites y rejilla
        self.ax.set_xlim(-self.map_size/2, self.map_size/2)
        self.ax.set_ylim(-self.map_size/2, self.map_size/2)
        self.ax.grid(True)
        self.ax.set_title('Mapa SLAM')
        
        # Mostrar grid de ocupación
        extent = [-self.map_size/2, self.map_size/2, -self.map_size/2, self.map_size/2]
        self.ax.imshow(self.occupancy_grid.T, extent=extent, 
                      cmap='Blues', alpha=0.5, origin='lower')
        
        # Dibujar puntos detectados
        if self.points:
            points = np.array(self.points)
            self.ax.scatter(points[:, 0], points[:, 1], 
                          c='blue', s=2, alpha=0.6, label='Obstáculos')
        
        # Dibujar trayectoria del robot
        if len(self.robot_path) > 1:
            path = np.array(self.robot_path)
            self.ax.plot(path[:, 0], path[:, 1], 
                        'r--', linewidth=1, alpha=0.7, label='Trayectoria')
        
        # Dibujar posición actual del robot
        if self.current_pose:  # Usar la pose guardada
            x, y, theta = self.current_pose
            self.ax.plot(x, y, 'ro', markersize=8, label='Robot')
            
            # Mostrar dirección del robot
            direction_len = 0.3
            dx = direction_len * np.cos(theta)
            dy = direction_len * np.sin(theta)
            self.ax.arrow(x, y, dx, dy,
                         head_width=0.1, head_length=0.2, fc='r', ec='r')
        
        self.ax.legend(loc='upper right')
        plt.pause(0.01) 