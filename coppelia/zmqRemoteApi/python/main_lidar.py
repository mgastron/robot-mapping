'''
main_lidar.py

Implementación simple de mapeo usando LIDAR en el robot Pioneer P3DX,
similar al mapeo de colisiones del main.py.
'''

import robotica
import numpy as np
import matplotlib.pyplot as plt
from avoid import wall_following
import time
import traceback

class LidarMapper:
    def __init__(self):
        """
        Inicializa el mapeador de LIDAR similar al main.py pero usando todo el rango
        """
        # Configuración de visualización
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.setup_plot()
        
        # Lista de puntos detectados
        self.points = []
        
        # Debug info
        self.max_distance_detected = 0
        
    def setup_plot(self):
        """Configura la visualización del mapa."""
        self.ax.set_xlim(-10, 10)  # Aumentado el rango de visualización
        self.ax.set_ylim(-10, 10)
        self.ax.grid(True)
        self.ax.set_title('Mapa LIDAR')
    
    def update_map(self, lidar_data, robot_pose):
        """
        Actualiza el mapa usando las lecturas del LIDAR.
        Guarda solo los puntos más lejanos detectados.
        """
        x, y, theta = robot_pose
        
        # Agrupar lecturas por sectores angulares para mantener solo las más lejanas
        angle_resolution = np.pi/len(lidar_data)  # Resolución angular del LIDAR
        
        # Procesar lecturas del LIDAR
        for i, distance in enumerate(lidar_data):
            # Usar todo el rango del LIDAR, ignorando solo lecturas inválidas
            if 0.1 < distance:  # Sin límite superior para aprovechar largo alcance
                # Calcular ángulo para esta lectura
                angle = theta - np.pi/2 + (i * np.pi/len(lidar_data))
                
                # Calcular punto donde se detectó el obstáculo
                point_x = x + distance * np.cos(angle)
                point_y = y + distance * np.sin(angle)
                
                # Actualizar máxima distancia detectada (para debug)
                if distance > self.max_distance_detected:
                    self.max_distance_detected = distance
                    print(f"Nueva máxima distancia detectada: {distance:.2f}m")
                
                # Añadir el punto detectado
                self.points.append((point_x, point_y))
    
    def display(self):
        """Muestra el mapa actual."""
        self.ax.clear()
        self.setup_plot()
        
        if self.points:
            # Convertir lista de puntos a array numpy
            points = np.array(self.points)
            
            # Dibujar puntos detectados
            self.ax.plot(points[:, 0], points[:, 1], 'b.', markersize=2)
            
            # Mostrar información sobre el rango
            self.ax.text(-9.5, 9.5, f'Max dist: {self.max_distance_detected:.2f}m', 
                        bbox=dict(facecolor='white', alpha=0.7))
        
        plt.pause(0.01)

def main(args=None):
    print("Iniciando conexión con CoppeliaSim...")
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX', use_lidar=True)
    
    if not hasattr(robot, 'lidar'):
        print("ERROR: No se encontró el LIDAR")
        return
    
    # Inicializar sistema de mapeo
    mapper = LidarMapper()
    
    print("Iniciando simulación...")
    coppelia.start_simulation()
    
    iteration = 0
    try:
        while coppelia.is_running():
            iteration += 1
            
            try:
                # Obtener datos de sensores
                lidar_data = robot.get_lidar()
                sonar_data = robot.get_sonar()
                robot_pose = robot.get_position()
                
                # Debug info cada 100 iteraciones
                if iteration % 100 == 0 and lidar_data:
                    max_dist = max(d for d in lidar_data if d > 0.1)
                    print(f"\nIteración {iteration}")
                    print(f"Distancia máxima actual: {max_dist:.2f}m")
                
                # Actualizar mapa y mostrar
                if lidar_data:
                    mapper.update_map(lidar_data, robot_pose)
                    if iteration % 5 == 0:
                        mapper.display()
                
                # Usar el wall_following original
                lspeed, rspeed = wall_following(sonar_data)
                robot.set_speed(lspeed, rspeed)
                
                time.sleep(0.05)
                
            except Exception as e:
                print(f"Error en iteración {iteration}:")
                print(traceback.format_exc())
                robot.set_speed(0, 0)
                time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario")
    finally:
        robot.set_speed(0, 0)
        print("Guardando mapa final...")
        mapper.display()
        plt.savefig("lidar_map.png", dpi=300, bbox_inches='tight')
        coppelia.stop_simulation()

if __name__ == '__main__':
    main() 