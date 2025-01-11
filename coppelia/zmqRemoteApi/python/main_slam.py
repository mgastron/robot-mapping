'''
main_slam.py

Implementación de SLAM usando LIDAR en el robot Pioneer P3DX.
'''

import robotica
from slam import SLAM
from avoid import wall_following
import matplotlib.pyplot as plt
import time
import traceback

def main(args=None):
    # Inicializar conexión con CoppeliaSim
    print("Iniciando conexión con CoppeliaSim...")
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX', use_lidar=True)
    
    # Verificar que el LIDAR está disponible
    print("Verificando LIDAR...")
    if not hasattr(robot, 'lidar'):
        print("ERROR: No se pudo encontrar el LIDAR en el robot")
        return
    
    # Inicializar SLAM
    print("Inicializando sistema SLAM...")
    slam = SLAM(map_size=20, resolution=0.05)
    
    # Factor de velocidad para el robot
    speed_factor = 1.0
    
    # Detener el robot antes de iniciar
    robot.set_speed(0, 0)
    
    print("Iniciando simulación...")
    coppelia.start_simulation()
    
    # Esperar a que la simulación se estabilice
    print("Esperando a que la simulación se estabilice...")
    time.sleep(1)
    
    # Asegurarse de que el robot esté detenido
    robot.set_speed(0, 0)
    time.sleep(0.5)
    
    # Obtener posición inicial
    initial_pose = robot.get_position()
    print(f"Posición inicial: x={initial_pose[0]:.2f}, y={initial_pose[1]:.2f}, theta={initial_pose[2]:.2f}")
    
    # Esperar a que el LIDAR se inicialice
    print("\nConfigurando LIDAR...")
    max_attempts = 3
    for attempt in range(max_attempts):
        lidar_data = robot.get_lidar()
        if lidar_data and len(lidar_data) > 0:
            print(f"LIDAR inicializado correctamente con {len(lidar_data)} puntos")
            print(f"Rango de distancias: {min(lidar_data):.2f} - {max(lidar_data):.2f} metros")
            break
        print(f"Intento {attempt + 1}/{max_attempts} de inicializar LIDAR")
        time.sleep(0.5)
    
    # Verificar el entorno antes de comenzar
    print("\nVerificando entorno inicial...")
    sonar_data = robot.get_sonar()
    min_dist = min(sonar_data)
    if min_dist < 0.5:
        print(f"ADVERTENCIA: Obstáculo cercano detectado a {min_dist:.2f} metros")
        # Retroceder un poco si hay obstáculos muy cerca
        robot.set_speed(-0.5, -0.5)
        time.sleep(1)
        robot.set_speed(0, 0)
        time.sleep(0.5)
    
    print("\nComenzando navegación...")
    iteration = 0
    last_speeds = (0, 0)  # Para suavizar los cambios de velocidad
    
    try:
        while coppelia.is_running():
            iteration += 1
            
            try:
                # Obtener datos de los sensores
                lidar_data = robot.get_lidar()
                sonar_data = robot.get_sonar()
                robot_pose = robot.get_position()
                
                # Calcular nuevas velocidades usando la función específica para SLAM
                lspeed, rspeed = avoid.slam_navigation(sonar_data, lidar_data)
                
                # Suavizar cambios de velocidad
                lspeed = 0.7 * last_speeds[0] + 0.3 * lspeed
                rspeed = 0.7 * last_speeds[1] + 0.3 * rspeed
                last_speeds = (lspeed, rspeed)
                
                # Aplicar velocidades
                robot.set_speed(lspeed * speed_factor, rspeed * speed_factor)
                
                # Actualizar mapa
                if lidar_data:
                    slam.update_map(lidar_data, robot_pose)
                    if iteration % 5 == 0:  # Actualizar visualización cada 5 iteraciones
                        slam.display()
                
                time.sleep(0.05)
                
            except Exception as e:
                print(f"Error en iteración {iteration}:")
                print(traceback.format_exc())
                robot.set_speed(0, 0)  # Detener el robot en caso de error
                time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario")
    finally:
        # Detener el robot
        print("Deteniendo el robot...")
        robot.set_speed(0, 0)
        
        # Guardar el mapa final
        try:
            print("Guardando mapa final...")
            slam.display()
            plt.savefig("slam_map.png", dpi=300, bbox_inches='tight')
            print("Mapa guardado como slam_map.png")
        except Exception as e:
            print(f"Error al guardar el mapa: {str(e)}")
        
        print("Deteniendo simulación...")
        coppelia.stop_simulation()
        print("Simulación terminada")

if __name__ == '__main__':
    main() 