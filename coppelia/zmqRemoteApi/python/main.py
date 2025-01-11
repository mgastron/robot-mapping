'''
main.py

Main program for robot mapping implementation using OccupancyMap visualization.
'''

import robotica
from mapping import OccupancyMap
from avoid import avoid  # Importamos la función de navegación de avoid.py

def main(args=None):
    # Inicializar la conexión con Coppelia
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    
    # Inicializar el mapa
    occupancy_map = OccupancyMap(map_size=10, resolution=0.1)
    
    # Factor de velocidad para hacer el robot más rápido
    speed_factor = 2.0  # Puedes ajustar este valor para hacer el robot más rápido o más lento
    
    # Iniciar la simulación
    coppelia.start_simulation()
    
    try:
        # Crear el mapa mientras la simulación está corriendo
        while coppelia.is_running():
            # Obtener lecturas del sonar
            readings = robot.get_sonar()
            
            # Obtener la posición actual del robot
            robot_pos = robot.get_position()
            
            # Actualizar y mostrar el mapa
            occupancy_map.update(robot_pos)
            occupancy_map.display()
            
            # Mover el robot usando el algoritmo de avoid.py con velocidad aumentada
            lspeed, rspeed = avoid(readings)
            robot.set_speed(lspeed * speed_factor, rspeed * speed_factor)
        
    except KeyboardInterrupt:
        print("\nMapping interrupted by user")
    finally:
        # Guardar el mapa antes de terminar
        occupancy_map.save_to_file("robot_map")
        coppelia.stop_simulation()
        print("Mapping completed. Map saved as robot_map.png")

if __name__ == '__main__':
    main()