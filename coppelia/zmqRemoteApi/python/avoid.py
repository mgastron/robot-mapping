'''
avoid.py

Implementation of wall-following algorithm for the Pioneer P3DX robot.
'''

import robotica
import math

def wall_following(readings):
    """
    Implementa un algoritmo de wall-following usando los sensores ultrasónicos.
    """
    # Constantes
    DESIRED_WALL_DIST = 0.6    # Distancia deseada a la pared
    BASE_SPEED = 1.0           # Velocidad base
    DANGER_DIST = 0.4          # Distancia de peligro
    CORNER_DIST = 0.5          # Distancia para detectar esquinas
    
    # Obtener lecturas relevantes
    front = min(readings[7], readings[8])  # Sensores frontales centrales
    right_front = readings[3]              # Sensor diagonal derecho frontal
    right_side = readings[4]               # Sensor lateral derecho
    left_front = readings[12]              # Sensor diagonal izquierdo frontal
    
    # Detectar situaciones especiales
    front_blocked = front < DANGER_DIST
    corner_detected = right_front < CORNER_DIST and right_side < CORNER_DIST
    too_close = right_side < DANGER_DIST
    no_wall = right_side > 1.0
    
    # Manejar cada situación
    if front_blocked:
        # Giro pronunciado a la izquierda cuando hay obstáculo al frente
        return -0.5, 1.0
        
    elif corner_detected:
        # Giro amplio a la izquierda en las esquinas
        return 1.0, -0.2
        
    elif too_close:
        # Alejarse de la pared cuando está muy cerca
        return 1.0, 0.2
        
    elif no_wall:
        # Buscar la pared girando a la derecha
        return 1.0, 0.5
        
    else:
        # Seguimiento normal de pared
        error = DESIRED_WALL_DIST - right_side
        
        # Ajuste suave proporcional al error
        adjustment = 0.7 * error
        
        # Limitar el ajuste para movimientos más suaves
        adjustment = max(-0.4, min(0.4, adjustment))
        
        # Velocidades base diferentes para mejor control
        lspeed = BASE_SPEED + adjustment
        rspeed = BASE_SPEED - adjustment
        
        # Limitar velocidades finales
        lspeed = max(0.0, min(1.5, lspeed))
        rspeed = max(0.0, min(1.5, rspeed))
        
        return lspeed, rspeed

# Función por defecto para compatibilidad
avoid = wall_following

def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    coppelia.start_simulation()
    while coppelia.is_running():
        readings = robot.get_sonar()
        lspeed, rspeed = wall_following(readings)
        robot.set_speed(lspeed, rspeed)
    coppelia.stop_simulation()

if __name__ == '__main__':
    main()
