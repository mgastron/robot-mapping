'''
robotica.py

Provides the communication between CoppeliaSim robotics simulator and
external Python applications via the ZeroMQ remote API.

Copyright (C) 2024 Javier de Lope

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np
import cv2
import time
import traceback

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class Coppelia():

    def __init__(self):
        print('*** connecting to coppeliasim')
        client = RemoteAPIClient()
        self.sim = client.getObject('sim')

    def start_simulation(self):
        # print('*** saving environment')
        self.default_idle_fps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.sim.startSimulation()

    def stop_simulation(self):
        # print('*** stopping simulation')
        self.sim.stopSimulation()
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            time.sleep(0.1)
        # print('*** restoring environment')
        self.sim.setInt32Param(self.sim.intparam_idle_fps, self.default_idle_fps)
        print('*** done')

    def is_running(self):
        return self.sim.getSimulationState() != self.sim.simulation_stopped


class P3DX():

    num_sonar = 16
    sonar_max = 1.0

    def __init__(self, sim, robot_id, use_camera=False, use_lidar=False):
        self.sim = sim
        print('*** getting handles', robot_id)
        print(f'\nBuscando sensores para el robot {robot_id}...')
        
        # Obtener handle principal del robot
        self.handle = self.sim.getObject(f'/{robot_id}')
        print(f'Handle principal del robot: {self.handle}')
        
        # Motores
        self.left_motor = self.sim.getObject(f'/{robot_id}/leftMotor')
        self.right_motor = self.sim.getObject(f'/{robot_id}/rightMotor')
        print(f'Motores encontrados: izquierdo={self.left_motor}, derecho={self.right_motor}')
        
        # Sensores ultrasónicos
        print('\nBuscando sensores ultrasónicos...')
        self.sonar = []
        for i in range(self.num_sonar):
            try:
                sensor = self.sim.getObject(f'/{robot_id}/ultrasonicSensor[{i}]')
                self.sonar.append(sensor)
                print(f'  Sonar {i}: handle={sensor}')
            except Exception as e:
                print(f'  Error al obtener sonar {i}: {str(e)}')
        
        # Cámara (opcional)
        if use_camera:
            try:
                self.camera = self.sim.getObject(f'/{robot_id}/camera')
                print(f'\nCámara encontrada: handle={self.camera}')
            except Exception as e:
                print(f'\nError al obtener la cámara: {str(e)}')
        
        # LIDAR (opcional)
        if use_lidar:
            try:
                # Intentar diferentes nombres comunes para el LIDAR
                lidar_names = ['/lidar', '/laser', '/hokuyo', '/sick', '/rplidar']
                for name in lidar_names:
                    try:
                        self.lidar = self.sim.getObject(f'/{robot_id}{name}')
                        print(f'\nLIDAR encontrado como "{name}": handle={self.lidar}')
                        break
                    except:
                        continue
                
                if not hasattr(self, 'lidar'):
                    print('\nNo se encontró el LIDAR con ningún nombre común')
                    # Intentar buscar cualquier objeto hijo que podría ser un LIDAR
                    children = self.sim.getObjectsInTree(self.handle)
                    print(f'Objetos encontrados en el robot:')
                    for child in children:
                        name = self.sim.getObjectAlias(child)
                        type_name = self.sim.getObjectType(child)
                        print(f'  - {name} (tipo: {type_name}, handle: {child})')
            except Exception as e:
                print(f'\nError al buscar el LIDAR: {str(e)}')

    def get_sonar(self):
        readings = []
        for i in range(self.num_sonar):
            res,dist,_,_,_ = self.sim.readProximitySensor(self.sonar[i])
            readings.append(dist if res == 1 else self.sonar_max)
        return readings

    def get_image(self):
        img, resX, resY = self.sim.getVisionSensorCharImage(self.camera)
        img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
        img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
        return img

    def get_lidar(self):
        """
        Obtiene los datos del sensor LIDAR y los combina con los sensores ultrasónicos.
        Retorna: Lista de distancias combinadas
        """
        try:
            # Obtener datos del sensor de proximidad principal
            result, distance, point, _, _ = self.sim.handleProximitySensor(self.lidar)
            
            # Obtener lecturas de los sensores ultrasónicos
            sonar_readings = self.get_sonar()
            
            # Crear array de lecturas combinadas
            num_points = 180  # Un punto por grado
            scan_data = []
            
            # Obtener orientación del robot
            _, _, theta = self.get_position()
            
            # Convertir lecturas del sonar a formato similar al LIDAR
            sonar_angles = np.linspace(-np.pi/2, np.pi/2, self.num_sonar)
            
            for i in range(num_points):
                angle = -np.pi/2 + (i * np.pi/num_points)
                
                # Encontrar la lectura del sonar más cercana a este ángulo
                sonar_idx = np.argmin(np.abs(sonar_angles - angle))
                sonar_dist = sonar_readings[sonar_idx]
                
                # Si el LIDAR detectó algo, usar esa lectura
                if result == 1 and abs(angle) < np.pi/4:  # LIDAR frontal
                    adjusted_distance = distance * np.cos(angle)
                    if adjusted_distance > 0.1:
                        scan_data.append(min(adjusted_distance, sonar_dist))
                else:
                    # Usar lectura del sonar
                    scan_data.append(sonar_dist)
            
            return scan_data
            
        except Exception as e:
            print(f"Error al leer sensores: {str(e)}")
            print(traceback.format_exc())
            return None

    def set_speed(self, left_speed, right_speed):
        self.sim.setJointTargetVelocity(self.left_motor, left_speed)
        self.sim.setJointTargetVelocity(self.right_motor, right_speed)

    def get_position(self):
        """
        Obtiene la posición y orientación actual del robot.
        Retorna: tupla (x, y, theta) con la posición y orientación
        """
        # Obtener la posición y orientación del robot desde CoppeliaSim
        position = self.sim.getObjectPosition(self.handle, -1)  # -1 significa coordenadas absolutas
        orientation = self.sim.getObjectOrientation(self.handle, -1)
        
        # Extraer las coordenadas x, y y el ángulo theta (orientación en z)
        x = position[0]
        y = position[1]
        theta = orientation[2]  # Usamos el ángulo en z como orientación
        
        return (x, y, theta)


def main(args=None):
    coppelia = Coppelia()
    robot = P3DX(coppelia.sim, 'PioneerP3DX')
    robot.set_speed(+1.2, -1.2)
    coppelia.start_simulation()
    while (t := coppelia.sim.getSimulationTime()) < 3:
        print(f'Simulation time: {t:.3f} [s]')
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()
