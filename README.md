# Robot Pioneer P3DX - Sistema de Navegación

Sistema de navegación autónoma para el robot Pioneer P3DX implementado en CoppeliaSim, utilizando Python y la API remota ZMQ.

## Descripción
Este proyecto implementa un sistema de navegación autónoma basado en wall-following para el robot Pioneer P3DX. El robot utiliza sensores ultrasónicos para detectar y evitar obstáculos mientras navega por el entorno.

## Características Principales
- Navegación autónoma usando wall-following
- Detección y evasión de obstáculos
- Manejo inteligente de esquinas
- Control suave de velocidad
- Integración con CoppeliaSim

## Requisitos Previos
- Python 3.x
- CoppeliaSim EDU/PRO
- Bibliotecas Python (ver requirements.txt)

## Instalación

1. Clona este repositorio:
git clone https://github.com/mgastron/robot-mapping.git

2. Instala las dependencias:
pip install -r requirements.txt

3. Configura CoppeliaSim:
    - Instala CoppeliaSim
   - Asegúrate de que el puerto ZMQ esté configurado correctamente
   - Carga la escena del Pioneer P3DX incluida en la carpeta `scenes`

## Estructura del Proyecto
robot-mapping/
├── coppelia/
│ └── zmqRemoteApi/
│ └── python/
│ ├── avoid.py # Algoritmo de navegación
│ ├── robotica.py # Interfaz con CoppeliaSim
│ ├── main.py # Programa principal
│ └── mapping.py # Funciones de mapeo
├── scenes/ # Escenas de CoppeliaSim
├── README.md
└── requirements.txt


## Uso

1. Abre CoppeliaSim y carga una escena del Pioneer P3DX
2. Ejecuta el programa principal:
python main.py


## Funcionamiento
El robot utiliza sus sensores ultrasónicos para:
- Detectar obstáculos en su entorno
- Mantener una distancia segura de las paredes
- Navegar evitando colisiones
- Manejar situaciones especiales como esquinas

## Contribuciones
Las contribuciones son bienvenidas. Por favor, abre un issue primero para discutir los cambios que te gustaría hacer.

## Licencia
Este proyecto está bajo la Licencia MIT. Ver el archivo LICENSE para más detalles.

## Autor
Matías Gasstron
Andrea Orama
Omar Teixeira
