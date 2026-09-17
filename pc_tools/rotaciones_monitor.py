#!/usr/bin/env python3
"""
Monitor para el potenciometro

- Muestra la posición actual (ticks) y velocidad (RPM) en la terminal.
- Grafica en vivo el historial de posición.

Uso:
    source /opt/ros/jazzy/setup.bash
    python3 potenciometro_monitor.py
"""
import csv
import math
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Import para ROS2
import rclpy
from rclpy.node import Node

# TODO: definir tipo de mensaje usado por los topicos
from std_msgs.msg import 

class RotacionesMonitor(Node):
    def __init__(self):
        super().__init__('rotaciones_monitor')

        # Publicadores de topicos /vector y /euler
        self.create_publisher(, '/vector', self., 10)
        self.create_publisher(, '/euler', self., 10)

        # Suscripcion al topico /angulo_rotado publicado por el nodo de microros
        self.create_subscription(, 'vector_rotado')
        self.create_subscription(, 'vector_microrotado')



    # Funcion de callback para el topico /vector_rotado
    def vector_rotado_cb(self, msg):
        pass

    # Funcion de callback para el topico /vector_microrotado
    def vector_microrotado_cb(self, msg):
        pass

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = RotacionesMonitor()

    # TODO: Recepcion de los datos para publicarlos en los topicos /vector y /euler

    # Configuración de la gráfica
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))

    # --- Grafico 1 : Vector sin rotar ---
    ax1.set_title("Vector sin rotar")

    # --- Subplot 2: Voltaje Rotado usando Matrices ---
    ax2.set_title("Vector Rotado (Matrices)")

    # --- Subplot 3: Voltaje Rotado usando Micro Rotaciones ---
    ax3.set_title("Vector Rotado (Micro Rotaciones)")

    def update_plot(_frame):
        rclpy.spin_once(node, timeout_sec=0.01)

        # Actualizar los datos de los gráficos con los valores recibidos de los topicos
        pass

    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    
if __name__ == '__main__':
    main()