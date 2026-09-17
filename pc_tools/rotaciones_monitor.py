#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import customtkinter as ctk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class RotacionesNode(Node):
    def __init__(self):
        super().__init__('rotaciones_monitor')
        
        # Publicadores
        self.pub_vector = self.create_publisher(Vector3, '/vector', 10)
        self.pub_euler = self.create_publisher(Vector3, '/euler', 10)
        
        # Suscriptores
        self.sub_rot = self.create_subscription(Vector3, '/vector_rotado', self.cb_rotado, 10)
        self.sub_micro = self.create_subscription(Vector3, '/vector_microrotado', self.cb_micro, 10)
        
        # Variables de estado de lectura (lo que llega del ESP32)
        self.v_original = [1.0, 0.0, 0.0]
        self.v_rotado = [1.0, 0.0, 0.0]
        self.v_microrotado = [1.0, 0.0, 0.0]
        self.data_updated = True  

        # Variables de estado objetivo (lo que el usuario ingresa en la UI)
        self.target_v = [1.0, 0.0, 0.0]
        self.target_e = [0.0, 0.0, 0.0]

        # Timer para publicar continuamente el objetivo a 2 Hz (cada 0.5 segundos)
        self.timer_publicacion = self.create_timer(0.5, self.publicar_datos)

    def cb_rotado(self, msg):
        self.v_rotado = [msg.x, msg.y, msg.z]
        self.data_updated = True

    def cb_micro(self, msg):
        self.v_microrotado = [msg.x, msg.y, msg.z]
        self.data_updated = True
        
    def set_target(self, v, e):
        # Actualizamos los objetivos cuando el usuario hace clic en el botón
        self.target_v = v
        self.target_e = e
        self.get_logger().info(f'Nuevos objetivos fijados -> Vector: {v} | Euler: {e}')

    def publicar_datos(self):
        # Esta función se ejecuta sola cada 0.5 segundos mandando el último estado deseado
        self.v_original = self.target_v
        
        msg_v = Vector3(x=float(self.target_v[0]), y=float(self.target_v[1]), z=float(self.target_v[2]))
        msg_e = Vector3(x=float(self.target_e[0]), y=float(self.target_e[1]), z=float(self.target_e[2]))
        
        self.pub_vector.publish(msg_v)
        
        # Le damos 50ms al ESP32 para que procese el vector antes de mandarle el euler
        time.sleep(0.05)
        self.pub_euler.publish(msg_e)


class RotacionesUI(ctk.CTk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        self.title("Monitor de Rotaciones ROS 2")
        self.geometry("1200x550") 
        ctk.set_appearance_mode("dark")
        
        # Layout principal
        self.frame_controles = ctk.CTkFrame(self, width=250)
        self.frame_controles.pack(side="left", fill="y", padx=10, pady=10)
        
        self.frame_graficos = ctk.CTkFrame(self)
        self.frame_graficos.pack(side="right", fill="both", expand=True, padx=10, pady=10)
        
        # Controles: Vector Original
        ctk.CTkLabel(self.frame_controles, text="Vector Original (X, Y, Z):", font=("Arial", 14, "bold")).pack(pady=(15, 5))
        self.ent_vx = ctk.CTkEntry(self.frame_controles, placeholder_text="X"); self.ent_vx.pack(pady=5)
        self.ent_vy = ctk.CTkEntry(self.frame_controles, placeholder_text="Y"); self.ent_vy.pack(pady=5)
        self.ent_vz = ctk.CTkEntry(self.frame_controles, placeholder_text="Z"); self.ent_vz.pack(pady=5)
        
        self.ent_vx.insert(0, "1.0"); self.ent_vy.insert(0, "0.0"); self.ent_vz.insert(0, "0.0")

        # Controles: Ángulos de Euler
        ctk.CTkLabel(self.frame_controles, text="Rotación (Roll, Pitch, Yaw) [rad]:", font=("Arial", 14, "bold")).pack(pady=(25, 5))
        self.ent_roll = ctk.CTkEntry(self.frame_controles, placeholder_text="Roll (X)"); self.ent_roll.pack(pady=5)
        self.ent_pitch = ctk.CTkEntry(self.frame_controles, placeholder_text="Pitch (Y)"); self.ent_pitch.pack(pady=5)
        self.ent_yaw = ctk.CTkEntry(self.frame_controles, placeholder_text="Yaw (Z)"); self.ent_yaw.pack(pady=5)

        self.ent_roll.insert(0, "0.0"); self.ent_pitch.insert(0, "0.0"); self.ent_yaw.insert(0, "0.0")

        # Botón para publicar
        self.btn_enviar = ctk.CTkButton(self.frame_controles, text="Actualizar Rotación", command=self.enviar_ros)
        self.btn_enviar.pack(pady=30)

        # Configuración de Matplotlib
        self.fig = plt.figure(figsize=(10, 4.5), facecolor='#2b2b2b')
        self.fig.subplots_adjust(bottom=0.25, wspace=0.25)
        
        self.ax1 = self.fig.add_subplot(131, projection='3d')
        self.ax2 = self.fig.add_subplot(132, projection='3d')
        self.ax3 = self.fig.add_subplot(133, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_graficos)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        self.actualizar_ros_y_graficos()
        
    def enviar_ros(self):
        try:
            v = [float(self.ent_vx.get()), float(self.ent_vy.get()), float(self.ent_vz.get())]
            e = [float(self.ent_roll.get()), float(self.ent_pitch.get()), float(self.ent_yaw.get())]
            # Ahora el botón solo actualiza las variables internas, el timer se encarga de enviarlo
            self.ros_node.set_target(v, e)
        except ValueError:
            print("Error: Por favor ingresa solo valores numéricos en los campos.")

    def draw_vector(self, ax, title, vec, color):
        ax.clear()
        ax.set_facecolor('#2b2b2b')
        ax.set_title(title, color='white', pad=10)
        
        max_val = max(abs(vec[0]), abs(vec[1]), abs(vec[2]))
        lim = max(1.0, max_val * 1.2)
        
        ax.set_xlim([-lim, lim]); ax.set_ylim([-lim, lim]); ax.set_zlim([-lim, lim])
        ax.set_xlabel('X', color='white'); ax.set_ylabel('Y', color='white'); ax.set_zlabel('Z', color='white')
        ax.tick_params(colors='white')
        
        ax.xaxis.set_pane_color((0.2, 0.2, 0.2, 1.0))
        ax.yaxis.set_pane_color((0.2, 0.2, 0.2, 1.0))
        ax.zaxis.set_pane_color((0.2, 0.2, 0.2, 1.0))

        ax.quiver(0, 0, 0, vec[0], vec[1], vec[2], color=color, arrow_length_ratio=0.1, linewidth=2.5)

        texto_coords = f"X: {vec[0]:.3f} | Y: {vec[1]:.3f} | Z: {vec[2]:.3f}"
        ax.text2D(0.5, -0.15, texto_coords, transform=ax.transAxes, ha='center', color='white', fontsize=11, fontweight='bold')

    def actualizar_ros_y_graficos(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.01)
        
        if self.ros_node.data_updated:
            self.draw_vector(self.ax1, "Vector sin rotar", self.ros_node.v_original, '#4287f5')
            self.draw_vector(self.ax2, "Vector Rotado", self.ros_node.v_rotado, '#42f554')
            self.draw_vector(self.ax3, "Micro Rotaciones", self.ros_node.v_microrotado, '#f54242')
            self.canvas.draw()
            self.ros_node.data_updated = False
            
        self.after(50, self.actualizar_ros_y_graficos)


def main():
    rclpy.init()
    node = RotacionesNode()
    
    app = RotacionesUI(node)
    app.mainloop()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()