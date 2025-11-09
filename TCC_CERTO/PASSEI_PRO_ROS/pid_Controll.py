#!/usr/bin/env python3

import os
import rclpy
import threading
import queue
import time
import math
import pygame
import random
from rclpy.node import Node

# Importa a mensagem customizada do ROS 2
from robo.msg import ESP

# Importa as classes de visão e display
from vision import VisionController
from display import DisplayController

# --- Constantes de Configuração Display ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 360

# --- Constantes de Configuração Neutro da cabeca ---
SERVO_NEUTRAL_X = 110
SERVO_NEUTRAL_Y = 72
SERVO_NEUTRAL_Z = 105
SERVO_OE_NEUTRAL = 90
SERVO_OD_NEUTRAL = 90

SERVO_LIMITS = {
    "x": (100, 120),
    "y": (52, 92),
    "z": (75, 135),
}

# # --- Limites servo x ---
# Servo_X_min = 100
# Servo_X_max = 120

# # --- Limites servo y ---
# Servo_Y_min = 60
# Servo_Y_max = 100

# # --- Limites servo Z ---
# Servo_Z_min = 50
# Servo_Z_max = 150

HAPPY_EAR_AMPLITUDE = 35
HAPPY_EAR_FREQUENCY = 0.5

# SERVO_MAX_ANGLE = 180
# SERVO_MIN_ANGLE = 0


# --- Dicionário de Expressões ---
EXPRESSIONS = {
    "normal": {
        "face_tracking": True,
        "x": "tracking",     # pra cima e pra baixo
        "y": "neutral",     #inclina
        "z": "tracking",    #vira lateralmente
        "oe": 90,
        "od": 90,
    },
    "happy": {
        "face_tracking": True,
        "x": "tracking",
        "y": "neutral",
        "z": "tracking",
        "oe": "wave",
        "od": "wave",
    },
    "bravo": {
        "face_tracking": True,
        "x": SERVO_NEUTRAL_X + 10,
        "y": "neutral",
        "z": "tracking",
        "oe": 135,
        "od": 45,
    },
    "tired": {
        "face_tracking": False,
        "x": SERVO_NEUTRAL_X + 10,
        "y": "random",
        "z": "neutral",
        "oe": 75,
        "od": 105,
    },
    "confuso": {
        "face_tracking": True,
        "x": SERVO_NEUTRAL_X + 5,
        "y": "oscillate",
        "z": "tracking",
        "oe": 100,
        "od": 80,
    },
    "irritado": {
        "face_tracking": True,
        "x": SERVO_NEUTRAL_X + 10,
        "y": "neutral",
        "z": "tracking",
        "oe": 45,
        "od": 135,
    },
    "triste": {
        "face_tracking": True,
        "x": SERVO_NEUTRAL_X + 10,
        "y": SERVO_NEUTRAL_Y + 10,
        "z": "tracking",
        "oe": 110,
        "od": 70,
    },
    "right": {
        "face_tracking": False,
        "x": SERVO_NEUTRAL_X + 20,
        "y": SERVO_NEUTRAL_Y + 10,
        "z": SERVO_NEUTRAL_Z+20,
        "oe": 110,
        "od": 70,
    },
    "left": {
        "face_tracking": False,
        "x": SERVO_NEUTRAL_X + 20,
        "y": SERVO_NEUTRAL_Y + 10,
        "z": SERVO_NEUTRAL_Z-20,
        "oe": 110,
        "od": 70,
    },
}

class ESPPublisher(Node):
    def __init__(self):
        super().__init__('esp_publisher')
        self.get_logger().info('Nó de Controle por Visão iniciado.')
        
        # Definicao das mensagens e topicos no ROS2
        self.publisher_ = self.create_publisher(ESP, '/esp_control', 1)
        timer_period = 0.1 # 10Hz    
        self.timer = self.create_timer(timer_period, self.timer_callback) #Roda a função timer_callback a cada 0.02 segundos
        self.data_queue = queue.Queue(maxsize=2) #limita os frames disponíveis para análise
        
        # Inicia o Pygame
        # Cria a classe que desenha os rostos no display, pelo frame do pygame
        pygame.init()
        info = pygame.display.Info()
        width, height = info.current_w, info.current_h
        screen = pygame.display.set_mode((width, height), pygame.FULLSCREEN)
        self.clock = pygame.time.Clock()
        self.display_drawer = DisplayController(screen)   
        self.vision_worker = VisionController(self.data_queue) # Cria a classe que trata as imagens do data_queue 
        
        # Sistema para proteção das variaveis do timer
        self.lock = threading.Lock()
        # Variáveis com a mensagem para a ESP
        self.linear_y  =  0.0
        self.angular_z =  0.0
        self.servo_x   =  SERVO_NEUTRAL_X
        self.servo_y   =  SERVO_NEUTRAL_Y
        self.servo_z   =  SERVO_NEUTRAL_Z
        self.servo_oe  =  SERVO_OE_NEUTRAL
        self.servo_od  =  SERVO_OD_NEUTRAL
        
        # --- Definicao dos targets de posicao dos servos ---
        self.target_linear_y  = 0.0
        self.target_angular_z = 0.0
        self.target_servo_x   = SERVO_NEUTRAL_X
        self.target_servo_y   = SERVO_NEUTRAL_Y
        self.target_servo_z   = SERVO_NEUTRAL_Z
        self.target_servo_oe  = SERVO_OE_NEUTRAL
        self.target_servo_od  = SERVO_OD_NEUTRAL

        # Controle de expressões
        self.current_gesture = "normal"
        self.running = True

        self.happy_start_time = None
        self.expression_start_time = time.time()
        self.face_tracking_enabled = True
        
    # Funcao que envia periodicamente o comando para a Jetson
    def timer_callback(self):
        msg = ESP()
        with self.lock:
            self.servo_x  = self.smooth_move(self.servo_x,  self.target_servo_x, 0.05)
            self.servo_y  = self.smooth_move(self.servo_y,  self.target_servo_y, 0.05)
            self.servo_z  = self.smooth_move(self.servo_z,  self.target_servo_z, 0.1)
            self.servo_oe = self.smooth_move(self.servo_oe, self.target_servo_oe, 0.1)
            self.servo_od = self.smooth_move(self.servo_od, self.target_servo_od, 0.1)
            msg.linear_y  = float(self.linear_y)
            msg.angular_z = float(self.angular_z)
            msg.servo_x   = int(self.servo_x)
            msg.servo_y   = int(self.servo_y)
            msg.servo_z   = int(self.servo_z)
            msg.servo_oe  = int(self.servo_oe)
            msg.servo_od  = int(self.servo_od)
        self.publisher_.publish(msg)       
        
    def start_workers(self):
        self.vision_worker.start()
        self.get_logger().info('Thread de visão iniciada.')

    def stop_workers(self):
        if self.vision_worker:
            self.vision_worker.stop()     
        self.get_logger().info('Thread de visão finalizada.')
        
    def _map_value(self, value, from_min, from_max, to_min, to_max):
        if from_max == from_min: return to_min
        mapped = (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
        return max(to_min, min(to_max, mapped))
    
    def smooth_move(self, current, target, alpha=0.2):
        return current + (target - current) * alpha
    
    # --- Emoções e expressões ---
    def update_expression(self, gesture):
        """Atualiza expressão atual se mudou."""
        if gesture != self.current_gesture:
            self.current_gesture = gesture
            self.expression_start_time = time.time()
            self.happy_start_time = self.expression_start_time
            self.apply_expression(gesture)

    def apply_expression(self, gesture):
        """Aplica a emoção definida no dicionário EXPRESSIONS."""
        expr = EXPRESSIONS.get(gesture, EXPRESSIONS["normal"])
        self.face_tracking_enabled = expr["face_tracking"]

        t = time.time() - self.expression_start_time

        # Cabeça (x, y, z)
        if expr["x"] == "tracking":
            pass
        elif expr["x"] == "neutral":
            self.target_servo_x = self.smooth_move(self.target_servo_x, SERVO_NEUTRAL_X)
        elif expr["x"] == "oscillate":
            self.target_servo_x = SERVO_NEUTRAL_X + 5 * math.sin(2 * math.pi * 0.5 * t)
        elif isinstance(expr["x"], (int, float)):
            self.target_servo_x = self.smooth_move(self.target_servo_x, expr["x"])

        if expr["y"] == "neutral":
            self.target_servo_y = self.smooth_move(self.target_servo_y, SERVO_NEUTRAL_Y)
        elif expr["y"] == "oscillate":
            self.target_servo_y = SERVO_NEUTRAL_Y + 10 * math.sin(2 * math.pi * 0.5 * t)
        elif expr["y"] == "random":
            self.target_servo_y = SERVO_NEUTRAL_Y + random.choice([-10, 10])
        elif isinstance(expr["y"], (int, float)):
            self.target_servo_y = self.smooth_move(self.target_servo_y, expr["y"])

        if expr["z"] == "tracking":
            pass  # será atualizado pelo rastreamento facial
        elif expr["z"] == "neutral":
            self.target_servo_z = self.smooth_move(self.target_servo_z, SERVO_NEUTRAL_Z)
        elif isinstance(expr["z"], (int, float)):
            self.target_servo_z = self.smooth_move(self.target_servo_z, expr["z"])

        # Orelhas (oe/od)
        if expr["oe"] == "wave":
            self.target_servo_oe = SERVO_OE_NEUTRAL + 45 + HAPPY_EAR_AMPLITUDE * math.sin(2 * math.pi * t * HAPPY_EAR_FREQUENCY)
        elif isinstance(expr["oe"], (int, float)):
            self.target_servo_oe = self.smooth_move(self.target_servo_oe, expr["oe"])

        if expr["od"] == "wave":
            self.target_servo_od = SERVO_OD_NEUTRAL - 45 - HAPPY_EAR_AMPLITUDE * math.sin(2 * math.pi * t * HAPPY_EAR_FREQUENCY)
        elif isinstance(expr["od"], (int, float)):
            self.target_servo_od = self.smooth_move(self.target_servo_od, expr["od"])

    # ----------------- IDLE -----------------
    def _idle_behavior(self):
        # mais chances de olhar pros lados
        if self.display_drawer.idle_done:
            choice = random.choices(
                ["right", "left", "happy", "tired"],
                weights=[0.3, 0.3, 0.2, 0.2],
                k=1
            )[0]
            self.update_expression(choice)
            # print(choice)
            self.display_drawer.draw_single_frame(choice, is_idle=True)

        else:
        # ainda está rodando o idle atual, continua executando
            self.display_drawer.draw_single_frame(None, is_idle=True)

        
    def _process_vision_data(self, data):
        gesture = data.get("gesture")
        face_coords = data.get("face", None)

        if not face_coords and not self.display_drawer.transition_state:
            self._idle_behavior()

        else:
            # Atualiza expressão atual
            self.display_drawer.idle_done = True
            self.display_drawer.draw_single_frame(gesture, is_idle=False)

            self.update_expression(gesture)

            face_x, face_y, face_w, face_h = face_coords
            face_center_x = face_x + face_w // 2
            face_center_y = face_y + face_h // 2

            # Segue horizontalmente com o servo_z (lateral)
            self.target_servo_z = self._map_value(face_center_x, 0, FRAME_WIDTH, *SERVO_LIMITS["z"])

            # Segue verticalmente com o servo_x (cima e baixo)
            self.target_servo_x = self._map_value(face_center_y, 0, FRAME_HEIGHT, *SERVO_LIMITS["x"])   

    def run(self):
        self.start_workers()
        
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT or \
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                        rclpy.shutdown() # Sai do while
                        break

            try:
                vision_data = self.data_queue.get_nowait()
                self._process_vision_data(vision_data)

            except queue.Empty:
                pass     
            
            pygame.display.flip()
            self.clock.tick(24) # limita o fps do pygame                   
            rclpy.spin_once(self, timeout_sec=0.01)
            
        self.destroy_node()
        
def main(args=None):
    # Espera até existir DISPLAY
    use_display = os.environ.get('DISPLAY')
    while not use_display:
        print("Nenhum display encontrado, esperando...")
        time.sleep(1)  # espera 1 segundo
        use_display = os.environ.get('DISPLAY')
    print(f"Display encontrado: {use_display}, inicializando nó...")

    # Se o display existe, inicializa ROS2
    rclpy.init(args=args)
    node = ESPPublisher()

    try:
        node.run()  # roda o loop principal do nó (com pygame)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando com Ctrl+C")
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
