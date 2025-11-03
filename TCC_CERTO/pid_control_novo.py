#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import threading
import queue
import time
import math
import pygame
import random

from robo.msg import ESP
from vision import VisionController
from display import DisplayController

# --- Constantes de Configuração ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 360

SERVO_NEUTRAL_X = 90   # Cima/Baixo
SERVO_NEUTRAL_Y = 84   # Inclinação
SERVO_NEUTRAL_Z = 90   # Lateral
SERVO_OE_NEUTRAL = 90  # Orelha Esquerda
SERVO_OD_NEUTRAL = 90  # Orelha Direita

HAPPY_EAR_AMPLITUDE = 35
HAPPY_EAR_FREQUENCY = 0.5

SERVO_LIMITS = {
    "x": (70, 110),
    "y": (54, 114),
    "z": (55, 115),
}

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
    "feliz": {
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
    "saco_cheio": {
        "face_tracking": True,
        "x": SERVO_NEUTRAL_X + 10,
        "y": "random",
        "z": "tracking",
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
        "face_tracking": False,
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
        "z": SERVO_NEUTRAL_Z-20,
        "oe": 110,
        "od": 70,
    },
    "left": {
        "face_tracking": False,
        "x": SERVO_NEUTRAL_X + 20,
        "y": SERVO_NEUTRAL_Y + 10,
        "z": SERVO_NEUTRAL_Z+20,
        "oe": 110,
        "od": 70,
    },
}

class ESPPublisher(Node):
    def __init__(self, screen):
        super().__init__('esp_publisher')
        self.get_logger().info('Nó de Controle por Visão iniciado.')

        self.publisher_ = self.create_publisher(ESP, '/esp_control', 10)
        self.data_queue = queue.Queue(maxsize=2)

        self.display_drawer = DisplayController(screen, self.data_queue) if screen else None
        self.vision_worker = VisionController(self.data_queue)

        # Estado dos servos
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.target_servo_x = float(SERVO_NEUTRAL_X)
        self.target_servo_y = float(SERVO_NEUTRAL_Y)
        self.target_servo_z = float(SERVO_NEUTRAL_Z)
        self.target_servo_oe = 90.0
        self.target_servo_od = 90.0

        # Controle de expressões
        self.current_gesture = "normal"
        self.happy_start_time = None
        self.expression_start_time = time.time()
        self.face_tracking_enabled = True

        # Controle de idle
        self.idle_timer = 0
        self.idle_side = True
        self.idle_ear_amp = 5

        # Controle de distância e calibragem
        self.ultima_distancia = None
        self.K_CALIBRACAO = 95.0

    # --- Funções utilitárias ---
    def smooth_move(self, current, target, alpha=0.2):
        return current + (target - current) * alpha

    def _map_value(self, value, from_min, from_max, to_min, to_max):
        if from_max == from_min:
            return to_min
        mapped = (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
        return max(to_min, min(to_max, mapped))

    # --- Controle da visão ---
    def start_workers(self):
        self.vision_worker.start()
        self.get_logger().info('Thread de visão iniciada.')

    def stop_workers(self):
        if self.vision_worker:
            self.vision_worker.stop()
        self.get_logger().info('Thread de visão finalizada.')

    # --- Emoções e expressões ---
    def update_expression(self, gesture):
        """Atualiza expressão atual se mudou."""
        if gesture != self.current_gesture:
            self.current_gesture = gesture
            self.expression_start_time = time.time()
            self.happy_start_time = self.expression_start_time
            self.apply_expression(gesture)

          # Evolução de bravo → irritado
        elif self.current_gesture == "bravo":
            elapsed = time.time() - self.expression_start_time
            if elapsed >= 5:
                self.get_logger().info("Bravo por muito tempo → evoluindo para irritado...")
                self.current_gesture = "irritado"
                self.expression_start_time = time.time()
                self.apply_expression("irritado")

    def apply_expression(self, gesture):
        """Aplica a emoção definida no dicionário EXPRESSIONS."""
        expr = EXPRESSIONS.get(gesture, EXPRESSIONS["normal"])
        self.face_tracking_enabled = expr["face_tracking"]

        t = time.time() - self.expression_start_time

        # Cabeça (x, y, z)
        if expr["x"] == "tracking":
            pass
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


    # --- Ações de inatividade ---
    # def perform_idle_action(self):
    #     t = time.time()
    #     if t >= self.idle_timer:
    #         action = random.choice(["cabeca_inclinado", "cabeca_frente_tras", "orelhas_senoide"])
    #         if action == "cabeca_inclinado":
    #             offset = 10 if self.idle_side else -10
    #             self.target_servo_y = SERVO_NEUTRAL_Y + offset
    #             self.idle_side = not self.idle_side
    #         elif action == "cabeca_frente_tras":
    #             self.target_servo_x = SERVO_NEUTRAL_X + random.choice([-10, 10])
    #         elif action == "orelhas_senoide":
    #             amp = self.idle_ear_amp * math.sin(2 * math.pi * 0.5 * t)
    #             self.target_servo_oe = SERVO_OE_NEUTRAL + amp
    #             self.target_servo_od = SERVO_OD_NEUTRAL - amp
    #         self.idle_timer = t + random.uniform(2, 5)

    # --- Processamento de dados de visão ---
    def _process_vision_data(self, data):
        face_coords = data.get('face')
        gesture = data.get('gesture', 'normal')

        # Atualiza expressão atual
        self.update_expression(gesture)

        if face_coords and self.face_tracking_enabled:
            face_x, face_y, face_w, face_h = face_coords
            face_center_x = face_x + face_w // 2
            face_center_y = face_y + face_h // 2

            # Rastreia o rosto
            self.target_servo_z = self._map_value(face_center_x, 0, FRAME_WIDTH, *SERVO_LIMITS["z"])
            self.target_servo_x = self._map_value(face_center_y, 0, FRAME_HEIGHT, *SERVO_LIMITS["x"])

            distancia_atual = self.K_CALIBRACAO / (face_w + 2)
            if self.ultima_distancia is None:
                self.ultima_distancia = distancia_atual

            if distancia_atual > 1.0:
                if distancia_atual > self.ultima_distancia + 0.1:
                    self.target_linear_y = 0.2
                else:
                    self.target_linear_y = 0.0
            else:
                self.target_linear_y = 0.0

            self.ultima_distancia = distancia_atual
        else:
            self.target_linear_y = 0.0

        # Atualiza comportamento da emoção ativa
        # self.apply_expression(gesture)

# --- Função de inicialização ---
def has_display():
    return os.environ.get('DISPLAY')

def main(args=None):
    use_display = has_display()
    pygame.init()
    info = pygame.display.Info()
    width, height = info.current_w, info.current_h
    screen = pygame.display.set_mode((width, height), pygame.FULLSCREEN)
    clock = pygame.time.Clock()

    # Inicialização ROS
    rclpy.init(args=args)
    node = ESPPublisher(screen)
    node.start_workers()
    running = True

    try:
        while running:
            if use_display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                        running = False

            rclpy.spin_once(node, timeout_sec=0)
            vision_data = None
            try:
                vision_data = node.data_queue.get_nowait()
                node._process_vision_data(vision_data)
            except queue.Empty:
                pass

            msg = ESP()
            msg.linear_y = float(node.target_linear_y)
            msg.angular_z = float(node.target_angular_z)
            msg.servo_x = int(node.target_servo_x)
            msg.servo_y = int(node.target_servo_y)
            msg.servo_z = int(node.target_servo_z)
            msg.servo_oe = int(node.target_servo_oe)
            msg.servo_od = int(node.target_servo_od)
            node.publisher_.publish(msg)

            if use_display and node.display_drawer:
                # desenha o frame e obtém a emoção atual do display
                current_emotion = node.display_drawer.draw_single_frame(vision_data)
                # 🔹 sincroniza emoção com ESPPublisher (motores, servos etc)
                if current_emotion != node.current_gesture:
                    node.update_expression(current_emotion)

                pygame.display.flip()
                clock.tick(24)

    except KeyboardInterrupt:
        node.get_logger().info('Interrupção por teclado detectada.')
    finally:
        node.get_logger().info('Encerrando workers e finalizando o nó.')

        # Envia 3 mensagens neutras para garantir parada
        for _ in range(3):
            final_msg = ESP()
            final_msg.linear_y = 0.0
            final_msg.angular_z = 0.0
            final_msg.servo_x = SERVO_NEUTRAL_X
            final_msg.servo_y = SERVO_NEUTRAL_Y
            final_msg.servo_z = SERVO_NEUTRAL_Z
            final_msg.servo_oe = SERVO_OE_NEUTRAL
            final_msg.servo_od = SERVO_OD_NEUTRAL
            node.publisher_.publish(final_msg)
            rclpy.spin_once(node, timeout_sec=0.1)

        node.stop_workers()
        node.destroy_node()
        rclpy.shutdown()
        if use_display and node.display_drawer:
            pygame.quit()

if __name__ == '__main__':
    main()
