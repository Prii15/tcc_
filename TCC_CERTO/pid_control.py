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

# Importa a mensagem customizada do ROS 2
from robo.msg import ESP

# Importa as classes de visão e display
from vision import VisionController
from display import DisplayController

# --- Constantes de Configuração ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
SERVO_NEUTRAL_X = 90
SERVO_NEUTRAL_Y = 84
SERVO_NEUTRAL_Z = 90
SERVO_OE_NEUTRAL = 90
SERVO_OD_NEUTRAL = 90

HAPPY_EAR_AMPLITUDE = 35
HAPPY_EAR_FREQUENCY = 0.5

#Limites servo x
Servo_X_min = 70
Servo_X_max = 110

#Limites servo y
Servo_Y_min = 54
Servo_Y_max = 114

#Limites servo z
Servo_Z_min = 55
Servo_Z_max = 115


class ESPPublisher(Node): 
    def __init__(self, screen):
        super().__init__('esp_publisher')
        self.get_logger().info('Nó de Controle por Visão iniciado.')

        self.publisher_ = self.create_publisher(ESP, '/esp_control', 10)
        self.data_queue = queue.Queue(maxsize=2)

        if screen:
            self.display_drawer = DisplayController(screen, self.data_queue)
        else:
            
            self.display_drawer = None
        
        self.vision_worker = VisionController(self.data_queue) 
        # self.vision_thread = threading.Thread(target=self.vision_worker.run)

        # Estado dos alvos (targets) para os motores e servos
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.target_servo_x = float(SERVO_NEUTRAL_X)    # pra cima e pra baixo
        self.target_servo_y = float(SERVO_NEUTRAL_Y)    # inclina
        self.target_servo_z = float(SERVO_NEUTRAL_Z)    # vira lateralmente
        self.target_servo_oe = 90.0
        self.target_servo_od = 90.0
        self.happy_start_time = None
        self.fofo_lado_direita = True

        # Controle de distância da pessoa
        self.ultima_distancia = None
        self.K_CALIBRACAO = 95.0  # ajuste depois conforme teste (px -> metros)


        # self.publish_thread = threading.Thread(target=self._process_vision_data)
        # self.publish_thread.daemon = True
        # self.publish_thread.start()

    def start_workers(self):
        self.vision_worker.start()
        self.get_logger().info('Thread de visão iniciada.')

    def stop_workers(self):
        if self.vision_worker:
            self.vision_worker.stop()
        
        self.get_logger().info('Thread de visão finalizada.')
    
    def perform_idle_action(self):
        t = time.time()

        # Inicializa variáveis de controle caso não existam
        if not hasattr(self, 'idle_timer'):
            self.idle_timer = t + random.uniform(2, 5)
            self.idle_side = True  # alterna lados
            self.idle_head_pos = SERVO_NEUTRAL_Y
            self.idle_ear_amp = 5

        action = 'normal'

        # Executa ação apenas quando o timer expira
        if t >= self.idle_timer:
            # Escolhe aleatoriamente uma ação
            action = random.choice(['cabeca_inclinado','cabeca_lado', 'cabeca_frente_tras', 'orelhas_senoide'])

            if action == 'normal':
                # Cabeça e orelhas neutras
                self.target_servo_x = SERVO_NEUTRAL_X
                self.target_servo_y = SERVO_NEUTRAL_Y
                self.target_servo_z = SERVO_NEUTRAL_Z
                self.target_servo_oe = SERVO_OE_NEUTRAL
                self.target_servo_od = SERVO_OD_NEUTRAL

            elif action == 'cabeca_inclinado':
                # Alterna lado
                if self.idle_side:
                    self.target_servo_y = SERVO_NEUTRAL_Y + 10
                else:
                    self.target_servo_y = SERVO_NEUTRAL_Y - 10
                self.idle_side = not self.idle_side
            
            elif action == 'cabeca_lado':
                # Pequena inclinação para frente/tras
                self.target_servo_x = SERVO_NEUTRAL_Z + random.choice([-25, 25])# Pequena inclinação para frente/tras

            elif action == 'cabeca_frente_tras':
                # Pequena inclinação para frente/tras
                self.target_servo_x = SERVO_NEUTRAL_X + random.choice([-10, 10])

            elif action == 'orelhas_senoide':
                # Movimento senoidal simples para as orelhas
                self.target_servo_oe = SERVO_OE_NEUTRAL + self.idle_ear_amp * math.sin(2*math.pi*0.5*t)
                self.target_servo_od = SERVO_OD_NEUTRAL - self.idle_ear_amp * math.sin(2*math.pi*0.5*t)

            # Define próximo timer
            self.idle_timer = t + random.uniform(2, 5)

        # Suaviza movimentos para parecer natural
        self.target_servo_x = self.target_servo_x * 0.7 + SERVO_NEUTRAL_X * 0.1
        self.target_servo_y = self.target_servo_y * 0.7 + SERVO_NEUTRAL_Y * 0.1


    def _map_value(self, value, from_min, from_max, to_min, to_max):
        if from_max == from_min: return to_min
        mapped = (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
        return max(to_min, min(to_max, mapped))

    def _process_vision_data(self, data):
        
        face_coords = data.get('face')
        gesture = data.get('gesture')

        if face_coords:
            face_x, face_y, face_w, face_h = face_coords
            face_center_x = face_x + face_w // 2
            face_center_y = face_y + face_h // 2

            # Segue horizontalmente com o servo_z (lateral)
            self.target_servo_z = self._map_value(face_center_x, 0, FRAME_WIDTH, Servo_Z_min, Servo_Z_max)
            # Segue verticalmente com o servo_x (cima e baixo)
            self.target_servo_x = self._map_value(face_center_y, 0, FRAME_HEIGHT, Servo_X_min, Servo_X_max)
      
            # Aproximação da distância usando a largura do rosto detectado
            distancia_atual = self.K_CALIBRACAO / (face_w + 2)  # metros aprox

            if self.ultima_distancia is None:
                self.ultima_distancia = distancia_atual

            if distancia_atual > 1.0:  # só anda se tiver a mais de 1m
                if distancia_atual > self.ultima_distancia + 0.1:  # está se afastando
                    # self.get_logger().info(f"Pessoa se afastando: {distancia_atual:.2f} m")
                    self.target_linear_y = 0.2  # velocidade para frente
                else:
                    self.target_linear_y = 0.0
            else:
                self.target_linear_y = 0.0

            # Atualiza memória
            self.ultima_distancia = distancia_atual
        else:
            # Suaviza retorno para neutro quando não há rosto
            # self.target_servo_z = self.target_servo_z * 0.95 + SERVO_NEUTRAL_X * 0.05
            self.target_servo_x = max(Servo_X_min, min(Servo_X_max, self.target_servo_x))

            self.target_linear_y = 0.0
            
        if self.happy_start_time is None:
            self.happy_start_time = time.time()

        if gesture == "feliz":
            current_time = time.time() - self.happy_start_time
            #self.get_logger().info(f"Tempo atual: {current_time}")
            # Movimento senoidal para orelhas
            # As orelhas se movem para a frente e para trás de forma cíclica
            angle_offset1 = 45 + HAPPY_EAR_AMPLITUDE * math.sin(2 * math.pi*current_time * HAPPY_EAR_FREQUENCY)
            angle_offset2 = 45 + HAPPY_EAR_AMPLITUDE * math.sin(2 * math.pi*current_time * (HAPPY_EAR_FREQUENCY+0.2))
            self.target_servo_oe = SERVO_OE_NEUTRAL + angle_offset1
            self.target_servo_od = SERVO_OD_NEUTRAL - angle_offset1

        elif gesture == "bravo":
            
            self.target_servo_od = 135
            self.target_servo_oe = 45

            self.target_servo_y = 104

        elif gesture == "fofo":
            current_time = time.time() - self.happy_start_time
            #self.get_logger().info(f"Tempo atual: {current_time}")

            if self.fofo_lado_direita:
                # Inclina a cabeça para a direita
                self.target_servo_y = 69
            else:
                # Inclina a cabeça para a esquerda
                self.target_servo_y = 99

            # Alterna o lado para a próxima chamada
            self.fofo_lado_direita = not self.fofo_lado_direita

            # Movimento senoidal para orelhas
            # As orelhas se movem para a frente e para trás de forma cíclica
            angle_offset1 = 45 + HAPPY_EAR_AMPLITUDE * math.sin(2 * math.pi*current_time * HAPPY_EAR_FREQUENCY)
            angle_offset2 = 45 + HAPPY_EAR_AMPLITUDE * math.sin(2 * math.pi*current_time * (HAPPY_EAR_FREQUENCY+0.2))
            self.target_servo_oe = SERVO_OE_NEUTRAL + angle_offset1
            self.target_servo_od = SERVO_OD_NEUTRAL - angle_offset1
            
        else:
            self.happy_start_time = None
            # Chama a função de idle
            self.perform_idle_action()


def has_display():
    return os.environ.get('DISPLAY') 
    # or os.path.exists('/dev/fb0')

def main(args=None):
    use_display = has_display()

    screen = None
    clock = None

    # if use_display:
    #     print(">>> MODO GRÁFICO (DESKTOP) DETECTADO. Usando configuração padrão. <<<")

    #     try:
            # Inicialização Gráfica
            # Força uso do framebuffer para desenhar no display da Jetson
            # os.environ["SDL_VIDEODRIVER"] = "fbcon"
            # os.environ["SDL_FBDEV"] = "/dev/fb0"
            # os.environ["SDL_NOMOUSE"] = "1"

    pygame.init()
    info = pygame.display.Info()
    width, height = info.current_w, info.current_h
    screen = pygame.display.set_mode((width, height), pygame.FULLSCREEN)
    clock = pygame.time.Clock()

        # except Exception as e:
        #     print(f"[AVISO] Falha ao iniciar o display: {e}")
        #     use_display = False
        #     screen = None


    # Inicialização do ROS
    rclpy.init(args=args)
    node = ESPPublisher(screen)

    node.start_workers()
    running = True
    
    try:
        while running:
            if use_display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or \
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                        running = False
            
            rclpy.spin_once(node, timeout_sec=0)

            vision_data = None # Começa com dados nulos

            try:
                # 1. Pega os dados da fila UMA ÚNICA VEZ e processa eles para o ROS2
                vision_data = node.data_queue.get_nowait()
                node._process_vision_data(vision_data)
            except queue.Empty:
                pass
            
            # Preenche a mensagem ROS antes de publicar
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
                node.display_drawer.draw_single_frame(vision_data)
                pygame.display.flip()
                clock.tick(24)

    except KeyboardInterrupt:
        node.get_logger().info('Interrupção por teclado detectada.')
    finally:
        node.get_logger().info('Encerrando workers e finalizando o nó.')
        
        # Envia uma última mensagem com os servos no centro
        final_msg = ESP()
        final_msg.linear_y = 0.0
        final_msg.angular_z = 0.0
        final_msg.servo_x = int(SERVO_NEUTRAL_X)
        final_msg.servo_y = int(SERVO_NEUTRAL_Y)
        final_msg.servo_z = 90
        final_msg.servo_oe = 90
        final_msg.servo_od = 90
        node.publisher_.publish(final_msg)

        # Dá tempo para que o ROS envie a mensagem
        rclpy.spin_once(node, timeout_sec=0.1)

        node.stop_workers()
        node.destroy_node()
        rclpy.shutdown()
        if use_display and node.display_drawer:
            pygame.quit()

if __name__ == '__main__':
    main()
