##!/usr/bin/env python3
import pygame
import time
# import queue
import random
## import os
## from ament_index_python.packages import get_package_share_directory

class DisplayController:
    def __init__(self, screen):
        self.face_pos = None
        self.screen = screen
        self.WIDTH, self.HEIGHT = self.screen.get_size()

        # --- CONTROLE DE TRANSIÇÕES ---
        self.transition_state = None
        self.chain_transition = None
        self.next_expression = None

        # --- CONTROLE DE EMOÇÕES ---
        self.emotion_start_time = None
        self.emotion_timeout = 5  # tempo limite em segundos para mudar bravo → irritado
        


        # Obtém o caminho absoluto até o pacote
        # pkg_path = get_package_share_directory('robo')

        # Caminho completo até os sprites
        # sprites_path = os.path.join(pkg_path, 'spritesheet')

        # --- SPRITESHEET 1: expressões e piscadas ---
        self.sprite_w1, self.sprite_h1 = 800, 449
        self.cols1, self.rows1 = 25, 9
        # self.sheet1 = pygame.image.load(os.path.join(sprites_path, 'spritesheet_piscar.png')).convert_alpha()
        self.sheet1 = pygame.image.load("spritesheet_piscar.png").convert_alpha()

        # --- SPRITESHEET 2: movimentos laterais ---
        self.sprite_w2, self.sprite_h2 = 800, 449
        self.cols2, self.rows2 = 13, 4  # agora 4 linhas: dir, volta_dir, esq, volta_esq
        # self.sheet2 = pygame.image.load(os.path.join(sprites_path, 'spritesheet_deslocar.png')).convert_alpha()
        self.sheet2 = pygame.image.load("spritesheet_deslocar.png").convert_alpha()

        # --- SPRITESHEET 2: movimentos laterais ---
        self.sprite_w3, self.sprite_h3 = 800, 449
        self.cols3, self.rows3 = 25, 6
        # self.sheet3 = pygame.image.load(os.path.join(sprites_path, 'spritesheet_transicoes.png')).convert_alpha()
        self.sheet3 = pygame.image.load("spritesheet_transicoes.png").convert_alpha()

        # Dicionários de referência
        self.expressions1 = {
            "saco_cheio": (0, 25),
            "bravo": (1, 25),
            "feliz": (2, 25),
            "confuso": (3, 25),
            "irritado": (4, 25),
            "triste": (5, 25),
            "normal": (6, 25),
            "normal_dir": (7, 25),
            "normal_esq": (8, 25),
        }

        # NOVA ORGANIZAÇÃO DOS MOVIMENTOS
        self.movements2 = {
            "move_dir": (0, 13),         # meio -> direita
            "return_dir": (1, 13),       # direita -> meio
            "move_esq": (2, 13),         # meio -> esquerda
            "return_esq": (3, 13),       # esquerda -> meio
        }

        # Dicionários de referência
        self.expressions3 = {
            "saco_cheio_trans": (0, 25),
            "bravo_trans": (1, 25),
            "feliz_trans": (2, 25),
            "confuso_trans": (3, 25),
            "irritado_trans": (4, 25),
            "triste_trans": (5, 25)
        }

        # Estado atual
        self.expression = "normal"
        self.frame = 0
        # self.fps = 24
        self.clock = pygame.time.Clock()
        self.last_switch_time = time.time()
        self.state = "hold"  # 'hold' ou 'blink'
        self.hold_duration = random.uniform(2.0, 5.0)
        self.blink_start = 1
        self.blink_end = 24

        # Frames cortados
        self.frames1 = self._cut_frames(self.sheet1, self.cols1, self.rows1, self.sprite_w1, self.sprite_h1)
        self.frames2 = self._cut_frames(self.sheet2, self.cols2, self.rows2, self.sprite_w2, self.sprite_h2)
        self.frames3 = self._cut_frames(self.sheet3, self.cols3, self.rows3, self.sprite_w3, self.sprite_h3)


        # --- IDLE SYSTEM ---
        self.idle_state = "idle"
        self.idle_timer = time.time()
        self.idle_delay = random.uniform(3, 6)
        self.idle_blink_count = 0
        self.idle_done = True  # indica se terminou o comportamento idle atual

    # ----------------- AUXILIARES -----------------

    def _cut_frames(self, sheet, cols, rows, w, h):
        frames = []
        for r in range(rows):
            row = []
            for c in range(cols):
                rect = pygame.Rect(c * w, r * h, w, h)
                row.append(sheet.subsurface(rect))
            frames.append(row)
        return frames

    def _get_frame(self, expr, index):
        if expr in self.expressions1:
            row, count = self.expressions1[expr]
            frames = self.frames1
            cols = self.cols1
        elif expr in self.movements2:
            row, count = self.movements2[expr]
            frames = self.frames2
            cols = self.cols2
        elif expr in self.expressions3:
            row, count = self.expressions3[expr]
            frames = self.frames3
            cols = self.cols3
        else:
            expr = "normal"
            row, count = self.expressions1[expr]
            frames = self.frames1
            cols = self.cols1

        index %= count
        return frames[row][index % cols]

    def _reset_blink_timer(self):
        """Define um intervalo aleatório entre piscadas."""
        self.hold_duration = random.uniform(2.0, 5.0)
        self.last_switch_time = time.time()
        self.state = "hold"

    # ----------------- IDLE -----------------
    def _can_change_state(self):
        """Só permite transições se não estiver piscando nem em transição."""
        return self.state != "blink" and not self.transition_state

    def _idle_behavior(self, choice):
        now = time.time()
        self.idle_done = False  # começou um novo idle
        # self.idle_state = idle

        # --- ESTADO INICIAL (decide ação) ---
        # if self.idle_state == "idle":
        # if now - self.idle_timer >= self.idle_delay:
        self.idle_timer = now
        
        # mais chances de olhar pros lados
        # choice = random.choices(
        #     ["right", "left", "happy", "tired"],
        #     weights=[0.3, 0.3, 0.2, 0.2],
        #     k=1
        # )[0]

        if choice == "happy":
            if not self.transition_state:
                self._start_transition("feliz")
                self.idle_state = "wait_emotion"
                self.idle_timer = now
        elif choice == "tired":
            if not self.transition_state:
                self._start_transition("saco_cheio")
                self.idle_state = "wait_emotion"
                self.idle_timer = now
                # --- MOVIMENTOS (DIREITA / ESQUERDA) ---
        elif choice == "right":
            # se não estiver normal, faz a transição antes de mover
            if self.expression != "normal":
                self._start_transition("normal")
                self.idle_state = "prep_move_right"
                
            else:
                self.idle_state = "move_right"
                self.frame = 0

        elif choice == "left":
            if self.expression != "normal":
                self._start_transition("normal")
                self.idle_state = "prep_move_left"
                
            else:
                self.idle_state = "move_left"
                self.frame = 0

        # --- PREPARAÇÕES ---
        if self.idle_state in ("prep_move_right", "prep_move_left"):
            print("ta em preparacao")
            if self._can_change_state() and not self.transition_state and self.expression == "normal":
                self.idle_state = "move_right" if self.idle_state == "prep_move_right" else "move_left"
                self.frame = 0
                self.expression = "move_dir" if self.idle_state == "move_right" else "move_esq"

        # --- MOVIMENTO PARA DIREITA ---
        elif self.idle_state == "move_right":
            print("chegou em ir direita")
            self.expression = "move_dir"
            self.frame += 1
            if self.frame >= 12:
                self.frame = 0
                self.idle_blink_count = 0
                self.idle_state = "look_right"
                self.expression = "normal_dir"
                self._reset_blink_timer()

        # --- OLHA PARA DIREITA (pisca 2x) ---
        elif self.idle_state == "look_right":
            print("chegou em piscar direita")
            if self.state == "hold":
                # espera hold antes de iniciar blink
                if time.time() - self.last_switch_time >= self.hold_duration:
                    self.state = "blink"
                    self.frame = self.blink_start
                    self.last_switch_time = now
                    self.idle_blink_count += 1

            elif self.state == "blink":
                self.frame += 1
                if self.frame > self.blink_end:
                    self.state = "hold"
                    self.frame = 0
                    self.last_switch_time = now
                    # se piscou 2x, volta
                    if self.idle_blink_count >= 2:
                        self.idle_state = "return_center_right"
                        self.frame = 0

        # --- VOLTA PARA O CENTRO (DIREITA) ---
        elif self.idle_state == "return_center_right":
            print("chegou em voltar para centro")
            self.expression = "return_dir"
            self.frame += 1
            if self.frame >= 12:
                self.frame = 0
                self.expression = "normal"
                self.idle_state = "fim"
                # self.idle_timer = now
                self.idle_delay = random.uniform(3, 6)
                self._reset_blink_timer()
            

        # --- MOVIMENTO PARA ESQUERDA ---
        elif self.idle_state == "move_left":
            print("chegou em move esq")
            self.expression = "move_esq"
            self.frame += 1
            if self.frame >= 12:
                self.frame = 0
                self.idle_blink_count = 0
                self.idle_state = "look_left"
                self.expression = "normal_esq"
                self._reset_blink_timer()

        # --- OLHA PARA ESQUERDA (pisca 2x) ---
        elif self.idle_state == "look_left":
            print("chegou em pisca esq")
            if self.state == "hold":
                if time.time() - self.last_switch_time >= self.hold_duration:
                    self.state = "blink"
                    self.frame = self.blink_start
                    self.last_switch_time = time.time()
                    self.idle_blink_count += 1
            elif self.state == "blink":
                self.frame += 1
                if self.frame > self.blink_end:
                    self.state = "hold"
                    self.frame = 0
                    self.last_switch_time = time.time()
                    if self.idle_blink_count >= 2:
                        self.idle_state = "return_center_left"
                        self.frame = 0

        # --- VOLTA PARA O CENTRO (ESQUERDA) ---
        elif self.idle_state == "return_center_left":
            print("chegou em volta centro esq")
            self.expression = "return_esq"
            self.frame += 1
            if self.frame >= 12:
                self.frame = 0
                self.expression = "normal"
                self.idle_state = "fim"
                # self.idle_timer = now
                self.idle_delay = random.uniform(3, 6)
                self._reset_blink_timer()
            

        # --- EMOÇÕES (feliz / cansado) ---
        elif self.idle_state == "wait_emotion":
            if now - self.idle_timer >= random.uniform(2, 4):
                if not self.transition_state:
                    self._start_transition("normal")
                    self.idle_state = "fim"
                    # self.idle_timer = now
                    self.idle_delay = random.uniform(3, 6)
                    self._reset_blink_timer()

        elif self.idle_state == "fim":
            self.idle_done = True  # sinaliza para o main que terminou
            return
            

    def _start_transition(self, target_expression):
        """
        Define qual transição precisa ocorrer com base no estado atual e no novo.
        """
        now = time.time()
        if self.transition_state:
            return  # evita conflito de transições simultâneas

        # Emoção -> Normal
        if target_expression == "normal" and f"{self.expression}_trans" in self.expressions3:
            back_trans = f"{self.expression}_trans"
            if back_trans in self.expressions3:
                self.transition_state = "to_normal"
                self.expression = back_trans
                self.frame = self.expressions3[back_trans][1] - 1
                self.transition_total = self.expressions3[back_trans][1]
                self.next_expression = "normal"

                # print(f"[Display] Target expression: {target_expression} | Expressão atual: {self.expression} | Transição: {self.transition_state}")
                return

        # Normal -> Emoção
        elif self.expression == "normal":
            fwd_trans = f"{target_expression}_trans"
            print(fwd_trans)
            if fwd_trans in self.expressions3:
                self.transition_state = "to_emotion"
                self.expression = fwd_trans
                self.frame = 0
                self.transition_total = self.expressions3[fwd_trans][1]
                self.next_expression = target_expression
                # print(f"[Display] Target expression: {target_expression} | Expressão atual: {self.expression} | Transição: {self.transition_state}")
                return

        # Emoção -> Outra Emoção (dupla transição)
        else:
            from_expr = self.expression
            to_expr = target_expression
            back_trans = f"{from_expr}_trans"
            fwd_trans = f"{to_expr}_trans"

            if back_trans in self.expressions3 and fwd_trans in self.expressions3:
                self.transition_state = "to_normal"
                self.expression = back_trans
                self.frame = self.expressions3[back_trans][1] - 1
                self.transition_total = self.expressions3[back_trans][1]
                self.next_expression = "normal"
                self.chain_transition = fwd_trans
                self.final_expression = to_expr
                return

        # fallback: troca direta
        # self.expression = target_expression
        # self.frame = 0
        # self.state = "hold"
        # self.last_switch_time = now
        # self.transition_state = None

    # -----------------------------------------------------

    def _handle_transition_chain(self):
        """Controla as animações de transição e cadeia entre emoções."""
        if self.transition_state == "to_emotion":
            self.frame += 1
            if self.frame >= self.transition_total:
                self.expression = self.next_expression
                self.frame = 0
                self.transition_state = None
                self._reset_blink_timer()

        elif self.transition_state == "to_normal":
            # self.frame -= 1
            self.frame = max(0, self.frame - 1)
            if self.frame <= 0:
                if self.chain_transition:
                    # Encadeia transição normal → nova emoção
                    self.transition_state = "to_emotion"
                    self.expression = self.chain_transition
                    self.frame = 0
                    self.transition_total = self.expressions3[self.chain_transition][1]
                    self.next_expression = self.final_expression
                    self.chain_transition = None
                else:
                    self.expression = "normal"
                    self.transition_state = None
                    self.frame = 0
                    self._reset_blink_timer()

                
    # ----------------- DESENHO -----------------

    def draw_single_frame(self, new_gesture, is_idle=False):
        now = time.time()
        # self.face_pos = data.get("face")

        # if self.face_pos == None:
        #     new_gesture = "saco_cheio"
        #     if new_gesture and new_gesture != self.expression and self._can_change_state():
        #         self._start_transition(new_gesture)
        
        # --- LEITURA DE DATA DO SISTEMA DE VISÃO ---
        # else:
        # new_gesture = data.get("gesture")

        if is_idle:
            self._idle_behavior(new_gesture)

        else:
            if new_gesture and new_gesture != self.expression and self._can_change_state():
                # if new_gesture and new_gesture != self.expression:
                self._start_transition(new_gesture)
        

        # --- PROCESSO DE TRANSIÇÃO ---
        if self.transition_state:
            self._handle_transition_chain()
            frame_img = self._get_frame(self.expression, int(self.frame))
        else:
            # --- LÓGICA DE PISCAR (só se não estiver em transição) ---
            if self.state == "hold":
                self.frame = 0
                if now - self.last_switch_time >= self.hold_duration:
                    self.state = "blink"
                    self.frame = self.blink_start
                    self.last_switch_time = now

            elif self.state == "blink":
                self.frame += 1
                if self.frame > self.blink_end:
                    self.state = "hold"
                    self.frame = 0
                    self.last_switch_time = now
                    self._reset_blink_timer()

            frame_img = self._get_frame(self.expression, self.frame)

            

        # --- DESENHO ---
        self.screen.fill((0, 0, 0))
        rect = frame_img.get_rect(center=(self.WIDTH // 2, self.HEIGHT // 2))
        self.screen.blit(frame_img, rect)
        pygame.display.flip()
        # return self.expression
        # self.clock.tick(self.fps)


    # def draw_single_frame(self, data=None):
    #     now = time.time()

    #     if data:
    #         self.face_pos = data.get("face")
    #         new_gesture = data.get("gesture")
    #         if new_gesture and new_gesture != self.expression:
    #             self.expression = new_gesture
    #             self.frame = 0
    #             self.state = "blink"
    #             self.last_switch_time = now

    #     # --- se não há face detectada, entra no idle ---
    #     if self.face_pos is None:
    #         self._idle_behavior()

    #     # Lógica de piscada
    #     if self.state == "hold":
    #         self.frame = 0
    #         if now - self.last_switch_time >= self.hold_duration:
    #             self.state = "blink"
    #             self.frame = self.blink_start
    #             self.last_switch_time = now
    #     elif self.state == "blink":
    #         self.frame += 1
    #         if self.frame > self.blink_end:
    #             self.state = "hold"
    #             self.frame = 0
    #             self.last_switch_time = now
    #             self._reset_blink_timer()

    #     # --- Desenho ---
    #     self.screen.fill((0, 0, 0))
    #     frame_img = self._get_frame(self.expression, self.frame)
    #     rect = frame_img.get_rect(center=(self.WIDTH // 2, self.HEIGHT // 2))
    #     self.screen.blit(frame_img, rect)
    #     pygame.display.flip()
        # self.clock.tick(self.fps)


# ================== MAIN =====================
# -------------------- LOOP DE TESTE INTERATIVO --------------------
# if __name__ == "__main__":
#     import pygame
#     import random

#     pygame.init()
#     screen = pygame.display.set_mode((800, 449))
#     pygame.display.set_caption("Simulador de Visão - Controle de Gestos e Idle")

#     controller = DisplayController(screen)

#     gestures = [
#         "normal", "feliz", "triste",
#         "irritado", "saco_cheio", "confuso",
#         "normal_dir", "normal_esq", "move_dir", "move_esq"
#     ]
#     gesture_index = 0
#     gesture_detected = "normal"

#     font = pygame.font.SysFont(None, 28)
#     clock = pygame.time.Clock()
#     running = True

#     print("Controles:")
#     print("← → : alterna gestos")
#     print("1–6 : seleciona gesto específico")
#     print("ESPACO : piscar (blink)")
#     print("I : comportamento idle (sorteio aleatório)")
#     print("ESC : sair\n")

#     while running:
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False

#             elif event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_ESCAPE:
#                     running = False

#                 elif event.key == pygame.K_RIGHT:
#                     gesture_index = (gesture_index + 1) % len(gestures)
#                     gesture_detected = gestures[gesture_index]

#                 elif event.key == pygame.K_LEFT:
#                     gesture_index = (gesture_index - 1) % len(gestures)
#                     gesture_detected = gestures[gesture_index]

#                 elif event.key == pygame.K_SPACE:
#                     gesture_detected = "blink"

#                 # --- tecla para acionar o comportamento IDLE ---
#                 elif event.key == pygame.K_i:
#                     choice = random.choices(
#                         ["move_dir", "move_esq", "feliz", "cansado"],
#                         weights=[0.3, 0.3, 0.2, 0.2],
#                         k=1
#                     )[0]
#                     gesture_detected = choice
#                     print(f"Idle sorteou: {choice}")
#                     controller._idle_behavior(choice)

#                 # atalhos diretos para gestos principais
#                 elif event.key == pygame.K_1: gesture_detected = "normal"
#                 elif event.key == pygame.K_2: gesture_detected = "feliz"
#                 elif event.key == pygame.K_3: gesture_detected = "triste"
#                 elif event.key == pygame.K_4: gesture_detected = "irritado"
#                 elif event.key == pygame.K_5: gesture_detected = "saco_cheio"
#                 elif event.key == pygame.K_6: gesture_detected = "confuso"

#         # limpa tela
#         screen.fill((0, 0, 0))

#         # desenha o frame com base no gesto "detectado"
#         controller.draw_single_frame(gesture_detected)

#         # mostra qual gesto está ativo
#         label = font.render(f"Gesto detectado: {gesture_detected}", True, (0, 255, 255))
#         screen.blit(label, (10, 10))

#         pygame.display.flip()
#         clock.tick(24)

#     pygame.quit()
