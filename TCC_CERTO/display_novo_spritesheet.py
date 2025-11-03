#!/usr/bin/env python3
import pygame
import time
import queue
import random


class DisplayController:
    def __init__(self, screen, data_queue=None):
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

        # --- SPRITESHEET 1: expressões e piscadas ---
        self.sprite_w1, self.sprite_h1 = 800, 449
        self.cols1, self.rows1 = 25, 9
        self.sheet1 = pygame.image.load("spritesheet_piscar.png").convert_alpha()

        # --- SPRITESHEET 2: movimentos laterais ---
        self.sprite_w2, self.sprite_h2 = 800, 449
        self.cols2, self.rows2 = 13, 4  # agora 4 linhas: dir, volta_dir, esq, volta_esq
        self.sheet2 = pygame.image.load("spritesheet_deslocar.png").convert_alpha()

        # --- SPRITESHEET 2: movimentos laterais ---
        self.sprite_w3, self.sprite_h3 = 800, 449
        self.cols3, self.rows3 = 25, 6
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

    def _idle_behavior(self):
        now = time.time()

        # --- ESTADO INICIAL (decide ação) ---
        if self.idle_state == "idle":
            if now - self.idle_timer >= self.idle_delay:
                self.idle_timer = now
                # mais chances de olhar pros lados
                choice = random.choices(
                    ["right", "left", "happy", "tired"],
                    weights=[0.3, 0.3, 0.2, 0.2],
                    k=1
                )[0]

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
                        return
                    else:
                        self.idle_state = "move_right"
                        self.frame = 0

                elif choice == "left":
                    if self.expression != "normal":
                        self._start_transition("normal")
                        self.idle_state = "prep_move_left"
                        return
                    else:
                        self.idle_state = "move_left"
                        self.frame = 0

        # --- PREPARAÇÕES ---
        elif self.idle_state in ("prep_move_right", "prep_move_left"):
            if self._can_change_state() and not self.transition_state and self.expression == "normal":
                self.idle_state = "move_right" if self.idle_state == "prep_move_right" else "move_left"
                self.frame = 0

        # --- MOVIMENTO PARA DIREITA ---
        elif self.idle_state == "move_right":
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
            if self.state == "hold":
                # espera hold antes de iniciar blink
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
                    # se piscou 2x, volta
                    if self.idle_blink_count >= 2:
                        self.idle_state = "return_center_right"
                        self.frame = 0

        # --- VOLTA PARA O CENTRO (DIREITA) ---
        elif self.idle_state == "return_center_right":
            self.expression = "return_dir"
            self.frame += 1
            if self.frame >= 12:
                self.frame = 0
                self.expression = "normal"
                self.idle_state = "idle"
                self.idle_timer = now
                self.idle_delay = random.uniform(3, 6)
                self._reset_blink_timer()

        # --- MOVIMENTO PARA ESQUERDA ---
        elif self.idle_state == "move_left":
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
            self.expression = "return_esq"
            self.frame += 1
            if self.frame >= 12:
                self.frame = 0
                self.expression = "normal"
                self.idle_state = "idle"
                self.idle_timer = now
                self.idle_delay = random.uniform(3, 6)
                self._reset_blink_timer()

        # --- EMOÇÕES (feliz / cansado) ---
        elif self.idle_state == "wait_emotion":
            if now - self.idle_timer >= random.uniform(2, 4):
                if not self.transition_state:
                    self._start_transition("normal")
                    self.idle_state = "idle"
                    self.idle_timer = now
                    self.idle_delay = random.uniform(3, 6)
                    self._reset_blink_timer()

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
                return

        # Normal -> Emoção
        elif self.expression == "normal":
            fwd_trans = f"{target_expression}_trans"
            if fwd_trans in self.expressions3:
                self.transition_state = "to_emotion"
                self.expression = fwd_trans
                self.frame = 0
                self.transition_total = self.expressions3[fwd_trans][1]
                self.next_expression = target_expression
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

    def draw_single_frame(self, data=None):
        now = time.time()

        # # --- Verifica se precisa evoluir bravo → irritado ---
        # if new_gesture != self.expression:
        #     self.emotion_start_time = time.time() if new_gesture == "bravo" else None
        #     if now - self.emotion_start_time >= self.emotion_timeout:
        #         print("Bravo por muito tempo → evoluindo para irritado...")
        #         self._start_transition("irritado")
        #         self.emotion_start_time = None  # evita repetição

        # --- LEITURA DE DATA DO SISTEMA DE VISÃO ---
        if data:
            self.face_pos = data.get("face")
            new_gesture = data.get("gesture")

            # Só reage se houver rosto detectado
            if self.face_pos is not None:
                if new_gesture and new_gesture != self.expression and self._can_change_state():
                    self._start_transition(new_gesture)
            else:
                # sem rosto → idle
                self._idle_behavior()
        else:
            # sem dados → idle
            self._idle_behavior()

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
        return self.expression
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

# def main():
#     pygame.init()
#     screen = pygame.display.set_mode((800, 480))
#     pygame.display.set_caption("Display Emocional")

#     clock = pygame.time.Clock()
#     data_queue = queue.Queue()
#     controller = DisplayController(screen, data_queue)

#     running = True
#     while running:
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False

#         # processa dados da fila
#         data = None
#         try:
#             data = data_queue.get_nowait()
#         except queue.Empty:
#             pass

#         controller.draw_single_frame(data)
#         clock.tick(24)

#     pygame.quit()


# if __name__ == "__main__":
#     main()
