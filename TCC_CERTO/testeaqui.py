#!/usr/bin/env python3
import pygame
import sys
import math
import queue

class DisplayController:
    def __init__(self, screen, data_queue):
        """
        Inicializa o controlador de display.
        """
        self.screen = screen
        self.data_queue = data_queue
        
        self.WIDTH, self.HEIGHT = self.screen.get_size()
        
        self.BLUE = (0, 204, 229)
        self.BLACK = (0, 0, 0)

        self.face_pos = None
        self.expression = "fofo"   # já inicia no modo fofo
        self.animation_frame = 0
        self.time_eye_open = 180
        self.time_to_blink = 12

    def _calculate_blink(self, frame):
        cycle_total = self.time_eye_open + self.time_to_blink
        cycle_frame = frame % cycle_total
        if cycle_frame < self.time_eye_open:
            return 0.0
        else:
            relative_time = cycle_frame - self.time_eye_open
            half_blink = self.time_to_blink / 2
            if relative_time < half_blink:
                return relative_time / half_blink
            else:
                return 1.0 - ((relative_time - half_blink) / half_blink)

    def _draw_eye_fofo(self, surface, color, center, size, scale, angle, closure):
        new_size = (size[0] * scale, size[1])
        adjusted_width = max(2, new_size[0] * (1 - closure))
        
        eye_surface = pygame.Surface((new_size[0], new_size[1]), pygame.SRCALPHA)
        eye_surface.fill((0,0,0,0))

        pygame.draw.ellipse(eye_surface, color, ((new_size[0] - adjusted_width) / 2, 0, adjusted_width, new_size[1]))
        rotated_eye = pygame.transform.rotate(eye_surface, angle)
        rect = rotated_eye.get_rect(center=center)
        surface.blit(rotated_eye, rect.topleft)


    def draw_single_frame(self):
        self.screen.fill(self.BLACK)

        eye_pos_x = self.WIDTH // 2
        eye_pos_y = self.HEIGHT // 2
        scale = 1.0

        eye_size = (110, 130)
        eye_distance = 120
        eye_tilt = 75
        closure = self._calculate_blink(self.animation_frame)

        # desenha expressão "fofo"
        self._draw_eye_fofo(self.screen, self.BLUE, (eye_pos_x - eye_distance, eye_pos_y+20), eye_size, scale, eye_tilt+25, closure)
        self._draw_eye_fofo(self.screen, self.BLUE, (eye_pos_x + eye_distance, eye_pos_y+20), eye_size, scale, -eye_tilt-25, closure)

        self.animation_frame += 1


def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Teste Expressão Fofo")
    clock = pygame.time.Clock()

    data_queue = queue.Queue()
    controller = DisplayController(screen, data_queue)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        controller.draw_single_frame()
        pygame.display.flip()
        clock.tick(60)  # 60 FPS

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
