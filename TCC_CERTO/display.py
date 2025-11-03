#!/usr/bin/env python3

import pygame
import math
import queue

class DisplayController:
   
    def __init__(self, screen, data_queue):
      
        self.screen = screen
        self.data_queue = data_queue
        
        self.WIDTH, self.HEIGHT = self.screen.get_size()
        
        self.BLUE = (1, 226, 160)
        self.BLACK = (0, 0, 0)

        self.face_pos = None
        self.expression = "curioso"
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

    
    def _draw_eye(self, surface, color, center, size, scale, angle, closure):
        new_size = (size[0] * scale, size[1])
        adjusted_width = max(2, new_size[0] * (1 - closure))
        
        eye_surface = pygame.Surface((new_size[0], new_size[1]), pygame.SRCALPHA)
        eye_surface.fill((0,0,0,0))

        pygame.draw.ellipse(eye_surface, color, ((new_size[0] - adjusted_width) / 2, 0, adjusted_width, new_size[1]))
        rotated_eye = pygame.transform.rotate(eye_surface, angle)
        rect = rotated_eye.get_rect(center=center)
        surface.blit(rotated_eye, rect.topleft)

    def _draw_eye_happy(self, surface, color, center, size, scale, angle, closure):
        new_size = (size[0] * scale, size[1])
        adjusted_width = max(2, new_size[0] * (1 - closure))
        
        eye_surface = pygame.Surface((new_size[0], new_size[1]), pygame.SRCALPHA)
        eye_surface.fill((0,0,0,0))

        pygame.draw.ellipse(eye_surface, color, ((new_size[0] - adjusted_width) / 2, 0, adjusted_width, new_size[1]))
        rotated_eye = pygame.transform.rotate(eye_surface, angle)

        rect = rotated_eye.get_rect(center=center)
        surface.blit(rotated_eye, rect.topleft)


        cobertura_surface = pygame.Surface((new_size[0], new_size[1]), pygame.SRCALPHA)
        cobertura_surface.fill((0, 0, 0, 0))

        pygame.draw.ellipse(cobertura_surface, (0,0,0), ((new_size[0] - adjusted_width) / 2, 0, adjusted_width/2, new_size[1]))
        rotated_cobertura = pygame.transform.rotate(cobertura_surface, angle)

        rect_cobertura = rotated_cobertura.get_rect(center=center)
        surface.blit(rotated_cobertura, rect_cobertura.topleft)

    def _draw_eye_bravo(self, surface, color, center, size, scale, angle, closure):
        new_size = (size[0] * scale, size[1])
        adjusted_width = max(2, new_size[0] * (1 - closure))
        
        eye_surface = pygame.Surface((new_size[0], new_size[1]), pygame.SRCALPHA)
        eye_surface.fill((0,0,0,0))
        pygame.draw.ellipse(eye_surface, color, ((new_size[0] - adjusted_width) / 2, 0, adjusted_width, new_size[1]))
        
        eye_cut_surface = pygame.Surface((new_size[0], new_size[1]), pygame.SRCALPHA)
        eye_cut_surface.fill((0,0,0,0))
        pygame.draw.rect(eye_cut_surface, self.BLACK, (0, 0, new_size[0]/2.5, new_size[1]))

        eye_surface.blit(eye_cut_surface, (0, 0))

        rotated_eye = pygame.transform.rotate(eye_surface, angle)

        rect = rotated_eye.get_rect(center=center)
        surface.blit(rotated_eye, rect.topleft)

    def _draw_eye_fofo(self, surface, color, center, size, scale, angle, closure):
        new_size = (size[0] * scale, size[1] * 1.2)  # mais redondinho
        adjusted_width = max(2, new_size[0] * (1 - closure))

        # olho base
        eye_surface = pygame.Surface((new_size[0], new_size[1]), pygame.SRCALPHA)
        eye_surface.fill((0,0,0,0))
        pygame.draw.ellipse(eye_surface, color, ((new_size[0] - adjusted_width) / 2, 0, adjusted_width, new_size[1]))

        rotated_eye = pygame.transform.rotate(eye_surface, angle)
        rect = rotated_eye.get_rect(center=center)
        surface.blit(rotated_eye, rect.topleft)

        # brilho branco no olho (detalhe "fofo")
        brilho_radius = int(min(new_size) * 0.15)
        brilho_x = rect.centerx - brilho_radius
        brilho_y = rect.centery - brilho_radius
        pygame.draw.circle(surface, (255, 255, 255), (brilho_x, brilho_y), brilho_radius)


    def draw_single_frame(self, data=None):
        """
        Desenha um único quadro na tela.
        """
        if data:
            self.face_pos = data.get('face')
            new_gesture = data.get('gesture')
            if new_gesture != self.expression and new_gesture is not None:
                self.expression = new_gesture
                self.animation_frame = self.time_eye_open

        self.screen.fill(self.BLACK)

        eye_pos_x = self.WIDTH // 2
        eye_pos_y = self.HEIGHT // 2
        current_expression = self.expression
        scale = 1.3

        if self.face_pos is None:
            current_expression = "normal"
            # offset = int(math.sin(pygame.time.get_ticks() / 700) * 150)
            # eye_pos_x += offset

        eye_size = (110, 130)
        eye_distance = 140
        eye_tilt = 75
        closure = self._calculate_blink(self.animation_frame)

        if current_expression in ["normal", "segue"]:
            self._draw_eye(self.screen, self.BLUE, (eye_pos_x - eye_distance, eye_pos_y), eye_size, scale, eye_tilt, closure)
            self._draw_eye(self.screen, self.BLUE, (eye_pos_x + eye_distance, eye_pos_y), eye_size, scale, -eye_tilt, closure)
        elif current_expression == "feliz":
            self._draw_eye_happy(self.screen, self.BLUE, (eye_pos_x - eye_distance, eye_pos_y), eye_size, scale, eye_tilt, closure)
            self._draw_eye_happy(self.screen, self.BLUE, (eye_pos_x + eye_distance, eye_pos_y), eye_size, scale, 180-eye_tilt, closure)
        elif current_expression == "bravo":
            self._draw_eye_bravo(self.screen, self.BLUE, (eye_pos_x - eye_distance, eye_pos_y), eye_size, scale, eye_tilt-180, closure)
            self._draw_eye_bravo(self.screen, self.BLUE, (eye_pos_x + eye_distance, eye_pos_y), eye_size, scale, -eye_tilt, closure)

        elif current_expression == "fofo":
            self._draw_eye_fofo(self.screen, self.BLUE, (eye_pos_x - eye_distance, eye_pos_y+20), eye_size, scale, eye_tilt-10, closure)
            self._draw_eye_fofo(self.screen, self.BLUE, (eye_pos_x + eye_distance, eye_pos_y+20), eye_size, scale, -eye_tilt+10, closure)

        self.animation_frame += 1