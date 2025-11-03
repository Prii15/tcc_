#!/usr/bin/env python3

import cv2
import mediapipe as mp
import serial
import time
import threading

from mediapipe.python.solutions import drawing_utils as mp_drawing

class VisionController:
    def __init__(self, data_queue):
        self.arduino = None
        self.last_send_time = 0
        self.data_queue = data_queue # Fila para comunicar com a thread de display
        self.running = False
        self.thread = threading.Thread(target=self._run_detection)

        # # Controle de "lock" de pessoa
        # self.locked = False
        # self.last_face_coords = None
        # self.last_face_time = 0

        # Controle de gesto consistente
        self.last_gesture = None
        # self.gesture_start_time = 0
        # self.MIN_GESTURE_TIME = 0.5  # segundos

        # Inicializa a câmera
        self.cap = cv2.VideoCapture(0)
        #self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Inicializa MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False, max_num_hands=1,
            min_detection_confidence=0.5, min_tracking_confidence=0.5
        )
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(min_detection_confidence=0.3)

    def _detect_gesture(self, mao_resultado):
        gesto = "normal"
        pos_x = None

        if mao_resultado.multi_hand_landmarks:
            mao = mao_resultado.multi_hand_landmarks[0]
            dedos_estendidos = 0
            for ponta, art in [(8, 6), (12, 10), (16, 14), (20, 18)]:
                if mao.landmark[ponta].y < mao.landmark[art].y:
                    dedos_estendidos += 1

            if dedos_estendidos >= 4:
                gesto = "feliz"
            elif dedos_estendidos == 0:
                gesto = "bravo"
            elif dedos_estendidos == 2:
                gesto = "cansado"
            elif dedos_estendidos == 1:
                gesto = "triste"
                # pos_x = mao.landmark[0].x
            elif dedos_estendidos == 3:
                gesto = "confuso"  # gesto especial para liberar o lock
            else:
                gesto = "normal"

        return gesto, pos_x

    #detecção de rosto
    def _detect_face(self, frame_rgb):
        face_coordinates = None
        results = self.face_detection.process(frame_rgb)
        if results.detections:
            detection = results.detections[0] # Pega apenas o primeiro rosto
            bboxC = detection.location_data.relative_bounding_box
            ih, iw, _ = frame_rgb.shape
            x, y, w, h = int(bboxC.xmin * iw), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)
            face_coordinates = (x, y, w, h)
        return face_coordinates
    
    def _filter_stable_gesture(self, gesture):
        MIN_GESTURE_FRAMES = 12  # frames necessários para confirmar mudança

        # Inicializa variáveis persistentes
        if not hasattr(self, "gesture_frame_count"):
            self.gesture_frame_count = 0
            self.last_gesture = "normal"
            self.stable_gesture = "normal"

        # Caso não haja gesto detectado
        if gesture is None or gesture == "normal":
            # Se já está "normal", zera contagem
            if self.stable_gesture == "normal":
                self.gesture_frame_count = 0
            else:
                # Conta frames sem gesto; se passar do limite, volta ao normal
                self.gesture_frame_count += 1
                if self.gesture_frame_count >= MIN_GESTURE_FRAMES:
                    self.stable_gesture = "normal"
                    self.last_gesture = "normal"
                    self.gesture_frame_count = 0
            return self.stable_gesture

        # Se detectou um gesto novo
        if gesture == self.last_gesture:
            # Mesmo gesto → soma contagem
            self.gesture_frame_count += 1
        else:
            # Gesto diferente → reinicia contagem
            self.gesture_frame_count = 1
            self.last_gesture = gesture

        # Só muda se mantiver o mesmo gesto por frames suficientes
        if self.gesture_frame_count >= MIN_GESTURE_FRAMES:
            self.stable_gesture = gesture
            self.gesture_frame_count = 0

        return self.stable_gesture



    def _run_detection(self):
        self.running = True

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Erro ao capturar frame.")
                break

            frame = cv2.flip(frame, 1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Detecta rosto
            face_coords = self._detect_face(frame_rgb)

            # Se não tem rosto, pode liberar lock após tempo ocioso
            current_time = time.time()
            if face_coords:
                # if not self.locked:
                #     # Primeira detecção → trava
                #     self.locked = True
                #     self.last_face_coords = face_coords
                #     self.last_face_time = current_time
                # else:
                #     # Já tem lock → apenas atualiza tempo
                self.last_face_time = current_time
            else:
                # # Se perdeu o rosto por mais de 3s, reseta lock
                # if self.locked and (current_time - self.last_face_time > 3):
                #     print("Lock liberado: rosto sumiu.")
                #     self.locked = False
                self.last_face_coords = None

            # Detecta gesto
            mao_resultado = self.hands.process(frame_rgb)
            gesture, hand_pos_x = self._detect_gesture(mao_resultado)

            # Aplica filtragem de estabilidade
            stable_gesture = self._filter_stable_gesture(gesture)

            # Se há lock ativo, ignora gestos de desbloqueio de outros
            # if self.locked:
            #     if stable_gesture == "desbloquear":
            #         print("Gesto de desbloqueio detectado! Liberando lock.")
            #         self.locked = False
            #         continue  # volta ao início do loop
            # else:
            #     # Se não tem lock, ignora todos os gestos até travar de novo
            #     if not face_coords:
            #         continue

            # Só envia se gesto foi estável
            if stable_gesture:
                self.data_queue.put({'face': face_coords, 'gesture': stable_gesture})

        # Liberação de recursos
        self.cap.release()
        self.hands.close()
        self.face_detection.close()
        print("Thread de visão finalizada e recursos liberados.")

    # def _run_detection(self):
    #     self.running = True
    #     frame_id = 0  # contador de frames

    #     while self.running:
    #         ret, frame = self.cap.read()
    #         if not ret:
    #             print("Erro ao capturar frame.")
    #             break

    #         # frame_id += 1
    #         # if frame_id % 2 != 0:  # <<--- AQUI ele processa só 1 a cada 2 frames
    #         #     continue

    #         frame = cv2.flip(frame, 1)
    #         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    #         # 1. Processa o rosto
    #         face_coords = self._detect_face(frame_rgb)

    #         # 2. Processa as mãos UMA ÚNICA VEZ
    #         mao_resultado = self.hands.process(frame_rgb)
            
    #         # 3. Detecta o gesto usando o resultado anterior
    #         gesture, hand_pos_x = self._detect_gesture(mao_resultado)
            
    #         # 4. Desenha as marcações usando o mesmo resultado
    #         # if mao_resultado.multi_hand_landmarks:
    #         #     for hand_landmarks in mao_resultado.multi_hand_landmarks:
    #         #         mp_drawing.draw_landmarks(
    #         #             frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

    #         if face_coords:
    #             x, y, w, h = face_coords
    #             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
    #         # 5. Mostra a janela de debug
    #         # cv2.imshow('Debug da Câmera', frame)
    #         # if cv2.waitKey(1) & 0xFF == ord('q'):
    #         #     self.running = False
    #         #     break
            
    #         # 6. Envia os dados para o Arduino (se ativo)
    #         # self._send_to_arduino(gesture, hand_pos_x)

    #         # 7. Envia os dados para o display
    #         self.data_queue.put({'face': face_coords, 'gesture': gesture})
        
    #     # Limpeza ao sair do loop
    #     self.cap.release()
    #     self.hands.close()
    #     self.face_detection.close()
    #     # if self.arduino:
    #     #     self.arduino.close()
    #     print("Thread de visão finalizada e recursos liberados.")

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join() # Espera a thread terminar