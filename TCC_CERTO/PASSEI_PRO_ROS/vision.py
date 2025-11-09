#!/usr/bin/env python3

import cv2
import mediapipe as mp
import threading

from mediapipe.python.solutions import drawing_utils as mp_drawing

class VisionController:
    def __init__(self, data_queue):
        self.last_send_time = 0
        self.data_queue = data_queue # Fila para comunicar com a thread de display
        self.running = False
        self.thread = threading.Thread(target=self._run_detection)

        # Controle de gesto consistente
        self.last_gesture = None

        # --- Controle de duração de bravo/irritado ---
        self.bravo_irritado_counter = 0
        self.bravo_irritado_state = None

        # Inicializa a câmera
        self.cap = cv2.VideoCapture(0)
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

    #detecção mao
    def _detect_gesture(self, mao_resultado):
        gesto = "normal"

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
                gesto = "irritado"
            elif dedos_estendidos == 1:
                gesto = "triste"
            elif dedos_estendidos == 3:
                gesto = "confuso"
            else:
                gesto = "normal"

        return gesto

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
    
    # filtro de gesto estavel
    def _filter_stable_gesture(self, gesture):
        MIN_GESTURE_FRAMES = 10  # frames necessários para confirmar mudança

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

    # --- BRAVO ↔ IRRITADO ---
    def _update_bravo_irritado(self, gesture):
        """Se o gesto for bravo ou irritado, alterna entre eles a cada 116 frames."""
        FRAME_THRESHOLD = 116

        self.bravo_irritado_counter += 1

        # Se nunca iniciou estado, começa com o atual
        if self.bravo_irritado_state is None:
            self.bravo_irritado_state = gesture

        # Quando passa o limite → alterna
        if self.bravo_irritado_counter >= FRAME_THRESHOLD:
            self.bravo_irritado_counter = 0
            self.bravo_irritado_state = (
                "irritado" if self.bravo_irritado_state == "bravo" else "bravo"
            )

        # Retorna o estado atual alternado
        return self.bravo_irritado_state

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

            # Detecta gesto
            mao_resultado = self.hands.process(frame_rgb)
            gesture = self._detect_gesture(mao_resultado)

            # Aplica filtragem de estabilidade
            stable_gesture = self._filter_stable_gesture(gesture)

            if stable_gesture in ["bravo", "irritado"]:
                stable_gesture = self._update_bravo_irritado(stable_gesture)

            # Só envia se gesto foi estável
            if stable_gesture:
                self.data_queue.put({'face': face_coords, 'gesture': stable_gesture})


        # Liberação de recursos
        self.cap.release()
        self.hands.close()
        self.face_detection.close()
        print("Thread de visão finalizada e recursos liberados.")

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join() # Espera a thread terminar