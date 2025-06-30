import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import numpy as np
from collections import deque

class EmotionDetector(Node):
    def __init__(self):
        super().__init__('detector_emociones')
        self.publisher_ = self.create_publisher(String, 'emocion_actual', 10)
        self.ultima_emocion = ""
        self.emociones_ultimas = deque(maxlen=7)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.get_logger().info("Detector de emociones iniciado. Presiona ESC para salir.")
        self.get_logger().info("Emociones detectables: Feliz, Triste, Enojado, Sorprendido, Neutro")
        self.timer = self.create_timer(0.03, self.detectar_emocion_en_frame)

    def detectar_emocion_en_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Error: No se puede leer de la cámara")
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb)

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                emocion, _ = self.analizar_emocion(face_landmarks.landmark, frame.shape)
                emocion_suave = self.suavizar_emocion(emocion)
                if emocion_suave != self.ultima_emocion:
                    self.ultima_emocion = emocion_suave
                    self.publicar_emocion(emocion_suave)
                cv2.putText(frame, f"Emocion: {emocion_suave}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        else:
            cv2.putText(frame, "No se detecta rostro", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Detector de Emociones", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def publicar_emocion(self, emocion):
        msg = String()
        msg.data = emocion
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicada emoción: {emocion}')

    def suavizar_emocion(self, emocion):
        self.emociones_ultimas.append(emocion)
        contador = {}
        for em in self.emociones_ultimas:
            contador[em] = contador.get(em, 0) + 1
        return max(contador.items(), key=lambda x: x[1])[0]

    def distancia(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def analizar_emocion(self, landmarks, shape):
        h, w = shape[:2]
        def punto(idx): return [landmarks[idx].x * w, landmarks[idx].y * h]

        labio_sup_centro, labio_inf_centro = punto(13), punto(14)
        comisura_izq, comisura_der = punto(78), punto(308)
        ojo_izq_sup, ojo_izq_inf = punto(159), punto(145)
        ojo_der_sup, ojo_der_inf = punto(386), punto(374)
        ojo_izq_izq, ojo_izq_der = punto(33), punto(133)
        ojo_der_izq, ojo_der_der = punto(362), punto(263)
        ceja_izq_interior, ceja_izq_centro = punto(70), punto(107)
        ceja_der_interior, ceja_der_centro = punto(300), punto(336)

        apertura_boca = self.distancia(labio_sup_centro, labio_inf_centro)
        ancho_boca = self.distancia(comisura_izq, comisura_der)
        curvatura_sonrisa = ((labio_inf_centro[1] - comisura_izq[1]) +
                             (labio_inf_centro[1] - comisura_der[1])) / 2
        apertura_ojo_izq = self.distancia(ojo_izq_sup, ojo_izq_inf)
        apertura_ojo_der = self.distancia(ojo_der_sup, ojo_der_inf)
        apertura_ojos_promedio = (apertura_ojo_izq + apertura_ojo_der) / 2
        ancho_ojo_izq = self.distancia(ojo_izq_izq, ojo_izq_der)
        ancho_ojo_der = self.distancia(ojo_der_izq, ojo_der_der)
        ancho_ojos_promedio = (ancho_ojo_izq + ancho_ojo_der) / 2
        altura_cejas = (self.distancia(ceja_izq_centro, ojo_izq_sup) +
                        self.distancia(ceja_der_centro, ojo_der_sup)) / 2
        distancia_entre_cejas = self.distancia(ceja_izq_interior, ceja_der_interior)

        # Ratios
        ratio_apertura_boca = apertura_boca / ancho_boca if ancho_boca > 0 else 0
        ratio_apertura_ojos = apertura_ojos_promedio / ancho_ojos_promedio if ancho_ojos_promedio > 0 else 0
        ratio_curvatura_sonrisa = curvatura_sonrisa / ancho_boca if ancho_boca > 0 else 0
        ratio_altura_cejas = altura_cejas / ancho_boca if ancho_boca > 0 else 0
        ratio_distancia_cejas = distancia_entre_cejas / ancho_boca if ancho_boca > 0 else 0

        # Umbrales
        if (ratio_apertura_ojos > 0.35 and ratio_apertura_boca > 0.4 and ratio_altura_cejas > 0.25):
            return "Sorprendido", {}
        if (ratio_curvatura_sonrisa > 0.02 and ratio_apertura_boca < 0.6):
            return "Feliz", {}
        if (ratio_distancia_cejas < 1.8 and ratio_altura_cejas < 0.18 and ratio_apertura_ojos < 0.25):
            return "Enojado", {}
        if (ratio_curvatura_sonrisa < -0.015 and ratio_altura_cejas < 0.15):
            return "Triste", {}
        return "Neutro", {}

def main(args=None):
    rclpy.init(args=args)
    detector = EmotionDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
