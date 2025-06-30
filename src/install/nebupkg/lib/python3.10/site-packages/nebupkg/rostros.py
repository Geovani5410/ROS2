#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import pickle
import os
import time
from datetime import datetime
import face_recognition
import threading

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        
        # Publisher para enviar el nombre reconocido
        self.name_publisher = self.create_publisher(String, 'recognized_person', 10)
        
        # Timer para ejecutar reconocimiento periódicamente
        self.timer = self.create_timer(3.0, self.recognition_callback)
        
        # Variables del sistema de reconocimiento
        self.known_faces = []
        self.known_names = []
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.data_file = 'face_data.pkl'
        self.recognition_threshold = 0.6
        
        # Variables de control
        self.cap = None
        self.last_recognized_person = ""
        self.last_recognition_time = 0
        self.registering = False  # Flag para modo registro
        
        # Inicializar cámara
        self.init_camera()
        
        # Cargar rostros conocidos
        self.load_known_faces()
        
        self.get_logger().info('🎥 Nodo de Reconocimiento Facial iniciado')
        self.get_logger().info(f'📊 Rostros conocidos cargados: {len(self.known_names)}')
        
        # Iniciar thread para comandos interactivos
        self.command_thread = threading.Thread(target=self.command_interface, daemon=True)
        self.command_thread.start()
    
    def init_camera(self):
        """Inicializar la cámara"""
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error('❌ No se puede abrir la cámara')
                return False
            
            # Configurar resolución
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            self.get_logger().info('✅ Cámara inicializada correctamente')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Error inicializando cámara: {e}')
            return False
    
    def load_known_faces(self):
        """Cargar rostros conocidos desde archivo"""
        if os.path.exists(self.data_file):
            try:
                with open(self.data_file, 'rb') as f:
                    data = pickle.load(f)
                    self.known_faces = data['faces']
                    self.known_names = data['names']
                self.get_logger().info(f'✅ Cargados {len(self.known_names)} rostros conocidos: {self.known_names}')
            except Exception as e:
                self.get_logger().error(f'❌ Error cargando datos: {e}')
        else:
            self.get_logger().info('📁 No hay archivo de datos, iniciando con base vacía')
    
    def save_known_faces(self):
        """Guardar rostros conocidos en archivo"""
        try:
            data = {
                'faces': self.known_faces,
                'names': self.known_names
            }
            with open(self.data_file, 'wb') as f:
                pickle.dump(data, f)
            self.get_logger().info(f'💾 Datos guardados: {len(self.known_names)} rostros')
        except Exception as e:
            self.get_logger().error(f'❌ Error guardando datos: {e}')
    
    def check_liveness(self, frame):
        """Verificación básica de liveness"""
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            
            if len(faces) > 0:
                for (x, y, w, h) in faces:
                    face_region = gray[y:y+h, x:x+w]
                    variance = cv2.Laplacian(face_region, cv2.CV_64F).var()
                    return variance > 100  # Umbral para detectar textura real
            return False
        except Exception as e:
            self.get_logger().error(f'❌ Error en liveness detection: {e}')
            return False
    
    def recognize_face(self, frame):
        """Reconocer rostro en el frame"""
        try:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            face_locations = face_recognition.face_locations(rgb_frame)
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
            
            names = []
            for face_encoding in face_encodings:
                matches = face_recognition.compare_faces(self.known_faces, face_encoding, tolerance=self.recognition_threshold)
                name = "Desconocido"
                
                if True in matches:
                    face_distances = face_recognition.face_distance(self.known_faces, face_encoding)
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = self.known_names[best_match_index]
                
                names.append(name)
            
            return face_locations, names
        except Exception as e:
            self.get_logger().error(f'❌ Error en reconocimiento: {e}')
            return [], []
    
    def register_new_face(self, frame, name):
        """Registrar un nuevo rostro"""
        try:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            face_locations = face_recognition.face_locations(rgb_frame)
            
            if len(face_locations) == 1:  # Solo un rostro en la imagen
                face_encoding = face_recognition.face_encodings(rgb_frame, face_locations)[0]
                self.known_faces.append(face_encoding)
                self.known_names.append(name)
                self.save_known_faces()
                self.get_logger().info(f'✅ Rostro de {name} registrado exitosamente')
                return True
            elif len(face_locations) == 0:
                self.get_logger().warning('⚠️ No se detectó ningún rostro para registro')
                return False
            else:
                self.get_logger().warning(f'⚠️ Se detectaron {len(face_locations)} rostros, se necesita exactamente 1')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ Error registrando rostro: {e}')
            return False
    
    def command_interface(self):
        """Interfaz de comandos para registro"""
        print("\n" + "="*50)
        print("🎥 SISTEMA DE RECONOCIMIENTO FACIAL")
        print("="*50)
        print("Comandos disponibles:")
        print("  'r' + Enter  → Registrar nueva persona")
        print("  'l' + Enter  → Listar personas registradas")
        print("  'q' + Enter  → Salir del programa")
        print("="*50 + "\n")
        
        while rclpy.ok():
            try:
                comando = input("👤 Ingresa comando (r/l/q): ").strip().lower()
                
                if comando == 'q':
                    print("👋 Cerrando sistema...")
                    rclpy.shutdown()
                    break
                    
                elif comando == 'l':
                    print(f"\n📋 Personas registradas ({len(self.known_names)}):")
                    if self.known_names:
                        for i, name in enumerate(self.known_names, 1):
                            print(f"  {i}. {name}")
                    else:
                        print("  (Ninguna persona registrada)")
                    print()
                    
                elif comando == 'r':
                    nombre = input("📝 Nombre de la persona: ").strip()
                    if not nombre:
                        print("❌ El nombre no puede estar vacío\n")
                        continue
                        
                    if nombre in self.known_names:
                        print(f"⚠️  {nombre} ya está registrado\n")
                        continue
                    
                    if self.cap is None or not self.cap.isOpened():
                        print("❌ Cámara no disponible\n")
                        continue
                    
                    print(f"📸 Registrando a {nombre}...")
                    print("   1. Posiciónate frente a la cámara")
                    print("   2. Asegúrate de que solo tu rostro sea visible")
                    print("   3. Presiona Enter cuando estés listo...")
                    
                    input("   Presiona Enter para capturar → ")
                    
                    self.registering = True
                    print("📷 Capturando...")
                    
                    best_frame = None
                    max_face_size = 0
                    
                    for i in range(10):
                        ret, frame = self.cap.read()
                        if ret:
                            frame = cv2.flip(frame, 1)
                            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            face_locations = face_recognition.face_locations(rgb_frame)
                            
                            if len(face_locations) == 1:
                                top, right, bottom, left = face_locations[0]
                                face_size = (bottom - top) * (right - left)
                                
                                if face_size > max_face_size:
                                    max_face_size = face_size
                                    best_frame = frame.copy()
                        
                        time.sleep(0.1)
                    
                    self.registering = False
                    
                    if best_frame is not None:
                        if self.register_new_face(best_frame, nombre):
                            print(f"✅ {nombre} registrado correctamente!")
                        else:
                            print("❌ Error en el registro")
                    else:
                        print("❌ No se pudo capturar un rostro válido")
                    
                    print()
                    
                else:
                    print("❌ Comando no reconocido. Usa 'r', 'l' o 'q'\n")
                    
            except KeyboardInterrupt:
                print("\n👋 Cerrando sistema...")
                rclpy.shutdown()
                break
            except Exception as e:
                print(f"❌ Error: {e}")
    
    def recognition_callback(self):
        """Callback principal para reconocimiento"""
        if self.cap is None or not self.cap.isOpened() or self.registering:
            return
        
        try:
            ret, frame = self.cap.read()
            if not ret:
                return
            
            frame = cv2.flip(frame, 1)
            
            if not self.check_liveness(frame):
                return
            
            face_locations, names = self.recognize_face(frame)
            
            current_time = time.time()
            
            for name in names:
                if name != "Desconocido":
                    if name != self.last_recognized_person:
                        msg = String()
                        msg.data = name
                        self.name_publisher.publish(msg)
                        
                        timestamp = datetime.now().strftime("%H:%M:%S")
                        self.get_logger().info(f'👤 [{timestamp}] Reconocido: {name}')
                        
                        self.last_recognized_person = name
                        self.last_recognition_time = current_time
                        
                        break
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en callback de reconocimiento: {e}')
    
    def destroy_node(self):
        """Limpiar recursos al cerrar"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FaceRecognitionNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

