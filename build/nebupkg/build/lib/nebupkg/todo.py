
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import speech_recognition as sr
import threading
import signal
import sys
import os
from std_msgs.msg import String
from std_msgs.msg import Bool

class WakeWordNode(Node):
    def __init__(self):
        super().__init__('wakeword_node')
        
        # Publishers
        self.status_publisher = self.create_publisher(String, '/wakeword/status', 10)
        self.active_publisher = self.create_publisher(Bool, '/wakeword/active', 10)
        
        # Palabras de activación
        self.wake_words = ["nebu","Hola", "despertar", "activar", "hola nebu", "hola"]
        
        # Palabras de desactivación
        self.sleep_words = ["adiós", "adios", "dormir", "desactivar", "hasta luego", "bye"]
        
        # Comandos de los nodos
        self.comandos = [
        "source /opt/ros/humble/setup.bash && ros2 run nebupkg emociones",
        "source /opt/ros/humble/setup.bash && ros2 run nebupkg rostros",
        "source /opt/ros/humble/setup.bash && ros2 run nebupkg model_node",
        "source /opt/ros/humble/setup.bash && ros2 run nebupkg voz_node",
        "source /opt/ros/humble/setup.bash && ros2 run nebupkg nodotts",

        ]
        
        # Control de estado
        self.nodos_activos = False
        self.procesos = []
        self.escuchando = True
        
        # Configuración del reconocimiento de voz
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Publicar estado inicial
        self.publish_status("Sistema iniciado")
        self.publish_active_status(False)
        
        # Iniciar calibración
        self.calibrar_microfono()
        
        # Iniciar hilo de escucha
        self.audio_thread = threading.Thread(target=self.escuchar_continuamente)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.get_logger().info("Wake Word Node iniciado correctamente")
        
    def calibrar_microfono(self):
        """Calibra el micrófono para ruido ambiente"""
        self.get_logger().info("Calibrando micrófono para ruido ambiente...")
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.get_logger().info("Calibración completada. Sistema listo.")
            self.publish_status("Sistema calibrado y listo")
        except Exception as e:
            self.get_logger().error(f"Error en calibración: {e}")
            
    def publish_status(self, mensaje):
        """Publica el estado del sistema"""
        msg = String()
        msg.data = mensaje
        self.status_publisher.publish(msg)
        
    def publish_active_status(self, activo):
        """Publica si los nodos están activos"""
        msg = Bool()
        msg.data = activo
        self.active_publisher.publish(msg)
        
    def iniciar_nodos(self):
        """Inicia todos los nodos ROS2"""
        if self.nodos_activos:
            self.get_logger().info("Los nodos ya están activos.")
            return
            
        self.get_logger().info("🚀 Activando nodos...")
        self.publish_status("Activando nodos...")
        self.procesos = []
        
        for i, cmd in enumerate(self.comandos):
            try:
                proceso = subprocess.Popen(
                    ["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self.procesos.append(proceso)
                self.get_logger().info(f"✅ Nodo {i+1} iniciado")
                time.sleep(1)
                
            except Exception as e:
                self.get_logger().error(f"❌ Error iniciando nodo {i+1}: {e}")
        
        self.nodos_activos = True
        self.publish_active_status(True)
        self.publish_status("Nodos activados")
        self.get_logger().info("🎉 Todos los nodos están activos!")
        
    def detener_nodos(self):
        """Detiene todos los nodos ROS2"""
        if not self.nodos_activos:
            self.get_logger().info("Los nodos ya están inactivos.")
            return
            
        self.get_logger().info("🛑 Deteniendo nodos...")
        self.publish_status("Deteniendo nodos...")
        
        try:
            subprocess.run(["pkill", "-f", "nebupkg"], check=False)
            subprocess.run(["pkill", "-f", "gnome-terminal"], check=False)
        except Exception as e:
            self.get_logger().error(f"Error cerrando terminales: {e}")
        
        for proceso in self.procesos:
            try:
                proceso.terminate()
                proceso.wait(timeout=3)
            except:
                try:
                    proceso.kill()
                except:
                    pass
        
        self.procesos = []
        self.nodos_activos = False
        self.publish_active_status(False)
        self.publish_status("Nodos detenidos")
        self.get_logger().info("✅ Nodos detenidos correctamente")
        
    def procesar_comando(self, texto: str):
        """Procesa el texto reconocido y ejecuta acciones"""
        texto_lower = texto.lower().strip()
        self.get_logger().info(f"Comando detectado: '{texto}'")
        
        # Verificar palabras de activación
        for wake_word in self.wake_words:
            if wake_word.lower() in texto_lower:
                if not self.nodos_activos:
                    self.iniciar_nodos()
                else:
                    self.get_logger().info("Los nodos ya están activos.")
                return
        
        # Verificar palabras de desactivación
        for sleep_word in self.sleep_words:
            if sleep_word.lower() in texto_lower:
                if self.nodos_activos:
                    self.detener_nodos()
                else:
                    self.get_logger().info("Los nodos ya están inactivos.")
                return
                
    def escuchar_continuamente(self):
        """Función principal de escucha continua"""
        self.get_logger().info("🎤 Sistema de wake word activo. Esperando comandos...")
        self.get_logger().info(f"Palabras de activación: {', '.join(self.wake_words)}")
        self.get_logger().info(f"Palabras de desactivación: {', '.join(self.sleep_words)}")
        
        while self.escuchando and rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                
                try:
                    texto = self.recognizer.recognize_google(audio, language='es-ES')
                    self.procesar_comando(texto)
                    
                except sr.UnknownValueError:
                    pass
                except sr.RequestError as e:
                    self.get_logger().error(f"Error en el servicio de reconocimiento: {e}")
                    time.sleep(1)
                    
            except sr.WaitTimeoutError:
                pass
            except Exception as e:
                self.get_logger().error(f"Error inesperado: {e}")
                time.sleep(1)
                
    def destruir_nodo(self):
        """Limpia recursos al destruir el nodo"""
        self.escuchando = False
        self.detener_nodos()
        
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WakeWordNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destruir_nodo()
        rclpy.shutdown()

if __name__ == '__main__':
    main()