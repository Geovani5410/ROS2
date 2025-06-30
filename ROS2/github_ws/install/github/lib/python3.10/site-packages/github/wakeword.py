#!/usr/bin/env python3
import subprocess
import time
import speech_recognition as sr
import threading
import signal
import sys
import os
from typing import List, Dict, Optional

class WakeWordNodeManager:
    def __init__(self):
        # Palabras de activación (puedes personalizar estas)
        self.wake_words = ["nebu", "despertar", "activar", "hola"]
        
        # Palabras de desactivación
        self.sleep_words = ["adiós", "adios", "dormir", "desactivar", "hasta luego", "bye"]
        
        # Comandos de los nodos
        self.comandos = [
            "source /opt/ROS2/github_ws/setup.bash && ros2 run github git",
        ]
        
        # Control de estado
        self.nodos_activos = False
        self.procesos = []
        self.escuchando = True
        
        # Configuración del reconocimiento de voz
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Ajustar para ruido ambiente
        print("Calibrando micrófono para ruido ambiente...")
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
        print("Calibración completada. Sistema listo.")
        
    def iniciar_nodos(self):
        """Inicia todos los nodos ROS2"""
        if self.nodos_activos:
            print("Los nodos ya están activos.")
            return
            
        print("🚀 Activando nodos...")
        self.procesos = []
        
        for i, cmd in enumerate(self.comandos):
            try:
                # Ejecutar cada comando en una terminal separada
                proceso = subprocess.Popen(
                    ["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self.procesos.append(proceso)
                print(f"✅ Nodo {i+1} iniciado")
                time.sleep(1)  # Pequeña pausa entre inicios
                
            except Exception as e:
                print(f"❌ Error iniciando nodo {i+1}: {e}")
        
        self.nodos_activos = True
        print("🎉 Todos los nodos están activos!")
        
    def detener_nodos(self):
        """Detiene todos los nodos ROS2"""
        if not self.nodos_activos:
            print("Los nodos ya están inactivos.")
            return
            
        print("🛑 Deteniendo nodos...")
        
        # Intentar cerrar las terminales de gnome-terminal
        try:
            subprocess.run(["pkill", "-f", "nebupkg"], check=False)
            subprocess.run(["pkill", "-f", "gnome-terminal"], check=False)
        except Exception as e:
            print(f"Error cerrando terminales: {e}")
        
        # Limpiar procesos
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
        print("✅ Nodos detenidos correctamente")
        
    def procesar_comando(self, texto: str) -> bool:
        """Procesa el texto reconocido y ejecuta acciones"""
        texto_lower = texto.lower().strip()
        print(f"Comando detectado: '{texto}'")
        
        # Verificar palabras de activación
        for wake_word in self.wake_words:
            if wake_word.lower() in texto_lower:
                if not self.nodos_activos:
                    self.iniciar_nodos()
                    return True
                else:
                    print("Los nodos ya están activos.")
                    return True
        
        # Verificar palabras de desactivación
        for sleep_word in self.sleep_words:
            if sleep_word.lower() in texto_lower:
                if self.nodos_activos:
                    self.detener_nodos()
                    return True
                else:
                    print("Los nodos ya están inactivos.")
                    return True
        
        # Comando especial para salir del programa
        if any(word in texto_lower for word in ["salir del sistema", "cerrar programa", "exit"]):
            print("Cerrando sistema...")
            self.detener_nodos()
            self.escuchando = False
            return False
            
        return True
        
    def escuchar_continuamente(self):
        """Función principal de escucha continua"""
        print("🎤 Sistema de wake word activo. Esperando comandos...")
        print(f"Palabras de activación: {', '.join(self.wake_words)}")
        print(f"Palabras de desactivación: {', '.join(self.sleep_words)}")
        print("Di 'salir del sistema' para cerrar el programa.\n")
        
        while self.escuchando:
            try:
                # Escuchar audio del micrófono
                with self.microphone as source:
                    # Timeout corto para permitir interrupciones
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                
                try:
                    # Reconocer usando Google Speech Recognition
                    texto = self.recognizer.recognize_google(audio, language='es-ES')
                    
                    if not self.procesar_comando(texto):
                        break
                        
                except sr.UnknownValueError:
                    # No se pudo entender el audio, continuar escuchando
                    pass
                except sr.RequestError as e:
                    print(f"Error en el servicio de reconocimiento: {e}")
                    time.sleep(1)
                    
            except sr.WaitTimeoutError:
                # Timeout normal, continuar escuchando
                pass
            except KeyboardInterrupt:
                print("\n🛑 Interrupción detectada. Cerrando sistema...")
                break
            except Exception as e:
                print(f"Error inesperado: {e}")
                time.sleep(1)
        
        # Limpiar al salir
        self.detener_nodos()
        print("👋 Sistema cerrado correctamente.")

def signal_handler(sig, frame):
    """Manejador de señales para cierre limpio"""
    print('\n🛑 Señal de interrupción recibida. Cerrando...')
    sys.exit(0)

def main():
    """Función principal"""
    # Configurar manejador de señales
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Crear y ejecutar el sistema
        manager = WakeWordNodeManager()
        manager.escuchar_continuamente()
        
    except Exception as e:
        print(f"Error fatal: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()