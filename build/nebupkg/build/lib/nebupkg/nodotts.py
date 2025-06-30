#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import requests
import pygame
import io
import threading

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        # Configuración de ElevenLabs
        self.API_KEY = "sk_3267e616e69b75556e9bb7b152df9da115b213911bb65abc"
        self.VOICE_ID = "8tm9IYjg8ybDoxe8w6Rk"
        self.TTS_URL = f"https://api.elevenlabs.io/v1/text-to-speech/{self.VOICE_ID}"

        # Subscriptor para solicitudes de TTS
        self.tts_request_subscriber = self.create_subscription(
            String, 'tts_request', self.tts_request_callback, 10)

        # Publicadores
        self.tts_status_publisher = self.create_publisher(Bool, 'tts_status', 10)
        self.mic_control_publisher = self.create_publisher(Bool, 'mic_control', 10)

        # Inicializar pygame mixer
        pygame.mixer.init()

        self.get_logger().info('🔊 Nodo TTS iniciado')

    def tts_request_callback(self, msg):
        """Callback para procesar solicitudes de TTS"""
        text = msg.data.strip()
        self.get_logger().info(f'📥 Solicitud TTS recibida: "{text}"')
        
        # Procesar TTS en un hilo separado para no bloquear
        threading.Thread(target=self.process_tts, args=(text,), daemon=True).start()

    def process_tts(self, text):
        """Procesa el texto a voz y lo reproduce"""
        try:
            # Señalar que TTS ha comenzado
            self.signal_tts_start()
            
            # Generar audio
            audio_data = self.text_to_speech(text)
            
            if audio_data:
                # Reproducir audio
                self.play_audio(audio_data)
                self.get_logger().info(f'✅ TTS completado para: "{text}"')
            else:
                self.get_logger().error(f'❌ Error generando TTS para: "{text}"')
            
            # Señalar que TTS ha terminado
            self.signal_tts_complete()
            
        except Exception as e:
            self.get_logger().error(f'Error en proceso TTS: {e}')
            self.signal_tts_complete()

    def text_to_speech(self, text):
        """Convierte texto a voz usando ElevenLabs API"""
        try:
            headers = {
                "Accept": "audio/mpeg",
                "Content-Type": "application/json",
                "xi-api-key": self.API_KEY
            }
            data = {
                "text": text,
                "model_id": "eleven_monolingual_v1",
                "voice_settings": {
                    "stability": 0.5,
                    "similarity_boost": 0.75
                }
            }
            
            self.get_logger().info('🌐 Enviando solicitud a ElevenLabs...')
            response = requests.post(self.TTS_URL, json=data, headers=headers)
            
            if response.status_code == 200:
                self.get_logger().info('✅ Audio generado exitosamente')
                return response.content
            else:
                self.get_logger().error(f'Error API ElevenLabs: {response.status_code}')
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error TTS: {e}")
            return None

    def play_audio(self, audio_data):
        """Reproduce el audio usando pygame"""
        try:
            self.get_logger().info('🎵 Reproduciendo audio...')
            audio_io = io.BytesIO(audio_data)
            pygame.mixer.music.load(audio_io)
            pygame.mixer.music.play()
            
            # Esperar hasta que termine la reproducción
            while pygame.mixer.music.get_busy():
                pygame.time.wait(100)
                
            self.get_logger().info('🎵 Reproducción terminada')
            
        except Exception as e:
            self.get_logger().error(f'Error al reproducir audio: {e}')

    def signal_tts_start(self):
        """Señala que TTS ha comenzado"""
        self.tts_status_publisher.publish(Bool(data=False))  # TTS ocupado
        self.mic_control_publisher.publish(Bool(data=False))  # Desactivar micrófono
        self.get_logger().info('📡 TTS iniciado → Micrófono desactivado')

    def signal_tts_complete(self):
        """Señala que TTS ha terminado"""
        self.tts_status_publisher.publish(Bool(data=True))   # TTS libre
        self.mic_control_publisher.publish(Bool(data=True))  # Activar micrófono
        self.get_logger().info('📡 TTS completado → Micrófono activado')

    def destroy_node(self):
        """Limpia recursos al destruir el nodo"""
        self.get_logger().info('🧹 Limpiando recursos TTS...')
        pygame.mixer.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n🛑 Interrupción por teclado')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print('👋 Nodo TTS cerrado')

if __name__ == '__main__':
    main()