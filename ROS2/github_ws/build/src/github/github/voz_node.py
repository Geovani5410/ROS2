import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import whisper
import time
import os
import serial  # Para comunicación serial

SAMPLE_RATE = 16000
THRESHOLD = 1.0
MAX_SEGMENT_DURATION = 1000

whisper_model = whisper.load_model("base")

def detectar_voz(audio, threshold=THRESHOLD):
    return np.any(np.abs(audio) > threshold)

class VoiceTranscriberNode(Node):
    def __init__(self):
        super().__init__('voice_transcriber_node')
        self.publisher_ = self.create_publisher(String, 'transcripcion_voz', 10)
        self.subscription = self.create_subscription(Bool, 'tts_complete', self.mic_control_callback, 10)
        self.get_logger().info('🎤 Nodo de transcripción de voz con Whisper iniciado')

        self.timer = self.create_timer(0.1, self.escuchar_continuamente)

        self.escuchando = False
        self.audio_buffer = []
        self.inicio_segmento = None
        self.tiempo_inicio_silencio = 0
        self.mic_enabled = False

        # Conexión serial con Arduino en Ubuntu (/dev/ttyACM0)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # dev0 para Arduino
            #self.serial_port = None
            time.sleep(2)
            self.get_logger().info("📡 Conectado al Arduino por serial en /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"🚫 Error al conectar con Arduino: {e}")
            self.serial_port = None

    def mic_control_callback(self, msg):
        self.mic_enabled = msg.data
        estado = "🟢 activado" if self.mic_enabled else "🔴 desactivado"
        self.get_logger().info(f'🎙️ Micrófono {estado} por señal externa')

    def escuchar_continuamente(self):
        if not self.mic_enabled:
            return

        data = sd.rec(int(0.5 * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()

        if detectar_voz(data):
            if not self.escuchando:
                self.get_logger().info("🔊 Voz detectada, grabando...")
                self.escuchando = True
                self.inicio_segmento = time.time()
                self.audio_buffer = []

                # Enviar señal al Arduino para encender LED
                if self.serial_port:
                    self.serial_port.write(b'ON\n')

            self.audio_buffer.append(data)
            self.tiempo_inicio_silencio = 0
        elif self.escuchando:
            if self.tiempo_inicio_silencio == 0:
                self.tiempo_inicio_silencio = time.time()
            elif time.time() - self.tiempo_inicio_silencio > 1.5:
                self.get_logger().info("🛑 Silencio detectado. Procesando transcripción...")
                self.finalizar_grabacion()

        if self.escuchando and (time.time() - self.inicio_segmento > MAX_SEGMENT_DURATION):
            self.get_logger().warn("⏱️ Tiempo máximo alcanzado. Procesando transcripción...")
            self.finalizar_grabacion()

    def finalizar_grabacion(self):
        self.escuchando = False
        self.tiempo_inicio_silencio = 0

        # Apagar LED
        if self.serial_port:
            self.serial_port.write(b'OFF\n')

        if self.audio_buffer:
            audio_final = np.concatenate(self.audio_buffer, axis=0)
            wav.write("grabacion.wav", SAMPLE_RATE, audio_final)
            self.transcribir_con_whisper("grabacion.wav")
            os.remove("grabacion.wav")
            self.audio_buffer = []

    def transcribir_con_whisper(self, ruta_audio):
        try:
            result = whisper_model.transcribe(ruta_audio, language="es")
            texto = result["text"].strip()
            if texto:
                self.get_logger().info(f"📝 Transcripción: {texto}")
                self.publisher_.publish(String(data=texto))
            else:
                self.get_logger().warn("🤷 Whisper no detectó contenido claro.")
        except Exception as e:
            self.get_logger().error(f"🚫 Error al transcribir con Whisper: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTranscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

