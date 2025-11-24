import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import serial
import numpy as np
import matplotlib.animation as animation
from collections import deque
import time
import glob
from scipy import signal

class SignalProcessor:
    def __init__(self, port=None, baudrate=115200, buffer_size=2000):  # Buffer más grande
        if port is None:
            port = self.find_serial_port()
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            print(f"✓ Conectado al puerto {port}")
            print(f"✓ Configuración: {baudrate} bauds")
            
            self.ser.reset_input_buffer()
            time.sleep(1)
            
        except serial.SerialException as e:
            print(f"✗ Error conectando al puerto {port}: {e}")
            print("\nPuertos disponibles:")
            for p in self.list_serial_ports():
                print(f"  {p}")
            raise
        
        self.buffer_size = buffer_size
        self.data_buffer = deque(maxlen=buffer_size)
        self.time_buffer = deque(maxlen=buffer_size)
        self.start_time = time.time()
        
        # Configuración para FFT - AUMENTADA a 200 Hz
        self.sample_rate = 200  # Hz - DUPLICADA para ver hasta 100 Hz
        self.fft_size = 1024    # Tamaño mayor para mejor resolución
        self.max_frequency = 100  # Hz - Límite máximo de frecuencia a mostrar
        
        self.check_serial_connection()
    
    def find_serial_port(self):
        ports = self.list_serial_ports()
        if not ports:
            raise Exception("No se encontraron puertos seriales")
        
        print("Puertos seriales encontrados:")
        for port in ports:
            print(f"  {port}")
        
        for port in ports:
            if 'USB' in port or 'ACM' in port:
                print(f"Usando puerto: {port}")
                return port
        
        return ports[0]
    
    def list_serial_ports(self):
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyS*')
        return ports
    
    def check_serial_connection(self):
        print("Verificando conexión serial...")
        test_data = []
        start_time = time.time()
        timeout = 3
        
        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                try:
                    raw_line = self.ser.readline()
                    line = raw_line.decode('utf-8').strip()
                    value = float(line)
                    test_data.append(value)
                    print(f"Dato recibido: {value}")
                    
                    if len(test_data) >= 5:
                        break
                        
                except Exception as e:
                    continue
        
        if test_data:
            print(f"✓ Conexión OK - Se recibieron {len(test_data)} datos")
            return True
        else:
            print("✗ No se recibieron datos")
            return False
    
    def read_data(self):
        """Lee múltiples datos del buffer para reducir latencia"""
        try:
            data_points = []
            while self.ser.in_waiting > 0:
                raw_line = self.ser.readline()
                line = raw_line.decode('utf-8').strip()
                value = float(line)
                data_points.append(value)
            
            return data_points[-1] if data_points else None
            
        except Exception:
            return None
    
    def compute_fft(self, data):
        """Calcula la FFT para el espectro de frecuencia (0-100 Hz)"""
        if len(data) < self.fft_size:
            return None, None
            
        # Usar los últimos puntos para FFT
        signal_data = np.array(list(data))[-self.fft_size:]
        
        # Remover el valor DC (componente constante)
        signal_data = signal_data - np.mean(signal_data)
        
        # Aplicar ventana de Hann para reducir fugas espectrales
        window = np.hanning(len(signal_data))
        windowed_signal = signal_data * window
        
        # Calcular FFT
        fft_result = np.fft.fft(windowed_signal)
        fft_magnitude = np.abs(fft_result)[:self.fft_size//2]  # Solo frecuencias positivas
        frequencies = np.fft.fftfreq(self.fft_size, 1.0/self.sample_rate)[:self.fft_size//2]
        
        # Filtrar solo frecuencias de 0 a 100 Hz
        mask = frequencies <= self.max_frequency
        frequencies_filtered = frequencies[mask]
        fft_magnitude_filtered = fft_magnitude[mask]
        
        return frequencies_filtered, fft_magnitude_filtered
    
    def process_realtime(self):
        print("\n" + "="*50)
        print("INICIANDO VISUALIZACIÓN CON ESPECTRO DE FRECUENCIAS (0-100 Hz)")
        print("="*50)
        print(f"Frecuencia de muestreo: {self.sample_rate} Hz")
        print(f"Frecuencia máxima observable: {self.sample_rate/2} Hz")
        print("Presiona Ctrl+C en la ventana de gráfico para detener")
        
        # Crear 2 subplots: señal temporal y espectro de frecuencia
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        plt.subplots_adjust(hspace=0.4)
        
        self.data_count = 0
        self.last_print_time = time.time()
        
        def animate(frame):
            new_value = self.read_data()
            
            if new_value is not None:
                self.data_count += 1
                current_time = time.time() - self.start_time
                
                self.data_buffer.append(new_value)
                self.time_buffer.append(current_time)
                
                # Mostrar estadísticas cada 3 segundos
                if time.time() - self.last_print_time >= 3:
                    print(f"Datos recibidos: {self.data_count} - Buffer: {len(self.data_buffer)}")
                    self.last_print_time = time.time()
                
                if len(self.data_buffer) > 50:
                    data_array = np.array(self.data_buffer)
                    time_array = np.array(self.time_buffer)
                    
                    # Limpiar ambos plots en cada frame
                    ax1.clear()
                    ax2.clear()
                    
                    # 1. Plot señal temporal (original)
                    ax1.plot(time_array, data_array, 'b-', linewidth=1.0, label='Señal en bruto')
                    ax1.set_title(f'Señal Temporal - {len(data_array)} puntos - Fs: {self.sample_rate} Hz')
                    ax1.set_ylabel('Valor ADC')
                    ax1.legend()
                    ax1.grid(True, alpha=0.3)
                    
                    # 2. Plot FFT (Espectro de Frecuencia 0-100 Hz)
                    freqs, fft_mag = self.compute_fft(self.data_buffer)
                    if freqs is not None and fft_mag is not None:
                        ax2.plot(freqs, fft_mag, 'r-', linewidth=1.5, label='Espectro FFT')
                        ax2.set_title(f'Espectro de Frecuencia (0-{self.max_frequency} Hz)')
                        ax2.set_ylabel('Magnitud')
                        ax2.set_xlabel('Frecuencia (Hz)')
                        ax2.legend()
                        ax2.grid(True, alpha=0.3)
                        ax2.set_xlim(0, self.max_frequency)  # Fijar límite de 0-100 Hz
                        
                        # Encontrar y mostrar frecuencia dominante (solo en el rango 0-100 Hz)
                        if len(freqs) > 0:
                            dominant_freq_idx = np.argmax(fft_mag)
                            dominant_freq = freqs[dominant_freq_idx]
                            dominant_mag = fft_mag[dominant_freq_idx]
                            
                            # Marcar frecuencia dominante en el gráfico
                            ax2.plot(dominant_freq, dominant_mag, 'ro', markersize=8, 
                                    label=f'Frec. dominante: {dominant_freq:.1f} Hz')
                            ax2.legend()
                        
                    else:
                        needed = self.fft_size - len(self.data_buffer)
                        ax2.text(0.5, 0.5, f'Recolectando datos...\nFaltan {needed} puntos', 
                                ha='center', va='center', transform=ax2.transAxes, fontsize=10)
                        ax2.set_title(f'Espectro de Frecuencia (0-{self.max_frequency} Hz) - Esperando datos')
                    
                    # Ajustar ejes automáticamente
                    ax1.relim()
                    ax1.autoscale_view()
            
            elif frame % 100 == 0:
                print("Esperando datos...")
        
        try:
            ani = animation.FuncAnimation(
                fig, animate, interval=20, cache_frame_data=False, blit=False  # Intervalo más corto
            )
            
            plt.tight_layout()
            plt.show()
            
        except KeyboardInterrupt:
            print("\nVisualización detenida por el usuario")
        finally:
            print(f"\nTotal de datos procesados: {self.data_count}")

# Uso del sistema
if __name__ == "__main__":
    try:
        print("Iniciando sistema de adquisición con análisis espectral...")
        print("CONFIGURACIÓN: Frecuencia de muestreo 200 Hz para ver hasta 100 Hz")
        print("Asegúrate de que el ESP32 esté enviando datos a 200 Hz")
        
        processor = SignalProcessor(baudrate=115200)
        processor.process_realtime()
        
    except KeyboardInterrupt:
        print("\nAplicación interrumpida por el usuario")
    except Exception as e:
        print(f"Error: {e}")
        print("\nAsegúrate de tener instalado scipy:")
        print("pip install scipy")
    finally:
        if 'processor' in locals():
            processor.ser.close()
            print("Conexión serial cerrada")