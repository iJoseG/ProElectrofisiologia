import matplotlib
# Forzar backend interactivo
matplotlib.use('TkAgg')  # O 'Qt5Agg' si prefieres Qt
import matplotlib.pyplot as plt

import serial
import numpy as np
from scipy import signal
import matplotlib.animation as animation
from collections import deque
import time

class SignalProcessor:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, buffer_size=1000):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"✓ Conectado al puerto {port}")
        except serial.SerialException as e:
            print(f"✗ Error conectando al puerto {port}: {e}")
            print("Verifica el puerto con: ls /dev/ttyUSB* /dev/ttyACM*")
            raise
        
        self.buffer_size = buffer_size
        self.data_buffer = deque(maxlen=buffer_size)
        self.time_buffer = deque(maxlen=buffer_size)
        self.start_time = time.time()
        
        # Configuración del filtro
        self.sample_rate = 100  # Hz
        self.lowcut = 0.5       # Frecuencia de corte baja (Hz)
        self.highcut = 45.0     # Frecuencia de corte alta (Hz)
        
    def apply_bandpass_filter(self, data):
        """Filtro pasabanda para señales bioeléctricas"""
        nyquist = 0.5 * self.sample_rate
        low = self.lowcut / nyquist
        high = self.highcut / nyquist
        
        # Filtro Butterworth
        b, a = signal.butter(4, [low, high], btype='band')
        filtered_data = signal.filtfilt(b, a, data)
        return filtered_data
    
    def apply_notch_filter(self, data, freq=50, Q=30):
        """Filtro de muesca para eliminar ruido de línea (50/60 Hz)"""
        nyquist = 0.5 * self.sample_rate
        freq_norm = freq / nyquist
        b, a = signal.iirnotch(freq_norm, Q)
        filtered_data = signal.filtfilt(b, a, data)
        return filtered_data
    
    def read_data(self):
        """Leer datos del puerto serial"""
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                return float(line)
        except (ValueError, UnicodeDecodeError):
            return None
        return None
    
    def process_realtime(self):
        """Procesamiento en tiempo real"""
        print("Iniciando visualización en tiempo real...")
        print("Presiona Ctrl+C para detener")
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        plt.subplots_adjust(hspace=0.5)
        
        # Variable para mantener la animación
        self.ani = animation.FuncAnimation(
            fig, self._animate, interval=50, 
            fargs=(ax1, ax2), cache_frame_data=False, blit=False
        )
        
        plt.tight_layout()
        plt.show()
    
    def _animate(self, frame, ax1, ax2):
        """Función de animación"""
        new_value = self.read_data()
        
        if new_value is not None:
            current_time = time.time() - self.start_time
            
            # Agregar a buffers
            self.data_buffer.append(new_value)
            self.time_buffer.append(current_time)
            
            if len(self.data_buffer) > 10:
                # Convertir a arrays numpy
                data_array = np.array(self.data_buffer)
                time_array = np.array(self.time_buffer)
                
                # Aplicar filtros
                bandpass_filtered = self.apply_bandpass_filter(data_array)
                notch_filtered = self.apply_notch_filter(bandpass_filtered)
                
                # Limpiar plots
                ax1.clear()
                ax2.clear()
                
                # Plot señal original
                ax1.plot(time_array, data_array, 'b-', alpha=0.7, label='Original', linewidth=1)
                ax1.set_title('Señal Original')
                ax1.set_ylabel('Amplitud (ADC)')
                ax1.legend()
                ax1.grid(True, alpha=0.3)
                
                # Plot señal filtrada
                ax2.plot(time_array, notch_filtered, 'r-', label='Filtrada', linewidth=1)
                ax2.set_title('Señal Filtrada')
                ax2.set_ylabel('Amplitud (ADC)')
                ax2.set_xlabel('Tiempo (s)')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
                
                # Ajustar ejes
                for ax in [ax1, ax2]:
                    ax.relim()
                    ax.autoscale_view()
        
        return ax1, ax2
    
    def analyze_signal(self, duration=10):
        """Análisis más detallado de la señal"""
        print(f"Capturando señal por {duration} segundos...")
        
        data = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            value = self.read_data()
            if value is not None:
                data.append(value)
            time.sleep(1/self.sample_rate)
        
        if len(data) > 0:
            data = np.array(data)
            
            # Aplicar filtros
            filtered_data = self.apply_bandpass_filter(data)
            filtered_data = self.apply_notch_filter(filtered_data)
            
            # Análisis espectral
            f, Pxx = signal.welch(filtered_data, self.sample_rate, nperseg=1024)
            
            # Estadísticas
            mean_val = np.mean(filtered_data)
            std_val = np.std(filtered_data)
            max_val = np.max(filtered_data)
            min_val = np.min(filtered_data)
            
            print(f"\n--- ANÁLISIS DE SEÑAL ---")
            print(f"Media: {mean_val:.2f}")
            print(f"Desviación estándar: {std_val:.2f}")
            print(f"Rango: {min_val:.2f} a {max_val:.2f}")
            print(f"Número de muestras: {len(data)}")
            
            # Graficar resultados
            fig, axes = plt.subplots(2, 2, figsize=(15, 10))
            
            # Señal temporal original
            time_axis = np.arange(len(data)) / self.sample_rate
            axes[0,0].plot(time_axis, data, 'b-', alpha=0.7)
            axes[0,0].set_title('Señal Original')
            axes[0,0].set_xlabel('Tiempo (s)')
            axes[0,0].set_ylabel('Amplitud')
            axes[0,0].grid(True)
            
            # Señal temporal filtrada
            axes[0,1].plot(time_axis, filtered_data, 'r-')
            axes[0,1].set_title('Señal Filtrada')
            axes[0,1].set_xlabel('Tiempo (s)')
            axes[0,1].set_ylabel('Amplitud')
            axes[0,1].grid(True)
            
            # Espectro de frecuencia
            axes[1,0].semilogy(f, Pxx)
            axes[1,0].set_title('Densidad Espectral de Potencia')
            axes[1,0].set_xlabel('Frecuencia [Hz]')
            axes[1,0].set_ylabel('PSD [V**2/Hz]')
            axes[1,0].grid(True)
            
            # Histograma
            axes[1,1].hist(filtered_data, bins=50, alpha=0.7, edgecolor='black')
            axes[1,1].set_title('Distribución de Amplitudes')
            axes[1,1].set_xlabel('Amplitud')
            axes[1,1].set_ylabel('Frecuencia')
            axes[1,1].grid(True)
            
            plt.tight_layout()
            plt.show()
            
            return filtered_data, f, Pxx
        
        return None, None, None

# Uso del sistema
if __name__ == "__main__":
    try:
        # Ajusta el puerto serial según tu sistema
        processor = SignalProcessor(port='/dev/ttyUSB0')  # o '/dev/ttyACM0'
        
        # Para visualización en tiempo real
        processor.process_realtime()
        
    except KeyboardInterrupt:
        print("\nCerrando aplicación...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'processor' in locals():
            processor.ser.close()
            print("Conexión serial cerrada")