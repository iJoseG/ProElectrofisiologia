import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import serial
import numpy as np
import matplotlib.animation as animation
from collections import deque
import time
import glob

class SignalProcessor:
    def __init__(self, port=None, baudrate=115200, buffer_size=500):
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
        self.last_update_time = time.time()
        
        # Variables para detección de pulsos
        self.peaks = []  # Lista de tiempos de picos detectados
        self.peak_values = []  # Valores de los picos
        self.heart_rate = 0  # Pulsos por minuto
        self.total_beats = 0  # Total de pulsos detectados
        self.last_peak_time = 0
        self.min_peak_interval = 0.3  # Mínimo intervalo entre picos (segundos)
        
        # Parámetros para detección de picos
        self.threshold = 30  # Valor inicial del umbral
        self.peak_height = 50  # Altura mínima del pico
        self.signal_mean = 0
        self.signal_std = 0
        
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
    
    def detect_peaks(self, data_window, time_window):
        """Detección robusta de picos para ECG"""
        if len(data_window) < 10:
            return []
        
        # Convertir a arrays numpy
        data_array = np.array(data_window)
        time_array = np.array(time_window)
        
        # Calcular estadísticas para umbral adaptativo
        self.signal_mean = np.mean(data_array)
        self.signal_std = np.std(data_array)
        
        # Umbral adaptativo (ajustar según tu señal)
        dynamic_threshold = self.signal_mean + 2 * self.signal_std
        self.threshold = max(30, dynamic_threshold)  # Mínimo 2000
        
        detected_peaks = []
        current_time = time_window[-1]
        
        # Buscar picos en los últimos datos (ventana deslizante)
        window_size = min(20, len(data_window))
        recent_data = list(data_window)[-window_size:]
        recent_times = list(time_window)[-window_size:]
        
        for i in range(2, len(recent_data)-2):
            # Condición de pico: punto más alto en una ventana pequeña
            if (recent_data[i] > recent_data[i-1] and 
                recent_data[i] > recent_data[i-2] and
                recent_data[i] > recent_data[i+1] and 
                recent_data[i] > recent_data[i+2] and
                recent_data[i] > self.threshold and
                recent_data[i] - min(recent_data[max(0,i-5):i+5]) > self.peak_height):
                
                peak_time = recent_times[i]
                
                # Verificar que haya pasado suficiente tiempo desde el último pico
                if current_time - self.last_peak_time > self.min_peak_interval:
                    detected_peaks.append((peak_time, recent_data[i]))
                    self.last_peak_time = peak_time
        
        return detected_peaks
    
    def calculate_heart_rate(self):
        """Calcula los pulsos por minuto basado en los últimos picos"""
        if len(self.peaks) < 2:
            return 0
        
        # Usar los últimos 8 picos para cálculo más estable
        recent_peaks = self.peaks[-8:]
        
        if len(recent_peaks) < 2:
            return 0
        
        # Calcular intervalos entre picos
        intervals = []
        for i in range(1, len(recent_peaks)):
            interval = recent_peaks[i][0] - recent_peaks[i-1][0]
            if interval > 0.3:  # Filtrar intervalos muy cortos (ruido)
                intervals.append(interval)
        
        if not intervals:
            return 0
        
        # Calcular frecuencia cardíaca promedio
        avg_interval = np.mean(intervals)
        heart_rate = 60.0 / avg_interval
        
        # Filtrar valores extremos
        if heart_rate < 40 or heart_rate > 200:
            return self.heart_rate  # Mantener valor anterior si es extremo
        
        return heart_rate
    
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
    
    def process_realtime(self):
        print("\n" + "="*50)
        print("INICIANDO VISUALIZACIÓN EN TIEMPO REAL")
        print("="*50)
        print("Presiona Ctrl+C en la ventana de gráfico para detener")
        
        fig, ax = plt.subplots(figsize=(14, 8))
        
        self.data_count = 0
        self.last_print_time = time.time()
        
        def animate(frame):
            new_value = self.read_data()
            
            if new_value is not None:
                self.data_count += 1
                current_time = time.time() - self.start_time
                
                self.data_buffer.append(new_value)
                self.time_buffer.append(current_time)
                
                # Detectar picos
                if len(self.data_buffer) > 20:
                    new_peaks = self.detect_peaks(self.data_buffer, self.time_buffer)
                    
                    for peak_time, peak_value in new_peaks:
                        if peak_time not in [p[0] for p in self.peaks]:  # Evitar duplicados
                            self.peaks.append((peak_time, peak_value))
                            self.total_beats += 1
                            print(f"¡Pulso detectado! Total: {self.total_beats}")
                
                # Calcular frecuencia cardíaca cada 2 segundos
                if len(self.peaks) >= 2 and time.time() - self.last_print_time >= 2:
                    self.heart_rate = self.calculate_heart_rate()
                    print(f"Pulsos: {self.total_beats} - Frecuencia: {self.heart_rate:.1f} BPM")
                    self.last_print_time = time.time()
                
                if len(self.data_buffer) > 2:
                    data_array = np.array(self.data_buffer)
                    time_array = np.array(self.time_buffer)
                    
                    ax.clear()
                    
                    # Graficar señal principal
                    ax.plot(time_array, data_array, 'b-', linewidth=1.5, label='Señal ECG', alpha=0.7)
                    
                    # Graficar picos detectados
                    if self.peaks:
                        peak_times = [p[0] for p in self.peaks if p[0] in time_array]
                        peak_values = [p[1] for p in self.peaks if p[0] in time_array]
                        ax.plot(peak_times, peak_values, 'ro', markersize=6, label='Pulsos detectados')
                    
                    # Línea de umbral
                    ax.axhline(y=self.threshold, color='r', linestyle='--', alpha=0.5, label='Umbral')
                    
                    # Configuración del gráfico
                    ax.set_title(f'Electrocardiograma en Tiempo Real', fontsize=14, fontweight='bold')
                    ax.set_ylabel('Valor ADC', fontsize=12)
                    ax.set_xlabel('Tiempo (s)', fontsize=12)
                    ax.legend(loc='upper right')
                    ax.grid(True, alpha=0.3)
                    
                    # Mostrar información de pulsos en el gráfico
                    info_text = f'Pulsos totales: {self.total_beats}\nFrecuencia cardíaca: {self.heart_rate:.1f} BPM\nUmbral: {self.threshold:.0f}'
                    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=12,
                           verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
                    
                    # Ajustar ejes automáticamente
                    ax.relim()
                    ax.autoscale_view()
            
            elif frame % 50 == 0:
                print("Esperando datos...")
        
        try:
            ani = animation.FuncAnimation(
                fig, animate, interval=20, cache_frame_data=False, blit=False
            )
            
            plt.tight_layout()
            plt.show()
            
        except KeyboardInterrupt:
            print("\nVisualización detenida por el usuario")
        finally:
            print(f"\nResumen final:")
            print(f"Total de datos procesados: {self.data_count}")
            print(f"Total de pulsos detectados: {self.total_beats}")
            if self.heart_rate > 0:
                print(f"Frecuencia cardíaca final: {self.heart_rate:.1f} BPM")

# Uso del sistema
if __name__ == "__main__":
    try:
        print("Iniciando sistema de monitorización de ECG...")
        print("Algoritmo de detección de pulsos activado")
        
        processor = SignalProcessor(baudrate=115200)
        processor.process_realtime()
        
    except KeyboardInterrupt:
        print("\nAplicación interrumpida por el usuario")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'processor' in locals():
            processor.ser.close()
            print("Conexión serial cerrada")