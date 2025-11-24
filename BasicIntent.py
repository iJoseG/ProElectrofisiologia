import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import serial
import numpy as np
from scipy import signal
import matplotlib.animation as animation
from collections import deque
import time
import glob

class SignalProcessor:
    def __init__(self, port=None, baudrate=115200, buffer_size=1000):
        # Buscar puerto automáticamente si no se especifica
        if port is None:
            port = self.find_serial_port()
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"✓ Conectado al puerto {port}")
            print(f"✓ Configuración: {baudrate} bauds, timeout: 1s")
            
            # Limpiar buffer serial
            self.ser.reset_input_buffer()
            time.sleep(2)  # Esperar a que se estabilice la conexión
            
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
        
        # Configuración del filtro
        self.sample_rate = 100  # Hz
        self.lowcut = 0.5       # Frecuencia de corte baja (Hz)
        self.highcut = 45.0     # Frecuencia de corte alta (Hz)
        
        # Verificar conexión
        self.check_serial_connection()
    
    def find_serial_port(self):
        """Encontrar puerto serial automáticamente"""
        ports = self.list_serial_ports()
        if not ports:
            raise Exception("No se encontraron puertos seriales")
        
        print("Puertos seriales encontrados:")
        for port in ports:
            print(f"  {port}")
        
        # Priorizar ESP32 (generalmente ttyUSB o ttyACM)
        for port in ports:
            if 'USB' in port or 'ACM' in port:
                print(f"Usando puerto: {port}")
                return port
        
        return ports[0]
    
    def list_serial_ports(self):
        """Listar puertos seriales disponibles"""
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyS*')
        return ports
    
    def check_serial_connection(self):
        """Verificar que lleguen datos del serial"""
        print("Verificando conexión serial...")
        test_data = []
        start_time = time.time()
        timeout = 5  # 5 segundos máximo
        
        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                try:
                    raw_line = self.ser.readline()
                    line = raw_line.decode('utf-8').strip()
                    value = float(line)
                    test_data.append(value)
                    print(f"Dato recibido: {value}")
                    
                    # Si tenemos algunos datos, es suficiente
                    if len(test_data) >= 5:
                        break
                        
                except UnicodeDecodeError:
                    print(f"Error de decodificación. Bytes crudos: {raw_line}")
                except ValueError as e:
                    print(f"Error convirtiendo a número: '{line}' - {e}")
                except Exception as e:
                    print(f"Error inesperado: {e}")
        
        if test_data:
            print(f"✓ Conexión OK - Se recibieron {len(test_data)} datos")
            print(f"  Ejemplo de datos: {test_data[:5]}")
            return True
        else:
            print("✗ No se recibieron datos después de 5 segundos")
            print("\nSolución de problemas:")
            print("1. Verifica que el ESP32 esté conectado y encendido")
            print("2. Verifica el código del ESP32 (debe enviar datos por Serial)")
            print("3. Prueba con: screen /dev/ttyUSB0 115200")
            print("4. Verifica los cables USB")
            return False
    
    def apply_bandpass_filter(self, data):
        """Filtro pasabanda para señales bioeléctricas"""
        if len(data) < 10:
            return data
            
        nyquist = 0.5 * self.sample_rate
        low = self.lowcut / nyquist
        high = self.highcut / nyquist
        
        try:
            # Filtro Butterworth
            b, a = signal.butter(4, [low, high], btype='band')
            filtered_data = signal.filtfilt(b, a, data)
            return filtered_data
        except Exception as e:
            print(f"Error en filtro pasabanda: {e}")
            return data
    
    def apply_notch_filter(self, data, freq=50, Q=30):
        """Filtro de muesca para eliminar ruido de línea"""
        if len(data) < 10:
            return data
            
        nyquist = 0.5 * self.sample_rate
        freq_norm = freq / nyquist
        
        try:
            b, a = signal.iirnotch(freq_norm, Q)
            filtered_data = signal.filtfilt(b, a, data)
            return filtered_data
        except Exception as e:
            print(f"Error en filtro de muesca: {e}")
            return data
    
    def read_data(self):
        """Leer datos del puerto serial"""
        try:
            if self.ser.in_waiting > 0:
                raw_line = self.ser.readline()
                line = raw_line.decode('utf-8').strip()
                return float(line)
        except UnicodeDecodeError:
            print(f"Error de decodificación. Bytes: {raw_line}")
        except ValueError:
            print(f"No se pudo convertir a número: '{line}'")
        except Exception as e:
            print(f"Error leyendo dato: {e}")
        
        return None
    
    def process_realtime(self):
        """Procesamiento en tiempo real"""
        print("\n" + "="*50)
        print("INICIANDO VISUALIZACIÓN EN TIEMPO REAL")
        print("="*50)
        print("Presiona Ctrl+C en la ventana de gráfico para detener")
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        plt.subplots_adjust(hspace=0.5)
        
        # Contador de datos recibidos
        self.data_count = 0
        
        def animate(frame):
            new_value = self.read_data()
            
            if new_value is not None:
                self.data_count += 1
                current_time = time.time() - self.start_time
                
                # Agregar a buffers
                self.data_buffer.append(new_value)
                self.time_buffer.append(current_time)
                
                # Mostrar progreso cada 50 datos
                if self.data_count % 50 == 0:
                    print(f"Datos recibidos: {self.data_count}")
                
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
                    ax1.set_title(f'Señal Original - {len(data_array)} puntos')
                    ax1.set_ylabel('Amplitud (ADC)')
                    ax1.legend()
                    ax1.grid(True, alpha=0.3)
                    
                    # Plot señal filtrada
                    ax2.plot(time_array, notch_filtered, 'r-', label='Filtrada', linewidth=1)
                    ax2.set_title('Señal Filtrada (Bandpass + Notch)')
                    ax2.set_ylabel('Amplitud (ADC)')
                    ax2.set_xlabel('Tiempo (s)')
                    ax2.legend()
                    ax2.grid(True, alpha=0.3)
                    
                    # Ajustar ejes automáticamente
                    ax1.relim()
                    ax1.autoscale_view()
                    ax2.relim()
                    ax2.autoscale_view()
            
            elif frame % 100 == 0:  # Mostrar mensaje cada 100 frames sin datos
                print("Esperando datos... Verifica el ESP32")
        
        try:
            # Crear animación
            ani = animation.FuncAnimation(
                fig, animate, interval=50, cache_frame_data=False, blit=False
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
        print("Iniciando sistema de adquisición de señales...")
        
        # El puerto se detectará automáticamente
        processor = SignalProcessor(baudrate=115200)
        
        # Iniciar visualización
        processor.process_realtime()
        
    except KeyboardInterrupt:
        print("\nAplicación interrumpida por el usuario")
    except Exception as e:
        print(f"Error: {e}")
        print("\nPara solución de problemas:")
        print("1. Verifica que el ESP32 esté programado correctamente")
        print("2. Ejecuta: ls /dev/ttyUSB* /dev/ttyACM*")
        print("3. Prueba con: screen /dev/ttyUSB0 115200")
    finally:
        if 'processor' in locals():
            processor.ser.close()
            print("Conexión serial cerrada")