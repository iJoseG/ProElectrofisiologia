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
    def __init__(self, port=None, baudrate=115200, buffer_size=500):  # Buffer más pequeño
        if port is None:
            port = self.find_serial_port()
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)  # Timeout más corto
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
            # Leer todos los datos disponibles
            while self.ser.in_waiting > 0:
                raw_line = self.ser.readline()
                line = raw_line.decode('utf-8').strip()
                value = float(line)
                data_points.append(value)
            
            # Devolver el último dato (más reciente)
            return data_points[-1] if data_points else None
            
        except Exception:
            return None
    
    def process_realtime(self):
        print("\n" + "="*50)
        print("INICIANDO VISUALIZACIÓN EN TIEMPO REAL")
        print("="*50)
        print("Presiona Ctrl+C en la ventana de gráfico para detener")
        
        fig, ax = plt.subplots(figsize=(12, 6))
        
        self.data_count = 0
        self.last_print_time = time.time()
        
        def animate(frame):
            new_value = self.read_data()
            
            if new_value is not None:
                self.data_count += 1
                current_time = time.time() - self.start_time
                
                self.data_buffer.append(new_value)
                self.time_buffer.append(current_time)
                
                # Mostrar estadísticas cada 2 segundos
                if time.time() - self.last_print_time >= 2:
                    print(f"Datos recibidos: {self.data_count} - Buffer: {len(self.data_buffer)}")
                    self.last_print_time = time.time()
                
                if len(self.data_buffer) > 2:
                    data_array = np.array(self.data_buffer)
                    time_array = np.array(self.time_buffer)
                    
                    ax.clear()
                    
                    # Plot con colores diferentes para mejor visualización
                    ax.plot(time_array, data_array, 'b-', linewidth=1.5, label='Señal en bruto')
                    ax.set_title(f'Señal en Tiempo Real - {len(data_array)} puntos - {self.data_count} totales')
                    ax.set_ylabel('Valor ADC (0-4095)')
                    ax.set_xlabel('Tiempo (s)')
                    ax.legend()
                    ax.grid(True, alpha=0.3)
                    
                    # Ajustar ejes automáticamente
                    ax.relim()
                    ax.autoscale_view()
            
            elif frame % 50 == 0:
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
        print("Iniciando sistema de adquisición de señales...")
        
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