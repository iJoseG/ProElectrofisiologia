#define SENSOR_PIN 34
#define MUESTRAS 256
#define FRECUENCIA_MUESTREO 500

// Arrays para FFT
float vReal[MUESTRAS];
float vImag[MUESTRAS];
float espectro[MUESTRAS/2];

// Variables para el filtro notch
float notchHistorial[4] = {0}; // Historial de muestras
float notchSalida = 0;

// Coeficientes del filtro notch para 60.5 Hz
// Frecuencia a eliminar: 60.5 Hz, Frecuencia de muestreo: 500 Hz
float notchB0 = 0.9600;
float notchB1 = -1.8719;
float notchB2 = 0.9600;
float notchA1 = -1.8719;
float notchA2 = 0.9200;

unsigned long tiempoAnterior = 0;
unsigned long intervaloMuestreo = 1000000 / FRECUENCIA_MUESTREO;
int indiceMuestra = 0;

void setup() {
  Serial.begin(115200);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  Serial.println("Iniciando adquisición de señal EMG con filtro notch 60.5 Hz...");
}

void loop() {
  unsigned long tiempoActual = micros();
  
  if (tiempoActual - tiempoAnterior >= intervaloMuestreo) {
    tiempoAnterior = tiempoActual;
    
    // Leer sensor y aplicar filtro notch inmediatamente
    int rawValue = analogRead(SENSOR_PIN);
    float voltaje = (rawValue * 3.3) / 4095.0;
    
    // Aplicar filtro notch
    float señalFiltrada = aplicarFiltroNotch(voltaje);
    
    vReal[indiceMuestra] = señalFiltrada;
    vImag[indiceMuestra] = 0.0;
    
    indiceMuestra++;
    
    if (indiceMuestra >= MUESTRAS) {
      calcularFFTManual();
      indiceMuestra = 0;
    }
  }
}

// Filtro notch IIR para 60.5 Hz
float aplicarFiltroNotch(float entrada) {
  // Ecuación en diferencias del filtro notch:
  // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  
  float salida = notchB0 * entrada + 
                 notchB1 * notchHistorial[0] + 
                 notchB2 * notchHistorial[1] - 
                 notchA1 * notchHistorial[2] - 
                 notchA2 * notchHistorial[3];
  
  // Actualizar historial
  notchHistorial[1] = notchHistorial[0];  // x[n-2] = x[n-1]
  notchHistorial[0] = entrada;            // x[n-1] = x[n]
  notchHistorial[3] = notchHistorial[2];  // y[n-2] = y[n-1]
  notchHistorial[2] = salida;             // y[n-1] = y[n]
  
  return salida;
}

void calcularFFTManual() {
  // Aplicar ventana de Hann
  aplicarVentanaHann(vReal, MUESTRAS);
  
  // Calcular FFT
  fft(vReal, vImag, MUESTRAS);
  
  // Calcular magnitud del espectro
  calcularMagnitudEspectro();
  
  // Enviar resultados
  enviarEspectroFrecuencias();
}

// Algoritmo FFT Cooley-Tukey (radix-2)
void fft(float* real, float* imag, int n) {
  int i, j, k, m;
  float theta, wtemp, wpr, wpi, wr, wi, tempr, tempi;
  
  // Bit-reversal permutation
  j = 0;
  for (i = 0; i < n - 1; i++) {
    if (i < j) {
      tempr = real[i];
      real[i] = real[j];
      real[j] = tempr;
      
      tempi = imag[i];
      imag[i] = imag[j];
      imag[j] = tempi;
    }
    
    k = n >> 1;
    while (k <= j) {
      j -= k;
      k >>= 1;
    }
    j += k;
  }
  
  // Danielson-Lanczos algorithm
  int mmax = 1;
  while (n > mmax) {
    int istep = mmax << 1;
    theta = -PI / mmax;
    wtemp = sin(0.5 * theta);
    wpr = -2.0 * wtemp * wtemp;
    wpi = sin(theta);
    wr = 1.0;
    wi = 0.0;
    
    for (m = 0; m < mmax; m++) {
      for (i = m; i < n; i += istep) {
        j = i + mmax;
        tempr = wr * real[j] - wi * imag[j];
        tempi = wr * imag[j] + wi * real[j];
        
        real[j] = real[i] - tempr;
        imag[j] = imag[i] - tempi;
        
        real[i] += tempr;
        imag[i] += tempi;
      }
      
      wtemp = wr;
      wr = wtemp * wpr - wi * wpi + wr;
      wi = wi * wpr + wtemp * wpi + wi;
    }
    
    mmax = istep;
  }
}

void aplicarVentanaHann(float* vector, int n) {
  for (int i = 0; i < n; i++) {
    float factor = 0.5 * (1 - cos(2 * PI * i / (n - 1)));
    vector[i] *= factor;
  }
}

void calcularMagnitudEspectro() {
  for (int i = 0; i < MUESTRAS/2; i++) {
    espectro[i] = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
  }
}

void enviarEspectroFrecuencias() {
  Serial.println("=== ESPECTRO FRECUENCIAL EMG (CON NOTCH 60.5 Hz) ===");
  
  bool encontroNotch = false;
  float magnitud60Hz = 0;
  float magnitud120Hz = 0;
  
  for (int i = 1; i < MUESTRAS/2; i++) {
    float frecuencia = (i * 1.0 * FRECUENCIA_MUESTREO) / MUESTRAS;
    float magnitud = espectro[i];
    
    // Detectar componentes en 60Hz y 120Hz para monitorear efectividad
    if (abs(frecuencia - 60.5) < 1.0) {
      magnitud60Hz = magnitud;
    }
    if (abs(frecuencia - 121.0) < 1.0) {
      magnitud120Hz = magnitud;
    }
    
    if (frecuencia >= 20 && frecuencia <= 500 && magnitud > 0.001) {
      Serial.print("F: ");
      Serial.print(frecuencia, 1);
      Serial.print(" Hz | Mag: ");
      Serial.println(magnitud, 4);
    }
  }
  
  Serial.println("=== FIN ESPECTRO ===");
  
  // Mostrar efectividad del filtro notch
  Serial.print("*** Magnitud en 60.5 Hz: ");
  Serial.println(magnitud60Hz, 4);
  Serial.print("*** Magnitud en 120 Hz: ");
  Serial.println(magnitud120Hz, 4);
  Serial.println();
  
  encontrarPicoDominante();
}

void encontrarPicoDominante() {
  float maxMagnitud = 0;
  int indiceMax = 0;
  
  for (int i = 1; i < MUESTRAS/2; i++) {
    float frecuencia = (i * 1.0 * FRECUENCIA_MUESTREO) / MUESTRAS;
    // Excluir banda alrededor de 60.5 Hz para encontrar picos reales de EMG
    if (frecuencia >= 20 && frecuencia <= 500 && 
        abs(frecuencia - 60.5) > 2.0 && abs(frecuencia - 121.0) > 2.0) {
      if (espectro[i] > maxMagnitud) {
        maxMagnitud = espectro[i];
        indiceMax = i;
      }
    }
  }
  
  float frecuenciaPico = (indiceMax * 1.0 * FRECUENCIA_MUESTREO) / MUESTRAS;
  Serial.print("*** PICO DOMINANTE (sin 60Hz): ");
  Serial.print(frecuenciaPico, 1);
  Serial.print(" Hz | Magnitud: ");
  Serial.println(maxMagnitud, 4);
  Serial.println();
}

// Función para ajustar el filtro notch a diferentes frecuencias (si necesitas cambiar)
void configurarFiltroNotch(float frecuenciaNotch, float fs) {
  float omega = 2.0 * PI * frecuenciaNotch / fs;
  float alpha = sin(omega) / (2.0 * 0.707); // Q factor de aproximadamente 4
  
  notchB0 = 1.0;
  notchB1 = -2.0 * cos(omega);
  notchB2 = 1.0;
  float a0 = 1.0 + alpha;
  notchA1 = -2.0 * cos(omega) / a0;
  notchA2 = (1.0 - alpha) / a0;
  
  // Normalizar coeficientes b
  notchB0 = notchB0 / a0;
  notchB1 = notchB1 / a0;
  notchB2 = notchB2 / a0;
}