#define SENSOR_PIN 34
#define MUESTRAS 256
#define FRECUENCIA_MUESTREO 500

// Variables para DOBLE FILTRO NOTCH
float notch1Historial[4] = {0}; // Primer filtro
float notch2Historial[4] = {0}; // Segundo filtro

// Coeficientes para primer filtro notch (Q ≈ 30)
float notch1B0 = 0.9900;
float notch1B1 = -1.9848;
float notch1B2 = 0.9900;
float notch1A1 = -1.9848;
float notch1A2 = 0.9801;

// Coeficientes para segundo filtro notch (Q ≈ 30)
float notch2B0 = 0.9900;
float notch2B1 = -1.9848;
float notch2B2 = 0.9900;
float notch2A1 = -1.9848;
float notch2A2 = 0.9801;

unsigned long tiempoAnterior = 0;
unsigned long intervaloMuestreo = 1000000 / FRECUENCIA_MUESTREO;

void setup() {
  Serial.begin(115200);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // Configurar ambos filtros notch
  configurarDobleNotch(60.5, FRECUENCIA_MUESTREO, 30.0, 30.0);
  
  // Encabezados para Serial Plotter
  Serial.println("EMG_SinFiltrar EMG_Filtrado");
}

void loop() {
  unsigned long tiempoActual = micros();
  
  if (tiempoActual - tiempoAnterior >= intervaloMuestreo) {
    tiempoAnterior = tiempoActual;
    
    int rawValue = analogRead(SENSOR_PIN);
    float voltaje = (rawValue * 3.3) / 4095.0;
    
    // Aplicar DOBLE filtro notch
    float señalFiltrada = aplicarDobleNotch(voltaje);
    
    // Enviar datos al Serial Plotter
    Serial.print(voltaje);
    Serial.print(" ");
    Serial.println(señalFiltrada);
  }
}

// Aplicar dos filtros notch en cascada
float aplicarDobleNotch(float entrada) {
  // PRIMER FILTRO NOTCH
  float etapa1 = notch1B0 * entrada + 
                 notch1B1 * notch1Historial[0] + 
                 notch1B2 * notch1Historial[1] - 
                 notch1A1 * notch1Historial[2] - 
                 notch1A2 * notch1Historial[3];
  
  // Actualizar historial primer filtro
  notch1Historial[1] = notch1Historial[0];
  notch1Historial[0] = entrada;
  notch1Historial[3] = notch1Historial[2];
  notch1Historial[2] = etapa1;
  
  // SEGUNDO FILTRO NOTCH
  float etapa2 = notch2B0 * etapa1 + 
                 notch2B1 * notch2Historial[0] + 
                 notch2B2 * notch2Historial[1] - 
                 notch2A1 * notch2Historial[2] - 
                 notch2A2 * notch2Historial[3];
  
  // Actualizar historial segundo filtro
  notch2Historial[1] = notch2Historial[0];
  notch2Historial[0] = etapa1;
  notch2Historial[3] = notch2Historial[2];
  notch2Historial[2] = etapa2;
  
  return etapa2;
}

// Configurar ambos filtros notch
void configurarDobleNotch(float frecuenciaNotch, float fs, float Q1, float Q2) {
  // Configurar primer filtro
  float omega = 2.0 * PI * frecuenciaNotch / fs;
  float alpha1 = sin(omega) / (2.0 * Q1);
  
  float b0 = 1.0;
  float b1 = -2.0 * cos(omega);
  float b2 = 1.0;
  float a0 = 1.0 + alpha1;
  
  notch1B0 = b0 / a0;
  notch1B1 = b1 / a0;
  notch1B2 = b2 / a0;
  notch1A1 = -2.0 * cos(omega) / a0;
  notch1A2 = (1.0 - alpha1) / a0;
  
  // Configurar segundo filtro
  float alpha2 = sin(omega) / (2.0 * Q2);
  a0 = 1.0 + alpha2;
  
  notch2B0 = b0 / a0;
  notch2B1 = b1 / a0;
  notch2B2 = b2 / a0;
  notch2A1 = -2.0 * cos(omega) / a0;
  notch2A2 = (1.0 - alpha2) / a0;
}