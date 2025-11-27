#include <Servo.h>

// Crear objetos para los servos
Servo servo1;
Servo servo2;

// Pines de conexión
const int pinServo1 = 9;
const int pinServo2 = 10;

void setup() {
  Serial.begin(9600);
  
  // Adjuntar servos a los pines
  servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  
  Serial.println("Calibracion de Servomotores");
  Serial.println("Comandos:");
  Serial.println("1: Mover servo1 a 0°");
  Serial.println("2: Mover servo1 a 90°");
  Serial.println("3: Mover servo1 a 180°");
  Serial.println("4: Mover servo2 a 0°");
  Serial.println("5: Mover servo2 a 90°");
  Serial.println("6: Mover servo2 a 180°");
  Serial.println("s: Detener servos");
  Serial.println("c: Barrido completo");
}

void loop() {
  if (Serial.available()) {
    char comando = Serial.read();
    
    switch(comando) {
      case '1':
        servo1.write(0);
        Serial.println("Servo1 en 0°");
        break;
        
      case '2':
        servo1.write(90);
        Serial.println("Servo1 en 90°");
        break;
        
      case '3':
        servo1.write(180);
        Serial.println("Servo1 en 180°");
        break;
        
      case '4':
        servo2.write(0);
        Serial.println("Servo2 en 0°");
        break;
        
      case '5':
        servo2.write(90);
        Serial.println("Servo2 en 90°");
        break;
        
      case '6':
        servo2.write(180);
        Serial.println("Servo2 en 180°");
        break;
        
      case 's':
        // Los servos mantienen su posición actual
        Serial.println("Servos detenidos");
        break;
        
      case 'c':
        barridoCompleto();
        break;
    }
  }
}

void barridoCompleto() {
  Serial.println("Iniciando barrido completo...");
  
  // Barrido del servo1
  for (int pos = 0; pos <= 180; pos += 10) {
    servo1.write(pos);
    Serial.print("Servo1: ");
    Serial.print(pos);
    Serial.println("°");
    delay(500);
  }
  
  // Barrido del servo2
  for (int pos = 0; pos <= 180; pos += 10) {
    servo2.write(pos);
    Serial.print("Servo2: ");
    Serial.print(pos);
    Serial.println("°");
    delay(500);
  }
  
  // Volver a posición central
  servo1.write(90);
  servo2.write(90);
  Serial.println("Barrido completado - Posicion central");
}