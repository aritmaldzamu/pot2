# 📚 Práctica 1: Comunicación Serial con ESP32-C6 en Arduino  

---

## 1) Resumen

- **Equipo / Autor(es):**  _Karen Nájera y Arith Maldonado_  
- **Curso / Asignatura:** _Elementos programables II_  
- **Fecha:** _18/09/2025_  
- **Descripción breve:**  
  En esta práctica se implementó un código en **Arduino IDE** para establecer comunicación serial con un **ESP32-C6**. Se explicó el funcionamiento de los diferentes tipos de variables (`int`, `char`, `string`, `float`, `bool`) y la cantidad de datos que puede almacenar cada uno. Además, se analizaron las diferencias entre los dos puertos de comunicación del ESP32 (UART y USB nativo) y cómo este recibe mensajes enviados desde el monitor serial.

---

## 2) Objetivos

- **General:**  
  _Comprender el funcionamiento básico de la comunicación serial en el ESP32-C6 usando Arduino IDE._  

- **Específicos:**  
  - Identificar y diferenciar los principales tipos de variables en Arduino.  
  - Implementar un programa que permita recibir y mostrar mensajes en el monitor serial.  
  - Analizar la diferencia entre el puerto **UART (serial clásico)** y el puerto **USB nativo** del ESP32.  
  - Verificar la correcta recepción y envío de caracteres mediante pruebas prácticas.  

---

## 3) Alcance y Exclusiones

- **Incluye:**  
  - Uso del ESP32-C6 como dispositivo de comunicación serial.  
  - Configuración del baud rate en el monitor serial.  
  - Recepción y envío de mensajes en el IDE de Arduino.  
  - Explicación teórica de los tipos de variables y su uso en la práctica.  

- **No incluye:**  
  - Conexión a sensores externos.  
  - Programación de librerías adicionales.  
  - Uso de comunicación inalámbrica (Wi-Fi / Bluetooth).  

---

## 4) Resultados

Durante la práctica se logró:  

- **Recepción de datos seriales:** El ESP32-C6 recibió correctamente mensajes enviados desde el monitor serial, aunque en un inicio aparecieron caracteres extraños debido a un **baud rate incorrecto**. Ajustando la velocidad a **38400 baudios** se solucionó el problema.  
- **Tipos de datos:**  
  - `int` → números enteros (16 bits).  
  - `char` → un carácter (1 byte).  
  - `string` → cadena de caracteres (mínimo 16 bits, máximo variable).  
  - `float` → números con decimales (32 bits).  
  - `bool` → valores lógicos (1 bit).  
- **Puertos de comunicación:**  
  - **UART (Universal Asynchronous Receiver-Transmitter):** puerto serial tradicional, útil para depuración o conexión con otros dispositivos.  
  - **USB nativo:** permite programar directamente el microcontrolador y también enviar datos sin necesidad de un conversor externo.  

---

**Código Implementado**

```cpp
char msg;

void setup() {
  Serial.begin(38400);   // Inicializa comunicación serial
}

void loop() {
  if (Serial.available()) {   // Verifica si hay datos
    msg = Serial.read();      // Lee el carácter
    Serial.print(msg);        // Lo reenvía al monitor
  }
}
