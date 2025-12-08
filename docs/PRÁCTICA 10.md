# Proyecto Final (PF): Plataforma de Balance 3-DOF con Visión por Computadora

---

## 1) Resumen
- Se desarrolló una **plataforma de balance de tres grados de libertad (3-DOF)** controlada mediante **visión por computadora** en tiempo real y comunicación inalámbrica **Bluetooth Serial** hacia un **ESP32**.
- Se implementaron **dos modos de operación**:
  - **Control por Mano (MediaPipe Hands):** La cámara identifica la orientación de la mano y traduce roll/pitch en ángulos lógicos para los servomotores.
  - **Control PD por Cámara Frontal:** La cámara observa la plataforma, detecta la posición de la pelota y aplica un control PD para regresarla al centro.
- El ESP32 convierte los ángulos en **PWM** para los servos e implementa una **rampa suave** para evitar golpes mecánicos.

---

## 2) Objetivos
- Implementar visión por computadora para el control en tiempo real de una plataforma 3-DOF.
- Procesar señales de **Bluetooth Serial** en el ESP32 para recibir comandos de ángulo.
- Controlar 3 servos usando ángulos lógicos con mapeo lógico–físico e interpolación suave (rampa).
- Implementar un control **PD** que estabilice una pelota sobre la plataforma.
- Validar el desempeño mecánico y de control mediante pruebas físicas (modo mano y modo pelota).

---

## 3) Materiales
- 1 × **ESP32 DevKit**  
- 3 × **Servomotores SG90** o similares  
- Estructura mecánica de **plataforma 3-DOF**  
- **Fuente de 5 V** para servos  
- 1 × **Cámara web**  
- PC con **Python 3** y librerías:
  - `opencv-python`  
  - `numpy`  
  - `mediapipe`  
  - `bluetooth` / PyBluez  
- Cables, protoboard y elementos mecánicos según el diseño

> **Nota eléctrica:** Todos los componentes deben compartir **GND** y los servos deben alimentarse con 5 V externos.

---

## 4) Código — Control por Mano (Python)

```python
# PEGA AQUÍ EL CÓDIGO COMPLETO DEL CONTROL POR MANO
```
