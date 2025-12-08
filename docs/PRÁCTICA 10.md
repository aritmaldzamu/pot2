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
import cv2
import numpy as np
import bluetooth
import time
import mediapipe as mp
 
# ========= BLUETOOTH =========
device_mac = "F0:24:F9:0C:4C:DE"   # <-- MISMA MAC
port = 1
 
# ========= PARÁMETROS MAPEO =========
# Posiciones "neutras" de la plataforma en grados lógicos (0-180)
NEUT_IZQ  = 90
NEUT_DER  = 90
NEUT_VERT = 60
 
# Cuánto giro de mano consideramos útil (en grados)
MAX_HAND_TILT_X = 30.0   # ±30° de roll mano
MAX_HAND_TILT_Y = 30.0   # ±30° de pitch mano
 
# Cuánto puede moverse el servo alrededor de su neutro
MAX_SERVO_OFFSET_X = 40.0  # servos izq/der ±40°
MAX_SERVO_OFFSET_Y = 30.0  # servo vertical ±30°
 
# Invertir ejes si hace falta
INVERT_X = True
INVERT_Y = True
 
# Límite de frecuencia de envío
MIN_DT_CMD = 0.015  # 15 ms (~66 Hz máx)
 
last_cmd_time = 0.0
 
# Referencias de ángulo (se calibran con 'c')
base_angleX = None   # ángulo neutro entre índice ↔ anular
base_angleY = None   # ángulo neutro entre muñeca ↔ dedo medio
 
# ======== CONEXIÓN BLUETOOTH ========
sock = None
print("Intentando conectar al ESP32 por Bluetooth...", device_mac)
while True:
    try:
        sock = bluetooth.BluetoothSocket()
        sock.settimeout(10)
        sock.connect((device_mac, port))
        print("✅ Conectado al ESP32!")
        break
    except Exception as e:
        print("Error de conexión, reintentando:", e)
        time.sleep(1)
 
# ======== MEDIAPIPE HANDS ========
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils
 
# ======== CÁMARA ========
video = cv2.VideoCapture(0)  # usa 0 o 1 según tu cámara
 
def calc_angle(p1, p2):
    """Devuelve ángulo (rad, deg) de la línea p1->p2 respecto al eje X."""
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    theta = np.arctan2(dy, dx)          # radianes
    theta_deg = np.degrees(theta)       # grados
    return theta, theta_deg
 
while True:
    ok, frame = video.read()
    if not ok:
        break
 
    frame = cv2.flip(frame, 1)
    h, w = frame.shape[:2]
 
    now = time.time()
    send_allowed = (now - last_cmd_time) >= MIN_DT_CMD
 
    # Procesar mano
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
 
    hay_mano = False
    angleX_deg = None
    angleY_deg = None
 
    if results.multi_hand_landmarks:
        hay_mano = True
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
 
        # ====== PUNTOS CLAVE ======
        idx_tip  = hand_landmarks.landmark[8]   # índice
        mid_tip  = hand_landmarks.landmark[12]  # medio
        ring_tip = hand_landmarks.landmark[16]  # anular
        wrist    = hand_landmarks.landmark[0]   # muñeca
 
        def to_px(lm):
            return int(lm.x * w), int(lm.y * h)
 
        idx_pt  = to_px(idx_tip)
        mid_pt  = to_px(mid_tip)
        ring_pt = to_px(ring_tip)
        wrist_pt= to_px(wrist)
 
        # Dibujar puntos (rojos)
        for p in [idx_pt, mid_pt, ring_pt, wrist_pt]:
            cv2.circle(frame, p, 8, (0, 0, 255), -1)
 
        # ====== LÍNEA VERDE: índice ↔ anular (roll / X) ======
        cv2.line(frame, idx_pt, ring_pt, (0, 255, 0), 3)
        _, angleX_deg = calc_angle(idx_pt, ring_pt)
 
        # ====== LÍNEA MAGENTA: muñeca ↔ dedo medio (pitch / Y) ======
        cv2.line(frame, wrist_pt, mid_pt, (255, 0, 255), 3)
        _, angleY_deg = calc_angle(wrist_pt, mid_pt)
 
        # Centro aproximado de la mano (turquesa)
        cx = int((wrist_pt[0] + idx_pt[0] + ring_pt[0]) / 3)
        cy = int((wrist_pt[1] + idx_pt[1] + ring_pt[1]) / 3)
        cv2.circle(frame, (cx, cy), 8, (255, 255, 0), -1)
 
        # Mostrar ángulos
        cv2.putText(frame, f"AX:{angleX_deg:7.2f} deg", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(frame, f"AY:{angleY_deg:7.2f} deg", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
 
        # ====== SI YA HAY CALIBRACIÓN, MAPEAR DIRECTO A ÁNGULOS DE SERVOS ======
        if base_angleX is not None and base_angleY is not None:
            # Diferencias de ángulo respecto a la mano neutra
            dAX = angleX_deg - base_angleX   # roll relativo
            dAY = angleY_deg - base_angleY   # pitch relativo
 
            # Normalizar al rango [-1, 1] según el máximo giro que consideras
            dAX_norm = np.clip(dAX / MAX_HAND_TILT_X, -1.0, 1.0)
            dAY_norm = np.clip(dAY / MAX_HAND_TILT_Y, -1.0, 1.0)
 
            # Invertir si quieres que gire al revés
            if INVERT_X:
                dAX_norm = -dAX_norm
            if INVERT_Y:
                dAY_norm = -dAY_norm
 
            # Offset en grados de servo
            servo_off_X = dAX_norm * MAX_SERVO_OFFSET_X
            servo_off_Y = dAY_norm * MAX_SERVO_OFFSET_Y
 
            # Ángulos LÓGICOS absolutos para cada servo
            ang_izq  = NEUT_IZQ  - servo_off_X
            ang_der  = NEUT_DER  + servo_off_X
            ang_vert = NEUT_VERT - servo_off_Y
 
            # Limitar 0..180
            ang_izq  = int(np.clip(ang_izq,  0, 180))
            ang_der  = int(np.clip(ang_der,  0, 180))
            ang_vert = int(np.clip(ang_vert, 0, 180))
 
            cv2.putText(frame, f"IZQ:{ang_izq} DER:{ang_der} VERT:{ang_vert}",
                        (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0,255,255), 2)
 
            # Enviar al ESP32
            if send_allowed:
                try:
                    cmd = f"ANG:{ang_izq},{ang_der},{ang_vert}\n"
                    sock.send(cmd.encode())
                    # print("CMD ->", cmd.strip())
                    last_cmd_time = now
                except Exception as e:
                    print("⚠️ Error al enviar ANG:", e)
        else:
            cv2.putText(frame, "Presiona 'c' para calibrar mano neutra y mandar ZERO",
                        (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)
 
    else:
        cv2.putText(frame, "NO HAND", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)
        # Sin mano -> opcional: mandar LOST
        if send_allowed:
            try:
                sock.send(b"LOST\n")
                last_cmd_time = now
            except Exception as e:
                print("⚠️ Error al enviar LOST:", e)
 
    cv2.imshow("Hand Angle Direct Control", frame)
    key = cv2.waitKey(1) & 0xFF
 
    if key == ord('q'):
        break
    elif key == ord('c'):
        # Calibrar mano neutra si hay ángulos válidos
        if angleX_deg is not None and angleY_deg is not None:
            base_angleX = angleX_deg
            base_angleY = angleY_deg
            print(f"Calibrado! base_angleX={base_angleX:.2f}, base_angleY={base_angleY:.2f}")
        # Mandar comando ZERO para llevar servos a 180 físicos
        try:
            sock.send(b"ZERO\n")
            last_cmd_time = time.time()
            print("Comando ZERO enviado (servos a 180 fisico).")
        except Exception as e:
            print("⚠️ Error al enviar ZERO:", e)
 
video.release()
sock.close()
cv2.destroyAllWindows()
 
```
---

## 5) Código — Firmware ESP32 (Arduino)

```python
#include <Arduino.h>
#include "BluetoothSerial.h"
 
BluetoothSerial SerialBT;
 
// === Buffer para lectura BT no bloqueante ===
String btBuffer;
 
// === Pines de los servos ===
#define SERVO_IZQ   25
#define SERVO_DER   15
#define SERVO_VERT  33
 
// === PWM ===
const uint32_t FREQ_HZ = 50;
const uint8_t  RES_BITS = 12;
const uint16_t DUTY_MIN = 205;   // ~1.0 ms
const uint16_t DUTY_MAX = 410;   // ~2.0 ms
 
// Convierte grados físicos 0..180 a duty
uint16_t dutyFromDeg(int deg){
  deg = constrain(deg,0,180);
  return map(deg,0,180,DUTY_MIN,DUTY_MAX);
}
 
// Convierte de ángulo lógico (0..180, centro 90) a físico (invertido)
int logicalToPhysical(int logicalDeg){
  logicalDeg = constrain(logicalDeg, 0, 180);
  // 0 lógico → 180 físico, 180 lógico → 0 físico
  return 180 - logicalDeg;
}
 
// Escribe usando grados lógicos
void writeServoLogical(int pin, int logicalDeg){
  int fisico = logicalToPhysical(logicalDeg);
  ledcWrite(pin, dutyFromDeg(fisico));
}
 
// Configurar servo con ángulo lógico inicial
void configServo(int pin, int initialLogical){
  pinMode(pin,OUTPUT);
  ledcAttach(pin,FREQ_HZ,RES_BITS);   // usa el pin como canal
  writeServoLogical(pin,initialLogical);
}
 
// === Rango y rampa ===
const int LIM_MIN = 0;
const int LIM_MAX = 180;
const int PASO_RAMPA = 45;          // tamaño de paso en rampa
const uint32_t DT_RAMP_MS = 2;
const uint32_t TIMEOUT_MS = 700;
 
// Estado en grados LÓGICOS
int posIzq = 90;
int posDer = 90;
int posVert= 60;
 
int tgtIzq = 90;
int tgtDer = 90;
int tgtVert= 60;
 
uint32_t tPrevRamp = 0;
uint32_t tLastCmd  = 0;
 
// Rampa suave hacia el objetivo
void aplicarRampa(){
  uint32_t now = millis();
  if(now - tPrevRamp < DT_RAMP_MS) return;
  tPrevRamp = now;
 
  auto go = [&](int actual,int target){
    if(actual < target) return min(actual + PASO_RAMPA, target);
    if(actual > target) return max(actual - PASO_RAMPA, target);
    return actual;
  };
 
  posIzq = go(posIzq, tgtIzq);
  posDer = go(posDer, tgtDer);
  posVert= go(posVert,tgtVert);
 
  // Escribimos usando grados LÓGICOS, se invierten adentro
  writeServoLogical(SERVO_IZQ, posIzq);
  writeServoLogical(SERVO_DER, posDer);
  writeServoLogical(SERVO_VERT,posVert);
}
 
// Parsea "ANG:izq,der,vert"
bool parseAngulos(const String &msg, int &aIzq, int &aDer, int &aVert){
  if (!msg.startsWith("ANG:")) return false;
  String data = msg.substring(4);  // después de "ANG:"
 
  int c1 = data.indexOf(',');
  if (c1 < 0) return false;
  int c2 = data.indexOf(',', c1 + 1);
  if (c2 < 0) return false;
 
  String sIzq  = data.substring(0, c1);
  String sDer  = data.substring(c1 + 1, c2);
  String sVert = data.substring(c2 + 1);
 
  sIzq.trim();
  sDer.trim();
  sVert.trim();
 
  aIzq  = sIzq.toInt();
  aDer  = sDer.toInt();
  aVert = sVert.toInt();
 
  return true;
}
 
// ======== SETUP ========
void setup(){
  Serial.begin(115200);
  SerialBT.begin("ESP32-BallPlatform");
 
  configServo(SERVO_IZQ, posIzq);
  configServo(SERVO_DER, posDer);
  configServo(SERVO_VERT,posVert);
 
  Serial.println("ESP32 listo");
  tLastCmd = millis();
}
 
// ======== LOOP ========
void loop(){
 
  // --- Lectura Bluetooth no bloqueante ---
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
 
    if (c == '\n') {
      // Tenemos una línea completa en btBuffer
      String msg = btBuffer;
      btBuffer = "";        // limpiar para el siguiente mensaje
 
      msg.trim();
      if (msg.length() > 0) {
        tLastCmd = millis();
 
        if (msg == "LOST") {
          // PC perdió mano/objetivo → "centro"
          tgtIzq  = 90;
          tgtDer  = 90;
          tgtVert = 60;
          Serial.println("Comando LOST: centro.");
 
        } else if (msg == "ZERO") {
          // 🔹 TODOS los servos a 180 físicos
          //    ⇒ 0 lógico por el mapeo invertido
          tgtIzq  = 0;
          tgtDer  = 0;
          tgtVert = 0;
          Serial.println("Comando ZERO: servos → 180° fisico");
 
        } else {
          int aIzq, aDer, aVert;
          if (parseAngulos(msg, aIzq, aDer, aVert)) {
            tgtIzq  = constrain(aIzq,  LIM_MIN, LIM_MAX);
            tgtDer  = constrain(aDer,  LIM_MIN, LIM_MAX);
            tgtVert = constrain(aVert, LIM_MIN, LIM_MAX);
            Serial.printf("ANG -> IZQ:%d DER:%d VERT:%d\n", tgtIzq, tgtDer, tgtVert);
          } else {
            Serial.print("Comando desconocido: ");
            Serial.println(msg);
          }
        }
      }
    } else if (c != '\r') {
      // Acumulamos caracteres, ignorando CR
      btBuffer += c;
    }
  }
 
  // Si pasa mucho tiempo sin recibir comandos, vuelve al centro
  if(millis() - tLastCmd > TIMEOUT_MS){
    tgtIzq = 90;
    tgtDer = 90;
    tgtVert= 60;
  }
 
  aplicarRampa();
  delay(1);
}
 
```
---

## 6) Código — Control PD de la Pelota (Python)

```python
import cv2
import numpy as np
import bluetooth
import time
 
# ========= BLUETOOTH =========
device_mac = "F0:24:F9:0C:4C:DE"   # MISMA MAC DEL ESP32
port = 1
 
# ========= PARÁMETROS SERVOS =========
NEUT_IZQ  = 90
NEUT_DER  = 90
NEUT_VERT = 60
 
# ----- MODO TEST (para exagerar el movimiento) -----
TEST_MODE = True   # Cambia a False cuando ya quieras algo más fino
 
if TEST_MODE:
    # Mucho más movimiento de servos
    MAX_SERVO_OFFSET_X = 50.0   # antes 40
    MAX_SERVO_OFFSET_Y = 35.0   # antes 30
 
    # Control más agresivo y SIN derivada (más fácil ver el sentido)
    KpX = 2.0
    KdX = 0.0
 
    KpY = 2.0
    KdY = 0.0
else:
    # Valores más tranquilos para uso normal
    MAX_SERVO_OFFSET_X = 40.0
    MAX_SERVO_OFFSET_Y = 30.0
 
    KpX = 0.8
    KdX = 0.2
 
    KpY = 0.8
    KdY = 0.2
 
# ====== FLAGS DE ORIENTACIÓN (LOS VAS CAMBIANDO EN VIVO) ======
INVERT_X = False   # lo puedes cambiar con la tecla 'x'
INVERT_Y = False   # lo puedes cambiar con la tecla 'y'
SWAP_AXES = False  # si True, intercambia X<->Y (tecla 's')
 
alpha = 0.8          # filtro para derivada
MIN_DT_CMD = 0.015   # 15 ms (~66 Hz máx)
 
last_cmd_time = 0.0
last_time = time.time()
 
last_errx = 0.0
last_erry = 0.0
dxf = 0.0
dyf = 0.0
 
# Centro calibrado de la plataforma (en píxeles de la imagen)
centerX = None
centerY = None
 
# Última posición conocida de la pelota (para la tecla 'b')
last_ball_x = None
last_ball_y = None
 
# ========= CONEXIÓN BLUETOOTH =========
sock = None
print("Intentando conectar al ESP32 por Bluetooth...", device_mac)
while True:
    try:
        sock = bluetooth.BluetoothSocket()
        sock.settimeout(10)
        sock.connect((device_mac, port))
        print("✅ Conectado al ESP32!")
        break
    except Exception as e:
        print("Error de conexión, reintentando:", e)
        time.sleep(1)
 
# ========= CÁMARA (LA QUE VE LA PLATAFORMA) =========
# Cambia 1 a 0 si tu otra cámara es la que ve la plataforma
video = cv2.VideoCapture(1)
 
# ========= RANGO HSV PARA LA PELOTA (EJEMPLO: NARANJA) =========
LOWER = np.array([10, 150, 120], np.uint8)
UPPER = np.array([25, 255, 255], np.uint8)
 
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
 
while True:
    ok, frame = video.read()
    if not ok:
        break
 
    # Si la cámara te da la imagen al revés y quieres voltearla:
    # frame = cv2.flip(frame, 1)
 
    h, w = frame.shape[:2]
 
    # Si aún no hay centro calibrado, por defecto usa el centro de la imagen
    if centerX is None or centerY is None:
        centerX = w // 2
        centerY = h // 2
 
    now = time.time()
    dt = now - last_time if now > last_time else 0.01
    send_allowed = (now - last_cmd_time) >= MIN_DT_CMD
 
    # --- Detección de pelota por color ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
 
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 
    tiene_pelota = False
 
    if len(contornos) > 0:
        c = max(contornos, key=cv2.contourArea)
        (x, y), radio = cv2.minEnclosingCircle(c)
 
        if radio > 5:  # umbral mínimo para ruido
            tiene_pelota = True
 
            x = int(x)
            y = int(y)
            radio = int(radio)
 
            # Guardamos última posición de la pelota
            last_ball_x = x
            last_ball_y = y
 
            # Dibujar pelota
            cv2.circle(frame, (x, y), radio, (255, 0, 0), 2)
            cv2.circle(frame, (x, y), 3, (255, 0, 0), -1)
 
            # Dibujar líneas del centro calibrado
            cx = int(centerX)
            cy = int(centerY)
            cv2.line(frame, (cx, 0), (cx, h), (0, 255, 255), 1)
            cv2.line(frame, (0, cy), (w, cy), (0, 255, 255), 1)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)  # centro calibrado
 
            # Errores normalizados respecto al centro calibrado
            errx_img = (x - cx) / (w / 2)   # derecha +, izquierda -
            erry_img = (y - cy) / (h / 2)   # abajo +, arriba -
 
            # Posible intercambio de ejes
            if SWAP_AXES:
                errx_raw = erry_img
                erry_raw = errx_img
            else:
                errx_raw = errx_img
                erry_raw = erry_img
 
            # Mostrar errores crudos para debug
            cv2.putText(frame, f"Ex_raw:{errx_raw:+.2f} Ey_raw:{erry_raw:+.2f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
 
            # --- Control PD ---
            derx = (errx_raw - last_errx) / dt
            dery = (erry_raw - last_erry) / dt
 
            dxf = alpha * dxf + (1 - alpha) * derx
            dyf = alpha * dyf + (1 - alpha) * dery
 
            uX = KpX * errx_raw + KdX * dxf
            uY = KpY * erry_raw + KdY * dyf
 
            # Limitamos uX, uY a [-1,1]
            uX = float(np.clip(uX, -1.0, 1.0))
            uY = float(np.clip(uY, -1.0, 1.0))
 
            # Invertir si hace falta (lo cambias en vivo con 'x' y 'y')
            if INVERT_X:
                uX = -uX
            if INVERT_Y:
                uY = -uY
 
            # Offset de servos en grados
            servo_off_X = uX * MAX_SERVO_OFFSET_X
            servo_off_Y = uY * MAX_SERVO_OFFSET_Y
 
            # Ángulos lógicos absolutos
            ang_izq  = NEUT_IZQ  - servo_off_X
            ang_der  = NEUT_DER  + servo_off_X
            ang_vert = NEUT_VERT - servo_off_Y
 
            # Limitar a 0..180
            ang_izq  = int(np.clip(ang_izq,  0, 180))
            ang_der  = int(np.clip(ang_der,  0, 180))
            ang_vert = int(np.clip(ang_vert, 0, 180))
 
            cv2.putText(frame,
                        f"IZQ:{ang_izq} DER:{ang_der} VERT:{ang_vert}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2)
 
            # --- Enviar comando ANG: ---
            if send_allowed:
                try:
                    cmd = f"ANG:{ang_izq},{ang_der},{ang_vert}\n"
                    sock.send(cmd.encode())
                    # print("CMD ->", cmd.strip())
                    last_cmd_time = now
                except Exception as e:
                    print("⚠️ Error al enviar ANG:", e)
 
            last_errx = errx_raw
            last_erry = erry_raw
            last_time = now
 
    if not tiene_pelota:
        cv2.putText(frame, "PELOTA NO DETECTADA", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 255), 2)
        # Opcional: si pierdes la pelota, manda LOST para nivelar
        if send_allowed:
            try:
                sock.send(b"LOST\n")
                last_cmd_time = now
            except Exception as e:
                print("⚠️ Error al enviar LOST:", e)
 
    # Mostrar estado de flags
    status = f"invX:{INVERT_X}  invY:{INVERT_Y}  swap:{SWAP_AXES}  TEST:{TEST_MODE}"
    cv2.putText(frame, status, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
 
    cv2.imshow("Ball Balancing Control", frame)
    cv2.imshow("Mascara", mask)
 
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        # Tecla 'c' para nivelar manualmente (centro por firmware)
        try:
            sock.send(b"LOST\n")
            last_cmd_time = time.time()
            print("Comando LOST enviado (plataforma al centro).")
        except Exception as e:
            print("⚠️ Error al enviar LOST:", e)
    elif key == ord('x'):
        INVERT_X = not INVERT_X
        print("INVERT_X ->", INVERT_X)
    elif key == ord('y'):
        INVERT_Y = not INVERT_Y
        print("INVERT_Y ->", INVERT_Y)
    elif key == ord('s'):
        SWAP_AXES = not SWAP_AXES
        print("SWAP_AXES ->", SWAP_AXES)
    elif key == ord('b'):
        # Calibrar centro con la pelota en el centro físico
        if last_ball_x is not None and last_ball_y is not None:
            centerX = last_ball_x
            centerY = last_ball_y
            print(f"Centro calibrado en ({centerX}, {centerY})")
 
video.release()
sock.close()
cv2.destroyAllWindows()
```
---

## 7) Código — Firmware ESP32 (Arduino)

```python
#include <Arduino.h>
#include "BluetoothSerial.h"
 
BluetoothSerial SerialBT;
 
// === Buffer para lectura BT no bloqueante ===
String btBuffer;
 
// === Pines de los servos ===
#define SERVO_IZQ   25
#define SERVO_DER   15
#define SERVO_VERT  33
 
// === PWM ===
const uint32_t FREQ_HZ = 50;
const uint8_t  RES_BITS = 12;
const uint16_t DUTY_MIN = 205;   // ~1.0 ms
const uint16_t DUTY_MAX = 410;   // ~2.0 ms
 
// Convierte grados físicos 0..180 a duty
uint16_t dutyFromDeg(int deg){
  deg = constrain(deg,0,180);
  return map(deg,0,180,DUTY_MIN,DUTY_MAX);
}
 
// Convierte de ángulo lógico (0..180, centro 90) a físico (invertido)
int logicalToPhysical(int logicalDeg){
  logicalDeg = constrain(logicalDeg, 0, 180);
  // 0 lógico → 180 físico, 180 lógico → 0 físico
  return 180 - logicalDeg;
}
 
// Escribe usando grados lógicos
void writeServoLogical(int pin, int logicalDeg){
  int fisico = logicalToPhysical(logicalDeg);
  ledcWrite(pin, dutyFromDeg(fisico));
}
 
// Configurar servo con ángulo lógico inicial
void configServo(int pin, int initialLogical){
  pinMode(pin,OUTPUT);
  ledcAttach(pin,FREQ_HZ,RES_BITS);   // usa el pin como canal
  writeServoLogical(pin,initialLogical);
}
 
// === Rango y rampa ===
const int LIM_MIN = 0;
const int LIM_MAX = 180;
const int PASO_RAMPA = 45;          // tamaño de paso en rampa
const uint32_t DT_RAMP_MS = 2;
const uint32_t TIMEOUT_MS = 700;
 
// Estado en grados LÓGICOS
int posIzq = 90;
int posDer = 90;
int posVert= 60;
 
int tgtIzq = 90;
int tgtDer = 90;
int tgtVert= 60;
 
uint32_t tPrevRamp = 0;
uint32_t tLastCmd  = 0;
 
// Rampa suave hacia el objetivo
void aplicarRampa(){
  uint32_t now = millis();
  if(now - tPrevRamp < DT_RAMP_MS) return;
  tPrevRamp = now;
 
  auto go = [&](int actual,int target){
    if(actual < target) return min(actual + PASO_RAMPA, target);
    if(actual > target) return max(actual - PASO_RAMPA, target);
    return actual;
  };
 
  posIzq = go(posIzq, tgtIzq);
  posDer = go(posDer, tgtDer);
  posVert= go(posVert,tgtVert);
 
  // Escribimos usando grados LÓGICOS, se invierten adentro
  writeServoLogical(SERVO_IZQ, posIzq);
  writeServoLogical(SERVO_DER, posDer);
  writeServoLogical(SERVO_VERT,posVert);
}
 
// Parsea "ANG:izq,der,vert"
bool parseAngulos(const String &msg, int &aIzq, int &aDer, int &aVert){
  if (!msg.startsWith("ANG:")) return false;
  String data = msg.substring(4);  // después de "ANG:"
 
  int c1 = data.indexOf(',');
  if (c1 < 0) return false;
  int c2 = data.indexOf(',', c1 + 1);
  if (c2 < 0) return false;
 
  String sIzq  = data.substring(0, c1);
  String sDer  = data.substring(c1 + 1, c2);
  String sVert = data.substring(c2 + 1);
 
  sIzq.trim();
  sDer.trim();
  sVert.trim();
 
  aIzq  = sIzq.toInt();
  aDer  = sDer.toInt();
  aVert = sVert.toInt();
 
  return true;
}
 
// ======== SETUP ========
void setup(){
  Serial.begin(115200);
  SerialBT.begin("ESP32-BallPlatform");
 
  configServo(SERVO_IZQ, posIzq);
  configServo(SERVO_DER, posDer);
  configServo(SERVO_VERT,posVert);
 
  Serial.println("ESP32 listo");
  tLastCmd = millis();
}
 
// ======== LOOP ========
void loop(){
 
  // --- Lectura Bluetooth no bloqueante ---
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
 
    if (c == '\n') {
      // Tenemos una línea completa en btBuffer
      String msg = btBuffer;
      btBuffer = "";        // limpiar para el siguiente mensaje
 
      msg.trim();
      if (msg.length() > 0) {
        tLastCmd = millis();
 
        if (msg == "LOST") {
          // PC perdió mano/objetivo → "centro"
          tgtIzq  = 90;
          tgtDer  = 90;
          tgtVert = 60;
          Serial.println("Comando LOST: centro.");
 
        } else if (msg == "ZERO") {
          // 🔹 TODOS los servos a 180 físicos
          //    ⇒ 0 lógico por el mapeo invertido
          tgtIzq  = 0;
          tgtDer  = 0;
          tgtVert = 0;
          Serial.println("Comando ZERO: servos → 180° fisico");
 
        } else {
          int aIzq, aDer, aVert;
          if (parseAngulos(msg, aIzq, aDer, aVert)) {
            tgtIzq  = constrain(aIzq,  LIM_MIN, LIM_MAX);
            tgtDer  = constrain(aDer,  LIM_MIN, LIM_MAX);
            tgtVert = constrain(aVert, LIM_MIN, LIM_MAX);
            Serial.printf("ANG -> IZQ:%d DER:%d VERT:%d\n", tgtIzq, tgtDer, tgtVert);
          } else {
            Serial.print("Comando desconocido: ");
            Serial.println(msg);
          }
        }
      }
    } else if (c != '\r') {
      // Acumulamos caracteres, ignorando CR
      btBuffer += c;
    }
  }
 
  // Si pasa mucho tiempo sin recibir comandos, vuelve al centro
  if(millis() - tLastCmd > TIMEOUT_MS){
    tgtIzq = 90;
    tgtDer = 90;
    tgtVert= 60;
  }
 
  aplicarRampa();
  delay(1);
}
 
```
---

## 7) Descripción de Funcionamiento

- En el modo **Control por Mano**:
  - La cámara captura la imagen de la mano y se calculan ángulos de orientación (roll y pitch).
  - Dichos ángulos se convierten en valores lógicos (0–180°) para los servomotores.
  - Se envían comandos por Bluetooth con el formato `ANG:izq,der,vert`, además de comandos auxiliares como `ZERO` para referencia y `LOST` para regresar la plataforma a posición neutral.
  - Los servomotores ejecutan los cambios de ángulo con una rampa de transición para evitar movimientos bruscos.

- En el modo **Control PD por Cámara Frontal**:
  - La cámara identifica la posición de la pelota mediante segmentación por color.
  - Con la diferencia entre la posición deseada y la real, se calcula un error en los ejes X y Y.
  - Se aplica un controlador **PD**, donde:
    - `Kp` controla la corrección proporcional.
    - `Kd` filtra la derivada para mejorar estabilidad.
  - Los valores calculados se convierten a grados y se envían al ESP32 con el mismo formato de comando `ANG`.
  - El firmware interpola suavemente los movimientos para mejorar la precisión.

---

## 8) Resultados Obtenidos

- El control PD permitió corregir la posición de la pelota sobre la plataforma de manera eficiente.
- Ajustar los valores de ganancia fue clave para lograr una estabilización más rápida.
- Se comprobó que ampliar la distancia entre los servomotores mejoró el rango y la sensibilidad del control mecánico.
- La interpolación por rampa minimizó vibraciones y aumentó la estabilidad visual y mecánica del sistema.

---
## Video del Funcionamiento Final
🎥 https://www.youtube.com/watch?v=ID_DEL_VIDEO](https://www.youtube.com/watch?v=zepItAOh-Lk
🎥 https://www.youtube.com/watch?v=MQ0QVBZc3m0

---

## 9) Conclusiones

- Fue necesario trabajar con **ganancias PD diferenciadas** según el comportamiento observado (mayor proporcionalidad para mejorar el tiempo de respuesta).
- Se determinó que **ampliar la distancia física entre los servos** permitió mayor precisión en la inclinación y favoreció el control de la pelota.
- Se identificó la importancia de optimizar la **velocidad de respuesta de los servomotores**, ya que influyó directamente en la estabilidad de la plataforma.
- La estructura de control basada en visión y retroalimentación demostró ser viable para aplicaciones dinámicas de estabilización en tiempo real.

---

