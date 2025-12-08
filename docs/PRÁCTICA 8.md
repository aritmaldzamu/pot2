# 📚 Práctica 8: Control de LEDs con ESP32 mediante Servidor Web Interactivo
---

## 1) Resumen

- **Equipo / Autor(es):**  _Karen Najera y Arith Maldonado_
- **Curso / Asignatura:** _Elementos programables II_  
- **Fecha:** _29/09/2025_  
- **Descripción breve:** _En esta práctica se configuró un ESP32 como servidor web para controlar dos LEDs mediante sliders (control PWM) y uno adicional a través de un botón de encendido/apagado en una interfaz web. También se incorporó una caja de texto para enviar datos al ESP32 desde la misma página. Todo esto permite simular una interfaz de control básica para un sistema embebido conectado a red, sentando las bases para aplicaciones IoT._


---

## 2) Objetivos

- **General:** _Comprender e implementar un servidor web sobre el ESP32 que permita el control interactivo de periféricos en tiempo real a través de una interfaz web._
- **Específicos:**
  - _Configurar una página HTML embebida que incluya controles interactivos como sliders, botones y cajas de texto._
  - _Implementar funciones en el servidor web para recibir valores y aplicarlos al hardware físico (pines PWM y digitales)._
  - _Utilizar funciones de mapeo PWM para controlar brillo o velocidad de motores._

## 3) Alcance y Exclusiones

- **Incluye:** 
-_Control de dos salidas PWM mediante sliders en una interfaz web._

-_Control de un LED (ON/OFF) mediante un botón en la misma página._

-_Recepción de texto desde una caja de entrada y su impresión en el monitor serial._

-_No se implementa autenticación ni seguridad en el servidor web._

-_No se utilizan librerías externas avanzadas, únicamente WiFi.h y WebServer.h.._

-_No se contempla la regulación analógica directa del LED controlado por botón (solo ON/OFF)_

---

## 4) Resultados
_Al subir el programa al ESP32 y conectarlo a la red WiFi, el monitor serial muestra la dirección IP local asignada. Esta dirección es introducida en un navegador dentro de la misma red para abrir la interfaz web.

_La página principal permite controlar un LED mediante un botón que cambia dinámicamente su estado de “ON” a “OFF”. Para los otros dos LEDs, se accede a través de rutas específicas (/on1, /off1, /on2, /off2) en la URL, lo cual permite verificar el funcionamiento del servidor al interpretar diferentes solicitudes HTTP._

_El botón HTML permite encender o apagar un LED (pin LED_BUILTIN) de forma remota, con cambios dinámicos en el texto del botón y estado del LED._

_Los sliders permiten modificar valores de 0 a 180, los cuales son mapeados a un rango de señal PWM (205 a 410). Esto se refleja en la intensidad de salida en los pines definidos por #define pwm 3 y #define pwm1 2, ideal para el control de brillo de LEDs o velocidad de servomotores._

_Al escribir en la caja de texto y presionar “Enviar”, el valor se muestra en el monitor serial como confirmación de recepción._

_Todas las acciones generan peticiones HTTP (GET) que el servidor del ESP32 interpreta correctamente, generando una respuesta inmediata y modificando salidas físicas._

---

## 6) Archivos Adjuntos

``` cpp

#include <WiFi.h>
#include <WebServer.h>

#define pwm 3
#define pwm1 2

const char* ssid = "iPhone";
const char* password = "karennajera";

WebServer servidor(80);

const int ledPin = LED_BUILTIN;
String ledState = "OFF";
int sliderValue = 0;
int sliderValue1 = 0;

const char htmlTemplate[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8">
    <title>Control de LED ESP32</title>
  </head>
  <body>
    <h1>Control de LED ESP32</h1>
    <p>El LED está %LED_STATE%</p>
    <a href="/%LINK%"><button>%BUTTON_TEXT%</button></a>
    <br><br>
    <h2>Control del Slider</h2>
    <input type="range" min="0" max="180" value="%SLIDER_VALUE%" id="slider" oninput="updateSlider(this.value)">
    <span id="sliderValue">%SLIDER_VALUE%</span>
    <br><br>
    <h2>Segundo Slider</h2>
    <input type="range" min="0" max="180" value="%SLIDER_VALUE1%" id="slider1" oninput="updateSlider1(this.value)">
    <span id="sliderValue1">%SLIDER_VALUE1%</span>
    <br><br>
    <h2>Ingresar Texto</h2>
    <input type="text" id="txtInput" placeholder="Escribe algo...">
    <button onclick="sendText()">Enviar</button>

    <script>
      function updateSlider(value) {
        document.getElementById('sliderValue').innerText = value;
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/slider?value=" + encodeURIComponent(value), true);
        xhr.send();
      }
      function updateSlider1(value) {
        document.getElementById('sliderValue1').innerText = value;
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/slider1?value=" + encodeURIComponent(value), true);
        xhr.send();
      }
      function sendText() {
        var textValue = document.getElementById('txtInput').value;
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/textbox?value=" + encodeURIComponent(textValue), true);
        xhr.send();
      }
    </script>
  </body>
</html>
)rawliteral;

void handleroot() {
  String html = String(htmlTemplate);
  html.replace("%LED_STATE%", ledState);
  html.replace("%SLIDER_VALUE%", String(sliderValue));
  html.replace("%SLIDER_VALUE1%", String(sliderValue1));
  if (ledState == "OFF") {
    html.replace("%LINK%", "ON");
    html.replace("%BUTTON_TEXT%", "Encender");
  } else {
    html.replace("%LINK%", "OFF");
    html.replace("%BUTTON_TEXT%", "Apagar");
  }
  servidor.send(200, "text/html", html);
}

void handleOn() {
  digitalWrite(ledPin, HIGH);
  ledState = "ON";
  handleroot();
}

void handleOff() {
  digitalWrite(ledPin, LOW);
  ledState = "OFF";
  handleroot();
}

void handleSlider() {
  if (servidor.hasArg("value")) {
    sliderValue = servidor.arg("value").toInt();
    Serial.println("Valor del slider: " + String(sliderValue));
    int duty = map(sliderValue, 0, 180, 205, 410);
    ledcWrite(pwm, duty);
  }
  servidor.send(200, "text/plain", "OK");
}

void handleSlider1() {
  if (servidor.hasArg("value")) {
    sliderValue1 = servidor.arg("value").toInt();
    Serial.println("Valor del segundo slider: " + String(sliderValue1));
    int duty1 = map(sliderValue1, 0, 180, 205, 410);
    ledcWrite(pwm1, duty1);
  }
  servidor.send(200, "text/plain", "OK");
}

void handleTextbox() {
  if (servidor.hasArg("value")) {
    String textValue = servidor.arg("value");
    Serial.println("Texto recibido: " + textValue);
  }
  servidor.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  ledcAttachChannel(pwm, 50, 12, 0);
  ledcAttachChannel(pwm1, 50, 12, 1);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.println(WiFi.localIP());

  servidor.on("/", handleroot);
  servidor.on("/ON", handleOn);
  servidor.on("/OFF", handleOff);
  servidor.on("/slider", handleSlider);
  servidor.on("/slider1", handleSlider1);
  servidor.on("/textbox", handle
```

## 5) Conclusión
_La práctica permitió comprobar que el ESP32 puede actuar como un servidor web funcional que permite controlar periféricos en tiempo real desde una interfaz gráfica. El sistema implementado constituye una base sólida para aplicaciones más complejas del Internet de las Cosas (IoT), como domótica, monitoreo remoto o automatización industrial._

_La integración entre HTML, JavaScript y el servidor embebido del ESP32 permite la creación de interfaces intuitivas que facilitan la interacción hombre-máquina sin necesidad de aplicaciones externas. Además, el manejo de señales PWM desde el navegador extiende la utilidad del sistema para control preciso de actuadores. Se concluye que el ESP32 es una herramienta poderosa, flexible y accesible para el desarrollo de sistemas embebidos conectados a red.._