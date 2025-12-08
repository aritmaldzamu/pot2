# 📚 Práctica 7: Control de LEDs con ESP32 mediante Servidor Web
---

## 1) Resumen

- **Equipo / Autor(es):**  _Karen Najera y Arith Maldonado_
- **Curso / Asignatura:** _Elementos programables II_  
- **Fecha:** _22/09/2025_  
- **Descripción breve:** _En esta práctica se configuró un ESP32 como servidor web para controlar tres LEDs a través de una red WiFi local. Se implementó una interfaz web sencilla que permite encender y apagar un LED principal mediante un botón en la página, mientras que los otros LEDs pueden controlarse utilizando diferentes rutas URL. El sistema demuestra cómo integrar redes inalámbricas con hardware físico en proyectos de IoT._


---

## 2) Objetivos

- **General:** _Comprender el funcionamiento de un servidor web en el ESP32 y su aplicación en el control remoto de dispositivos electrónicos._
- **Específicos:**
  - _Configurar el ESP32 para conectarse a una red WiFi._
  - _Implementar un servidor web que responda a diferentes rutas (URLs)._
  - _Encender y apagar LEDs mediante botones en una página web y mediante acceso directo a URL._

## 3) Alcance y Exclusiones

- **Incluye:** 
-_Control de tres LEDs independientes a través de rutas específicas en el servidor._

-_Prueba del sistema accediendo desde un navegador web en la misma red._

-_Conexión del ESP32 a una red WiFi local._

-_No se implementa autenticación ni seguridad en el servidor web._

-_No se utilizan librerías externas de servidores avanzados, únicamente la librería nativa WebServer.h._

-_El control se limita a encendido y apagado, sin regulación de intensidad (PWM)._

---

## 4) Resultados
_Al cargar el programa en el ESP32 y abrir el monitor serial, se muestra la dirección IP asignada por la red WiFi, la cual se introduce en el navegador para acceder al servidor._

_La página principal permite controlar un LED mediante un botón que cambia dinámicamente su estado de “ON” a “OFF”. Para los otros dos LEDs, se accede a través de rutas específicas (/on1, /off1, /on2, /off2) en la URL, lo cual permite verificar el funcionamiento del servidor al interpretar diferentes solicitudes HTTP._

_El funcionamiento observado fue el siguiente:_
_Al presionar el botón en la página, el LED conectado al pin 20 cambia entre encendido y apagado._
_Al introducir manualmente la ruta /on1 o /off1, el LED en el pin 19 responde correctamente._
_De igual manera, con las rutas /on2 y /off2 se controla el LED del pin 21_
_Esto valida la correcta integración entre la programación del ESP32, el servidor web y el hardware físico._




<img src="../recursos/imgs/P7.jpg" alt="..." width="400px">

<img src="../recursos/imgs/P7'.jpg" alt="..." width="400px">
---

## 6) Archivos Adjuntos

```CPP
#include <WiFi.h>
#include <WebServer.h>
 
const char* ssid = "iPhone";
const char* password = "karennajera";
 
WebServer server(80);
 
const int ledPin = 20;  
const int ledPin1 = 19;
const int ledPin2 = 21;
String ledState = "OFF";
String ledState1 = "OFF";
String ledState2 = "OFF";
 
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>ESP32 LED Control</title></head><body>";
  html += "<h1>ESP32 LED Control</h1>";
  html += "<p>LED is " + ledState + "</p>";
  if (ledState == "OFF")
    html += "<a href=\"/on\"><button>Turn On</button></a>";
  else
    html += "<a href=\"/off\"><button>Turn Off</button></a>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}
 
void handleOn() {
  digitalWrite(ledPin, HIGH);
  ledState = "ON";
  handleRoot();
}
 
void handleOff() {
  digitalWrite(ledPin, LOW);
  ledState = "OFF";
  handleRoot();
}
 
void handleOn1() {
  digitalWrite(ledPin1, HIGH);
  ledState1 = "ON";
  handleRoot();
}
void handleOff1() {
  digitalWrite(ledPin1, LOW);
  ledState1 = "OFF";
  handleRoot();
}
void handleOn2() {
  digitalWrite(ledPin2, HIGH);
  ledState2 = "ON";
  handleRoot();
}
void handleOff2() {
  digitalWrite(ledPin2, LOW);
  ledState2 = "OFF";
  handleRoot();
}
 
void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
 
  server.on("/", handleRoot);
  server.on("/on", handleOn);
  server.on("/off", handleOff);
  server.on("/on1", handleOn1);
  server.on("/off1", handleOff1);
  server.on("/on2", handleOn2);
  server.on("/off2", handleOff2);
 
  server.begin();
  Serial.println("Server started");
}
 
void loop() {
  server.handleClient();
}


```

## 5) Conclusión
_La práctica permitió comprobar que el ESP32 puede actuar como un servidor web capaz de controlar periféricos en tiempo real. A través de una interfaz sencilla en HTML, se logró el encendido y apagado de LEDs de forma remota desde cualquier dispositivo conectado a la misma red._
_Este ejercicio constituye un ejemplo básico de aplicaciones IoT, donde los microcontroladores conectados a internet permiten interactuar con el entorno físico. Los conocimientos adquiridos son fundamentales para proyectos más complejos como automatización del hogar, control de maquinaria industrial o monitoreo de sistemas._