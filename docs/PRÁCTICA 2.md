# 📚 Práctica 2: Control de LED NeoPixel con Arduino mediante comunicación Serial.

---

## 1) Resumen

- **Equipo / Autor(es):**  _Karen Najera y Arith Maldonado_
- **Curso / Asignatura:** _Elementos programables II_  
- **Fecha:** _20/08/2025_  
- **Descripción breve:** _En esta práctica se implementa un programa en Arduino para controlar un LED NeoPixel a través de comandos enviados por el monitor serial. El usuario puede enviar instrucciones como “red”, “green” o “blue” y el LED cambiará su color de acuerdo al mensaje recibido. La librería Adafruit_NeoPixel permite el manejo de este tipo de LEDs direccionables de manera sencilla._

!!! tip "Consejo"
    Mantén este resumen corto (máx. 5 líneas). Lo demás va en secciones específicas.

---

## 2) Objetivos

- **General:** _Comprender el funcionamiento básico de un LED NeoPixel y su control mediante comunicación serial en Arduino._
- **Específicos:**
  - _Configurar el puerto serial para recibir datos desde el monitor de Arduino ID_
  - _Implementar la librería Adafruit_NeoPixel para inicializar y controlar el LED._
  - _Programar condiciones que permitan el cambio de color del LED en función del mensaje recibido._

## 3) Alcance y Exclusiones

- **Incluye:** _El código desarrollado tiene como finalidad recibir comandos de texto a través del puerto serial y traducirlos en cambios de color en un LED NeoPixel._

Solo se controla un LED (NUMPIXELS = 1).

-_El usuario puede escribir “red”, “green” o “blue” en el monitor serial._

-_Cada mensaje recibido activa el LED con la intensidad y color definido._

-_Se incorpora un retardo de 1 segundo para visualizar claramente cada cambio._

-_La lógica puede escalarse fácilmente para más LEDs o más colores.._

---

## 4) Resultados

 _Al realizar la práctica se comprobó que el sistema respondió de manera adecuada a los comandos enviados desde el monitor serial. Cada vez que se ingresó la palabra “red”, el LED NeoPixel se iluminó en color rojo con la intensidad programada; al escribir “green”, el LED cambió correctamente a color verde; y al introducir “blue”, se encendió en color azul._
**Código**
_El retardo de un segundo facilitó la observación de cada cambio de color antes de recibir un nuevo comando, lo que permitió validar visualmente el funcionamiento del programa. Además, se constató que el uso del carácter coma (,) como delimitador en la lectura de cadenas evitó errores de interpretación en los mensajes._




<img src="../recursos/imgs/P2.png" alt="..." width="400px">


_En general, el comportamiento del LED fue estable, sin presentar fallos de comunicación ni bloqueos durante las pruebas, lo cual confirma la correcta implementación de la librería y de la lógica de control._

**Conocimientos previos**
- _Programación básica en X_
- _Electrónica básica_
- _Git/GitHub_

---

## 6) Código
```CPP
// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library
 
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
 
// Which pin on the Arduino is connected to the NeoPixels?
#define PIN 8    // On Trinket or Gemma, suggest changing this to 1
 
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 1 // Popular NeoPixel ring size
 
// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
 
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels
 
String msj="";
 
void setup() {
   Serial.begin(115200);//Inicia Serial a 115200 bauds o 115200 char/s
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}
 
void loop() {
 
   if(Serial.available()>0){ //Entrar al if solo si se recibe algo por el serial
    msj = Serial.readStringUntil(',');//Ciclo hasta leer el char marcado
    Serial.println(msj); //Imprime el comando recibido
  }
 
  if(msj=="red")
  {
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay (1000);
  }
 
  else if(msj=="green"){
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();
    delay (1000);
  }
  else if(msj=="blue"){
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(0, pixels.Color(0, 0, 150));
    pixels.show();
    delay (1000);
  }
   
 
   
}
```
## 5) Conclusión
_Con esta práctica se demostró el uso básico de la librería Adafruit_NeoPixel para controlar LEDs direccionables mediante comunicación serial. El programa permite al usuario interactuar directamente con el hardware enviando comandos simples desde el monitor serial, logrando así un cambio de color en el LED. Esta lógica se puede ampliar a tiras LED más grandes y a una gama más amplia de colores, lo cual representa una aplicación fundamental en proyectos de iluminación decorativa, robótica y señalización._
