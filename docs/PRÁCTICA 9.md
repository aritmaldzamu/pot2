# 🛰️ Práctica 9 (P9): Comunicación ESP-NOW con 3 botones y 3 LEDs

---

## 1) Resumen
- **Rol del nodo:** ESP32-C6 en **WIFI_STA** usando **ESP-NOW**. Lee **3 botones** (GPIO 2, 3, 4) y **envía** su estado a **tres peers** (por MAC). Además **recibe** paquetes y conmuta **3 LEDs** (GPIO 10, 11, 12) según el **origen** y el **payload**.

---

## 2) Objetivos
- Configurar **ESP-NOW** y registrar **tres peers** por dirección MAC.
- Enviar el estado de cada botón al peer correspondiente.
- Recibir datos y accionar LEDs en función de la **MAC fuente** y del campo **b** del payload.
- Verificar el funcionamiento mediante **logs** en el Monitor Serie.

---

## 3) Materiales
- ESP32-C6 Dev Module  
- 3 pulsadores (conexión pull-down o pull-up)  
- 3 LEDs con resistencias (GPIO 10, 11, 12)  
- Protoboard y jumpers  
- Arduino IDE (core ESP32)

> **Nota eléctrica:** Comparte **GND** entre todos los elementos y usa resistencias para LEDs. Define consistentemente pull-up/pull-down.

---

## 4) Código (P9)

```cpp
#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0x7C, 0x2C, 0x67, 0x55, 0xD6, 0x88}; //mAC ADREES DEL OTRO ESP32
uint8_t broadcastAddress2[] = {0x7C, 0x2C, 0x67, 0x55, 0xD8, 0xDC};
uint8_t broadcastAddress3[] = {0x7C, 0x2C, 0x67, 0x55, 0xD4, 0xE0};
 
int led1 = 10;
int led2 = 11;
int led3 = 12;
int boton1 = 2;
int boton2 = 3;
int boton3 = 4;
 
//Enviar
// Estructura de datos (máx. 250 bytes)
 
typedef struct struct_msj {
  char a[32];
  int b;
  float c;
  bool d;
} struct_msj;
struct_msj datosEnviados;
struct_msj datosRecibidos;
struct_msj datosEnviados2;
struct_msj datosRecibidos2;
struct_msj datosEnviados3;
struct_msj datosRecibidos3;
 
esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfo2;
esp_now_peer_info_t peerInfo3;
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *tempData, int tam) {
  memcpy(&datosRecibidos, tempData, sizeof(datosRecibidos));
  // Imprimir información de la fuente (opcional)
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);
  Serial.printf("Datos recibidos de: %s\n", macStr);
  //Serial.printf("Bytes recibidos: %d\n", tam);
  if(info->src_addr[5] == 0x88)
  {
    if (datosRecibidos.b == 1) {
      digitalWrite(led1, HIGH);
    }
    else {
      digitalWrite(led1, LOW);
    }
  }
  if(info->src_addr[5] == 0xE0)
  {
    if (datosRecibidos.b==1) {
      digitalWrite(led2, HIGH);
    }
    else {
      digitalWrite(led2, LOW);
    }
  }
  if(info->src_addr[5] == 0xDC)
  {
    if (datosRecibidos.b == 1) {
      digitalWrite(led3, HIGH);
    }
    else {
      digitalWrite(led3, LOW);
    }
  }
 
  Serial.printf("Int: %d\n", datosRecibidos.b);
  //Serial.printf("Float: %.2f\n", datosRecibidos.c);
  //Serial.printf("Bool: %d\n\n", datosRecibidos.d);
}
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(boton1, INPUT);
  pinMode(boton2, INPUT);
  pinMode(boton3, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("No se pudo agregar el peer");
    return;
  }
  // Configurar peer
  memset(&peerInfo2, 0, sizeof(peerInfo2));
  memcpy(peerInfo2.peer_addr, broadcastAddress2, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo2) != ESP_OK)
  {
    Serial.println("No se pudo agregar el peer");
    return;
  }
    memset(&peerInfo3, 0, sizeof(peerInfo3));
  memcpy(peerInfo3.peer_addr, broadcastAddress3, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo3) != ESP_OK)
  {
    Serial.println("No se pudo agregar el peer");
    return;
  }
  Serial.println("ESP-NOW listo para enviar");
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop()
{
  //aqui imprime estado botones
  Serial.print("Botón 1: ");
  Serial.print(digitalRead(boton1));
  Serial.print(" | Botón 2: ");
  Serial.print(digitalRead(boton2));
  Serial.print(" | Botón 3: ");
  Serial.println(digitalRead(boton3));
 
  strcpy(datosEnviados.a, "Hola Mundo");
  datosEnviados.c = 3.14;
  datosEnviados.d = true;
 
 
  if(digitalRead(boton1) == HIGH){
    datosEnviados.b = 1;
    esp_err_t result1 = esp_now_send(broadcastAddress,
                                    (uint8_t *)&datosEnviados,
                                    sizeof(datosEnviados));
  }
  else if(digitalRead(boton1) == LOW){
    datosEnviados.b = 0;
    esp_err_t result1 = esp_now_send(broadcastAddress,
                                  (uint8_t *)&datosEnviados,
                                  sizeof(datosEnviados));
  }
  if(digitalRead(boton2) == HIGH){
  datosEnviados.b = 1;
  esp_err_t result2 = esp_now_send(broadcastAddress2,
                                  (uint8_t *)&datosEnviados,
                                  sizeof(datosEnviados));  
  }
  else if(digitalRead(boton2) == LOW){
    datosEnviados.b = 0;
    esp_err_t result2 = esp_now_send(broadcastAddress2,
                                  (uint8_t *)&datosEnviados,
                                  sizeof(datosEnviados));
  }
 
  if(digitalRead(boton3) == HIGH){
  datosEnviados.b = 1;
  esp_err_t result3 = esp_now_send(broadcastAddress3,
                                  (uint8_t *)&datosEnviados,
                                  sizeof(datosEnviados));
  }
  else if(digitalRead(boton3) == LOW){
    datosEnviados.b = 0;
    esp_err_t result3 = esp_now_send(broadcastAddress3,
                                  (uint8_t *)&datosEnviados,
                                  sizeof(datosEnviados));
  }
 
  delay(2000);
}

```
