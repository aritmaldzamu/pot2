# 📚Práctica 10 (P10): Procesamiento de Monedas con Operaciones Morfológicas
---

## 1) Resumen

- **Equipo / Autor(es):**  _Arith Maldonado_
- **Curso / Asignatura:** _Elementos programables II_  
- **Fecha:** _29/09/2025_  
- **Descripción breve:** _En esta práctica se procesó una imagen de un conjunto de monedas aplicando filtrado por color, umbralización y operaciones morfológicas (erosión y dilatación) para aislar los objetos de interés y retirar el fondo. Se identificaron contornos y se generó una máscara para extraer únicamente las monedas detectadas mediante procesamiento digital de imágenes._


---

## 2) Objetivos

- **General:** _Aplicar operaciones de morfología digital utilizando OpenCV para segmentar objetos en una imagen y aislar regiones mediante filtrado y contornos._
- **Específicos:**
  - _Separar canales de color para descartar fondo no deseado._
  - _Aplicar umbral para resaltar bordes brillantes presentes en las monedas._

## 3) Alcance y Exclusiones

- **Incluye:** 
-_Aplicación de umbralización para resaltar los bordes brillantes de las monedas._

-_Uso de operaciones morfológicas para cierre de contornos._

---

## 4) Resultados
Al procesar la imagen, el filtrado por color permitió eliminar el fondo rojo sin afectar las monedas.


La umbralización resaltó las zonas brillantes correspondientes al aro metálico de cada moneda.

Las operaciones morfológicas de dilatación y erosión permitieron cerrar los contornos incompletos, reduciendo ruido y rellenando discontinuidades.

La detección de contornos permitió identificar cada moneda aislada, generando una máscara que extrae únicamente las regiones válidas de la imagen.

Los resultados muestran con claridad las monedas aisladas del resto de la imagen, confirmando el funcionamiento del procesamiento aplicado.

---

## 6) Archivos Adjuntos

``` cpp

import cv2
import numpy as np
# --- carga y resize ---
img = cv2.imread("images/Coins4.jpg")
h, w = img.shape[:2]
MAX_W, MAX_H = 1080, 720
scale = min(MAX_W / w, MAX_H / h, 1.0)
if scale < 1.0:
    img = cv2.resize(img, (int(w*scale), int(h*scale)), interpolation=cv2.INTER_AREA)
cv2.imshow("Original", img)
# --- canales para filtrar rojo (BGR) ---
b = img[:, :, 0]
g = img[:, :, 1]
r = img[:, :, 2]
# 1) FILTRO: quitar fondo rojo (en BGR)
R_LOW_BG, G_HIGH_BG, B_HIGH_BG = 145, 110, 110
mask_bg_red = cv2.bitwise_and(
    cv2.inRange(r, R_LOW_BG, 255),
    cv2.bitwise_and(cv2.inRange(g, 0, G_HIGH_BG),
                    cv2.inRange(b, 0, B_HIGH_BG))
)
mask_not_red = cv2.bitwise_not(mask_bg_red)
# 2) ESCALA DE GRISES
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("Gris", gray)
# 3) UMBRAL ALTO PARA QUEDARTE CON EL ARO (borde brillante)
T_RING = 80
_, ring = cv2.threshold(gray, T_RING, 255, cv2.THRESH_BINARY)
# quitar cualquier aro que esté sobre el tapete rojo
ring = cv2.bitwise_and(ring, mask_not_red)
cv2.imshow("Aro en gris", ring)
# 4) MORFOLOGÍA: cerrar el aro (mismas ops de clase)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
ring_closed = cv2.dilate(ring, kernel, iterations=3)
ring_closed = cv2.erode (ring_closed, kernel, iterations=4)
cv2.imshow("Aro cerrado", ring_closed)
# 5) CONTORNOS sobre el aro cerrado
contours, _ = cv2.findContours(ring_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
img_contours = img.copy()
cv2.drawContours(img_contours, contours, -1, (0, 255, 0), 2)
cv2.imshow("Monedas por contorno", img_contours)
mask_fill = np.zeros_like(gray)
cv2.drawContours(mask_fill, contours, -1, 255, thickness=-1)  # -1 = rellenar
cv2.imshow("Mascara rellena desde contorno", mask_fill)
result = cv2.bitwise_and(img, img, mask=mask_fill)
cv2.imshow("Monedas rellenas (desde contorno)", result)
while True:
    if cv2.waitKey(20) == 27:
        break
cv2.destroyAllWindows()
```

## 5) Conclusión
_La práctica permitió comprobar que las operaciones morfológicas son fundamentales para la segmentación efectiva de objetos en una escena, especialmente cuando existen irregularidades o ruido visual. El filtrado basado en canales de color permitió descartar el fondo sin afectar el objeto de interés, mientras que la umbralización destacó características relevantes como el brillo del aro. Se concluye que el procesamiento mediante erosión, dilatación y contornos es una técnica eficiente para extraer objetos de interés en imágenes, representando una base sólida para futuras aplicaciones como conteo, clasificación o inspección automática._

