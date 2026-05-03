# USV_python

Este proyecto consiste en un prototipo funcional de un vehículo marino autónomo (USV) basado en **Raspberry Pi 4**. Utiliza propulsión diferencial y un sistema de guiado mediante GPS e IMU.

## 🚀 Funcionalidades
- **Control Autónomo:** Navegación por puntos de referencia (waypoints) mediante la fórmula de Haversine.
- **Control PID:** Algoritmo de corrección de rumbo sintonizado para estabilidad en agua ($K_p=1.8$, $K_i=0.01$, $K_d=0.15$).
- **Estación de Control Terrestre:** Interfaz web asíncrona (PWA) con telemetría en tiempo real y vídeo FPV.

## 🛠️ Hardware
- **Cerebro:** Raspberry Pi 4 Model B.
- **Sensórica:** GPS NEO-M8N, IMU BNO055 y sonda térmica DS18B20.
- **Propulsión:** Motores Brushed Injora 550 29T con ESC de 60A.

## 📂 Estructura del Repositorio
- `servidor_web.py`: Archivo principal con la lógica de control y servidor Flask.
- `/templates`: Interfaz de usuario en HTML.
- `/static`: Archivos de estilo y librerías CSS (Tailwind).
