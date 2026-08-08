# ====================================================================
# CÓDIGO PRINCIPAL
# ====================================================================
import threading
import time
import math
import os
import sys
import glob
import pigpio 
from flask import Flask, render_template, request, jsonify

import serial
import board
import busio
import adafruit_bno055

# --- INICIALIZAR FLASK ---
app = Flask(__name__)

# --- CONFIGURACIÓN DE PINES Y ESCs
PIN_ESC_IZQ = 27 
PIN_ESC_DER = 17

PULSE_STOP     = 1500
PULSE_MIN_FWD  = 1530
PULSE_MAX_FWD  = 1900 
PULSE_MIN_REV  = 1450
PULSE_MAX_REV  = 1100 
FACTOR_POTENCIA = 0.3 

# --- SENSORES Y ESTADO GLOBAL ---
IMU_ADDRESS = 0x29  
OFFSET_RUMBO = 0    
BAUD_RATE_GPS = 9600

datos_sensores = {
    "rumbo": 0.0, 
    "latitud": 0.0, 
    "longitud": 0.0, 
    "calibracion": 0,
    "distancia_wpt": 0.0, 
    "temperatura": 0.0
}

# --- CONFIGURACIÓN PID ---
kp = 1.8   # Proporcional
ki = 0.01  # Integral
kd = 0.5   # Derivativo

error_previo = 0
integral = 0
last_time = time.time()

MODO_AUTO = False
lista_waypoints = []

# --- CONFIGURACIÓN FAILSAFE ---
FAILSAFE_TIMEOUT = 2.0  # segundos sin comunicación antes de detener el USV
ultimo_contacto = time.monotonic()
failsafe_activado = False

# --- INICIALIZACIÓN PIGPIO ---
pi = pigpio.pi()
if not pi.connected:
    print("[ERROR CRÍTICO] Ejecuta sudo pigpiod")
    sys.exit() 

pi.set_mode(PIN_ESC_IZQ, pigpio.OUTPUT)
pi.set_mode(PIN_ESC_DER, pigpio.OUTPUT)
pi.set_servo_pulsewidth(PIN_ESC_IZQ, PULSE_STOP)
pi.set_servo_pulsewidth(PIN_ESC_DER, PULSE_STOP)

# ====================================================================
# FUNCIONES MATEMÁTICAS (Haversine)
# ====================================================================
def calcular_distancia(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calcular_rumbo_deseado(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

# ====================================================================
# LECTURA DE SENSORES
# ====================================================================
def leer_imu():
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor_imu = None
    while True:
        try:
            if sensor_imu is None: 
                sensor_imu = adafruit_bno055.BNO055_I2C(i2c, address=IMU_ADDRESS)
                print("[IMU] ✅ Sensor BNO055 conectado y leyendo correctamente.")
            
            euler = sensor_imu.euler
            calibracion = sensor_imu.calibration_status 
            
            if calibracion:
                datos_sensores["calibracion"] = calibracion[3] 
            if euler is not None and euler[0] is not None:
                datos_sensores["rumbo"] = (euler[0] + OFFSET_RUMBO) % 360
        except Exception as e:
            print(f"[IMU ERROR] ❌ Fallo de lectura: {e}")
            sensor_imu = None 
            time.sleep(0.5)
        time.sleep(0.1)

def leer_gps():
    puertos = ["/dev/serial0", "/dev/ttyS0", "/dev/ttyAMA0"]
    puerto_activo = next((p for p in puertos if os.path.exists(p)), None)
    if not puerto_activo: return
    try: gps = serial.Serial(puerto_activo, baudrate=BAUD_RATE_GPS, timeout=1)
    except: return

    while True:
        try:
            linea = gps.readline().decode('utf-8', errors='ignore').strip()
            if linea.startswith("$GNGGA") or linea.startswith("$GPGGA"):
                partes = linea.split(',')
                if len(partes) > 5 and partes[2] != "":
                    lat_raw, lon_raw = float(partes[2]), float(partes[4])
                    lat_deg = int(lat_raw / 100)
                    lat_dec = lat_deg + ((lat_raw - (lat_deg * 100)) / 60.0)
                    if partes[3] == 'S': lat_dec *= -1
                    lon_deg = int(lon_raw / 100)
                    lon_dec = lon_deg + ((lon_raw - (lon_deg * 100)) / 60.0)
                    if partes[5] == 'W': lon_dec *= -1
                    
                    datos_sensores["latitud"] = lat_dec
                    datos_sensores["longitud"] = lon_dec
        except: pass
        
# ====================================================================
# LECTURA TEMPERATURA
# ====================================================================        
def hilo_temperatura():
    base_dir = '/sys/bus/w1/devices/'
    try:
        # Busca automáticamente cualquier carpeta que empiece por "28-"
        device_folder = glob.glob(base_dir + '28*')[0]
        device_file = device_folder + '/w1_slave'
        print(f"[TEMP] ✅ Sensor detectado en: {device_folder}")
    except:
        device_file = None
        print("[TEMP] ⚠️ No se detectó el sensor DS18B20.")

    while True:
        if device_file:
            try:
                with open(device_file, 'r') as f:
                    lineas = f.readlines()
                # Si la primera línea acaba en YES, la lectura es buena
                if lineas[0].strip()[-3:] == 'YES':
                    pos_igual = lineas[1].find('t=')
                    if pos_igual != -1:
                        temp_str = lineas[1][pos_igual+2:]
                        # Convertimos a Celsius y guardamos en la variable global
                        datos_sensores["temperatura"] = float(temp_str) / 1000.0
            except:
                pass
        time.sleep(2) 
# ====================================================================
# CONTROL DE MOTORES INTOCABLE
# ====================================================================
def calcular_pulso(direccion):
    if direccion == 0: return PULSE_STOP
    if direccion == 1: return int(PULSE_MIN_FWD + (PULSE_MAX_FWD - PULSE_MIN_FWD) * FACTOR_POTENCIA)
    if direccion == -1: return int(PULSE_MIN_REV - (PULSE_MIN_REV - PULSE_MAX_REV) * FACTOR_POTENCIA)

def set_motores_dinamicos(dir_izq, dir_der):
    pi.set_servo_pulsewidth(PIN_ESC_IZQ, calcular_pulso(dir_izq))
    pi.set_servo_pulsewidth(PIN_ESC_DER, calcular_pulso(dir_der))

def stop_motors(): set_motores_dinamicos(0, 0)
def move_forward(): set_motores_dinamicos(1, 1)
def move_backward(): set_motores_dinamicos(-1, -1)
def turn_left(): set_motores_dinamicos(-1, 1)
def turn_right(): set_motores_dinamicos(1, -1)
def move_fwd_left():
    p_full = calcular_pulso(1)
    pi.set_servo_pulsewidth(PIN_ESC_IZQ, int(PULSE_STOP + (p_full - PULSE_STOP) * 0.5))
    pi.set_servo_pulsewidth(PIN_ESC_DER, p_full)
def move_fwd_right():
    p_full = calcular_pulso(1)
    pi.set_servo_pulsewidth(PIN_ESC_IZQ, p_full)
    pi.set_servo_pulsewidth(PIN_ESC_DER, int(PULSE_STOP + (p_full - PULSE_STOP) * 0.5))

# ====================================================================
# FAILSAFE
# ====================================================================
def registrar_contacto():
    global ultimo_contacto, failsafe_activado

    ultimo_contacto = time.monotonic()

    if failsafe_activado:
        print("[FAILSAFE] ✅ Comunicación recuperada.")
        failsafe_activado = False

def hilo_failsafe():
    global MODO_AUTO, failsafe_activado

    while True:
        tiempo_sin_contacto = time.monotonic() - ultimo_contacto

        if tiempo_sin_contacto > FAILSAFE_TIMEOUT and not failsafe_activado:
            failsafe_activado = True
            MODO_AUTO = False
            stop_motors()

            print(
                f"[FAILSAFE] ⚠️ Comunicación perdida durante "
                f"{tiempo_sin_contacto:.1f} s. MOTORES DETENIDOS."
            )

        time.sleep(0.1)

# ====================================================================
# HILO DEL PILOTO AUTOMÁTICO (ACTUALIZADO PARA RUTAS)
# ====================================================================
def hilo_autonomo():
    global MODO_AUTO, lista_waypoints, error_previo, integral, last_time
    while True:
        if MODO_AUTO and len(lista_waypoints) > 0:
            objetivo_actual = lista_waypoints[0] 
            destino_lat = objetivo_actual['lat']
            destino_lon = objetivo_actual['lon']

            lat = datos_sensores["latitud"]
            lon = datos_sensores["longitud"]
            rumbo = datos_sensores["rumbo"]

            if lat != 0.0 and lon != 0.0:
                # 1. Distancia
                dist = calcular_distancia(lat, lon, destino_lat, destino_lon)
                datos_sensores["distancia_wpt"] = dist
                
                # 2. Rumbo
                rumbo_obj = calcular_rumbo_deseado(lat, lon, destino_lat, destino_lon)

                # 3. Error
                error = rumbo_obj - rumbo
                if error > 180: error -= 360
                if error < -180: error += 360

                # 4. Lógica de Llegada
                if dist < 3.0:
                    print(f"✅ WAYPOINT ALCANZADO: {destino_lat}, {destino_lon}")
                    lista_waypoints.pop(0) #Borramos el punto alcanzado de la lista
                    
                    if len(lista_waypoints) == 0:
                        print("No quedan más puntos. Detenemos motores.")
                        stop_motors()
                        MODO_AUTO = False
                
                # 5. Lógica de Navegación con PID y Velocidad Dinámica
                else:
                    now = time.time()
                    dt = now - last_time
                    if dt <= 0: dt = 0.1
                    
                    # --- CÁLCULO PID ---
                    proporcional = error
                    integral += error * dt
                    integral = max(-50, min(50, integral)) # Anti-windup
                    derivativo = (error - error_previo) / dt
                    
                    salida_pid = (kp * proporcional) + (ki * integral) + (kd * derivativo)
                    
                    error_previo = error
                    last_time = now

                    # --- GESTIÓN DE POTENCIA DINÁMICA (SLIDER) ---
                    
                    # 1. El slider marca el LÍMITE MÁXIMO absoluto en tiempo real
                    pulso_limite_dinamico = int(PULSE_STOP + (PULSE_MAX_FWD - PULSE_STOP) * FACTOR_POTENCIA)
                    
                    # 2. Velocidad de crucero base (80% del límite del slider)
                    pulso_base = int(PULSE_STOP + (PULSE_MAX_FWD - PULSE_STOP) * (FACTOR_POTENCIA * 0.8))
                    
                    # 3. El PID suma potencia a un motor y se la resta al otro para girar
                    pulso_izq = pulso_base + salida_pid
                    pulso_der = pulso_base - salida_pid

                    # 4. CORTAFUEGOS DE SEGURIDAD
                    pulso_izq = max(PULSE_STOP, min(pulso_limite_dinamico, pulso_izq))
                    pulso_der = max(PULSE_STOP, min(pulso_limite_dinamico, pulso_der))

                    # Enviar señal a los motores
                    pi.set_servo_pulsewidth(PIN_ESC_IZQ, int(pulso_izq))
                    pi.set_servo_pulsewidth(PIN_ESC_DER, int(pulso_der))
        else:
            datos_sensores["distancia_wpt"] = 0.0
            integral = 0 # Resetear si no estamos en auto
            
        time.sleep(0.1) # Aumentamos a 10Hz para que el PID sea más estable

# ====================================================================
# RUTAS DEL SERVIDOR WEB (FLASK)
# ====================================================================
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/telemetria')
def telemetria():

    registrar_contacto()
    modo_str = "AUTO" if MODO_AUTO else "MANUAL"
    return jsonify({
        "rumbo": round(datos_sensores["rumbo"], 1),
        "latitud": round(datos_sensores["latitud"], 6),
        "longitud": round(datos_sensores["longitud"], 6),
        "calibracion": datos_sensores["calibracion"],
        "distancia": round(datos_sensores["distancia_wpt"], 1),
        "modo": modo_str,
        "puntos_restantes": len(lista_waypoints),
        "temperatura": round(datos_sensores["temperatura"], 1)
    })

@app.route('/comando', methods=['POST'])
def comando():
    global MODO_AUTO, lista_waypoints, FACTOR_POTENCIA

    registrar_contacto()
    
    data = request.json
    cmd = data.get('cmd', '').upper()
    
    if not (cmd in ["UP", "DOWN", "LEFT", "RIGHT", "UPL", "UPR", "STOP"]):
        print(f"[WEB] Comando recibido: {cmd}")

    if cmd.startswith("SPD|"):
        try:
            val = int(cmd.split("|")[1])
            FACTOR_POTENCIA = max(0, min(100, val)) / 100.0
            print(f">> Potencia ajustada al {val}%")
        except: pass
        return jsonify({"status": "ok"})
    if cmd == "ADD_WPT":
        try:
            lat = float(data.get('lat', 0.0))
            lon = float(data.get('lon', 0.0))
            if lat != 0.0 and lon != 0.0:
                lista_waypoints.append({'lat': lat, 'lon': lon})
                print(f"📍 Punto añadido. Total en ruta: {len(lista_waypoints)}")
        except: pass
        return jsonify({"status": "ok"})

    if cmd == "CLEAR_WPT":
        lista_waypoints.clear()
        MODO_AUTO = False
        stop_motors()
        print("🗑️ Ruta borrada. Barco en MODO MANUAL.")
        return jsonify({"status": "ok"})

    if cmd == "AUTO":
        if len(lista_waypoints) > 0: 
            MODO_AUTO = True
            print("▶ INICIANDO MISIÓN AUTÓNOMA")
        else:
            print("⚠️ Error: No puedes iniciar AUTO sin waypoints en la lista.")
        return jsonify({"status": "ok"})
        
    if cmd == "MANUAL" or cmd == "STOP":
        MODO_AUTO = False
        stop_motors()
        return jsonify({"status": "ok"})

    if not MODO_AUTO:
        if cmd == "UP": move_forward()
        elif cmd == "DOWN": move_backward()
        elif cmd == "LEFT": turn_left()
        elif cmd == "RIGHT": turn_right()
        elif cmd == "UPL": move_fwd_left()
        elif cmd == "UPR": move_fwd_right()
        
    return jsonify({"status": "ok"})

# ====================================================================
# ARRANQUE
# ====================================================================
if __name__ == "__main__":
    print("Iniciando hilos de sensores...")
    threading.Thread(target=leer_imu, daemon=True).start()
    threading.Thread(target=leer_gps, daemon=True).start()
    threading.Thread(target=hilo_autonomo, daemon=True).start()
    threading.Thread(target=hilo_temperatura, daemon=True).start()
    threading.Thread(target=hilo_failsafe, daemon=True).start()

    print("\n" + "="*50)
    print("   🚀 ESTACIÓN DE CONTROL WEB USV ONLINE")
    print("   Abre el navegador de tu móvil y entra en:")
    print("   http://192.168.4.1:5000  (Si usas Hotspot)")
    print("="*50 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        stop_motors()
        pi.set_servo_pulsewidth(PIN_ESC_IZQ, 0)
        pi.set_servo_pulsewidth(PIN_ESC_DER, 0)
        pi.stop()
