#!/usr/bin/env python3
import time, struct, threading, math, csv, os, random
from datetime import datetime
from pyrf24 import RF24, RF24_PA_LOW, RF24_250KBPS, RF24_CRC_16

# ================= HARDWARE Y RF =================
CE_PIN = 25
CSN_DEV = 0
ADDR = b"DRONE"
RF_CH = 110

# ================= PROTOCOLO =================
# Telemetría que llega de la STM32 (22 bytes)
TELEM_FMT = struct.Struct("<BBIhhhhhhHBB")
# Comandos hacia la STM32 (11 bytes + 1 byte de padding 'x' = 12 bytes)
CMD_FMT = struct.Struct("<BBHHBHBBx")

# Banderas (Flags) de la STM32
CMD_ARM       = 1 << 0
CMD_DISARM    = 1 << 1
CMD_ESTOP     = 1 << 2
CMD_SET_THR   = 1 << 3
CMD_SET_MOTOR = 1 << 4
CMD_IMU_EN    = 1 << 5
CMD_CAL_ALL   = 1 << 6

# ================= VARIABLES GLOBALES =================
radio = None
estado_actual = {
    "t_ms": 0, "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0,
    "status": 0, "cmd_echo": 0, "age_ms": 9999, "last_rx": 0
}
secuencia_comando = 0
motores_pwm = 1000 
mascara_motores = 0

# ================= FUNCIONES DE UTILIDAD =================
def decodificar_status(status):
    imu_en   = (status >> 4) & 1
    cal_busy = (status >> 3) & 1
    armed    = (status >> 2) & 1
    fs       = (status >> 5) & 1
    return imu_en, cal_busy, armed, fs

def calcular_angulos(ax, ay, az):
    try:
        roll  = math.degrees(math.atan2(ay / 1000.0, az / 1000.0))
        pitch = math.degrees(math.atan2(-ax / 1000.0, math.sqrt((ay/1000.0)**2 + (az/1000.0)**2)))
        return round(roll, 1), round(pitch, 1)
    except:
        return 0.0, 0.0

# ================= COMUNICACIÓN CORE =================
def hilo_recepcion():
    global estado_actual
    while True:
        if radio.available():
            data = radio.read(32)
            if len(data) >= TELEM_FMT.size and data[0] == 0xA1:
                (typ, tx_seq, t_ms, ax, ay, az, gx, gy, gz, status, cmd_echo, ping) = TELEM_FMT.unpack(data[:TELEM_FMT.size])
                
                estado_actual.update({
                    "t_ms": t_ms, "ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz,
                    "status": status, "cmd_echo": cmd_echo, "last_rx": time.time()
                })
        time.sleep(0.002)

def enviar_comando(flag, motor_mask=0, motor_us=1000, esperar_eco=True):
    global secuencia_comando, motores_pwm, mascara_motores
    secuencia_comando = (secuencia_comando + 1) & 0xFF
    
    if flag == CMD_SET_MOTOR:
        motores_pwm = motor_us
        mascara_motores = motor_mask

    paquete = CMD_FMT.pack(0xB1, secuencia_comando, flag, 1000, motor_mask, motor_us, 1, 0)
    radio.writeAckPayload(1, paquete)

    if esperar_eco:
        t0 = time.time()
        while time.time() - t0 < 0.5:
            if estado_actual["cmd_echo"] == secuencia_comando:
                break
            time.sleep(0.01)

        secuencia_comando = (secuencia_comando + 1) & 0xFF
        paquete_vacio = CMD_FMT.pack(0xB1, secuencia_comando, 0, 1000, mascara_motores, motores_pwm, 1, 0)
        radio.writeAckPayload(1, paquete_vacio)

def enviar_pwm_continuo(motor_mask, motor_us):
    global secuencia_comando, motores_pwm, mascara_motores
    secuencia_comando = (secuencia_comando + 1) & 0xFF
    motores_pwm = motor_us
    mascara_motores = motor_mask
    
    paquete = CMD_FMT.pack(0xB1, secuencia_comando, CMD_SET_MOTOR, 1000, motor_mask, motor_us, 1, 0)
    radio.writeAckPayload(1, paquete)

def parada_emergencia():
    enviar_comando(CMD_ESTOP, esperar_eco=False)
    time.sleep(0.1)
    enviar_comando(CMD_DISARM, esperar_eco=False)
    time.sleep(0.1)
    enviar_comando(CMD_SET_MOTOR, motor_mask=15, motor_us=1000)
    print("\n🚨 EMERGENCIA: Dron desarmado y motores detenidos.")

# ================= PRUEBA Y RECOLECCIÓN DE DATOS =================
def correr_prueba_csv():
    print("\n--- INICIANDO PRUEBA PARA IDENTIFICACIÓN DE SISTEMAS ---")
    
    # 1. Configuración de Motores
    print("\n[MÁSCARA DE MOTORES]")
    print("Ingresa los motores a excitar separados por comas (ej. 1, 1,4, 1,2,3, 1,2,3,4)")
    m_str = input("Motores activos: ").strip()
    mascara = 0
    if "1" in m_str: mascara |= (1<<0)
    if "2" in m_str: mascara |= (1<<1)
    if "3" in m_str: mascara |= (1<<2)
    if "4" in m_str: mascara |= (1<<3)
    
    if mascara == 0:
        print("❌ Error: Selecciona al menos un motor válido.")
        return

    # 2. Configuración de Señal
    print("\n[TIPO DE SEÑAL]")
    print("1. Escalón (Step - Salto abrupto)")
    print("2. Rampa (Ida y vuelta triangular)")
    print("3. Senoidal (Oscilación continua)")
    print("4. Aleatoria (Pseudo-random noise, excelente para System ID)")
    tipo_senal = input("Elige (1-4): ").strip()

    if tipo_senal not in ["1", "2", "3", "4"]:
        print("❌ Señal inválida.")
        return

    pwm_base = int(input("\nPWM de reposo / offset (ej. 1100): ") or "1100")
    
    if tipo_senal == "1" or tipo_senal == "2":
        pwm_pico = int(input("PWM pico deseado (ej. 1300): ") or "1300")
    else:
        amplitud = int(input("Amplitud de la onda/ruido en us (ej. 100): ") or "100")

    if tipo_senal == "3":
        freq_hz = float(input("Frecuencia senoidal en Hz (ej. 1.5): ") or "1.5")
    elif tipo_senal == "4":
        hold_ruido = float(input("Mantener cada valor random por (segundos, ej. 0.05): ") or "0.05")

    duracion = float(input("Duración total de la prueba en segundos (ej. 5): ") or "5")
    
    # Pre-vuelo
    print("\n⏳ Preparando STM32...")
    enviar_comando(CMD_CAL_ALL)
    print("Calibrando... (Espera ~4 segs)")
    time.sleep(5) 
    enviar_comando(CMD_ARM)
    print("✅ Motores armados.")
    
    # Archivo CSV
    os.makedirs("logs", exist_ok=True)
    nombre_senales = {"1": "Escalon", "2": "Rampa", "3": "Senoidal", "4": "Random"}
    nombre_archivo = f"logs/M_{m_str.replace(',','-')}_{nombre_senales[tipo_senal]}_{datetime.now().strftime('%H%M%S')}.csv"
    
    try:
        with open(nombre_archivo, "w", newline="") as f:
            escritor = csv.writer(f)
            # NUEVO: Encabezado con los 4 motores separados para MATLAB
            escritor.writerow(["tiempo_s", "m1_us", "m2_us", "m3_us", "m4_us", "gx", "gy", "gz", "roll_acc", "pitch_acc"])
            
            print(f"📈 Grabando datos en: {nombre_archivo}")
            print("\n🚨 PRESIONA 'Ctrl+C' EN CUALQUIER MOMENTO PARA ABORTAR Y APAGAR MOTORES 🚨\n")
            
            # Forzar todos a 1000 inicial
            enviar_comando(CMD_SET_MOTOR, motor_mask=15, motor_us=1000)
            time.sleep(0.5)

            t_inicio = time.time()
            ultimo_random_t = t_inicio
            pwm_actual = pwm_base
            
            while True:
                t = time.time() - t_inicio
                if t > duracion:
                    break

                if tipo_senal == "1": 
                    if t < duracion * 0.2 or t > duracion * 0.8:
                        pwm_actual = pwm_base
                    else:
                        pwm_actual = pwm_pico

                elif tipo_senal == "2":
                    mitad = duracion / 2.0
                    if t <= mitad:
                        pwm_actual = pwm_base + (pwm_pico - pwm_base) * (t / mitad)
                    else:
                        pwm_actual = pwm_pico - (pwm_pico - pwm_base) * ((t - mitad) / mitad)

                elif tipo_senal == "3":
                    pwm_actual = pwm_base + amplitud * math.sin(2 * math.pi * freq_hz * t)

                elif tipo_senal == "4":
                    if time.time() - ultimo_random_t >= hold_ruido:
                        pwm_actual = pwm_base + random.randint(-amplitud, amplitud)
                        ultimo_random_t = time.time()

                pwm_seguro = int(max(1000, min(2000, pwm_actual)))

                # NUEVO: Determinar el PWM exacto de cada motor para el CSV
                m1_out = pwm_seguro if "1" in m_str else 1000
                m2_out = pwm_seguro if "2" in m_str else 1000
                m3_out = pwm_seguro if "3" in m_str else 1000
                m4_out = pwm_seguro if "4" in m_str else 1000

                enviar_pwm_continuo(mascara, pwm_seguro)
                roll, pitch = calcular_angulos(estado_actual["ax"], estado_actual["ay"], estado_actual["az"])
                
                # Guardamos la fila con los 4 motores independientes
                escritor.writerow([
                    t, m1_out, m2_out, m3_out, m4_out, 
                    estado_actual["gx"]/10.0, estado_actual["gy"]/10.0, estado_actual["gz"]/10.0, 
                    roll, pitch
                ])
                time.sleep(0.01)

            print("✅ Fase de excitación finalizada. Volviendo a 1000us...")
            enviar_comando(CMD_SET_MOTOR, motor_mask=15, motor_us=1000)
            time.sleep(0.5)

    except KeyboardInterrupt:
        # Aquí entra el botón de pánico físico (Ctrl+C)
        parada_emergencia()
        return

    print("✅ Prueba completamente guardada.")
    parada_emergencia()

# ================= MENÚ PRINCIPAL =================
def main():
    global radio
    radio = RF24(CE_PIN, CSN_DEV)
    if not radio.begin():
        print("❌ Error de NRF24L01 (SPI/Cableado).")
        return

    radio.setChannel(RF_CH)
    radio.setDataRate(RF24_250KBPS)
    radio.setCRCLength(RF24_CRC_16)
    radio.setPALevel(RF24_PA_LOW)
    radio.setAutoAck(True)
    radio.enableDynamicPayloads()
    radio.enableAckPayload()
    radio.openReadingPipe(1, ADDR)
    radio.startListening()

    t = threading.Thread(target=hilo_recepcion, daemon=True)
    t.start()
    
    print("⏳ Esperando conexión con STM32...")
    time.sleep(2)
    if time.time() - estado_actual["last_rx"] > 1.0:
        print("❌ No hay telemetría. Revisa que la STM32 esté encendida.")
        return
    print("✅ Conexión establecida.")

    while True:
        try:
            print("\n--- MENÚ DESCONTROL ---")
            print("1. Ver Estado (Telemetría)")
            print("2. Calibrar IMU")
            print("3. Armar Motores")
            print("4. Desarmar Motores")
            print("5. Prueba de Sistemas (Escalón, Rampa, Senoidal, Random)")
            print("6. Salir")
            opcion = input("Elige una opción: ").strip()

            if opcion == "1":
                imu, cal, arm, fs = decodificar_status(estado_actual["status"])
                roll, pitch = calcular_angulos(estado_actual["ax"], estado_actual["ay"], estado_actual["az"])
                edad = int((time.time() - estado_actual["last_rx"]) * 1000)
                print(f"\n[ESTADO] Retardo: {edad}ms | Roll: {roll}° Pitch: {pitch}° | Gyro X: {estado_actual['gx']/10.0} | Calibrando: {cal} | Armado: {arm}")
            
            elif opcion == "2":
                enviar_comando(CMD_CAL_ALL)
                print("✅ Comando de calibración enviado.")
            
            elif opcion == "3":
                enviar_comando(CMD_ARM)
                print("✅ Comando de armado enviado.")
            
            elif opcion == "4":
                enviar_comando(CMD_DISARM)
                print("✅ Comando de desarmado enviado.")
            
            elif opcion == "5":
                correr_prueba_csv()
            
            elif opcion == "6":
                parada_emergencia()
                break

        except KeyboardInterrupt:
            parada_emergencia()
            break

if __name__ == "__main__":
    main()
