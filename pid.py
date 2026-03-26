#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, threading, math, os
from pyrf24 import RF24, RF24_PA_LOW, RF24_250KBPS, RF24_CRC_16
from collections import deque

try:
    import matplotlib.pyplot as plt
    GRAPHING_ENABLED = True
except ImportError:
    print("⚠️ Matplotlib no está instalado. La gráfica en tiempo real estará desactivada.")
    GRAPHING_ENABLED = False

# ================= HARDWARE Y RF =================
CE_PIN = 25
CSN_DEV = 0
ADDR = b"DRONE"
RF_CH = 110

PKT_TELEM = 0xA1
PKT_CMD   = 0xB1

TELEM_FMT = struct.Struct("<BBIhhhhhhHBB")   # 24 bytes
CMD_FMT   = struct.Struct("<BBHHBHBBx")      # 12 bytes

# Flags
CMD_ARM       = 1 << 0
CMD_DISARM    = 1 << 1
CMD_ESTOP     = 1 << 2
CMD_SET_THR   = 1 << 3
CMD_SET_MOTOR = 1 << 4
CMD_IMU_EN    = 1 << 5
CMD_CAL_ALL   = 1 << 6
CMD_PING      = 1 << 7
CMD_SET_PID   = 1 << 8

MASK_ALL = (1<<0)|(1<<1)|(1<<2)|(1<<3)

def decode_status(st):
    return (st >> 4) & 1, (st >> 3) & 1, (st >> 2) & 1, (st >> 5) & 1

def calcular_angulo_acc(ax, ay, az):
    try:
        return math.degrees(math.atan2(ay / 1000.0, az / 1000.0))
    except:
        return 0.0

class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.telem = None
        self.last_rx = 0.0

    def update(self, t):
        with self.lock:
            self.telem = t
            self.last_rx = time.time()

    def snapshot(self):
        with self.lock:
            return self.telem, self.last_rx

class CmdState:
    def __init__(self):
        self.lock = threading.RLock() # Previene el Deadlock
        self.seq = 0
        self.flags = 0
        self.thr_us = 1000
        self.motor_mask = MASK_ALL
        self.motor_us = 1000
        self.imu_en = 1
        self.ping_id = 0

    def bump(self):
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def pack(self):
        with self.lock:
            return CMD_FMT.pack(
                PKT_CMD, self.seq & 0xFF, self.flags & 0xFFFF, self.thr_us & 0xFFFF,
                self.motor_mask & 0xFF, self.motor_us & 0xFFFF, self.imu_en & 0xFF, self.ping_id & 0xFF
            )

    def set(self, flags=None, thr=None, mm=None, mus=None, imu=None):
        with self.lock:
            if flags is not None: self.flags = int(flags)
            if thr is not None:   self.thr_us = int(thr)
            if mm is not None:    self.motor_mask = int(mm)
            if mus is not None:   self.motor_us = int(mus)
            if imu is not None:   self.imu_en = 1 if imu else 0
            self.bump()

def q88_from_float(x: float) -> int:
    if x > 127.0: x = 127.0
    if x < -128.0: x = -128.0
    return int(round(x * 256.0)) & 0xFFFF

def main():
    # === ¡AQUÍ ESTÁ LA ÚNICA LÍNEA QUE DEBES MODIFICAR! ===
    VALORES_MATLAB = {"kp": 0.7505, "ki": 0.04, "kd": 0.008, "sp": 0.0, "kp_ang": 2.8}
    # ======================================================

    radio = RF24(CE_PIN, CSN_DEV)
    print("=== Horus GCS: Control de ÁNGULO en Cascada ===")

    if not radio.begin():
        print("❌ radio.begin() falló")
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

    link = Shared()
    cmd = CmdState()
    stop_evt = threading.Event()
    
    roll_filtrado = 0.0
    last_t_ms = 0
    roll_history = deque(maxlen=200)
    sp_history = deque(maxlen=200)
    show_graph = False

    radio.writeAckPayload(1, cmd.pack())

    # --- FUNCIONES DE COMANDO (Definidas antes para poder usarlas al inyectar) ---
    def send(flags=0, thr=None, mm=None, mus=None, imu=None):
        cmd.set(flags=flags, thr=thr, mm=mm, mus=mus, imu=imu)
        seq = cmd.seq
        radio.writeAckPayload(1, cmd.pack())

        t_start = time.time()
        while time.time() - t_start < 0.5:
            telem, _ = link.snapshot()
            if telem and telem["cmd_echo"] == seq:
                break
            time.sleep(0.01)
        
        cmd.set(flags=0)
        radio.writeAckPayload(1, cmd.pack())

    def pid_set(which, val):
        sel = {"kp":0, "ki":1, "kd":2, "sp":3, "kp_ang":4}[which]
        q = q88_from_float(val)
        send(CMD_SET_PID, mm=sel, mus=q)
        VALORES_MATLAB[which] = val
        print(f"-> {which} = {val} (enviado al STM32)")

    def estop():
        send(CMD_ESTOP)
        time.sleep(0.1)
        send(CMD_DISARM)
        time.sleep(0.1)
        send(CMD_SET_MOTOR, mm=MASK_ALL, mus=1000)

    def print_status():
        telem, last_rx = link.snapshot()
        if not telem: return
        age = int((time.time()-last_rx)*1000)
        imu, cal, arm, fs = decode_status(telem["status"])
        print(f"[STATUS] age={age}ms | Roll_Estimado: {roll_filtrado:+.1f}° | Gx: {telem['gx']/10.0:+.1f}°/s | Cal: {cal} Arm: {arm}")

    # --- HILO RX ---
    def rx_thread():
        nonlocal roll_filtrado, last_t_ms
        while not stop_evt.is_set():
            if radio.available():
                data = radio.read(32)
                try:
                    radio.writeAckPayload(1, cmd.pack())
                except Exception:
                    pass

                if len(data) >= TELEM_FMT.size and data[0] == PKT_TELEM:
                    (typ, tx_seq, t_ms, ax, ay, az, gx, gy, gz, status, cmd_echo, ping_echo) = TELEM_FMT.unpack(data[:TELEM_FMT.size])

                    link.update({
                        "tx_seq": tx_seq, "t_ms": t_ms,
                        "ax": ax, "ay": ay, "az": az,
                        "gx": gx, "gy": gy, "gz": gz,
                        "status": status, "cmd_echo": cmd_echo, "ping_echo": ping_echo
                    })
                    
                    # Filtro Complementario en Python para gráfica suave
                    dt = (t_ms - last_t_ms) / 1000.0 if last_t_ms > 0 else 0.005
                    last_t_ms = t_ms
                    if dt > 0.1: dt = 0.005 

                    acc_roll = calcular_angulo_acc(ax, ay, az)
                    roll_filtrado = 0.98 * (roll_filtrado + (gx / 10.0) * dt) + 0.02 * acc_roll

                    if show_graph:
                        roll_history.append(roll_filtrado)
                        sp_history.append(VALORES_MATLAB["sp"])

            time.sleep(0.001)

    threading.Thread(target=rx_thread, daemon=True).start()

    print("⏳ Esperando RX...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        telem, last_rx = link.snapshot()
        if telem and (time.time() - last_rx) < 0.5:
            print("✅ RX OK")
            break
        time.sleep(0.05)
    else:
        print("❌ No hay telemetría. Revisa la STM32.")
        stop_evt.set()
        return

    # --- INYECCIÓN AUTOMÁTICA DE VALORES DEFAULT ---
    print("\n⚙️ Enviando ganancias PID por defecto al dron...")
    pid_set("kp", VALORES_MATLAB["kp"])
    time.sleep(0.05)
    pid_set("ki", VALORES_MATLAB["ki"])
    time.sleep(0.05)
    pid_set("kd", VALORES_MATLAB["kd"])
    time.sleep(0.05)
    pid_set("kp_ang", VALORES_MATLAB["kp_ang"])
    time.sleep(0.05)
    print("✅ Dron configurado y listo para volar.\n")

    # --- HILO DEL MENÚ CLI ---
    def cli_thread():
        nonlocal show_graph
        print("Comandos Vuelo: status | cal | arm | dis | stop | thr <us>")
        print("Comandos PID:   pid kp <f> | pid ki <f> | pid kd <f> | pid kp_ang <f>")
        print("Mando Setpoint: sp <grados_inclinacion> (ej. sp 15)")
        print("Gráfica:        graph on | graph off | quit\n")

        try:
            while not stop_evt.is_set():
                s = input("> ").strip().lower()
                if not s: continue
                if s == "quit":
                    stop_evt.set()
                    break
                if s == "status":
                    print_status()
                    continue
                if s == "cal":
                    send(CMD_CAL_ALL)
                    print("-> Calibrando... IMPORTANTE: Mantén el dron 100% nivelado.")
                    continue
                if s == "arm":
                    send(CMD_ARM)
                    print("-> arm enviado")
                    continue
                if s == "dis":
                    send(CMD_DISARM)
                    print("-> disarm enviado")
                    continue
                if s == "stop":
                    estop()
                    print("-> EMERGENCIA DETENIDA")
                    continue
                if s.startswith("thr "):
                    try:
                        v = int(s.split()[1])
                        send(CMD_SET_THR, thr=v)
                        print(f"-> Acelerador: {v}us")
                    except: pass
                    continue
                
                if s == "graph on":
                    if GRAPHING_ENABLED: show_graph = True; print("-> Gráfica Activada")
                    continue
                if s == "graph off":
                    show_graph = False; print("-> Gráfica Desactivada")
                    continue

                if s.startswith("pid "):
                    parts = s.split()
                    if len(parts) == 2 and parts[1] == "show":
                        print("Valores Actuales:", VALORES_MATLAB)
                        continue
                    if len(parts) != 3 or parts[1] not in ("kp","ki","kd","kp_ang"):
                        print("Uso: pid kp/ki/kd/kp_ang <float>")
                        continue
                    try: pid_set(parts[1], float(parts[2]))
                    except ValueError: pass
                    continue

                if s.startswith("sp "):
                    try: pid_set("sp", float(s.split()[1]))
                    except: print("Uso: sp <grados>")
                    continue

        except EOFError:
            stop_evt.set()

    threading.Thread(target=cli_thread, daemon=True).start()

    # --- HILO PRINCIPAL: Módulo Gráfico ---
    if GRAPHING_ENABLED:
        plt.ion()
        fig, ax = plt.subplots()
        fig.canvas.manager.set_window_title('Horus Telemetry - Ángulo Real')
        line_roll, = ax.plot([], [], label='Ángulo Roll (°)', color='blue')
        line_sp, = ax.plot([], [], label='Setpoint (°)', color='red', linestyle='--')
        ax.set_ylim(-45, 45)
        ax.set_xlim(0, 200)
        ax.legend(loc='upper right')
        ax.grid(True)

        while not stop_evt.is_set():
            if show_graph and len(roll_history) > 0:
                line_roll.set_data(range(len(roll_history)), list(roll_history))
                line_sp.set_data(range(len(sp_history)), list(sp_history))
                plt.pause(0.05)
            else:
                time.sleep(0.1)
    else:
        while not stop_evt.is_set():
            time.sleep(0.1)

    estop()
    print("\nAdios!")

if __name__ == "__main__":
    main()
