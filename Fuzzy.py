#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Horus GCS V9.2 — LQI / LQG / FUZZY Multiplexor
(Bug de desbordamiento corregido, variables empaquetadas a x10)
VERSIÓN EXPANDIDA Y COMENTADA
"""

import time
import struct
import threading
from collections import deque
import numpy as np
from scipy.linalg import solve_discrete_are, inv

from pyrf24 import RF24, RF24_PA_LOW, RF24_250KBPS, RF24_CRC_16

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── CONFIGURACIÓN FÍSICA DE LOS EJES ─────────────────────────────
# Si un eje estabiliza acelerando los motores equivocados, cambia
# el valor a True para invertir toda la matemática de ese eje.
INVERT_ROLL_AXIS  = True   
INVERT_PITCH_AXIS = False

# ── Hardware ─────────────────────────────────────────────────────
CE_PIN  = 25
CSN_DEV = 0
ADDR    = b"DRONE"
RF_CH   = 110

# ── Protocolo ────────────────────────────────────────────────────
PKT_TELEM  = 0xA1
PKT_CMD    = 0xB1
PKT_KALMAN = 0xB2

CMD_FMT    = struct.Struct("<BBHHhhhhhhhhHBBB")   # 27 bytes
KALMAN_FMT = struct.Struct("<BBhhhhhhhh")         # 18 bytes
TELEM_FMT  = struct.Struct("<BBIhhhhhHHHHHBB")    # 28 bytes

# ── Flags de comando ─────────────────────────────────────────────
CMD_ARM           = 1 << 0
CMD_DISARM        = 1 << 1
CMD_ESTOP         = 1 << 2
CMD_SET_THR       = 1 << 3
CMD_IMU_EN        = 1 << 4
CMD_CAL_ALL       = 1 << 5
CMD_PING          = 1 << 6
CMD_START_MOTORS  = 1 << 7
CMD_UPDATE_K      = 1 << 8
CMD_SET_AW        = 1 << 9  
CMD_UPDATE_FUZZY  = 1 << 10 # Bandera para actualizar triángulos Fuzzy

# ── Bits de status ───────────────────────────────────────────────
ST_MPU_OK     = 1 << 0
ST_ACCEL_OK   = 1 << 1
ST_ARMED      = 1 << 2
ST_CAL_BUSY   = 1 << 3
ST_IMU_EN     = 1 << 4
ST_FAILSAFE   = 1 << 5
ST_MOTORS_RUN = 1 << 6

# ═══════════════════════════════════════════════════════════════
#  Modelo LQI Físico y Kalman
# ═══════════════════════════════════════════════════════════════
class DroneLQIModel:
    def __init__(self, dt=0.02):
        self.dt = dt
        
        # --- EJE ROLL ---
        self.A_roll = np.array([
            [0.9976,   0.02669],
            [-0.03649, 0.9870]
        ])
        self.B_roll = np.array([
            [-0.0007416],
            [0.0003183]
        ])
        
        self.A_aug_roll = np.array([
            [self.A_roll[0,0], self.A_roll[0,1], 0.0],
            [self.A_roll[1,0], self.A_roll[1,1], 0.0],
            [self.dt,          0.0,              1.0]
        ])
        self.B_aug_roll = np.array([
            [self.B_roll[0,0]],
            [self.B_roll[1,0]],
            [0.0]
        ])

        # --- EJE PITCH ---
        self.A_pitch = np.array([
            [0.9939,   0.01319],
            [-0.03461, 0.9828]
        ])
        self.B_pitch = np.array([
            [-0.0007354],
            [0.007451]
        ])
        
        self.A_aug_pitch = np.array([
            [self.A_pitch[0,0], self.A_pitch[0,1], 0.0],
            [self.A_pitch[1,0], self.A_pitch[1,1], 0.0],
            [self.dt,           0.0,               1.0]
        ])
        self.B_aug_pitch = np.array([
            [self.B_pitch[0,0]],
            [self.B_pitch[1,0]],
            [0.0]
        ])

    def calc_k_roll(self, q_ang, q_rate, q_int, r_val):
        Q = np.diag([q_ang, q_rate, q_int])
        R = np.array([[r_val]])
        P = solve_discrete_are(self.A_aug_roll, self.B_aug_roll, Q, R)
        K = inv(self.B_aug_roll.T @ P @ self.B_aug_roll + R) @ (self.B_aug_roll.T @ P @ self.A_aug_roll)
        K_out = K.flatten()
        
        # INVERSIÓN FÍSICA SI ES REQUERIDA
        if INVERT_ROLL_AXIS:
            K_out = -K_out
        return K_out

    def calc_k_pitch(self, q_ang, q_rate, q_int, r_val):
        Q = np.diag([q_ang, q_rate, q_int])
        R = np.array([[r_val]])
        P = solve_discrete_are(self.A_aug_pitch, self.B_aug_pitch, Q, R)
        K = inv(self.B_aug_pitch.T @ P @ self.B_aug_pitch + R) @ (self.B_aug_pitch.T @ P @ self.A_aug_pitch)
        K_out = K.flatten()
        
        # INVERSIÓN FÍSICA SI ES REQUERIDA
        if INVERT_PITCH_AXIS:
            K_out = -K_out
        return K_out

    def calc_kalman_gain(self, axis, qw_pos, qw_vel, rv_pos, rv_vel):
        C = np.array([[1.0, 0.0], [0.0, 1.0]])
        Q_k = np.diag([qw_pos, qw_vel])
        R_k = np.diag([rv_pos, rv_vel])
        
        if axis == "roll":
            P = solve_discrete_are(self.A_roll.T, C.T, Q_k, R_k)
        else:
            P = solve_discrete_are(self.A_pitch.T, C.T, Q_k, R_k)
            
        L = P @ C.T @ inv(C @ P @ C.T + R_k)
        return L


# ═══════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════
def _i16_x10(v):
    return max(-32768, min(32767, int(round(v * 10.0))))

def _i16_x1000(v):
    return max(-32768, min(32767, int(round(v * 1000.0))))

class Shared:
    def __init__(self):
        self._lock = threading.Lock()
        self.telem = None
        self.last_rx = 0.0

    def update(self, t):
        with self._lock:
            self.telem = t
            self.last_rx = time.time()

    def snapshot(self):
        with self._lock:
            return self.telem, self.last_rx

class CmdState:
    def __init__(self):
        self._lock = threading.RLock()
        self.seq = 0
        self.flags = 0
        
        self.thr_us = 1120
        self.sp_roll = 0.0
        self.sp_pitch = 0.0
        
        # Variables de Tuning LQI en GCS (Pesos Q y R)
        self.q_roll  = [110.0, 10.0, 0.5]  
        self.r_roll  = 0.3
        
        self.q_pitch = [250.0, 30.0, 1.0]
        self.r_pitch = 0.3
        
        # Ganancias LQI predeterminadas (Intactas)
        self.k_roll_ang   = 9.058
        self.k_roll_rate  = 0.683
        self.k_roll_int   = 1.287
        
        self.k_pitch_ang  = 14.639
        self.k_pitch_rate = 12.095
        self.k_pitch_int  = 1.752
        
        # Geometría FUZZY predeterminada
        self.fz_e_max   = 30.0
        self.fz_r_max   = 200.0
        self.fz_out_max = 300.0
        
        self.aw_limit  = 65        
        self.imu_en = 1
        self.ping_id = 0
        self.ctrl_mode = 0  

    def pack(self):
        with self._lock:
            # Multiplexión: Si actualizamos Fuzzy, enviamos los límites a través de las variables K de Roll.
            # CORRECCIÓN DE DESBORDAMIENTO APLICADA (Usamos x10 en lugar de x1000 para estos valores grandes)
            if self.flags & CMD_UPDATE_FUZZY:
                kr_a_encoded = _i16_x10(self.fz_e_max)
                kr_r_encoded = _i16_x10(self.fz_r_max)
                kr_i_encoded = _i16_x10(self.fz_out_max)
            else:
                kr_a_encoded = _i16_x1000(self.k_roll_ang)
                kr_r_encoded = _i16_x1000(self.k_roll_rate)
                kr_i_encoded = _i16_x1000(self.k_roll_int)

            return CMD_FMT.pack(
                PKT_CMD,
                self.seq & 0xFF,
                self.flags & 0xFFFF,
                int(self.thr_us) & 0xFFFF,
                _i16_x10(self.sp_roll),
                _i16_x10(self.sp_pitch),
                kr_a_encoded,
                kr_r_encoded,
                kr_i_encoded,    
                _i16_x1000(self.k_pitch_ang),
                _i16_x1000(self.k_pitch_rate),
                _i16_x1000(self.k_pitch_int),   
                int(self.aw_limit) & 0xFFFF,
                self.imu_en & 0xFF,
                self.ping_id & 0xFF,
                self.ctrl_mode & 0xFF
            )

    def set(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self, k, v)
            self.seq = (self.seq + 1) & 0xFF

def decode_status(st):
    return {
        "mpu":     bool(st & ST_MPU_OK),
        "armed":   bool(st & ST_ARMED),
        "cal":     bool(st & ST_CAL_BUSY),
        "imu_en":  bool(st & ST_IMU_EN),
        "fs":      bool(st & ST_FAILSAFE),
        "motors":  bool(st & ST_MOTORS_RUN),
    }

# ═══════════════════════════════════════════════════════════════
#  GCS principal
# ═══════════════════════════════════════════════════════════════
class HorusGCS:
    GRAPH_LEN = 300  

    def __init__(self):
        self.shared = Shared()
        self.cmd = CmdState()
        self.stop_evt = threading.Event()
        self.radio = None
        self.pending_kalman_pkt = None
        
        self.lqi_model = DroneLQIModel(dt=0.02)
        
        self.kalman_params = {'roll': [1.0, 1.0, 100.0, 100.0], 'pitch': [1.0, 1.0, 100.0, 100.0]}
        self.L_roll_cache = np.array([[0.1, 0.0], [0.0, 0.1]])
        self.L_pitch_cache = np.array([[0.1, 0.0], [0.0, 0.1]])

        self.roll_hist  = deque(maxlen=self.GRAPH_LEN)
        self.pitch_hist = deque(maxlen=self.GRAPH_LEN)
        self.sp_r_hist  = deque(maxlen=self.GRAPH_LEN)
        self.sp_p_hist  = deque(maxlen=self.GRAPH_LEN)

    # ── Radio ────────────────────────────────────────────────────
    def _init_radio(self):
        self.radio = RF24(CE_PIN, CSN_DEV)
        if not self.radio.begin():
            print("❌ radio.begin() falló")
            return False
        self.radio.setChannel(RF_CH)
        self.radio.setDataRate(RF24_250KBPS)
        self.radio.setCRCLength(RF24_CRC_16)
        self.radio.setPALevel(RF24_PA_LOW)
        self.radio.setAutoAck(True)
        self.radio.enableDynamicPayloads()
        self.radio.enableAckPayload()
        self.radio.openReadingPipe(1, ADDR)
        self.radio.startListening()
        return True

    def _send(self, flags=0, timeout=0.5, **kw):
        kw["flags"] = flags
        self.cmd.set(**kw)
        target_seq = self.cmd.seq
        self.radio.writeAckPayload(1, self.cmd.pack())

        t0 = time.time()
        ok = False
        while time.time() - t0 < timeout:
            telem, _ = self.shared.snapshot()
            if telem and telem["cmd_echo"] == target_seq:
                ok = True
                break
            time.sleep(0.01)

        self.cmd.set(flags=0)
        self.radio.writeAckPayload(1, self.cmd.pack())
        return ok

    def _send_kalman(self):
        seq = (self.cmd.seq + 1) & 0xFF
        self.cmd.seq = seq
        
        self.pending_kalman_pkt = KALMAN_FMT.pack(
            PKT_KALMAN, 
            seq,
            _i16_x1000(self.L_roll_cache[0,0]), 
            _i16_x1000(self.L_roll_cache[0,1]),
            _i16_x1000(self.L_roll_cache[1,0]), 
            _i16_x1000(self.L_roll_cache[1,1]),
            _i16_x1000(self.L_pitch_cache[0,0]), 
            _i16_x1000(self.L_pitch_cache[0,1]),
            _i16_x1000(self.L_pitch_cache[1,0]), 
            _i16_x1000(self.L_pitch_cache[1,1])
        )

    def _estop(self):
        print("🛑 EMERGENCIA")
        for _ in range(3):
            self._send(flags=CMD_ESTOP, timeout=0.15)
            time.sleep(0.03)

    def _rx_thread(self):
        while not self.stop_evt.is_set():
            if self.radio.available():
                data = self.radio.read(32)
                
                try:
                    if self.pending_kalman_pkt:
                        self.radio.writeAckPayload(1, self.pending_kalman_pkt)
                        self.pending_kalman_pkt = None
                    else:
                        self.radio.writeAckPayload(1, self.cmd.pack())
                except Exception as e:
                    pass

                if len(data) >= TELEM_FMT.size and data[0] == PKT_TELEM:
                    try:
                        (typ, tx_seq, t_ms,
                         roll_x10, pitch_x10,
                         gx_x10, gy_x10, gz_x10,
                         status,
                         pwm_m1, pwm_m2, pwm_m3, pwm_m4,
                         cmd_echo, ping_echo) = TELEM_FMT.unpack(data[:TELEM_FMT.size])

                        roll  = roll_x10  / 10.0
                        pitch = pitch_x10 / 10.0

                        self.shared.update({
                            "tx_seq": tx_seq, "t_ms": t_ms,
                            "roll": roll, "pitch": pitch,
                            "gx": gx_x10 / 10.0, "gy": gy_x10 / 10.0, "gz": gz_x10 / 10.0,
                            "status": status,
                            "pwm_m1": pwm_m1, "pwm_m2": pwm_m2,
                            "pwm_m3": pwm_m3, "pwm_m4": pwm_m4,
                            "cmd_echo": cmd_echo, "ping_echo": ping_echo,
                        })

                        self.roll_hist.append(roll)
                        self.pitch_hist.append(pitch)
                        self.sp_r_hist.append(self.cmd.sp_roll)
                        self.sp_p_hist.append(self.cmd.sp_pitch)

                    except struct.error:
                        pass
            else:
                time.sleep(0.001)

    def _print_status(self):
        telem, last_rx = self.shared.snapshot()
        if not telem:
            print("❌ Sin telemetría todavía")
            return
            
        age = int((time.time() - last_rx) * 1000)
        st = decode_status(telem["status"])
        link = "🟢" if age < 500 else "🔴"
        
        mod_str = ["LQI (Complementario)", "LQG (Kalman)", "FUZZY (Difuso)"][self.cmd.ctrl_mode]
        
        # --- PANEL DE ESTADO COMPLETO Y DETALLADO ---
        print(f"\n[{link} {age}ms] MODO ACTIVO: {mod_str}")
        print(f"        Roll=  {telem['roll']:+6.1f}°  | SP= {self.cmd.sp_roll:+4.1f}°")
        print(f"        Pitch= {telem['pitch']:+6.1f}°  | SP= {self.cmd.sp_pitch:+4.1f}°")
        print(f"        Gx=    {telem['gx']:+6.1f}°/s | Gy= {telem['gy']:+6.1f}°/s")
        print(f"        PWM:   M1={telem['pwm_m1']}  M2={telem['pwm_m2']}  M3={telem['pwm_m3']}  M4={telem['pwm_m4']} | Thr={self.cmd.thr_us}")
        
        if self.cmd.ctrl_mode == 0 or self.cmd.ctrl_mode == 1:
            print(f"        Q_Roll : Ang={self.cmd.q_roll[0]:.1f}  Rate={self.cmd.q_roll[1]:.1f}  Int={self.cmd.q_roll[2]:.1f} | R={self.cmd.r_roll:.1f}")
            print(f"        Q_Pitch: Ang={self.cmd.q_pitch[0]:.1f} Rate={self.cmd.q_pitch[1]:.1f} Int={self.cmd.q_pitch[2]:.1f} | R={self.cmd.r_pitch:.1f}")
            print(f"        K_Roll = [{self.cmd.k_roll_ang:.3f}, {self.cmd.k_roll_rate:.3f}, {self.cmd.k_roll_int:.3f}]")
            print(f"        K_Pitch= [{self.cmd.k_pitch_ang:.3f}, {self.cmd.k_pitch_rate:.3f}, {self.cmd.k_pitch_int:.3f}]")
        
        if self.cmd.ctrl_mode == 1:
            kr = self.kalman_params['roll']
            kp = self.kalman_params['pitch']
            print(f"        [KALMAN] Roll : Qw_pos={kr[0]:.1f}  Qw_vel={kr[1]:.1f} | Rv_pos={kr[2]:.1f}  Rv_vel={kr[3]:.1f}")
            print(f"        [KALMAN] Pitch: Qw_pos={kp[0]:.1f} Qw_vel={kp[1]:.1f} | Rv_pos={kp[2]:.1f} Rv_vel={kp[3]:.1f}")

        if self.cmd.ctrl_mode == 2:
            print(f"        [FUZZY] Límites Activos -> Err_Max: {self.cmd.fz_e_max}° | Rate_Max: {self.cmd.fz_r_max}°/s | Out_Max: {self.cmd.fz_out_max}")

        print(f"        Arm={st['armed']} | Motors={st['motors']} | AW Limit={self.cmd.aw_limit} | Failsafe={st['fs']}")

    def _tune_lqi(self, axis, qa, qr, qi, r_val):
        try:
            if axis == "roll":
                K = self.lqi_model.calc_k_roll(qa, qr, qi, r_val)
                self.cmd.set(q_roll=[qa, qr, qi], r_roll=r_val, 
                             k_roll_ang=K[0], k_roll_rate=K[1], k_roll_int=K[2])
                print(f"✅ Riccati OK. K_Roll calculada y enviada.")
            elif axis == "pitch":
                K = self.lqi_model.calc_k_pitch(qa, qr, qi, r_val)
                self.cmd.set(q_pitch=[qa, qr, qi], r_pitch=r_val,
                             k_pitch_ang=K[0], k_pitch_rate=K[1], k_pitch_int=K[2])
                print(f"✅ Riccati OK. K_Pitch calculada y enviada.")
            
            return self._send(flags=CMD_UPDATE_K)
        except Exception as e:
            print(f"❌ Error al resolver la ecuación de Riccati: {e}")
            return False

    # ── CLI ──────────────────────────────────────────────────────
    def _cli_thread(self):
        help_text = """
╔══════════════════════════════════════════════════════════╗
║     Horus GCS V9.2 — LQI / LQG / FUZZY Multiplexer       ║
╠══════════════════════════════════════════════════════════╣
║ MODO DE CONTROL (Cambio en vivo):                        ║
║   mode lqi         LQI clásico (Filtro Complementario)   ║
║   mode lqg         LQI + Filtro Observador de Kalman     ║
║   mode fuzzy       Motor de Inferencia Difuso (25 Reglas)║
║                                                          ║
║ TUNING FUZZY (Moldea la superficie de control):          ║
║   tune fuzzy <E_max> <Rate_max> <Out_PWM>                ║
║     Ej: tune fuzzy 30 200 300                            ║
║                                                          ║
║ VUELO BÁSICO Y SETPOINTS:                                ║
║   cal              Calibra el giroscopio                 ║
║   arm              Arma motores en ralentí               ║
║   dis              Desarma motores                       ║
║   start            Inicia el control estabilizado        ║
║   stop             PARADA DE EMERGENCIA                  ║
║   status           Imprime el panel de estado completo   ║
║   thr <us>         Ajusta el throttle base 1000..1600    ║
║   roll <°>         Setea el ángulo objetivo de Roll      ║
║   pitch <°>        Setea el ángulo objetivo de Pitch     ║
║   aw <valor>       Ajusta el límite Anti-Windup          ║
║                                                          ║
║ TUNING LQI Y LQG:                                        ║
║   tune roll|pitch <Q_ang> <Q_rate> <Q_int> <R>           ║
║   kalman roll|pitch <Qw_pos> <Qw_vel> <Rv_pos> <Rv_vel>  ║
╚══════════════════════════════════════════════════════════╝
"""
        print(help_text)

        while not self.stop_evt.is_set():
            try:
                s = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                self.stop_evt.set()
                break
            if not s:
                continue
            parts = s.split()
            cmd = parts[0].lower()
            args = parts[1:]

            try:
                if cmd == "status":
                    self._print_status()
                elif cmd == "help" or cmd == "?":
                    print(help_text)
                    
                # --- CONTROL DE MODOS ---
                elif cmd == "mode" and len(args) == 1:
                    m = args[0].lower()
                    if m == "lqi":
                        ok = self._send(ctrl_mode=0)
                        print("🧠 Modo LQI Activado" if ok else "❌ Sin ACK")
                    elif m == "lqg":
                        ok = self._send(ctrl_mode=1)
                        print("🧠 Modo LQG Activado" if ok else "❌ Sin ACK")
                    elif m == "fuzzy":
                        ok = self._send(ctrl_mode=2)
                        print("🧠 Modo FUZZY Activado" if ok else "❌ Sin ACK")
                        
                # --- SINTONIZADOR FUZZY ---
                elif cmd == "tune" and len(args) == 4 and args[0].lower() == "fuzzy":
                    e_max = float(args[1])
                    r_max = float(args[2])
                    out_max = float(args[3])
                    
                    self.cmd.set(fz_e_max=e_max, fz_r_max=r_max, fz_out_max=out_max)
                    ok = self._send(flags=CMD_UPDATE_FUZZY)
                    
                    if ok:
                        print(f"✅ Límites Fuzzy Actualizados -> ErrMax: {e_max}°, RateMax: {r_max}°/s, OutMax: {out_max}")
                    else:
                        print("❌ Sin ACK")

                # --- COMANDOS BÁSICOS RESTAURADOS ---
                elif cmd == "cal":
                    ok = self._send(flags=CMD_CAL_ALL)
                    print("✅ Calibración iniciada" if ok else "❌ Sin ACK")
             
