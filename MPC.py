#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Horus GCS V11.2 — EXCLUSIVE MPC EDITION
(Interfaz de Control Simplificada y Monitoreo de Ganancias Q/R)
"""

import time, struct, threading
from collections import deque
import numpy as np
from scipy.linalg import eigvals

from pyrf24 import RF24, RF24_PA_LOW, RF24_250KBPS, RF24_CRC_16

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── CONFIGURACIÓN FÍSICA DE LOS EJES ─────────────────────────────
INVERT_ROLL_AXIS  = True   
INVERT_PITCH_AXIS = False

# ── Hardware ─────────────────────────────────────────────────────
CE_PIN  = 25
CSN_DEV = 0
ADDR    = b"DRONE"
RF_CH   = 110

# ── Protocolo Estricto V9.5 ──────────────────────────────────────
PKT_TELEM  = 0xA1
PKT_CMD    = 0xB1

CMD_FMT    = struct.Struct("<BBHHhhhhhhhhHBBB")   
TELEM_FMT  = struct.Struct("<BBIhhhhhHHHHHBB")    

# ── Flags de comando ─────────────────────────────────────────────
CMD_ARM           = 1 << 0
CMD_DISARM        = 1 << 1
CMD_ESTOP         = 1 << 2
CMD_SET_THR       = 1 << 3
CMD_IMU_EN        = 1 << 4
CMD_CAL_ALL       = 1 << 5
CMD_PING          = 1 << 6
CMD_START_MOTORS  = 1 << 7
CMD_STREAM_MATRIX = 1 << 12  
CMD_SET_U_MAX     = 1 << 13  

# ── Bits de status ───────────────────────────────────────────────
ST_MPU_OK     = 1 << 0
ST_ACCEL_OK   = 1 << 1
ST_ARMED      = 1 << 2
ST_CAL_BUSY   = 1 << 3
ST_IMU_EN     = 1 << 4
ST_FAILSAFE   = 1 << 5
ST_MOTORS_RUN = 1 << 6

class DroneMPCModel:
    def __init__(self, dt=0.005, N=10):
        self.dt = dt
        self.N = N  
        
        self.A_roll = np.array([[0.9976, 0.02669], [-0.03649, 0.9870]])
        self.B_roll = np.array([[-0.0007416], [0.0003183]])
        if INVERT_ROLL_AXIS: self.B_roll = -self.B_roll
        self.A_aug_roll, self.B_aug_roll = self._augment(self.A_roll, self.B_roll)

        self.A_pitch = np.array([[0.9939, 0.01319], [-0.03461, 0.9828]])
        self.B_pitch = np.array([[-0.0007354], [0.007451]])
        if INVERT_PITCH_AXIS: self.B_pitch = -self.B_pitch
        self.A_aug_pitch, self.B_aug_pitch = self._augment(self.A_pitch, self.B_pitch)

    def _augment(self, A, B):
        A_aug = np.array([
            [A[0,0], A[0,1], 0.0],
            [A[1,0], A[1,1], 0.0],
            [self.dt, 0.0,   1.0]
        ])
        B_aug = np.array([[B[0,0]], [B[1,0]], [0.0]])
        return A_aug, B_aug

    def condense(self, A, B, Q, R):
        nx, nu = A.shape[0], B.shape[1]
        Phi = np.zeros((self.N * nx, nx))
        A_p = np.eye(nx)
        for i in range(self.N):
            A_p = A_p @ A
            Phi[i*nx:(i+1)*nx, :] = A_p
            
        Gamma = np.zeros((self.N * nx, self.N * nu))
        for i in range(self.N):
            for j in range(i + 1):
                Gamma[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = np.linalg.matrix_power(A, i - j) @ B

        Q_bar = np.kron(np.eye(self.N), Q)
        R_bar = np.kron(np.eye(self.N), R)

        H = 2.0 * (Gamma.T @ Q_bar @ Gamma + R_bar)
        F = 2.0 * (Gamma.T @ Q_bar @ Phi)
        
        L_step = 1.0 / np.max(np.real(eigvals(H)))
        return H, F, L_step

def _i16_x10(v): return max(-32768, min(32767, int(round(v * 10.0))))

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
        with self._lock: return self.telem, self.last_rx

class CmdState:
    def __init__(self):
        self._lock = threading.RLock()
        self.seq = 0
        self.flags = 0
        self.thr_us = 1120
        self.sp_roll = 0.0
        self.sp_pitch = 0.0
        
        self.u_max = 300.0   
        self.imu_en = 1
        self.ping_id = 0
        self.ctrl_mode = 4  
        
        # Variables locales para mostrar en status (Inicializadas con valores nominales aprox)
        self.q_roll  = [10.0, 1.0, 0.5]  
        self.r_roll  = 1.0
        self.q_pitch = [10.0, 1.0, 0.5]
        self.r_pitch = 1.0
        
        self.matrix_chunk = [0.0, 0.0, 0.0, 0.0]

    def pack(self):
        with self._lock:
            if self.flags & CMD_STREAM_MATRIX:
                header = struct.pack("<BBHH", PKT_CMD, self.seq & 0xFF, self.flags & 0xFFFF, int(self.thr_us) & 0xFFFF)
                matrix_data = struct.pack("<ffff", *self.matrix_chunk)
                footer = struct.pack("<HBBB", int(self.u_max) & 0xFFFF, self.imu_en & 0xFF, self.ping_id & 0xFF, self.ctrl_mode & 0xFF)
                return header + matrix_data + footer
            else:
                return CMD_FMT.pack(
                    PKT_CMD, self.seq & 0xFF, self.flags & 0xFFFF, int(self.thr_us) & 0xFFFF,
                    _i16_x10(self.sp_roll), _i16_x10(self.sp_pitch),
                    0, 0, 0, 0, 0, 0,   
                    int(self.u_max) & 0xFFFF, self.imu_en & 0xFF, self.ping_id & 0xFF, self.ctrl_mode & 0xFF
                )

    def set(self, **kw):
        with self._lock:
            for k, v in kw.items(): setattr(self, k, v)
            self.seq = (self.seq + 1) & 0xFF

def decode_status(st):
    return {
        "mpu": bool(st & ST_MPU_OK), "armed": bool(st & ST_ARMED),
        "cal": bool(st & ST_CAL_BUSY), "imu_en": bool(st & ST_IMU_EN),
        "fs": bool(st & ST_FAILSAFE), "motors": bool(st & ST_MOTORS_RUN),
    }

class HorusGCS:
    GRAPH_LEN = 300  

    def __init__(self):
        self.shared = Shared()
        self.cmd = CmdState()
        self.stop_evt = threading.Event()
        self.radio = None
        
        self.mpc_model = DroneMPCModel(N=10)
        
        self.current_H_roll = np.eye(10)
        self.current_F_roll = np.zeros((10, 3))
        self.current_H_pitch = np.eye(10)
        self.current_F_pitch = np.zeros((10, 3))

        self.roll_hist  = deque(maxlen=self.GRAPH_LEN)
        self.pitch_hist = deque(maxlen=self.GRAPH_LEN)
        self.sp_r_hist  = deque(maxlen=self.GRAPH_LEN)
        self.sp_p_hist  = deque(maxlen=self.GRAPH_LEN)

    def _init_radio(self):
        self.radio = RF24(CE_PIN, CSN_DEV)
        if not self.radio.begin(): return False
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
        
        self.radio.flush_tx()
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
        self.radio.flush_tx()
        self.radio.writeAckPayload(1, self.cmd.pack())
        return ok

    def _estop(self):
        print("\r\n🛑 EMERGENCIA MPC")
        for _ in range(3):
            self._send(flags=CMD_ESTOP, timeout=0.15)
            time.sleep(0.03)

    def _rx_thread(self):
        while not self.stop_evt.is_set():
            if self.radio.available():
                data = self.radio.read(32)
                
                if isinstance(data, list): data = bytes(data)
                elif isinstance(data, bytearray): data = bytes(data)

                try: 
                    self.radio.writeAckPayload(1, self.cmd.pack())
                except Exception: 
                    pass

                if len(data) >= TELEM_FMT.size and data[0] == PKT_TELEM:
                    try:
                        parsed = TELEM_FMT.unpack(data[:TELEM_FMT.size])
                        
                        self.shared.update({
                            "tx_seq": parsed[1], "t_ms": parsed[2], 
                            "roll": parsed[3]/10.0, "pitch": parsed[4]/10.0,
                            "gx": parsed[5]/10.0, "gy": parsed[6]/10.0, "gz": parsed[7]/10.0,
                            "status": parsed[8], "pwm_m1": parsed[9], "pwm_m2": parsed[10],
                            "pwm_m3": parsed[11], "pwm_m4": parsed[12], 
                            "cmd_echo": parsed[13], "ping_echo": parsed[14]
                        })

                        self.roll_hist.append(parsed[3]/10.0)
                        self.pitch_hist.append(parsed[4]/10.0)
                        self.sp_r_hist.append(self.cmd.sp_roll)
                        self.sp_p_hist.append(self.cmd.sp_pitch)
                        
                    except Exception as e: 
                        print(f"\r\n⚠️ [DEBUG] Fallo al desempacar datos: {e}\r\n> ", end="")
            else: 
                time.sleep(0.002)

    def _print_status(self):
        telem, last_rx = self.shared.snapshot()
        if not telem: print("❌ Sin telemetría."); return
        age = int((time.time() - last_rx) * 1000)
        st = decode_status(telem["status"])
        link = "🟢" if age < 500 else "🔴"
        
        print(f"[{link} {age}ms] CEREBRO: PURE MPC ACTIVE")
        print(f"        Roll= {telem['roll']:+6.1f}° SP={self.cmd.sp_roll:+4.1f}° | Pitch= {telem['pitch']:+6.1f}° SP={self.cmd.sp_pitch:+4.1f}°")
        print(f"        Gx= {telem['gx']:+6.1f}°/s | Gy= {telem['gy']:+6.1f}°/s | Gz= {telem['gz']:+6.1f}°/s")
        print(f"        PWM: M1={telem['pwm_m1']} M2={telem['pwm_m2']} M3={telem['pwm_m3']} M4={telem['pwm_m4']} | Thr={self.cmd.thr_us}")
        
        # --- AÑADIDO: Impresión de Q y R del MPC ---
        print(f"        Q_Roll : Ang={self.cmd.q_roll[0]:.1f} Rate={self.cmd.q_roll[1]:.1f} Int={self.cmd.q_roll[2]:.1f} | R_Roll ={self.cmd.r_roll:.2f}")
        print(f"        Q_Pitch: Ang={self.cmd.q_pitch[0]:.1f} Rate={self.cmd.q_pitch[1]:.1f} Int={self.cmd.q_pitch[2]:.1f} | R_Pitch={self.cmd.r_pitch:.2f}")
        
        print(f"        Arm={st['armed']} Motors={st['motors']} MPU={st['mpu']} U_MAX={self.cmd.u_max} FS={st['fs']}")

    def _cli_thread(self):
        print("Escribe 'help' o '?' para ver los comandos.")
        while not self.stop_evt.is_set():
            try: s = input("> ").strip()
            except (EOFError, KeyboardInterrupt): self.stop_evt.set(); break
            if not s: continue
            parts = s.split()
            cmd, args = parts[0].lower(), parts[1:]

            try:
                if cmd == "status": self._print_status()
                elif cmd == "help" or cmd == "?":
                    print("""
╔══════════════════════════════════════════════════════════╗
║    Horus GCS V11.2 — EXCLUSIVE MPC INTERFACE             ║
╠══════════════════════════════════════════════════════════╣
║ TUNING MPC (Requiere Disarm):                            ║
║   tune roll <Q_ang> <Q_rate> <Q_int> <R>                 ║
║   tune pitch <Q_ang> <Q_rate> <Q_int> <R>                ║
║                                                          ║
║ Comandos de Vuelo Base:                                  ║
║   cal, arm, dis, stop, start, status, thr <us>           ║
║   roll <°>, pitch <°>, umax <valor>                      ║
╚══════════════════════════════════════════════════════════╝""")
                
                # --- AÑADIDO: Comando "tune" sin "mpc" ---
                elif cmd == "tune" and len(args) == 5:
                    axis_str = args[0].lower()
                    if axis_str in ["roll", "pitch"]:
                        qa, qv, qi, r_val = map(float, args[1:5])
                        telem, _ = self.shared.snapshot()
                        if telem and (telem["status"] & ST_ARMED):
                            print("⚠️ ERROR: Desarma el sistema (dis) antes de transmitir matrices.")
                            continue
                            
                        print(f"🔄 Rediseñando horizonte óptico para {axis_str.upper()}...")
                        Q = np.diag([qa, qv, qi])
                        R = np.array([[r_val]])
                        
                        if axis_str == "roll":
                            H, F, L_step = self.mpc_model.condense(self.mpc_model.A_aug_roll, self.mpc_model.B_aug_roll, Q, R)
                            self.current_H_roll, self.current_F_roll = H, F
                            axis_id = 0
                            self.cmd.set(q_roll=[qa, qv, qi], r_roll=r_val) # Guardar para el status
                        else:
                            H, F, L_step = self.mpc_model.condense(self.mpc_model.A_aug_pitch, self.mpc_model.B_aug_pitch, Q, R)
                            self.current_H_pitch, self.current_F_pitch = H, F
                            axis_id = 1
                            self.cmd.set(q_pitch=[qa, qv, qi], r_pitch=r_val) # Guardar para el status
                            
                        flat_floats = list(H.flatten()) + list(F.flatten()) + [float(L_step)]
                        chunks = [flat_floats[i:i+4] for i in range(0, len(flat_floats), 4)]
                        
                        print(f"📡 Transmitiendo ráfaga por ráfaga hacia la RAM...")
                        stream_success = True
                        for idx, chunk in enumerate(chunks):
                            while len(chunk) < 4: chunk.append(0.0)
                            thr_val = (idx << 8) | axis_id
                            
                            self.cmd.set(flags=CMD_STREAM_MATRIX, thr_us=thr_val, matrix_chunk=chunk)
                            target_seq = self.cmd.seq
                            
                            self.radio.flush_tx()
                            self.radio.writeAckPayload(1, self.cmd.pack())
                            
                            t0 = time.time()
                            chunk_ack = False
                            while time.time() - t0 < 0.4:
                                t_snap, _ = self.shared.snapshot()
                                if t_snap and t_snap["cmd_echo"] == target_seq:
                                    chunk_ack = True
                                    break
                                time.sleep(0.002)
                                
                            if not chunk_ack:
                                print(f"❌ Pérdida de sincronía en el bloque {idx}. Abortado.")
                                stream_success = False
                                break
                        
                        self.cmd.set(flags=0, thr_us=1120)
                        self.radio.flush_tx()
                        self.radio.writeAckPayload(1, self.cmd.pack())
                        if stream_success:
                            print(f"✅ ¡Éxito! Controlador {axis_str.upper()} reconfigurado.")

                elif cmd == "cal": 
                    if self._send(flags=CMD_CAL_ALL): print("✅ Calibración iniciada")
                    else: print("❌ Error de comunicación")
                elif cmd == "arm": 
                    if self._send(flags=CMD_ARM): print("🔒 ARMADO")
                    else: print("❌ Mando de armado perdido. (No hubo ACK)")
                elif cmd == "start": 
                    if self._send(flags=CMD_START_MOTORS): print("🚀 Motores en marcha y ejecutando MPC")
                    else: print("❌ Mando de start perdido. (No hubo ACK)")
                elif cmd == "dis": 
                    if self._send(flags=CMD_DISARM): print("✅ Desarmado")
                elif cmd == "stop": self._estop()
                elif cmd == "thr" and len(args) == 1: self._send(flags=CMD_SET_THR, thr_us=int(args[0])); print(f"🚀 Throttle -> {args[0]}")
                elif cmd == "roll" and len(args) == 1: self._send(sp_roll=float(args[0])); print(f"✅ SP Roll -> {args[0]}")
                elif cmd == "pitch" and len(args) == 1: self._send(sp_pitch=float(args[0])); print(f"✅ SP Pitch -> {args[0]}")
                elif cmd == "umax" and len(args) == 1: self._send(flags=CMD_SET_U_MAX, u_max=float(args[0])); print(f"✅ U_MAX -> {args[0]}")
                elif cmd == "quit": self._estop(); self.stop_evt.set()
            except Exception as e: print(f"Error en comando: {e}")

    def _run_graph(self):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
        fig.patch.set_facecolor("#0d0d0d")
        for ax in (ax1, ax2):
            ax.set_facecolor("#1a1a2e")
            ax.tick_params(colors="#cccccc")
            ax.grid(True, color="#222244", linestyle="--", linewidth=0.5)

        fig.suptitle("Horus GCS V11.2 — Pure MPC Trajectory Monitor", color="#e0e0ff", fontsize=13)
        ln_roll,    = ax1.plot([], [], color="#4fc3f7", lw=1.5, label="Roll Real")
        ln_sp_roll, = ax1.plot([], [], color="#ef5350", lw=1.0, ls="--", label="Setpoint")
        ln_pred_r,  = ax1.plot([], [], color="#00e676", lw=2.0, ls=":", label="Horizonte MPC (Roll)")
        ax1.set_ylim(-45, 45); ax1.legend(loc="upper right", facecolor="#111133", labelcolor="white", fontsize=8)

        ln_pitch,    = ax2.plot([], [], color="#a5d6a7", lw=1.5, label="Pitch Real")
        ln_sp_pitch, = ax2.plot([], [], color="#ffb74d", lw=1.0, ls="--", label="Setpoint")
        ln_pred_p,  = ax2.plot([], [], color="#ffd600", lw=2.0, ls=":", label="Horizonte MPC (Pitch)")
        ax2.set_ylim(-45, 45); ax2.legend(loc="upper right", facecolor="#111133", labelcolor="white", fontsize=8)
        plt.tight_layout(rect=[0, 0, 1, 0.95])

        def _update(_frame):
            if self.stop_evt.is_set(): plt.close(fig); return
            
            try:
                r_hist = list(self.roll_hist)
                p_hist = list(self.pitch_hist)
                sr_hist = list(self.sp_r_hist)
                sp_hist = list(self.sp_p_hist)
                
                n = len(r_hist)
                if n == 0: return
                xs = list(range(n))
                
                ln_roll.set_data(xs, r_hist)
                ln_sp_roll.set_data(xs, sr_hist)
                ln_pitch.set_data(xs, p_hist)
                ln_sp_pitch.set_data(xs, sp_hist)
                
                telem, _ = self.shared.snapshot()
                if telem:
                    try:
                        x0_r = np.array([[telem["roll"] - self.cmd.sp_roll], [telem["gx"]], [0.0]])
                        U_r = -np.linalg.inv(self.current_H_roll) @ self.current_F_roll @ x0_r
                        p_roll, xt = [], x0_r.copy()
                        for idx in range(self.mpc_model.N):
                            xt = self.mpc_model.A_aug_roll @ xt + self.mpc_model.B_aug_roll * np.clip(U_r[idx], -self.cmd.u_max, self.cmd.u_max)
                            p_roll.append(xt[0,0] + self.cmd.sp_roll)
                        ln_pred_r.set_data(list(range(n-1, n-1 + self.mpc_model.N)), p_roll)
                    except Exception: pass

                    try:
                        x0_p = np.array([[telem["pitch"] - self.cmd.sp_pitch], [telem["gy"]], [0.0]])
                        U_p = -np.linalg.inv(self.current_H_pitch) @ self.current_F_pitch @ x0_p
                        p_pitch, xt = [], x0_p.copy()
                        for idx in range(self.mpc_model.N):
                            xt = self.mpc_model.A_aug_pitch @ xt + self.mpc_model.B_aug_pitch * np.clip(U_p[idx], -self.cmd.u_max, self.cmd.u_max)
                            p_pitch.append(xt[0,0] + self.cmd.sp_pitch)
                        ln_pred_p.set_data(list(range(n-1, n-1 + self.mpc_model.N)), p_pitch)
                    except Exception: pass

                ax1.set_xlim(max(0, n - self.GRAPH_LEN), n + 12)
                ax2.set_xlim(max(0, n - self.GRAPH_LEN), n + 12)

            except Exception as e:
                print(f"\r\n⚠️ [DEBUG GRÁFICA] Error de renderizado: {e}\r\n> ", end="")

        self.ani = animation.FuncAnimation(fig, _update, interval=30, cache_frame_data=False)
        plt.show(block=True)
        self.stop_evt.set()

    def run(self):
        print("=== Horus GCS V11.2 - EXCLUSIVE DYNAMIC MPC ===")
        if not self._init_radio(): return
        self.radio.writeAckPayload(1, self.cmd.pack())
        threading.Thread(target=self._rx_thread, daemon=True).start()
        threading.Thread(target=self._cli_thread, daemon=True).start()
        try: self._run_graph()
        except KeyboardInterrupt: pass
        self._estop()
        self.stop_evt.set()

if __name__ == "__main__":
    HorusGCS().run()
