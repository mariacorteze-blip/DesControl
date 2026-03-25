#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Horus Link (Pi) - Menu + CSV logger + REALTIME gyro plot (3 axes)
Works with your STM32 BASE protocol:
  TELEM (22 bytes): <BBIhhhhhhHBB
  CMD   (12 bytes): <BBHHBHBBx

Features:
- Menu: status, cal, imu on/off, arm, dis, stop, motor set
- Manual roll logger (pair motors) -> CSV with t_s starting at 0.0
- Realtime gyro plot (gx,gy,gz) in dps (3 axes) using matplotlib
  Command: plot on/off   (or plot once)

Safety:
- Ctrl+C anywhere => ESTOP + DISARM + all motors 1000
"""

import time, struct, threading, math, csv, os, sys
from datetime import datetime
from collections import deque

from pyrf24 import RF24, RF24_PA_LOW, RF24_250KBPS, RF24_CRC_16

# ================ RF HW ================
CE_PIN = 25
CSN_DEV = 0
ADDR = b"DRONE"
RF_CH = 110

# ================ Protocol ================
PKT_TELEM = 0xA1
PKT_CMD   = 0xB1

# STM->Pi telemetry (22 bytes)
TELEM_FMT = struct.Struct("<BBIhhhhhhHBB")
# type, tx_seq, t_ms, ax,ay,az, gx,gy,gz, status, cmd_echo, ping

# Pi->STM command (12 bytes)
CMD_FMT = struct.Struct("<BBHHBHBBx")
# type, cmd_seq, flags, thr_us, motor_mask, motor_us, imu_enable, ping_id, pad

# Flags (STM base)
CMD_ARM       = 1 << 0
CMD_DISARM    = 1 << 1
CMD_ESTOP     = 1 << 2
CMD_SET_THR   = 1 << 3
CMD_SET_MOTOR = 1 << 4
CMD_IMU_EN    = 1 << 5
CMD_CAL_ALL   = 1 << 6

# Motor mask bits (STM base)
M1 = 1 << 0
M2 = 1 << 1
M3 = 1 << 2
M4 = 1 << 3
MASK_ALL  = M1 | M2 | M3 | M4
MASK_M1M4 = M1 | M4
MASK_M2M3 = M2 | M3

# ================= Shared State =================
radio = None

state_lock = threading.Lock()
estado = {
    "tx_seq": 0,
    "t_ms": 0,
    "ax": 0, "ay": 0, "az": 0,
    "gx": 0, "gy": 0, "gz": 0,
    "status": 0,
    "cmd_echo": 0,
    "ping": 0,
    "last_rx": 0.0
}

cmd_lock = threading.Lock()
cmd_seq = 0
last_motor_mask = MASK_ALL
last_motor_us = 1000
last_imu_en = 1
last_ping_id = 0

stop_evt = threading.Event()

# ================ Utils ================
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def decodificar_status(status):
    imu_en   = (status >> 4) & 1
    cal_busy = (status >> 3) & 1
    armed    = (status >> 2) & 1
    fs       = (status >> 5) & 1
    return imu_en, cal_busy, armed, fs

def calcular_angulos(ax_mg, ay_mg, az_mg):
    try:
        ax = ax_mg / 1000.0
        ay = ay_mg / 1000.0
        az = az_mg / 1000.0
        roll  = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
        return roll, pitch
    except Exception:
        return 0.0, 0.0

def link_age_ms():
    with state_lock:
        last = estado["last_rx"]
    if last <= 0:
        return 999999
    return int((time.time() - last) * 1000)

def snapshot_estado():
    with state_lock:
        return dict(estado)

# ================ RF Send Helpers ================
def _next_seq():
    global cmd_seq
    cmd_seq = (cmd_seq + 1) & 0xFF
    return cmd_seq

def write_ack_payload(flags, thr_us=1000, motor_mask=0, motor_us=1000, imu_enable=1, ping_id=0):
    global last_motor_mask, last_motor_us, last_imu_en, last_ping_id
    with cmd_lock:
        seq = _next_seq()
        last_motor_mask = motor_mask if (flags & CMD_SET_MOTOR) else last_motor_mask
        last_motor_us   = motor_us   if (flags & CMD_SET_MOTOR) else last_motor_us
        last_imu_en     = imu_enable
        last_ping_id    = ping_id

        pkt = CMD_FMT.pack(
            PKT_CMD,
            seq,
            flags & 0xFFFF,
            int(thr_us) & 0xFFFF,
            int(motor_mask) & 0xFF,
            int(motor_us) & 0xFFFF,
            int(imu_enable) & 0xFF,
            int(ping_id) & 0xFF
        )
        radio.writeAckPayload(1, pkt)

        # Keepalive after a "real" command: send a flags=0 to keep link fresh
        seq2 = _next_seq()
        pkt2 = CMD_FMT.pack(
            PKT_CMD,
            seq2,
            0,
            1000,
            int(last_motor_mask) & 0xFF,
            int(last_motor_us) & 0xFFFF,
            int(last_imu_en) & 0xFF,
            int(last_ping_id) & 0xFF
        )
        radio.writeAckPayload(1, pkt2)

    return seq

def estop_disarm():
    try:
        write_ack_payload(CMD_ESTOP, motor_mask=MASK_ALL, motor_us=1000, imu_enable=last_imu_en)
        time.sleep(0.15)
        write_ack_payload(CMD_DISARM, motor_mask=MASK_ALL, motor_us=1000, imu_enable=last_imu_en)
        time.sleep(0.15)
        write_ack_payload(CMD_SET_MOTOR, motor_mask=MASK_ALL, motor_us=1000, imu_enable=last_imu_en)
    except Exception:
        pass

# ================ Threads ================
def rx_thread():
    while not stop_evt.is_set():
        try:
            if radio.available():
                data = radio.read(32)
                if len(data) >= TELEM_FMT.size and data[0] == PKT_TELEM:
                    (typ, tx_seq, t_ms,
                     ax, ay, az, gx, gy, gz,
                     status, cmd_echo, ping) = TELEM_FMT.unpack(data[:TELEM_FMT.size])

                    with state_lock:
                        estado.update({
                            "tx_seq": tx_seq,
                            "t_ms": t_ms,
                            "ax": ax, "ay": ay, "az": az,
                            "gx": gx, "gy": gy, "gz": gz,
                            "status": status,
                            "cmd_echo": cmd_echo,
                            "ping": ping,
                            "last_rx": time.time()
                        })
        except Exception:
            pass
        time.sleep(0.001)

def ack_keepalive_thread():
    """
    Keep ACK payload stocked (ACK payload gets consumed).
    We resend last known "neutral" packet frequently.
    """
    global last_motor_mask, last_motor_us, last_imu_en, last_ping_id
    while not stop_evt.is_set():
        try:
            with cmd_lock:
                seq = _next_seq()
                pkt = CMD_FMT.pack(
                    PKT_CMD,
                    seq,
                    0,              # flags=0 keepalive
                    1000,
                    int(last_motor_mask) & 0xFF,
                    int(last_motor_us) & 0xFFFF,
                    int(last_imu_en) & 0xFF,
                    int(last_ping_id) & 0xFF
                )
                radio.writeAckPayload(1, pkt)
        except Exception:
            pass
        time.sleep(0.02)  # 50Hz

# ================ Realtime Gyro Plot ================
class GyroPlot:
    def __init__(self, seconds=8.0, hz=50.0):
        self.seconds = float(seconds)
        self.hz = float(hz)
        self.maxlen = int(self.seconds * self.hz)
        self.t0 = None
        self.buf_t = deque(maxlen=self.maxlen)
        self.buf_gx = deque(maxlen=self.maxlen)
        self.buf_gy = deque(maxlen=self.maxlen)
        self.buf_gz = deque(maxlen=self.maxlen)
        self.running = False
        self.thread = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def _run(self):
        # Import inside thread so program still works without matplotlib
        try:
            import matplotlib
            import matplotlib.pyplot as plt
        except Exception as e:
            print("❌ No pude importar matplotlib. Instala con:")
            print("   sudo apt-get install python3-matplotlib")
            print("   o  pip3 install matplotlib")
            self.running = False
            return

        # If no display, matplotlib may fail. We'll try anyway.
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_title("Gyroscope (dps) - Real Time")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("dps")

        line_gx, = ax.plot([], [], label="Gx")
        line_gy, = ax.plot([], [], label="Gy")
        line_gz, = ax.plot([], [], label="Gz")
        ax.legend(loc="upper right")
        ax.grid(True)

        self.t0 = time.time()
        next_t = time.time()

        while self.running and not stop_evt.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(0.001)
                continue
            next_t = now + (1.0 / max(5.0, self.hz))

            s = snapshot_estado()
            if s["last_rx"] <= 0:
                continue

            t_s = now - self.t0
            gx = s["gx"] / 10.0
            gy = s["gy"] / 10.0
            gz = s["gz"] / 10.0

            self.buf_t.append(t_s)
            self.buf_gx.append(gx)
            self.buf_gy.append(gy)
            self.buf_gz.append(gz)

            # Update plot
            if len(self.buf_t) >= 2:
                line_gx.set_data(self.buf_t, self.buf_gx)
                line_gy.set_data(self.buf_t, self.buf_gy)
                line_gz.set_data(self.buf_t, self.buf_gz)

                ax.set_xlim(max(0.0, self.buf_t[0]), self.buf_t[-1])
                # autoscale Y (simple)
                ymin = min(min(self.buf_gx), min(self.buf_gy), min(self.buf_gz))
                ymax = max(max(self.buf_gx), max(self.buf_gy), max(self.buf_gz))
                if abs(ymax - ymin) < 1e-3:
                    ymax = ymin + 1.0
                pad = 0.15 * (ymax - ymin)
                ax.set_ylim(ymin - pad, ymax + pad)

                fig.canvas.draw()
                fig.canvas.flush_events()

        try:
            plt.close(fig)
        except Exception:
            pass

# ================ Manual Roll Logger (CSV, fixed time 0.0) ================
def manual_roll_logger():
    print("\n=== MANUAL ROLL LOGGER (CSV) ===")
    print("Modo manual con teclas para M1+M4 vs M2+M3.")
    print("Antes: haz cal, imu on, arm (desde el menú).")
    print("Durante: a/z sube/baja M1+M4, d/c sube/baja M2+M3, x reset, p pausa, e estop, q salir.\n")

    if link_age_ms() > 500:
        print("❌ No hay telemetría (age > 500ms).")
        return

    s0 = snapshot_estado()
    imu, cal, arm, fs = decodificar_status(s0["status"])
    if arm != 1:
        print("❌ arm=0. Primero ejecuta 'arm'.")
        return

    base = int(input("Base PWM (todos) (ej 1100): ").strip() or "1100")
    base = clamp(base, 1000, 2000)
    step = int(input("Delta por tecla (ej 10): ").strip() or "10")
    step = clamp(step, 1, 50)
    settle = float(input("Segundos de estabilización (ej 2): ").strip() or "2")
    duration = float(input("Duración de logging (seg) (ej 12): ").strip() or "12")

    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"logs/manual_roll_{ts}_base{base}_step{step}.csv"
    print(f"✅ CSV -> {fname}")

    # poner base a todos
    write_ack_payload(CMD_SET_MOTOR, motor_mask=MASK_ALL, motor_us=base, imu_enable=1)
    time.sleep(max(0.0, settle))

    # setup
    d_left = 0
    d_right = 0
    paused = False
    t0 = time.time()
    t_ms0 = None
    last_send = 0.0
    last_print = 0.0

    def getch_nonblock():
        # cross-platform-ish nonblocking: use stdin with select if possible
        # If not supported, fall back to blocking-free by checking sys.stdin in a thread.
        import sys, select
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    # Put terminal in cbreak mode on unix
    try:
        import termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        term_ok = True
    except Exception:
        term_ok = False
        old = None

    try:
        with open(fname, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "t_s","t_stm_s","t_ms",
                "m1_us","m2_us","m3_us","m4_us",
                "gx_dps","gy_dps","gz_dps",
                "roll_acc_deg","pitch_acc_deg",
                "status_hex","imu","cal_busy","armed","failsafe",
                "age_ms","d_left","d_right","paused"
            ])

            print("\nRUN (Ctrl+C = ESTOP). Teclas: a/z, d/c, x, p, e, q\n")

            while True:
                now = time.time()
                t_s = now - t0
                if t_s > duration:
                    break

                k = getch_nonblock() if term_ok else None
                if k:
                    k = k.lower()
                    if k == 'a': d_left += step
                    elif k == 'z': d_left -= step
                    elif k == 'd': d_right += step
                    elif k == 'c': d_right -= step
                    elif k == 'x': d_left = 0; d_right = 0
                    elif k == 'p': paused = not paused
                    elif k == 'e':
                        print("\n🚨 ESTOP!")
                        estop_disarm()
                        return
                    elif k == 'q':
                        print("\nSalir seguro...")
                        break

                    d_left  = clamp(d_left,  -300, 300)
                    d_right = clamp(d_right, -300, 300)

                if paused:
                    m1 = m2 = m3 = m4 = base
                else:
                    m1 = clamp(base + d_left, 1000, 2000)
                    m4 = clamp(base + d_left, 1000, 2000)
                    m2 = clamp(base + d_right, 1000, 2000)
                    m3 = clamp(base + d_right, 1000, 2000)

                # send at ~50Hz
                if (now - last_send) >= 0.02:
                    last_send = now
                    write_ack_payload(CMD_SET_MOTOR, motor_mask=MASK_M1M4, motor_us=m1, imu_enable=1)
                    write_ack_payload(CMD_SET_MOTOR, motor_mask=MASK_M2M3, motor_us=m2, imu_enable=1)

                s = snapshot_estado()
                age = link_age_ms()
                imu_en, cal_busy, armed, fs = decodificar_status(s["status"])
                gx = s["gx"]/10.0
                gy = s["gy"]/10.0
                gz = s["gz"]/10.0
                rollA, pitchA = calcular_angulos(s["ax"], s["ay"], s["az"])

                t_ms = s["t_ms"]
                if t_ms0 is None and t_ms != 0:
                    t_ms0 = t_ms
                t_stm_s = ((t_ms - t_ms0)/1000.0) if (t_ms0 is not None) else 0.0

                w.writerow([
                    round(t_s, 6),
                    round(t_stm_s, 6),
                    t_ms,
                    m1, m2, m3, m4,
                    gx, gy, gz,
                    rollA, pitchA,
                    f"0x{s['status']:04X}",
                    imu_en, cal_busy, armed, fs,
                    age, d_left, d_right, int(paused)
                ])

                if (now - last_print) >= 0.2:
                    last_print = now
                    print(f"t={t_s:6.2f}s m1/m4={m1:4d} m2/m3={m2:4d} "
                          f"dl={d_left:+4d} dr={d_right:+4d} gx={gx:+7.2f} "
                          f"age={age:3d}ms st=0x{s['status']:04X} {'PAUSE' if paused else ''}")

                time.sleep(0.01)  # ~100Hz logging

    except KeyboardInterrupt:
        print("\n⚠️ Ctrl+C -> ESTOP + DISARM")
        estop_disarm()
        return
    finally:
        if term_ok and old is not None:
            try:
                import termios
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    # safe stop
    write_ack_payload(CMD_SET_MOTOR, motor_mask=MASK_ALL, motor_us=1000, imu_enable=1)
    time.sleep(0.2)
    write_ack_payload(CMD_DISARM, motor_mask=MASK_ALL, motor_us=1000, imu_enable=1)
    time.sleep(0.2)
    print(f"✅ Done. CSV saved: {fname}")

# ================ Menu / Main ================
def print_status_once():
    s = snapshot_estado()
    age = link_age_ms()
    imu_en, cal_busy, armed, fs = decodificar_status(s["status"])
    rollA, pitchA = calcular_angulos(s["ax"], s["ay"], s["az"])
    print(f"[STATUS] age={age}ms t_ms={s['t_ms']} "
          f"gx={s['gx']/10.0:+7.2f} gy={s['gy']/10.0:+7.2f} gz={s['gz']/10.0:+7.2f} "
          f"Racc={rollA:+6.1f} Pacc={pitchA:+6.1f} "
          f"imu={imu_en} cal={cal_busy} arm={armed} fs={fs} st=0x{s['status']:04X}")

def menu():
    print("\n=== Horus Menu (Pi) ===")
    print("status                -> imprime 1 línea (no spam)")
    print("cal                   -> calibrar gyro+acc (STM)")
    print("imu on / imu off       -> habilitar IMU (STM)")
    print("arm / dis / stop       -> armar / desarmar / ESTOP")
    print("motor <m1|m2|m3|m4|all> <us>   -> set pwm")
    print("log manual             -> logger manual roll (M1+M4 vs M2+M3) + CSV")
    print("plot on / plot off     -> gráfica gyro 3 ejes en tiempo real")
    print("quit\n")

def main():
    global radio, last_motor_mask, last_motor_us, last_imu_en

    print("=== DescontrOoOol ===")
    radio = RF24(CE_PIN, CSN_DEV)
    if not radio.begin():
        print("❌ radio.begin() falló (SPI/cableado/alimentación)")
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

    # preload (neutral)
    last_motor_mask = MASK_ALL
    last_motor_us = 1000
    last_imu_en = 1
    pkt0 = CMD_FMT.pack(PKT_CMD, 0, 0, 1000, MASK_ALL, 1000, 1, 0)
    radio.writeAckPayload(1, pkt0)

    threading.Thread(target=rx_thread, daemon=True).start()
    threading.Thread(target=ack_keepalive_thread, daemon=True).start()

    # wait RX
    print("⏳ Esperando telemetría...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        if link_age_ms() < 500:
            print("✅ RX OK")
            break
        time.sleep(0.05)
    else:
        print("❌ No RX packets. Revisa RF_ADDR/RF_CH y que STM esté transmitiendo.")
        return

    plotter = GyroPlot(seconds=8.0, hz=50.0)

    menu()

    try:
        while True:
            s = input("> ").strip().lower()
            if not s:
                continue

            if s == "quit":
                break

            if s == "menu":
                menu()
                continue

            if s == "status":
                print_status_once()
                continue

            if s == "cal":
                write_ack_payload(CMD_CAL_ALL, motor_mask=last_motor_mask, motor_us=last_motor_us, imu_enable=last_imu_en)
                print("✅ cal enviado (mira status: cal_busy=1 mientras calibra)")
                continue

            if s.startswith("imu "):
                onoff = s.split()[1]
                en = 1 if onoff == "on" else 0
                last_imu_en = en
                write_ack_payload(CMD_IMU_EN, motor_mask=last_motor_mask, motor_us=last_motor_us, imu_enable=en)
                print(f"✅ imu {onoff} enviado")
                continue

            if s == "arm":
                write_ack_payload(CMD_ARM, motor_mask=last_motor_mask, motor_us=last_motor_us, imu_enable=last_imu_en)
                print("✅ arm enviado (ver arm=1 en status)")
                continue

            if s == "dis":
                write_ack_payload(CMD_DISARM, motor_mask=last_motor_mask, motor_us=last_motor_us, imu_enable=last_imu_en)
                print("✅ disarm enviado")
                continue

            if s == "stop":
                estop_disarm()
                print("✅ ESTOP + DISARM + all=1000")
                continue

            if s.startswith("motor "):
                parts = s.split()
                if len(parts) != 3:
                    print("Uso: motor <m1|m2|m3|m4|all> <us>")
                    continue
                m = parts[1]
                us = clamp(int(parts[2]), 1000, 2000)
                if m == "m1": mask = M1
                elif m == "m2": mask = M2
                elif m == "m3": mask = M3
                elif m == "m4": mask = M4
                elif m == "all": mask = MASK_ALL
                else:
                    print("motor debe ser m1/m2/m3/m4/all")
                    continue
                last_motor_mask = mask
                last_motor_us = us
                write_ack_payload(CMD_SET_MOTOR, motor_mask=mask, motor_us=us, imu_enable=last_imu_en)
                print(f"✅ motor {m} -> {us}")
                continue

            if s == "log manual":
                manual_roll_logger()
                continue

            if s == "plot on":
                plotter.start()
                print("✅ plot ON (se abre ventana matplotlib). Si no ves ventana, estás en modo sin display.")
                continue

            if s == "plot off":
                plotter.stop()
                print("✅ plot OFF")
                continue

            print("?? unknown (type menu)")

    except KeyboardInterrupt:
        print("\n⚠️ Ctrl+C -> ESTOP + DISARM")
        estop_disarm()

    stop_evt.set()
    try:
        plotter.stop()
    except Exception:
        pass
    print("Bye.")

if __name__ == "__main__":
    main()
