#!/usr/bin/env python3
import time
import struct
import threading
import math

from pyrf24 import RF24, RF24_PA_LOW, RF24_250KBPS, RF24_CRC_16

# HW
CE_PIN = 25
CSN_DEV = 0
ADDR = b"DRONE"
RF_CH = 110

# Packet types
PKT_TELEM = 0xA1
PKT_CMD   = 0xB1

# STM->Pi telemetry (24 bytes)
TELEM_FMT = struct.Struct("<BBIhhhhhhHBB")
# type, tx_seq, t_ms, ax,ay,az, gx,gy,gz, status, cmd_echo, ping_echo

# Pi->STM command (12 bytes)
CMD_FMT = struct.Struct("<BBHHBHB B")
# type, cmd_seq, flags, thr_us, motor_mask, motor_us, imu_enable, ping_id

# flags
CMD_ARM       = 1 << 0
CMD_DISARM    = 1 << 1
CMD_ESTOP     = 1 << 2
CMD_SET_THR   = 1 << 3
CMD_SET_MOTOR = 1 << 4
CMD_IMU_EN    = 1 << 5
CMD_CAL_ALL   = 1 << 6
CMD_PING      = 1 << 7

# motor mask
M1 = 1 << 0
M2 = 1 << 1
M3 = 1 << 2
M4 = 1 << 3


def accel_to_roll_pitch_deg(ax_mg, ay_mg, az_mg):
    ax = ax_mg / 1000.0
    ay = ay_mg / 1000.0
    az = az_mg / 1000.0
    roll  = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    return roll, pitch


class CmdState:
    def __init__(self):
        self.cmd_seq = 0
        self.flags = 0
        self.thr_us = 1000
        self.motor_mask = 0
        self.motor_us = 1000
        self.imu_enable = 1
        self.ping_id = 0
        self.lock = threading.Lock()

    def bump(self):
        self.cmd_seq = (self.cmd_seq + 1) & 0xFF

    def build(self) -> bytes:
        with self.lock:
            return CMD_FMT.pack(
                PKT_CMD,
                self.cmd_seq & 0xFF,
                self.flags & 0xFFFF,
                self.thr_us & 0xFFFF,
                self.motor_mask & 0xFF,
                self.motor_us & 0xFFFF,
                self.imu_enable & 0xFF,
                self.ping_id & 0xFF
            )

    def set(self, flags, thr=None, mm=None, mus=None, imu=None, ping=False):
        with self.lock:
            self.flags = flags
            if thr is not None:
                self.thr_us = int(thr)
            if mm is not None:
                self.motor_mask = int(mm)
            if mus is not None:
                self.motor_us = int(mus)
            if imu is not None:
                self.imu_enable = 1 if imu else 0
            if ping:
                self.ping_id = (self.ping_id + 1) & 0xFF
            self.bump()


def main():
    radio = RF24(CE_PIN, CSN_DEV)

    print("=== Horus Link Console (Pi) ===")
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

    cmd = CmdState()
    radio.writeAckPayload(1, cmd.build())  # preload

    stop_evt = threading.Event()

    # ACK keepalive (importante: el ACK payload se consume)
    def ack_keepalive():
        while not stop_evt.is_set():
            try:
                radio.writeAckPayload(1, cmd.build())
            except Exception:
                pass
            time.sleep(0.02)  # 50 Hz

    threading.Thread(target=ack_keepalive, daemon=True).start()

    # Telemetry shared state
    imu_lock = threading.Lock()
    last_telem = None
    last_rx_time = time.time()
    last_cmd_echo = 0
    last_ping_echo = 0

    # Live view control
    view_evt = threading.Event()     # when set => show live IMU
    view_hz = 15.0                   # default print rate
    view_period = 1.0 / view_hz

    def wait_echo(target_echo, timeout_s):
        nonlocal last_cmd_echo
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if last_cmd_echo == target_echo:
                return True
            time.sleep(0.01)
        return False

    # Live view thread (NO bloquea input)
    def imu_view_thread():
        nonlocal last_telem, last_rx_time
        next_t = time.time()
        while not stop_evt.is_set():
            if not view_evt.is_set():
                time.sleep(0.05)
                continue

            now = time.time()
            if now < next_t:
                time.sleep(0.005)
                continue
            next_t = now + view_period

            with imu_lock:
                t = last_telem

            if not t:
                print("[IMU] (no data yet)")
                continue

            age_ms = int((time.time() - last_rx_time) * 1000)
            roll, pitch = accel_to_roll_pitch_deg(t["ax"], t["ay"], t["az"])

            # status bits
            imu_en   = (t["status"] >> 4) & 1
            cal_busy = (t["status"] >> 3) & 1
            armed    = (t["status"] >> 2) & 1
            fs       = (t["status"] >> 5) & 1

            print(
                f"[IMU] age={age_ms:4d}ms seq={t['tx_seq']:3d} t={t['t_ms']:7d} "
                f"A=({t['ax']:+5d},{t['ay']:+5d},{t['az']:+5d})mg "
                f"G=({t['gx']:+5d},{t['gy']:+5d},{t['gz']:+5d})0.1dps "
                f"R={roll:+6.1f}° P={pitch:+6.1f}° "
                f"imu={imu_en} cal={cal_busy} arm={armed} fs={fs}"
            )

    threading.Thread(target=imu_view_thread, daemon=True).start()

    print("\nCommands:")
    print("  cal            (gyro+acc juntos, tarda ~4s)")
    print("  imu on/off     (habilita IMU en STM)")
    print("  view on/off    (muestra IMU en vivo en la consola)")
    print("  view hz <n>    (cambia Hz del view, ej: view hz 10)")
    print("  arm | dis | stop")
    print("  t <us> | m1 <us> | m2 <us> | m3 <us> | m4 <us>")
    print("  ping")
    print("  quit\n")

    def console_thread():
        nonlocal view_period, view_hz, last_ping_echo
        while True:
            s = input("> ").strip().lower()
            if not s:
                continue

            if s == "quit":
                stop_evt.set()
                return

            if s == "view on":
                view_evt.set()
                print(f"✅ view ON ({view_hz:.1f} Hz)")
                continue

            if s == "view off":
                view_evt.clear()
                print("✅ view OFF")
                continue

            if s.startswith("view hz "):
                try:
                    v = float(s.split()[2])
                    if v < 1:
                        v = 1
                    if v > 50:
                        v = 50
                    view_hz = v
                    view_period = 1.0 / view_hz
                    print(f"✅ view Hz = {view_hz:.1f}")
                except Exception:
                    print("Uso: view hz <n>  (ej: view hz 10)")
                continue

            if s == "arm":
                cmd.set(CMD_ARM)
                target = cmd.cmd_seq
                print("-> arm (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s == "dis":
                cmd.set(CMD_DISARM)
                target = cmd.cmd_seq
                print("-> dis (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s == "stop":
                cmd.set(CMD_ESTOP)
                target = cmd.cmd_seq
                print("-> stop (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s.startswith("t "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_THR, thr=v)
                target = cmd.cmd_seq
                print(f"-> t {v} (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s.startswith("m1 "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_MOTOR, mm=M1, mus=v)
                target = cmd.cmd_seq
                print(f"-> m1 {v} (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s.startswith("m2 "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_MOTOR, mm=M2, mus=v)
                target = cmd.cmd_seq
                print(f"-> m2 {v} (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s.startswith("m3 "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_MOTOR, mm=M3, mus=v)
                target = cmd.cmd_seq
                print(f"-> m3 {v} (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s.startswith("m4 "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_MOTOR, mm=M4, mus=v)
                target = cmd.cmd_seq
                print(f"-> m4 {v} (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s == "cal":
                cmd.set(CMD_CAL_ALL)
                target = cmd.cmd_seq
                print("-> cal (gyro+acc) waiting ack (up to 8s)")
                ok = wait_echo(target, 8.0)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s.startswith("imu "):
                onoff = s.split()[1]
                cmd.set(CMD_IMU_EN, imu=(onoff == "on"))
                target = cmd.cmd_seq
                print(f"-> imu {onoff} (waiting ack)")
                ok = wait_echo(target, 1.5)
                print(("✅" if ok else "❌") + f" cmd_echo={target}")
                continue

            if s == "ping":
                cmd.set(CMD_PING, ping=True)
                target = cmd.cmd_seq
                ping_target = cmd.ping_id
                print("-> ping (waiting ack + ping_echo)")
                ok = wait_echo(target, 1.5)
                if not ok:
                    print("❌ cmd_echo timeout")
                    continue
                t0 = time.time()
                while time.time() - t0 < 1.5:
                    if last_ping_echo == ping_target:
                        print(f"✅ ping_echo={ping_target}")
                        break
                    time.sleep(0.01)
                else:
                    print("❌ ping_echo timeout")
                continue

            print("?? unknown")

    threading.Thread(target=console_thread, daemon=True).start()
    print("✅ Listening...\n")

    while not stop_evt.is_set():
        if radio.available():
            data = radio.read(32)
            now = time.time()
            last_rx_time = now

            if len(data) >= TELEM_FMT.size and data[0] == PKT_TELEM:
                (typ, tx_seq, t_ms,
                 ax, ay, az, gx, gy, gz,
                 status, cmd_echo_rx, ping_echo_rx) = TELEM_FMT.unpack(data[:TELEM_FMT.size])

                last_cmd_echo = cmd_echo_rx
                last_ping_echo = ping_echo_rx

                with imu_lock:
                    last_telem = {
                        "tx_seq": tx_seq,
                        "t_ms": t_ms,
                        "ax": ax, "ay": ay, "az": az,
                        "gx": gx, "gy": gy, "gz": gz,
                        "status": status
                    }

        time.sleep(0.001)

    stop_evt.set()
    print("Bye.")


if __name__ == "__main__":
    main()
