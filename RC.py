#!/usr/bin/env python3
import time
import struct
import threading
import sys

from pyrf24 import RF24, RF24_PA_LOW, RF24_250KBPS, RF24_CRC_16

CE_PIN = 25
CSN_DEV = 0
ADDR = b"DRONE"
RF_CH = 110

PKT_IMU = 0xA1
PKT_CMD = 0xB1

IMU_FMT = struct.Struct("<BBIhhhhhhH")      # 20 bytes
CMD_FMT = struct.Struct("<BBHHBHB BB")      # 12 bytes

CMD_CAL_GYRO   = 1 << 0
CMD_CAL_ACCEL  = 1 << 1
CMD_ARM        = 1 << 2
CMD_DISARM     = 1 << 3
CMD_SET_THR    = 1 << 4
CMD_SET_MOTOR  = 1 << 5
CMD_ESTOP      = 1 << 6

M1_BIT = 1 << 0
M2_BIT = 1 << 1
M3_BIT = 1 << 2
M4_BIT = 1 << 3

PROMPT = "> "

class CmdState:
    def __init__(self):
        self.cmd_seq = 0
        self.flags = 0
        self.thr_us = 1000
        self.motor_mask = 0
        self.motor_us = 1000
        self.imu_enable = 1
        self.lock = threading.Lock()

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
                0, 0
            )

    def bump(self):
        self.cmd_seq = (self.cmd_seq + 1) & 0xFF

    def set(self, flags, thr=None, mm=None, mus=None, imu=None):
        with self.lock:
            self.flags = int(flags) & 0xFFFF
            if thr is not None: self.thr_us = int(thr)
            if mm  is not None: self.motor_mask = int(mm) & 0xFF
            if mus is not None: self.motor_us = int(mus)
            if imu is not None: self.imu_enable = 1 if imu else 0
            self.bump()


def main():
    radio = RF24(CE_PIN, CSN_DEV)
    if not radio.begin():
        print("❌ begin() falló")
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
    radio.writeAckPayload(1, cmd.build())  # ACK inicial

    # UI flags
    show_imu = False
    rate_on = False
    ui_lock = threading.Lock()

    # stats
    total = 0
    imu_cnt = 0
    last_rate_t = time.time()
    last_imu_print = time.time()

    def safe_print(msg: str):
        # imprime sin destruir tanto el input, y re-pone el prompt
        sys.stdout.write("\r" + (" " * 80) + "\r")
        sys.stdout.write(msg + "\n")
        sys.stdout.write(PROMPT)
        sys.stdout.flush()

    def push_ack():
        radio.writeAckPayload(1, cmd.build())

    def print_help():
        safe_print(
            "Comandos:\n"
            "  calg | cala | arm | dis | stop\n"
            "  t <us> | m1 <us> | m2 <us> | m3 <us> | m4 <us>\n"
            "  imu on/off   (habilita/corta TX IMU desde STM)\n"
            "  show on/off  (imprime IMU en consola)\n"
            "  rate on/off  (imprime estadísticas cada 2s)\n"
            "  help | exit"
        )

    def console_thread():
        nonlocal show_imu, rate_on
        sys.stdout.write(PROMPT)
        sys.stdout.flush()

        while True:
            try:
                s = input().strip().lower()
            except EOFError:
                break

            if not s:
                sys.stdout.write(PROMPT)
                sys.stdout.flush()
                continue

            if s == "exit":
                safe_print("Bye.")
                # terminamos el proceso completo
                os._exit(0)

            if s in ("help", "h"):
                print_help()
                continue

            if s == "calg":
                cmd.set(CMD_CAL_GYRO); safe_print("[CMD] CAL_GYRO"); push_ack()
            elif s == "cala":
                cmd.set(CMD_CAL_ACCEL); safe_print("[CMD] CAL_ACCEL"); push_ack()
            elif s == "arm":
                cmd.set(CMD_ARM); safe_print("[CMD] ARM"); push_ack()
            elif s == "dis":
                cmd.set(CMD_DISARM); safe_print("[CMD] DISARM"); push_ack()
            elif s == "stop":
                cmd.set(CMD_ESTOP); safe_print("[CMD] ESTOP"); push_ack()

            elif s.startswith("t "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_THR, thr=v); safe_print(f"[CMD] THR={v}"); push_ack()

            elif s.startswith("m1 "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_MOTOR, mm=M1_BIT, mus=v); safe_print(f"[CMD] M1={v}"); push_ack()
            elif s.startswith("m2 "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_MOTOR, mm=M2_BIT, mus=v); safe_print(f"[CMD] M2={v}"); push_ack()
            elif s.startswith("m3 "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_MOTOR, mm=M3_BIT, mus=v); safe_print(f"[CMD] M3={v}"); push_ack()
            elif s.startswith("m4 "):
                v = int(s.split()[1])
                cmd.set(CMD_SET_MOTOR, mm=M4_BIT, mus=v); safe_print(f"[CMD] M4={v}"); push_ack()

            elif s.startswith("imu "):
                onoff = s.split()[1]
                ena = (onoff == "on")
                cmd.set(0, imu=ena)
                safe_print(f"[CMD] IMU_ENABLE={ena}")
                push_ack()

            elif s.startswith("show "):
                onoff = s.split()[1]
                with ui_lock:
                    show_imu = (onoff == "on")
                safe_print(f"[UI] show_imu={show_imu}")

            elif s.startswith("rate "):
                onoff = s.split()[1]
                with ui_lock:
                    rate_on = (onoff == "on")
                safe_print(f"[UI] rate_on={rate_on}")

            else:
                safe_print("?? comando no reconocido (usa help)")

    # start console thread
    t = threading.Thread(target=console_thread, daemon=True)
    t.start()

    safe_print("✅ Pi PRX listo (IMU RX + CMD por ACK). Escribe help para comandos.")

    # RX loop
    while True:
        if radio.available():
            data = radio.read(32)
            total += 1

            if len(data) >= IMU_FMT.size and data[0] == PKT_IMU:
                imu_cnt += 1
                typ, seq, t_ms, ax, ay, az, gx, gy, gz, status = IMU_FMT.unpack(data[:IMU_FMT.size])

                with ui_lock:
                    do_show = show_imu

                # imprime IMU solo si show_imu=on y a ~10Hz
                if do_show and (time.time() - last_imu_print) > 0.1:
                    last_imu_print = time.time()
                    safe_print(
                        f"seq={seq:3d} t={t_ms:7d} "
                        f"A(mg)=({ax:5d},{ay:5d},{az:5d}) "
                        f"G(0.1dps)=({gx:5d},{gy:5d},{gz:5d}) "
                        f"st=0x{status:04X}"
                    )

            # refresh ACK a veces (por si el receptor no ve teclado justo)
            if (total % 50) == 0:
                push_ack()

        # rate solo si lo activas
        with ui_lock:
            do_rate = rate_on

        if do_rate and (time.time() - last_rate_t) > 2.0:
            safe_print(f"[RATE] total={total}/2s imu={imu_cnt}/2s")
            total = 0
            imu_cnt = 0
            last_rate_t = time.time()

        time.sleep(0.001)


if __name__ == "__main__":
    main()
