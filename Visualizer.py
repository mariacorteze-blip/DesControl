import sys
import time
import threading
from collections import deque

import serial
import serial.tools.list_ports

import matplotlib.pyplot as plt


# ---------- Config ----------
BAUD = 115200
EXPECTED_COLS = 8  # t_ms,ax,ay,az,gx,gy,gz,acc_ok
PLOT_FPS = 25      # refresco del plot (no del serial)
MAX_POINTS = 1200  # ventana en muestras (~12s a 100Hz)
SERIAL_TIMEOUT = 0.0  # no-bloqueante


# ---------- Helpers ----------
def pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found.")
    # Heurística: preferir dispositivos típicos
    for p in ports:
        d = (p.description or "").lower()
        if any(k in d for k in ["stm", "stlink", "usb", "serial", "cp210", "ch340"]):
            return p.device
    return ports[0].device


def parse_csv_line(line: str):
    """
    Expected: t_ms,ax,ay,az,gx,gy,gz,acc_ok
    Returns (t_ms, gx, gy, gz) or None
    """
    line = line.strip()
    if not line:
        return None
    if line.startswith(("Boot", "MPU", "Gyro", "err,")):
        return None
    if line.startswith("t_ms,"):
        return None

    parts = line.split(",")
    if len(parts) < EXPECTED_COLS:
        return None

    try:
        t_ms = float(parts[0])
        gx = float(parts[4])
        gy = float(parts[5])
        gz = float(parts[6])
    except ValueError:
        return None

    return t_ms, gx, gy, gz


# ---------- Serial reader thread ----------
class SerialCSVReader:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser = None

        self._stop = threading.Event()
        self._thread = None

        # ring buffers for data
        self.t = deque(maxlen=MAX_POINTS)
        self.gx = deque(maxlen=MAX_POINTS)
        self.gy = deque(maxlen=MAX_POINTS)
        self.gz = deque(maxlen=MAX_POINTS)

        # stats
        self.lines_ok = 0
        self.lines_bad = 0
        self.bytes_in = 0
        self.last_line_ts = 0.0

        # internal byte buffer (for robust framing)
        self._rxbuf = bytearray()

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=SERIAL_TIMEOUT)
        # ayuda a bajar latencia en algunos drivers
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def _run(self):
        while not self._stop.is_set():
            try:
                n = self.ser.in_waiting
                if n:
                    chunk = self.ser.read(n)
                    self.bytes_in += len(chunk)
                    self._rxbuf.extend(chunk)

                    # extrae líneas completas (terminadas en \n)
                    while True:
                        idx = self._rxbuf.find(b"\n")
                        if idx < 0:
                            break
                        raw_line = self._rxbuf[:idx+1]
                        del self._rxbuf[:idx+1]

                        line = raw_line.decode(errors="ignore")
                        parsed = parse_csv_line(line)
                        if parsed is None:
                            self.lines_bad += 1
                            continue

                        t_ms, gx, gy, gz = parsed
                        self.t.append(t_ms)
                        self.gx.append(gx)
                        self.gy.append(gy)
                        self.gz.append(gz)

                        self.lines_ok += 1
                        self.last_line_ts = time.time()
                else:
                    # no busy-spin
                    time.sleep(0.001)
            except serial.SerialException:
                break
            except Exception:
                # si hay basura en el stream, no te crashea
                self.lines_bad += 1


# ---------- Main plotting ----------
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else pick_port()
    print(f"Opening {port} @ {BAUD}")

    reader = SerialCSVReader(port, BAUD)
    reader.start()

    plt.ion()
    fig, ax = plt.subplots()
    line_gx, = ax.plot([], [], label="gx")
    line_gy, = ax.plot([], [], label="gy")
    line_gz, = ax.plot([], [], label="gz")
    ax.legend()
    ax.set_xlabel("t (ms)")
    ax.set_ylabel("deg/s")

    last_ui = 0.0
    ui_period = 1.0 / PLOT_FPS

    try:
        while True:
            now = time.time()
            if now - last_ui >= ui_period:
                last_ui = now

                if len(reader.t) > 5:
                    t = list(reader.t)
                    gx = list(reader.gx)
                    gy = list(reader.gy)
                    gz = list(reader.gz)

                    line_gx.set_data(t, gx)
                    line_gy.set_data(t, gy)
                    line_gz.set_data(t, gz)

                    ax.relim()
                    ax.autoscale_view()

                # título con stats (te sirve para debug)
                age = now - reader.last_line_ts if reader.last_line_ts else 999.0
                ax.set_title(
                    f"OK:{reader.lines_ok} BAD:{reader.lines_bad} "
                    f"Bytes:{reader.bytes_in} Last:{age*1000:.0f}ms"
                )

                plt.pause(0.001)
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
