"""Serial, TCP, and simulated transports with a common interface."""

import json
import queue
import re
import socket
import threading
import time
import os

import serial

from .sim import SimDevice


def detect_auto_serial_port():
    """Return the best matching Arduino serial port, or None if none are found."""
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    for port in ports:
        if port.vid == 0x1A86 and port.pid == 0x7523:
            return port.device
    for port in ports:
        if port.vid in (0x2341, 0x2A03):
            return port.device
    for port in ports:
        if re.search(r"arduino|ch340|mega", port.description or "", re.IGNORECASE):
            return port.device
    return None


def resolve_serial_port(requested=None):
    """Resolve an explicit port, ARM_PORT, or a detected Arduino port."""
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    available = {port.device for port in ports}
    if requested and requested.lower() != "auto" and requested in available:
        return requested, False

    environment_port = os.environ.get("ARM_PORT")
    if environment_port and environment_port in available:
        return environment_port, False

    detected = detect_auto_serial_port()
    if detected:
        return detected, True
    return requested or "auto", False


class SerialTransport:
    def __init__(self, port, baud=115200, timeout=1.0, reset_wait=True):
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.rx_queue = queue.Queue()
        self._stop = threading.Event()
        self.reset_wait = reset_wait
        self.ready_seen = False
        self.boot_lines = []
        if reset_wait:
            deadline = time.time() + 3.0
            while time.time() < deadline:
                try:
                    line = self.ser.readline()
                except Exception:
                    break
                if not line:
                    continue
                text = line.decode("utf-8", errors="replace").rstrip("\r\n")
                if not text:
                    continue
                self.boot_lines.append(text)
                if "READY" in text:
                    self.ready_seen = True
                    break
            time.sleep(0.5)
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception:
                time.sleep(0.05)
                continue
            if not data:
                time.sleep(0.01)
                continue
            buf += data
            while True:
                idx = buf.find(b"{")
                if idx == -1:
                    buf = b""
                    break
                if idx > 0:
                    buf = buf[idx:]
                text = buf.decode("utf-8", errors="replace")
                try:
                    obj, end = json.JSONDecoder().raw_decode(text)
                except (json.JSONDecodeError, ValueError):
                    break
                self.rx_queue.put(text[:end].strip())
                buf = buf[end:]

    def send_line(self, line):
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()

    def close(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


class TcpTransport:
    def __init__(self, host, port, timeout=2.0):
        self.host = host
        self.port = port
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(0.1)
        self.rx_queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self.ready_seen = True
        self.boot_lines = []

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.sock.recv(4096)
                if not data:
                    break
                buf += data
                while True:
                    idx = buf.find(b"{")
                    if idx == -1:
                        buf = b""
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    text = buf.decode("utf-8", errors="replace")
                    try:
                        obj, end = json.JSONDecoder().raw_decode(text)
                    except (json.JSONDecodeError, ValueError):
                        break
                    self.rx_queue.put(text[:end].strip())
                    buf = buf[end:]
            except socket.timeout:
                continue
            except Exception:
                time.sleep(0.05)

    def send_line(self, line):
        self.sock.sendall((line + "\n").encode("ascii"))

    def close(self):
        self._stop.set()
        try:
            self.sock.close()
        except Exception:
            pass


class SimTransport:
    def __init__(self, config=None):
        self.device = SimDevice(config)
        self.rx_queue = queue.Queue()
        self.ready_seen = True
        self.boot_lines = []

    def send_line(self, line):
        for reply in self.device.process(line):
            self.rx_queue.put(json.dumps(reply) if not isinstance(reply, str) else reply)

    def close(self):
        pass
