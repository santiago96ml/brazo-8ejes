"""Serial, TCP, and simulated transports with a common interface."""

import json
import queue
import socket
import threading
import time

import serial

from .sim import SimDevice


class SerialTransport:
    def __init__(self, port, baud=115200, timeout=0.1):
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.rx_queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                line = self.ser.readline()
                if line:
                    self.rx_queue.put(line.decode("utf-8", errors="replace").rstrip("\r\n"))
            except Exception:
                time.sleep(0.05)

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

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.sock.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.rx_queue.put(line.decode("utf-8", errors="replace").rstrip("\r"))
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

    def send_line(self, line):
        for reply in self.device.process(line):
            self.rx_queue.put(json.dumps(reply) if not isinstance(reply, str) else reply)

    def close(self):
        pass