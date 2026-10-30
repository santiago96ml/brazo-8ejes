"""ArmController client over a transport."""

import json
import queue
import threading
import time


class ArmController:
    def __init__(self, transport, watchdog_ms=0, state_callback=None,
                 send_enable_on_connect=True, keep_enabled=False,
                 disable_on_exit=False):
        self.transport = transport
        self.watchdog_ms = watchdog_ms
        self.state_callback = state_callback
        self.keep_enabled = keep_enabled
        # Default policy: leave the arm enabled/holding on clean exit; the
        # firmware watchdog neutralizes after keepalives stop. Sending F/S is
        # opt-in via --disable.
        self.disable_on_exit = bool(disable_on_exit) and not keep_enabled
        self._reply_queue = queue.Queue()
        self._stop = threading.Event()
        self._dispatcher = threading.Thread(target=self._dispatch, daemon=True)
        self._dispatcher.start()
        self._keepalive_thread = None
        if send_enable_on_connect:
            try:
                self.transport.send_line("R")
            except Exception:
                pass
        if watchdog_ms and watchdog_ms > 0:
            self._keepalive_thread = threading.Thread(target=self._keepalive, daemon=True)
            self._keepalive_thread.start()

    def _dispatch(self):
        while not self._stop.is_set():
            try:
                raw = self.transport.rx_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                obj = json.loads(raw)
            except (ValueError, TypeError):
                obj = {"raw": raw}
            if isinstance(obj, dict) and obj.get("status") == "STATE":
                if self.state_callback is not None:
                    try:
                        self.state_callback(obj)
                    except Exception:
                        pass
                self._reply_queue.put(obj)
            else:
                self._reply_queue.put(obj)

    def _keepalive(self):
        interval = max(0.5, self.watchdog_ms / 2000.0)
        while not self._stop.is_set():
            try:
                self.transport.send_line("K")
            except Exception:
                break
            time.sleep(interval)

    def send_line(self, line):
        self.transport.send_line(line)

    def wait_ack(self, expected, timeout=2.0):
        deadline = time.time() + timeout
        pending = []
        found = None
        while time.time() < deadline and found is None:
            try:
                item = self._reply_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            if self._matches(item, expected):
                found = item
            else:
                pending.append(item)
        for p in pending:
            self._reply_queue.put(p)
        return found

    @staticmethod
    def _matches(item, expected):
        if not isinstance(item, dict):
            return False
        for key in ("ack", "status", "event"):
            if item.get(key) == expected:
                return True
        return False

    def close(self):
        self._stop.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=1.0)
        try:
            if self.disable_on_exit:
                self.transport.send_line("F")
                self.transport.send_line("S")
        except Exception:
            pass
        try:
            self.transport.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False