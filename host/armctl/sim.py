"""In-process emulation of the firmware for --sim and tests."""

import json
import time


class SimDevice:
    def __init__(self, config=None):
        self.config = config or {}
        joints_cfg = []
        if config and "joints" in config:
            joints_cfg = config["joints"]
        self.n = 8
        self.joints = []
        for i in range(self.n):
            jc = joints_cfg[i] if i < len(joints_cfg) else {}
            mode = jc.get("mode", "P")
            pmin = jc.get("pulse_min", 110)
            pstop = jc.get("pulse_stop", 307)
            pmax = jc.get("pulse_max", 500)
            limits = jc.get("limits_deg", [0, 180])
            mirror = jc.get("mirror", None)
            motor = jc.get("motor", i + 1)
            label = jc.get("label", "")
            self.joints.append({
                "c": i,
                "motor": motor,
                "label": label,
                "m": mode,
                "d": 90,
                "p": pstop,
                "tg": 90,
                "mv": 0,
                "pulse_min": pmin,
                "pulse_stop": pstop,
                "pulse_max": pmax,
                "min_deg": limits[0],
                "max_deg": limits[1],
                "mirror": mirror,
                "invert": False,
                "home_deg": 90,
                "timed_until": 0.0,
                "timed_pulse": None,
            })
        self.enabled = False
        self.estop = False
        self.estop_latched = False
        self.oe = "HIGH"
        self.watchdog_ms = (config or {}).get("watchdog_ms", 3000)
        self.last_keepalive = time.time()
        self.t0 = time.time()
        self.saved = False
        # normalize mirror source to index
        for j in self.joints:
            m = j.get("mirror")
            if isinstance(m, dict):
                src = m.get("source", m.get("src"))
                if isinstance(src, int) and 1 <= src <= 8:
                    src = src - 1
                m["source"] = src
                j["invert"] = bool(m.get("invert", False))

    def _motor_index(self, motor):
        # motor number -> channel index
        for j in self.joints:
            if j["motor"] == motor:
                return j["c"]
        return motor - 1

    def _deg_to_pulse(self, j, deg):
        rng = j["pulse_max"] - j["pulse_min"]
        return int(j["pulse_min"] + (deg / 180.0) * rng)

    def _mirror_update(self):
        for j in self.joints:
            m = j.get("mirror")
            if not isinstance(m, dict):
                continue
            src_idx = m.get("source")
            try:
                src = self.joints[int(src_idx)]
            except Exception:
                src = None
            if src is None:
                continue
            val = src["d"]
            if m.get("invert") or j.get("invert"):
                val = 180 - val
            j["d"] = val
            j["p"] = self._deg_to_pulse(j, val)
            j["tg"] = val

    def _tick(self):
        now = time.time()
        for j in self.joints:
            if j["timed_until"] and now >= j["timed_until"]:
                j["p"] = j["pulse_stop"]
                j["mv"] = 0
                j["timed_until"] = 0.0
                j["timed_pulse"] = None
        self._mirror_update()

    def process(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return []
        self._tick()
        replies = []
        c0 = cmd[0]

        if cmd == "I":
            replies.append({"status": "OK", "pca9685": "CONECTADO_0x40"})
            return replies
        if cmd == "V":
            replies.append({"status": "VERSION", "fw": "1.0.0", "board": "MEGA2560_PCA9685", "proto": 1})
            return replies
        if cmd == "S":
            for j in self.joints:
                j["p"] = j["pulse_stop"]
                j["mv"] = 0
                j["timed_until"] = 0.0
            self.estop = True
            self.estop_latched = True
            self.enabled = False
            self.oe = "HIGH"
            replies.append({"event": "EMERGENCY_STOP", "oe": "HIGH"})
            return replies
        if cmd == "R":
            self.estop = False
            self.estop_latched = False
            self.enabled = True
            self.oe = "LOW"
            replies.append({"event": "ENABLED", "oe": "LOW"})
            return replies
        if cmd == "F":
            for j in self.joints:
                j["p"] = j["pulse_stop"]
                j["mv"] = 0
            replies.append({"event": "ALL_NEUTRAL"})
            return replies
        if cmd == "H":
            for j in self.joints:
                j["d"] = j["home_deg"]
                j["p"] = self._deg_to_pulse(j, j["home_deg"]) if j["m"] == "P" else j["pulse_stop"]
                j["mv"] = 0
            self._mirror_update()
            replies.append({"ack": "HOME"})
            return replies
        if cmd == "E":
            self.saved = True
            replies.append({"ack": "SAVED"})
            return replies
        if cmd == "D":
            replies.append({"ack": "DEFAULTS"})
            return replies
        if cmd == "K":
            self.last_keepalive = time.time()
            replies.append({"ack": "K_OK"})
            return replies
        if cmd == "Q":
            replies.append(self.state())
            return replies

        if cmd.startswith("M"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            deg = float(parts[1])
            ms = int(parts[2]) if len(parts) > 2 else None
            j = self.joints[ch]
            if j["m"] != "P":
                replies.append({"ack": "ERR", "code": "MODE_CONT_USE_T", "motor": motor, "cmd": cmd})
                return replies
            if deg < j["min_deg"] or deg > j["max_deg"]:
                replies.append({"ack": "ERR", "code": "BAD_ARG", "motor": motor, "cmd": cmd})
                return replies
            j["tg"] = deg
            j["d"] = deg
            j["p"] = self._deg_to_pulse(j, deg)
            j["mv"] = 1
            # mirror pair
            self._mirror_update()
            replies.append({"ack": "M_OK", "motor": motor, "ch": ch, "deg": deg, "n": motor})
            return replies

        if cmd.startswith("P"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            pulse = int(parts[1])
            j = self.joints[ch]
            j["p"] = pulse
            j["mv"] = 0
            if j["pulse_max"] != j["pulse_min"]:
                j["d"] = (pulse - j["pulse_min"]) / (j["pulse_max"] - j["pulse_min"]) * 180.0
            self._mirror_update()
            replies.append({"ack": "P_OK", "motor": motor, "ch": ch, "pulse": pulse, "n": motor})
            return replies

        if cmd.startswith("T"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            offset = int(parts[1])
            ms = int(parts[2])
            if offset < -150 or offset > 150:
                replies.append({"ack": "ERR", "code": "BAD_ARG", "motor": motor, "cmd": cmd})
                return replies
            j = self.joints[ch]
            pulse = j["pulse_stop"] + offset
            j["p"] = pulse
            j["mv"] = 1
            j["timed_pulse"] = pulse
            j["timed_until"] = time.time() + ms / 1000.0
            # mirror pair inverted velocity
            m = j.get("mirror")
            # find joints that mirror this one
            for jj in self.joints:
                mm = jj.get("mirror")
                if isinstance(mm, dict) and mm.get("source") == ch:
                    inv = 1 if (mm.get("invert") or jj.get("invert")) else -1
                    # inverted: subtract offset
                    jj["p"] = jj["pulse_stop"] - offset
                    jj["mv"] = 1
                    jj["timed_pulse"] = jj["pulse_stop"] - offset
                    jj["timed_until"] = time.time() + ms / 1000.0
            replies.append({"ack": "T_OK", "motor": motor, "ch": ch, "offset": offset, "ms": ms, "n": motor})
            return replies

        if cmd.startswith("C"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            pulse = int(parts[1])
            j = self.joints[ch]
            j["pulse_stop"] = pulse
            j["p"] = pulse
            replies.append({"ack": "CALIBRATED", "motor": motor, "ch": ch, "pulse": pulse, "n": motor})
            return replies

        if cmd.startswith("B"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            mn = int(parts[1])
            mx = int(parts[2])
            j = self.joints[ch]
            j["pulse_min"] = mn
            j["pulse_max"] = mx
            replies.append({"ack": "RANGE", "motor": motor, "ch": ch, "min": mn, "max": mx})
            return replies

        if cmd.startswith("N"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            mode = parts[1]
            self.joints[ch]["m"] = mode
            replies.append({"ack": "MODE", "motor": motor, "ch": ch, "mode": mode})
            return replies

        if cmd.startswith("L"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            mn = int(parts[1])
            mx = int(parts[2])
            j = self.joints[ch]
            j["min_deg"] = mn
            j["max_deg"] = mx
            replies.append({"ack": "LIMITS", "motor": motor, "ch": ch, "min": mn, "max": mx})
            return replies

        if cmd.startswith("Y"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            src = parts[1]
            # src may be motor number
            try:
                src_i = int(src)
                if 1 <= src_i <= 8:
                    src_i = src_i - 1
            except ValueError:
                src_i = src
            self.joints[ch]["mirror"] = {"source": src_i, "invert": self.joints[ch].get("invert", False)}
            replies.append({"ack": "MIRROR", "motor": motor, "ch": ch, "src": src})
            return replies

        if cmd.startswith("J"):
            motor = int(cmd[1:])
            ch = self._motor_index(motor)
            self.joints[ch]["invert"] = True
            if self.joints[ch].get("mirror"):
                self.joints[ch]["mirror"]["invert"] = True
            replies.append({"ack": "INVERT", "motor": motor, "ch": ch})
            return replies

        if cmd.startswith("X"):
            body = cmd[1:]
            pairs = body.split(",")
            n = 0
            for pair in pairs:
                if not pair.strip():
                    continue
                chs, deg = pair.split(":")
                motor = int(chs)
                ch = self._motor_index(motor)
                d = float(deg)
                j = self.joints[ch]
                j["d"] = d
                j["tg"] = d
                j["p"] = self._deg_to_pulse(j, d) if j["m"] == "P" else j["p"]
                j["mv"] = 1
                n += 1
            self._mirror_update()
            replies.append({"ack": "X_OK", "n": n})
            return replies

        if cmd.startswith("O"):
            body = cmd[1:]
            parts = body.split(":")
            motor = int(parts[0])
            ch = self._motor_index(motor)
            deg = float(parts[1])
            self.joints[ch]["home_deg"] = deg
            replies.append({"ack": "HOME_SET", "motor": motor, "ch": ch, "deg": deg})
            return replies

        if cmd.startswith("W"):
            ms = int(cmd[1:])
            self.watchdog_ms = ms
            replies.append({"ack": "WATCHDOG", "ms": ms})
            return replies

        if cmd.startswith("A"):
            ms = int(cmd[1:])
            replies.append({"ack": "AUTO", "ms": ms})
            return replies

        replies.append({"ack": "ERR", "code": "BAD_ARG", "cmd": cmd})
        return replies

    def state(self):
        self._tick()
        chs = []
        for j in self.joints:
            chs.append({"c": j["c"], "n": j["motor"], "m": j["m"], "d": j["d"], "p": j["p"], "tg": j["tg"], "mv": j["mv"]})
        return {"status": "STATE", "t": int((time.time() - self.t0) * 1000), "en": 1 if self.enabled else 0,
                "estop": 1 if self.estop_latched else 0, "ch": chs}