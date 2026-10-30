"""Wire protocol v1 command builders and reply parsers (pure, no I/O)."""

import json
from dataclasses import dataclass, field


def cmd_info():
    return "I"


def cmd_version():
    return "V"


def cmd_stop():
    return "S"


def cmd_enable():
    return "R"


def cmd_neutral():
    return "F"


def cmd_move(motor, deg, ms=None):
    if ms is None:
        return "M%d:%s" % (motor, deg)
    return "M%d:%s:%d" % (motor, deg, ms)


def cmd_pulse(motor, pulse):
    return "P%d:%d" % (motor, pulse)


def cmd_spin(motor, offset, ms):
    try:
        off = int(offset)
    except (ValueError, TypeError):
        raise ValueError("spin offset must be an integer")
    if off < -150 or off > 150:
        raise ValueError("spin offset must be in -150..150")
    return "T%d:%d:%d" % (motor, off, ms)


def cmd_calibrate(motor, pulse):
    return "C%d:%d" % (motor, pulse)


def cmd_bounds(motor, mn, mx):
    return "B%d:%d:%d" % (motor, mn, mx)


def cmd_mode(motor, mode):
    return "N%d:%s" % (motor, mode)


def cmd_limits(motor, min_deg, max_deg):
    return "L%d:%d:%d" % (motor, min_deg, max_deg)


def cmd_mirror(motor, src):
    return "Y%d:%s" % (motor, src)


def cmd_invert(motor):
    return "J%d" % motor


def cmd_multi(moves):
    parts = []
    for motor, deg in moves:
        parts.append("%d:%s" % (motor, deg))
    return "X" + ",".join(parts)


def cmd_home():
    return "H"


def cmd_home_set(motor, deg):
    return "O%d:%s" % (motor, deg)


def cmd_save():
    return "E"


def cmd_defaults():
    return "D"


def cmd_watchdog(ms):
    return "W%d" % ms


def cmd_keepalive():
    return "K"


def cmd_state():
    return "Q"


def cmd_auto(ms):
    return "A%d" % ms


def cmd_wait(ms):
    return "W%d" % ms


def parse_line(line):
    line = line.strip()
    if not line:
        return None
    try:
        return json.loads(line)
    except (ValueError, TypeError):
        return {"raw": line}


@dataclass
class JointState:
    channel: int = 0
    motor: int = 0
    mode: str = "P"
    deg: float = 0.0
    pulse: int = 0
    target: float = 0.0
    moving: int = 0

    @classmethod
    def from_dict(cls, d):
        c = d.get("c", 0)
        n = d.get("n", None)
        if n is None:
            n = c + 1
        return cls(
            channel=c,
            motor=n,
            mode=d.get("m", "P"),
            deg=d.get("d", 0.0),
            pulse=d.get("p", 0),
            target=d.get("tg", 0.0),
            moving=d.get("mv", 0),
        )


def parse_state(reply):
    if not isinstance(reply, dict):
        return None
    chs = reply.get("ch", [])
    out = []
    for c in chs:
        if isinstance(c, dict):
            out.append(JointState.from_dict(c))
    return out


def motor_of(state):
    if isinstance(state, JointState):
        return state.motor
    if isinstance(state, dict):
        c = state.get("c", 0)
        n = state.get("n", None)
        return c + 1 if n is None else n
    return None


def is_ack(reply, code):
    if not isinstance(reply, dict):
        return False
    return reply.get("ack") == code or reply.get("status") == code or reply.get("event") == code


def reply_kind(reply):
    if not isinstance(reply, dict):
        return "raw"
    if "event" in reply:
        return "event"
    if "ack" in reply:
        return "ack"
    if "status" in reply:
        return "status"
    return "other"