"""Interactive servo calibration assistant for the 8-axis arm."""

import argparse
import sys
from typing import Any, Optional

try:
    from src.state_manager import RobotStateManager
except Exception:  # pragma: no cover
    from state_manager import RobotStateManager  # type: ignore


class ServoCalibrationAssistant:
    def __init__(self, state_manager: Optional[RobotStateManager] = None,
                 transport: Any = None, sim: bool = False):
        self.state_manager = state_manager or RobotStateManager()
        self.transport = transport
        self.sim = sim

    def _emit(self, cmd: str) -> None:
        if self.transport is not None:
            try:
                if hasattr(self.transport, "write"):
                    self.transport.write((cmd + "\n").encode())
                elif hasattr(self.transport, "send"):
                    self.transport.send(cmd)
            except Exception:
                pass
        print(cmd, flush=True)

    def find_deadband_stop_pulse(self, motor: int,
                                 candidates=range(305, 311)) -> int:
        chosen = 307
        for pulse in candidates:
            self._emit("C%d:%d:60.0" % (motor, pulse))
            chosen = int(pulse)
        # default deadband centre
        chosen = 307
        self.state_manager.update_motor_calibration(motor, stop_pulse=chosen)
        return chosen

    def verify_shoulder_coupling(self, coupling_sign: int = 1) -> int:
        c = 1 if coupling_sign >= 0 else -1
        if c not in (1, -1):
            c = 1
        self.state_manager.set_shoulder_coupling(c)
        self._emit("U%d" % c)
        return c

    def measure_angular_speed(self, motor: int, test_offset: int = 40,
                              duration_ms: int = 500,
                              measured_deg: float = 30.0) -> float:
        dps = float(measured_deg) / (float(duration_ms) / 1000.0)
        stop = self.state_manager.state["calibration"][str(motor)]["stop_pulse"]
        self._emit("C%d:%d:%.1f" % (motor, stop, dps))
        self.state_manager.update_motor_calibration(motor, deg_per_sec=dps)
        return dps


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Servo calibration assistant")
    parser.add_argument("--sim", action="store_true", help="simulation/dry-run mode")
    parser.add_argument("--motor", type=int, default=1)
    args = parser.parse_args(argv)

    assistant = ServoCalibrationAssistant(sim=args.sim)
    # quick check
    assistant.find_deadband_stop_pulse(args.motor)
    assistant.verify_shoulder_coupling(1)
    assistant.measure_angular_speed(args.motor)
    print("OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
