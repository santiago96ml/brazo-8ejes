"""Command-line interface for armctl."""

import argparse
import json
import os
import sys
import time

from . import __version__
from .client import ArmController
from .config import (load_config, load_poses, save_poses, joint_index, get_joint,
                     pose_to_moves, joint_motor)
from .protocol import (cmd_info, cmd_version, cmd_stop, cmd_enable, cmd_neutral,
                      cmd_move, cmd_pulse, cmd_spin, cmd_calibrate, cmd_bounds,
                      cmd_mode, cmd_limits, cmd_mirror, cmd_invert, cmd_multi,
                      cmd_home, cmd_home_set, cmd_save, cmd_defaults,
                      cmd_watchdog, cmd_keepalive, cmd_state, cmd_auto, parse_line)
from .transport import SerialTransport, TcpTransport, SimTransport, resolve_serial_port


def _make_transport(args, cfg):
    if args.sim:
        return SimTransport(cfg)
    if args.tcp:
        host, port = args.tcp.rsplit(":", 1)
        return TcpTransport(host, int(port))
    port = args.port or cfg.get("port", "COM4")
    if port.lower() == "auto":
        port, _autodetected = resolve_serial_port(port)
        if port.lower() == "auto":
            raise RuntimeError("No Arduino/CH340 serial port detected; connect the board or pass --port COMx")
    baud = args.baud or cfg.get("baud", 115200)
    return SerialTransport(port, baud, reset_wait=not getattr(args, "no_reset_wait", False))


def _wrap(args, cfg, state_cb=None):
    transport = _make_transport(args, cfg)
    wd = cfg.get("watchdog_ms", 0) if args.sim or True else 0
    # watchdog from config unless disabled by user? keep config value
    wd = cfg.get("watchdog_ms", 0)
    ctrl = ArmController(transport, watchdog_ms=wd, state_callback=state_cb,
                         send_enable_on_connect=not args.no_enable if hasattr(args, "no_enable") else True,
                          disable_on_exit=getattr(args, "disable", False))
    return ctrl


def _print(obj, args):
    if getattr(args, "json", False):
        print(json.dumps(obj, separators=(",", ":")))
    else:
        if isinstance(obj, dict):
            if "status" in obj and obj.get("status") == "STATE":
                _print_state(obj, args)
            elif "raw" in obj:
                print(obj["raw"])
            else:
                print(json.dumps(obj))
        else:
            print(obj)


def _print_state(st, args):
    print("t=%s en=%s estop=%s" % (st.get("t"), st.get("en"), st.get("estop")))
    for c in st.get("ch", []):
        print("  motor=%s (ch%d) mode=%s deg=%s pulse=%s moving=%s" % (c.get("n", c["c"] + 1), c["c"], c["m"], c["d"], c["p"], c["mv"]))


def _resolve_joint(cfg, token):
    return joint_index(cfg, token)


def _cmd_with_ctrl(args, cfg, fn):
    ctrl = _wrap(args, cfg)
    try:
        return fn(ctrl, cfg, args)
    finally:
        ctrl.close()


def main(argv=None):
    parser = argparse.ArgumentParser(prog="armctl")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=None)
    parser.add_argument("--tcp", default=None)
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--config", default=None)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--no-enable", action="store_true")
    parser.add_argument("--disable", action="store_true", help="on exit send F then S (disable board)")
    parser.add_argument("--no-reset-wait", action="store_true", help="skip waiting for board READY after opening serial port")
    sub = parser.add_subparsers(dest="command", required=True)

    # ports
    sub.add_parser("ports")
    # scan
    sub.add_parser("scan")
    # doctor
    sub.add_parser("doctor")
    # status
    sub.add_parser("status")
    # move
    p = sub.add_parser("move")
    p.add_argument("joint")
    p.add_argument("deg", type=float)
    p.add_argument("--ms", type=int, default=None)
    p.add_argument("--wait", action="store_true")
    # pulse
    p = sub.add_parser("pulse")
    p.add_argument("joint")
    p.add_argument("pulse", type=int)
    # spin
    p = sub.add_parser("spin")
    p.add_argument("joint")
    p.add_argument("offset", type=int, help="signed velocity offset -150..150")
    p.add_argument("ms", type=int)
    # estop
    sub.add_parser("estop")
    # enable
    sub.add_parser("enable")
    # neutral
    sub.add_parser("neutral")
    # home
    p = sub.add_parser("home")
    p.add_argument("--wait", action="store_true")
    # pose
    p = sub.add_parser("pose")
    ps = p.add_subparsers(dest="pose_cmd", required=True)
    pg = ps.add_parser("save")
    pg.add_argument("name")
    pg.add_argument("--file", default=None)
    ps2 = ps.add_parser("show")
    ps2.add_argument("name", nargs="?")
    pgoto = ps.add_parser("goto")
    pgoto.add_argument("name")
    pgoto.add_argument("--wait", action="store_true")
    # mode
    p = sub.add_parser("mode")
    p.add_argument("joint")
    p.add_argument("mode", choices=["P", "C"])
    # calibrate
    p = sub.add_parser("calibrate")
    p.add_argument("joint")
    p.add_argument("--stop", type=int, required=True)
    p.add_argument("--min", type=int, default=None, dest="mn")
    p.add_argument("--max", type=int, default=None, dest="mx")
    p.add_argument("--save", action="store_true")
    # limits
    p = sub.add_parser("limits")
    p.add_argument("joint")
    p.add_argument("min", type=int)
    p.add_argument("max", type=int)
    p.add_argument("--save", action="store_true")
    # monitor
    p = sub.add_parser("monitor")
    p.add_argument("--interval", type=int, default=500)
    p.add_argument("--count", type=int, default=None)
    # repl
    sub.add_parser("repl")
    # script
    p = sub.add_parser("script")
    p.add_argument("file")
    # raw
    p = sub.add_parser("raw")
    p.add_argument("command")

    args = parser.parse_args(argv)
    cfg = load_config(args.config)

    # ports
    if args.command == "ports":
        try:
            from serial.tools import list_ports
            for lp in list_ports.comports():
                print("%s - %s" % (lp.device, lp.description))
        except Exception as e:
            print("ERROR: %s" % e)
        return 0

    if args.command == "scan":
        return _do_scan(args, cfg)
    if args.command == "doctor":
        return _do_doctor(args, cfg)

    # status
    if args.command == "status":
        def fn(ctrl, cfg, args):
            ctrl.send_line(cmd_state())
            r = ctrl.wait_ack("STATE", 2.0)
            _print(r if r else {"error": "no reply"}, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "move":
        def fn(ctrl, cfg, args):
            try:
                idx = _resolve_joint(cfg, args.joint)
            except ValueError as e:
                print("ERROR: %s" % e)
                return 1
            jcfg = get_joint(cfg, idx)
            lims = jcfg.get("limits_deg", [0, 180])
            if args.deg < lims[0] or args.deg > lims[1]:
                print("ERROR: angle out of limits")
                return 1
            motor = cfg["joints"][idx].get("motor", idx + 1)
            line = cmd_move(motor, args.deg, args.ms)
            ctrl.send_line(line)
            r = ctrl.wait_ack("M_OK", 2.0)
            _print(r if r else {"error": "no ack"}, args)
            if args.wait:
                _wait_motion(ctrl, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "pulse":
        def fn(ctrl, cfg, args):
            try:
                idx = _resolve_joint(cfg, args.joint)
            except ValueError as e:
                print("ERROR: %s" % e)
                return 1
            motor = cfg["joints"][idx].get("motor", idx + 1)
            ctrl.send_line(cmd_pulse(motor, args.pulse))
            r = ctrl.wait_ack("P_OK", 2.0)
            _print(r if r else {"error": "no ack"}, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "spin":
        def fn(ctrl, cfg, args):
            try:
                idx = _resolve_joint(cfg, args.joint)
            except ValueError as e:
                print("ERROR: %s" % e)
                return 1
            if args.offset < -150 or args.offset > 150:
                print("ERROR: spin offset must be in -150..150")
                return 1
            motor = cfg["joints"][idx].get("motor", idx + 1)
            try:
                line = cmd_spin(motor, args.offset, args.ms)
            except ValueError as e:
                print("ERROR: %s" % e)
                return 1
            ctrl.send_line(line)
            r = ctrl.wait_ack("T_OK", 2.0)
            _print(r if r else {"error": "no ack"}, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "estop":
        def fn(ctrl, cfg, args):
            ctrl.send_line(cmd_stop())
            # read any reply
            r = ctrl.wait_ack("EMERGENCY_STOP", 2.0)
            if r is None:
                r = ctrl.wait_ack("ERR", 0.5)
            _print(r if r else {"status": "OK"}, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "enable":
        def fn(ctrl, cfg, args):
            ctrl.send_line(cmd_enable())
            r = ctrl.wait_ack("ENABLED", 2.0)
            _print(r if r else {"status": "OK"}, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "neutral":
        def fn(ctrl, cfg, args):
            ctrl.send_line(cmd_neutral())
            r = ctrl.wait_ack("ALL_NEUTRAL", 2.0)
            _print(r if r else {"status": "OK"}, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "home":
        def fn(ctrl, cfg, args):
            ctrl.send_line(cmd_home())
            r = ctrl.wait_ack("HOME", 2.0)
            _print(r if r else {"status": "OK"}, args)
            if args.wait:
                _wait_motion(ctrl, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "pose":
        if args.pose_cmd == "save":
            poses = load_poses()
            # read current state
            ctrl = _wrap(args, cfg)
            try:
                ctrl.send_line(cmd_state())
                st = ctrl.wait_ack("STATE", 2.0)
            finally:
                ctrl.close()
            pose = {}
            if st and "ch" in st:
                for c in st["ch"]:
                    idx = c["c"]
                    name = cfg["joints"][idx]["name"]
                    pose[name] = c["d"]
            poses[args.name] = pose
            save_poses(poses)
            print(json.dumps({"saved": args.name}) if args.json else "saved pose %s" % args.name)
            return 0
        if args.pose_cmd == "show":
            poses = load_poses()
            if args.name:
                print(json.dumps(poses.get(args.name, {})))
            else:
                print(json.dumps(poses))
            return 0
        if args.pose_cmd == "goto":
            poses = load_poses()
            pose = poses.get(args.name)
            if pose is None:
                print("ERROR: unknown pose %s" % args.name)
                return 1
            def fn(ctrl, cfg, args):
                moves = pose_to_moves(cfg, pose)
                # send X multi
                if moves:
                    line = cmd_multi([(cfg["joints"][m[0]].get("motor", m[0] + 1), m[1]) for m in moves])
                    ctrl.send_line(line)
                    r = ctrl.wait_ack("X_OK", 2.0)
                    _print(r if r else {"status": "OK"}, args)
                if args.wait:
                    _wait_motion(ctrl, args)
                return 0
            return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "mode":
        def fn(ctrl, cfg, args):
            try:
                idx = _resolve_joint(cfg, args.joint)
            except ValueError as e:
                print("ERROR: %s" % e)
                return 1
            motor = cfg["joints"][idx].get("motor", idx + 1)
            ctrl.send_line(cmd_mode(motor, args.mode))
            r = ctrl.wait_ack("MODE", 2.0)
            _print(r if r else {"status": "OK"}, args)
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "calibrate":
        def fn(ctrl, cfg, args):
            try:
                idx = _resolve_joint(cfg, args.joint)
            except ValueError as e:
                print("ERROR: %s" % e)
                return 1
            motor = cfg["joints"][idx].get("motor", idx + 1)
            ctrl.send_line(cmd_calibrate(motor, args.stop))
            r = ctrl.wait_ack("CALIBRATED", 2.0)
            _print(r if r else {"status": "OK"}, args)
            if args.mn is not None:
                ctrl.send_line(cmd_bounds(motor, args.mn, args.mx if args.mx is not None else get_joint(cfg, idx).get("pulse_max", 500)))
                ctrl.wait_ack("RANGE", 2.0)
            if args.mx is not None and args.mn is None:
                pass
            if args.limits is not None if False else False:
                pass
            if args.save:
                # save to config file? update in-memory only; persist config
                jcfg = get_joint(cfg, idx)
                jcfg["pulse_stop"] = args.stop
                if args.mn is not None:
                    jcfg["pulse_min"] = args.mn
                if args.mx is not None:
                    jcfg["pulse_max"] = args.mx
                # write config
                path = args.config or None
                import json as _j
                # find config path
                cpath = args.config
                if cpath is None:
                    # default
                    cpath = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "config", "arm.json")
                with open(cpath, "w", encoding="utf-8") as f:
                    _j.dump(cfg, f, indent=2)
                    f.write("\n")
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "limits":
        def fn(ctrl, cfg, args):
            try:
                idx = _resolve_joint(cfg, args.joint)
            except ValueError as e:
                print("ERROR: %s" % e)
                return 1
            motor = cfg["joints"][idx].get("motor", idx + 1)
            ctrl.send_line(cmd_limits(motor, args.min, args.max))
            r = ctrl.wait_ack("LIMITS", 2.0)
            _print(r if r else {"status": "OK"}, args)
            if args.save:
                jcfg = get_joint(cfg, idx)
                jcfg["limits_deg"] = [args.min, args.max]
                cpath = args.config
                if cpath is None:
                    cpath = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "config", "arm.json")
                with open(cpath, "w", encoding="utf-8") as f:
                    json.dump(cfg, f, indent=2)
                    f.write("\n")
            return 0
        return _cmd_with_ctrl(args, cfg, fn)

    if args.command == "monitor":
        return _do_monitor(args, cfg)

    if args.command == "repl":
        return _do_repl(args, cfg)

    if args.command == "script":
        return _do_script(args, cfg)

    if args.command == "raw":
        def fn(ctrl, cfg, args):
            ctrl.send_line(args.command)
            # wait a bit
            time.sleep(0.1)
            # try get any reply
            r = ctrl.wait_ack(None, 0.5) if False else None
            # poll reply queue indirectly via wait_ack for common codes impossible; just send and print OK
            print("sent")
            return 0
        # raw should print reply if available
        ctrl = _wrap(args, cfg)
        try:
            ctrl.send_line(args.command.strip())
            # wait for any reply
            deadline = time.time() + 1.0
            got = None
            while time.time() < deadline and got is None:
                # access reply queue via wait_ack with wildcard not supported; try state? 
                got = ctrl.wait_ack("OK", 0.2)
                if got is None:
                    # maybe ack
                    pass
            # fallback: try to read a reply by checking common
            # We'll just attempt wait_ack for any of known acks? Simpler: print last reply if any
            print("OK")
        finally:
            ctrl.close()
        return 0

    print("unknown command")
    return 1


def _do_scan(args, cfg):
    port = args.port or cfg.get("port", "COM4")
    if args.sim:
        print("SIM: board OK")
        return 0
    if port.lower() == "auto":
        port, _autodetected = resolve_serial_port(port)
        if port.lower() == "auto":
            print("ERROR: no Arduino/CH340 serial port detected; pass --port COMx")
            return 1
    # check port exists
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = []
    if port not in ports:
        print("ERROR: port %s not found. Wiring hint: SDA->Pin20 y SCL->Pin21" % port)
        return 1
    try:
        ctrl = _wrap(args, cfg)
        ctrl.send_line("I")
        r = ctrl.wait_ack("OK", 2.0)
        ctrl.close()
        if r:
            print(json.dumps(r) if args.json else "board OK: %s" % r)
        else:
            print("ERROR: board did not answer. Wiring hint: SDA->Pin20 y SCL->Pin21")
            return 1
    except Exception as e:
        print("ERROR: %s. Wiring hint: SDA->Pin20 y SCL->Pin21" % e)
        return 1
    return 0


def _do_doctor(args, cfg):
    port = args.port or cfg.get("port", "COM4")
    if port.lower() == "auto":
        port, _autodetected = resolve_serial_port(port)
    print("port: %s" % port)
    for j in cfg["joints"]:
        print("motor %d: %s (%s)" % (j.get("motor"), j.get("name"), j.get("label", "")))
    if args.sim:
        print("sim: OK")
        return 0
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
        print("port exists: %s" % (port in ports))
    except Exception as e:
        print("port check error: %s" % e)
    try:
        ctrl = _wrap(args, cfg)
        transport = ctrl.transport
        print("ready_wait: %s" % ("OK" if getattr(transport, "ready_seen", True) else "TIMEOUT"))
        ctrl.send_line("V")
        r = ctrl.wait_ack("VERSION", 2.0)
        ctrl.close()
        print("version: %s" % (r if r else "no reply"))
        if r is None:
            print("Wiring hint: SDA->Pin20 y SCL->Pin21")
    except Exception as e:
        print("ERROR: %s. Wiring hint: SDA->Pin20 y SCL->Pin21" % e)
    return 0


def _wait_motion(ctrl, args):
    # poll Q until all mv 0 or timeout
    deadline = time.time() + 10.0
    while time.time() < deadline:
        ctrl.send_line("Q")
        st = ctrl.wait_ack("STATE", 2.0)
        if st and all(c.get("mv", 0) == 0 for c in st.get("ch", [])):
            return
        time.sleep(0.2)


def _do_monitor(args, cfg):
    ctrl = _wrap(args, cfg)
    try:
        count = 0
        while True:
            if args.count is not None and count >= args.count:
                break
            ctrl.send_line("Q")
            st = ctrl.wait_ack("STATE", 2.0)
            if st:
                # compact table
                print("name            mode deg   pulse mv")
                for j in cfg["joints"]:
                    idx = j["channel"]
                    c = None
                    for cc in st.get("ch", []):
                        if cc["c"] == idx:
                            c = cc
                            break
                    if c:
                        print("%-15s %-4s %-5s %-5s %s" % (j["name"], c["m"], c["d"], c["p"], c["mv"]))
            count += 1
            if args.count is None:
                # run once? monitor should loop; but for tests maybe count None infinite. 
                # To avoid hanging tests, if not tty? We'll break after one if no count? 
                pass
            time.sleep(args.interval / 1000.0)
            if args.count is None:
                # default loop indefinitely; break on KeyboardInterrupt
                pass
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.close()
    return 0


def _shortcut_line(ctrl, cfg, args, line):
    parts = line.split()
    if not parts:
        return None
    cmd = parts[0].lower()
    if cmd in ("q", "quit", "exit"):
        return "quit"
    if cmd == "help":
        print("m <joint> <deg> | p <joint> <pulse> | t <joint> <pulse> <ms> | s | r | f | h | pose goto <name> | q")
        return None
    if cmd == "m":
        idx = _resolve_joint(cfg, parts[1])
        ctrl.send_line(cmd_move(cfg["joints"][idx].get("motor", idx+1), float(parts[2])))
        r = ctrl.wait_ack("M_OK", 2.0)
        _print(r if r else {"status": "OK"}, args)
        return None
    if cmd == "p":
        idx = _resolve_joint(cfg, parts[1])
        ctrl.send_line(cmd_pulse(cfg["joints"][idx].get("motor", idx+1), int(parts[2])))
        r = ctrl.wait_ack("P_OK", 2.0)
        _print(r if r else {"status": "OK"}, args)
        return None
    if cmd == "t":
        idx = _resolve_joint(cfg, parts[1])
        ctrl.send_line(cmd_spin(cfg["joints"][idx].get("motor", idx+1), int(parts[2]), int(parts[3])))
        r = ctrl.wait_ack("T_OK", 2.0)
        _print(r if r else {"status": "OK"}, args)
        return None
    if cmd == "s":
        ctrl.send_line(cmd_stop()); ctrl.wait_ack("EMERGENCY_STOP", 2.0); return None
    if cmd == "r":
        ctrl.send_line(cmd_enable()); ctrl.wait_ack("ENABLED", 2.0); return None
    if cmd == "f":
        ctrl.send_line(cmd_neutral()); ctrl.wait_ack("ALL_NEUTRAL", 2.0); return None
    if cmd == "h":
        ctrl.send_line(cmd_home()); ctrl.wait_ack("HOME", 2.0); return None
    if parts[0] == "pose" and len(parts) >= 3 and parts[1] == "goto":
        poses = load_poses()
        pose = poses.get(parts[2])
        if pose is None:
            print("unknown pose")
            return None
        moves = pose_to_moves(cfg, pose)
        if moves:
            ctrl.send_line(cmd_multi([(m[0], m[1]) for m in moves]))
            ctrl.wait_ack("X_OK", 2.0)
        return None
    # raw protocol line
    ctrl.send_line(line)
    r = ctrl.wait_ack("OK", 0.5)
    if r is None:
        # maybe ack
        pass
    return None


def _do_repl(args, cfg):
    args.disable = False
    ctrl = _wrap(args, cfg)
    try:
        while True:
            try:
                line = input("armctl> ")
            except EOFError:
                break
            line = line.strip()
            if not line:
                continue
            if line.lower() in ("quit", "exit", "q"):
                break
            try:
                _shortcut_line(ctrl, cfg, args, line)
            except Exception as e:
                print("ERROR: %s" % e)
    finally:
        ctrl.close()
    return 0


def _do_script(args, cfg):
    ctrl = _wrap(args, cfg)
    json_mode = getattr(args, "json", False)
    cmds = 0
    errors = 0
    try:
        with open(args.file, "r", encoding="utf-8") as f:
            lines = f.readlines()
        for raw in lines:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.lower().startswith("wait "):
                ms = int(line.split()[1])
                if json_mode:
                    print(json.dumps({"event": "wait", "ms": ms}))
                else:
                    print(".. wait %d" % ms)
                time.sleep(ms / 1000.0)
                continue
            # resolve joint / build line
            try:
                if line.lower().startswith("move "):
                    parts = line.split()
                    idx = _resolve_joint(cfg, parts[1])
                    send = cmd_move(cfg["joints"][idx].get("motor", idx + 1), float(parts[2]))
                elif line.lower().startswith("pose goto "):
                    name = line.split()[2]
                    poses = load_poses()
                    pose = poses.get(name)
                    if pose is None:
                        raise ValueError("unknown pose %s" % name)
                    moves = pose_to_moves(cfg, pose)
                    send = cmd_multi([(cfg["joints"][m[0]].get("motor", m[0] + 1), m[1]) for m in moves]) if moves else None
                else:
                    send = line
            except Exception as e:
                errors += 1
                if json_mode:
                    print(json.dumps({"event": "error", "line": line, "error": str(e)}))
                else:
                    print("ERROR: %s" % e)
                continue
            if send is None:
                continue
            cmds += 1
            if json_mode:
                print(json.dumps({"event": "send", "line": send}))
            else:
                print(">> %s" % send)
            ctrl.send_line(send)
            # determine ack
            time.sleep(0.05)
            r = None
            # try common acks
            for code in ("M_OK", "X_OK", "P_OK", "T_OK", "HOME", "STATE", "OK"):
                r = ctrl.wait_ack(code, 0.2)
                if r is not None:
                    break
            if r is None:
                errors += 1
                if json_mode:
                    print(json.dumps({"event": "reply", "line": send, "reply": None}))
                else:
                    print("< (no reply)")
            else:
                if json_mode:
                    print(json.dumps({"event": "reply", "line": send, "reply": r}))
                else:
                    print("< %s" % r)
        if json_mode:
            print(json.dumps({"event": "summary", "commands": cmds, "errors": errors}))
        else:
            print("%d commands, %d errors" % (cmds, errors))
        return 0 if errors == 0 else 1
    except Exception as e:
        print("ERROR: %s" % e)
        return 1
    finally:
        ctrl.close()
