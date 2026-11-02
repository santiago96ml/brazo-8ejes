#!/usr/bin/env python3
"""Discover and connect over SSH to a remote vision node (optional netbook brain).

The remote node is optional: this project runs fully on a single workstation. When a
second machine is used as a vision/HTTP node, configure these environment variables:

    VISION_NODE_USER       SSH user on the remote node      (default: robot)
    VISION_NODE_SSH_KEY    path to the private key          (default: ~/.ssh/id_ed25519)
    VISION_NODE_HOSTS      comma separated candidate hosts  (default: empty)
    VISION_NODE_PORT       SSH port                         (default: 22)

Nothing is hardcoded to a specific machine, network or user.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional

NODE_USER = os.environ.get("VISION_NODE_USER", "robot")
SSH_KEY_PATH = os.path.expanduser(os.environ.get("VISION_NODE_SSH_KEY", "~/.ssh/id_ed25519"))
NODE_PORT = int(os.environ.get("VISION_NODE_PORT", "22"))
KNOWN_HOSTS = [h.strip() for h in os.environ.get("VISION_NODE_HOSTS", "").split(",") if h.strip()]


def _probe_ssh(host: str, timeout: float = 0.6) -> Optional[str]:
    try:
        with socket.create_connection((host, NODE_PORT), timeout=timeout) as sock:
            sock.settimeout(0.6)
            try:
                banner = sock.recv(64).decode("utf-8", "replace").strip()
            except Exception:
                banner = "SSH-open"
            return banner
    except Exception:
        return None


def scan_for_ssh(subnet_prefix: str = "192.168.1") -> List[Dict[str, str]]:
    hosts = list(KNOWN_HOSTS) + [
        f"{subnet_prefix}.{i}"
        for i in range(1, 255)
        if f"{subnet_prefix}.{i}" not in KNOWN_HOSTS
    ]

    def _check(host: str):
        banner = _probe_ssh(host)
        return {"host": host, "banner": banner} if banner is not None else None

    found: List[Dict[str, str]] = []
    with ThreadPoolExecutor(max_workers=48) as ex:
        for res in ex.map(_check, hosts):
            if res is not None:
                found.append(res)
    return found


def verify_node_ssh(host: str, expected_hostname: str = "") -> Dict[str, object]:
    if not os.path.exists(SSH_KEY_PATH):
        return {"host": host, "ok": False, "error": "ssh key not found at %s" % SSH_KEY_PATH}
    cmd = [
        "ssh",
        "-i", SSH_KEY_PATH,
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=4",
        "-o", "StrictHostKeyChecking=accept-new",
        "-p", str(NODE_PORT),
        f"{NODE_USER}@{host}",
        "hostname && uname -r && ls -l /dev/video* 2>/dev/null || true",
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    lines = [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
    hostname = lines[0] if lines else ""
    ok = r.returncode == 0 and (not expected_hostname or hostname == expected_hostname)
    return {
        "host": host,
        "ok": ok,
        "returncode": r.returncode,
        "hostname": hostname,
        "stdout": r.stdout.strip(),
        "stderr": r.stderr.strip(),
    }


def deploy_vision_node(host: str, port: int = 8080) -> Dict[str, object]:
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    vision_file = os.path.join(repo_root, "src", "vision_locator.py")
    scp_cmd = [
        "scp",
        "-i", SSH_KEY_PATH,
        "-o", "BatchMode=yes",
        "-o", "StrictHostKeyChecking=accept-new",
        "-P", str(NODE_PORT),
        vision_file,
        f"{NODE_USER}@{host}:~/vision_locator.py",
    ]
    scp_res = subprocess.run(scp_cmd, capture_output=True, text=True)
    if scp_res.returncode != 0:
        return {"ok": False, "stage": "scp", "stderr": scp_res.stderr.strip()}

    start_cmd = [
        "ssh",
        "-i", SSH_KEY_PATH,
        "-o", "BatchMode=yes",
        "-o", "StrictHostKeyChecking=accept-new",
        "-p", str(NODE_PORT),
        f"{NODE_USER}@{host}",
        f"pkill -f 'vision_locator.py --serve' 2>/dev/null || true; "
        f"nohup python3 ~/vision_locator.py --serve --port {port} >/tmp/vision_locator.log 2>&1 & "
        f"sleep 1 && curl -s http://127.0.0.1:{port}/health",
    ]
    ssh_res = subprocess.run(start_cmd, capture_output=True, text=True)
    return {
        "ok": ssh_res.returncode == 0,
        "host": host,
        "vision_url": f"http://{host}:{port}/pose",
        "health": ssh_res.stdout.strip(),
        "stderr": ssh_res.stderr.strip(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="specific host to test first")
    parser.add_argument("--subnet", default="192.168.1", help="subnet prefix to sweep")
    parser.add_argument("--expected-hostname", default="", help="require this hostname over SSH")
    parser.add_argument("--deploy-vision", action="store_true", help="copy and start vision_locator.py remotely")
    args = parser.parse_args()

    report: Dict[str, object] = {
        "ssh_key_path": SSH_KEY_PATH,
        "ssh_key_exists": os.path.exists(SSH_KEY_PATH),
        "ssh_user": NODE_USER,
        "candidates": [],
        "node_found": None,
    }

    if args.host:
        res = verify_node_ssh(args.host, args.expected_hostname)
        report["direct_check"] = res
        if res.get("ok"):
            report["node_found"] = res
    else:
        candidates = scan_for_ssh(args.subnet)
        report["candidates"] = candidates
        for cand in candidates:
            res = verify_node_ssh(str(cand["host"]), args.expected_hostname)
            if res.get("ok"):
                report["node_found"] = res
                break

    found = report.get("node_found")
    if found and args.deploy_vision:
        report["vision_deploy"] = deploy_vision_node(str(found["host"]))

    print(json.dumps(report, indent=2))
    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
