#!/usr/bin/env python3
"""Auto-discover and connect via SSH to the Netbook Conectar Igualdad (Ubuntu Server).

Recovers the exact SSH credentials from the `tanque` project:
- User: `robot`
- Private key: `C:\\Users\\merca\\.ssh\\tanque_ed25519`
- Known MAC (Ethernet RTL8105E): `B8-AE-ED-10-65-59`
- Historical IPs: `192.168.1.47` (Ethernet Personal-EAF), `192.168.1.65` (WiFi RobticaCatamarca)
- Hostkey fingerprint: `SHA256:7bwTpuHJylmtnnFiJkKyAwteKaRUpuDXHtZLgQ2b7wE`
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional

SSH_KEY_PATH = os.path.expanduser(r"~\.ssh\tanque_ed25519")
SSH_USER = "robot"
KNOWN_IPS = ["192.168.1.47", "192.168.1.65"]
NETBOOK_ETH_MAC = "b8-ae-ed-10-65-59"


def _probe_port_22(ip: str, timeout: float = 0.6) -> Optional[str]:
    try:
        with socket.create_connection((ip, 22), timeout=timeout) as sock:
            sock.settimeout(0.6)
            try:
                banner = sock.recv(64).decode("utf-8", "replace").strip()
            except Exception:
                banner = "SSH-open"
            return banner
    except Exception:
        return None


def scan_lan_for_ssh(subnet_prefix: str = "192.168.1") -> List[Dict[str, str]]:
    ips = list(KNOWN_IPS) + [
        f"{subnet_prefix}.{i}"
        for i in range(1, 255)
        if f"{subnet_prefix}.{i}" not in KNOWN_IPS
    ]
    found: List[Dict[str, str]] = []

    def _check(ip: str):
        banner = _probe_port_22(ip)
        if banner is not None:
            return {"ip": ip, "banner": banner}
        return None

    with ThreadPoolExecutor(max_workers=48) as ex:
        for res in ex.map(_check, ips):
            if res is not None:
                found.append(res)
    return found


def verify_netbook_ssh(ip: str, key_path: str = SSH_KEY_PATH) -> Dict[str, object]:
    cmd = [
        "ssh",
        "-i",
        key_path,
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=4",
        "-o",
        "StrictHostKeyChecking=no",
        f"{SSH_USER}@{ip}",
        "hostname && uname -r && ip -br a && ls -l /dev/video* 2>/dev/null || true",
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    lines = [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
    hostname = lines[0] if lines else ""
    return {
        "ip": ip,
        "ok": r.returncode == 0 and hostname == "robot",
        "returncode": r.returncode,
        "hostname": hostname,
        "stdout": r.stdout.strip(),
        "stderr": r.stderr.strip(),
    }


def deploy_vision_node(ip: str, port: int = 8080, key_path: str = SSH_KEY_PATH) -> Dict[str, object]:
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    vision_file = os.path.join(repo_root, "src", "vision_locator.py")
    scp_cmd = [
        "scp",
        "-i",
        key_path,
        "-o",
        "BatchMode=yes",
        "-o",
        "StrictHostKeyChecking=no",
        vision_file,
        f"{SSH_USER}@{ip}:~/vision_locator.py",
    ]
    scp_res = subprocess.run(scp_cmd, capture_output=True, text=True)
    if scp_res.returncode != 0:
        return {"ok": False, "stage": "scp", "stderr": scp_res.stderr.strip()}

    start_cmd = [
        "ssh",
        "-i",
        key_path,
        "-o",
        "BatchMode=yes",
        "-o",
        "StrictHostKeyChecking=no",
        f"{SSH_USER}@{ip}",
        f"pkill -f 'vision_locator.py --serve' 2>/dev/null || true; "
        f"nohup python3 ~/vision_locator.py --serve --port {port} >/tmp/vision_locator.log 2>&1 & "
        f"sleep 1 && curl -s http://127.0.0.1:{port}/health",
    ]
    ssh_res = subprocess.run(start_cmd, capture_output=True, text=True)
    return {
        "ok": ssh_res.returncode == 0,
        "ip": ip,
        "vision_url": f"http://{ip}:{port}/pose",
        "health": ssh_res.stdout.strip(),
        "stderr": ssh_res.stderr.strip(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", help="Specific Netbook IP to test first")
    parser.add_argument("--subnet", default="192.168.1", help="Subnet prefix to sweep")
    parser.add_argument("--deploy-vision", action="store_true", help="Copy and start vision_locator.py on the Netbook")
    args = parser.parse_args()

    report: Dict[str, object] = {
        "ssh_key_path": SSH_KEY_PATH,
        "ssh_key_exists": os.path.exists(SSH_KEY_PATH),
        "ssh_user": SSH_USER,
        "netbook_eth_mac": NETBOOK_ETH_MAC,
        "candidates_port_22": [],
        "netbook_found": None,
    }

    if args.ip:
        res = verify_netbook_ssh(args.ip)
        report["direct_check"] = res
        if res["ok"]:
            report["netbook_found"] = res
    else:
        candidates = scan_lan_for_ssh(args.subnet)
        report["candidates_port_22"] = candidates
        for cand in candidates:
            res = verify_netbook_ssh(str(cand["ip"]))
            if res["ok"]:
                report["netbook_found"] = res
                break

    if report["netbook_found"] and args.deploy_vision:
        netbook_ip = str(report["netbook_found"]["ip"])  # type: ignore[index]
        report["vision_deploy"] = deploy_vision_node(netbook_ip)

    print(json.dumps(report, indent=2))
    return 0 if report["netbook_found"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
