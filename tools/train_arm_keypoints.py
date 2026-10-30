"""Training helpers for the 6-keypoint arm pose model (YOLO11-Pose).

Provides :func:`build_dataset_yaml`, :func:`check_environment`, :func:`train`
and :func:`export_onnx`. Importable and usable as a CLI.
"""
from __future__ import annotations

import os
import sys
from typing import Any, Dict, Optional

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

PIP_HINT = (
    "python -m pip install --upgrade ultralytics\n"
    "python -m pip install --index-url https://download.pytorch.org/whl/cu124 torch torchvision"
)

KEYPOINT_NAMES = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_tip")


def check_environment() -> dict:
    """Return installed versions and a ready flag. Never raises."""
    info: Dict[str, Any] = {
        "torch": None,
        "cuda": False,
        "ultralytics": None,
        "device": "cpu",
        "ready": False,
        "hint": PIP_HINT,
    }
    try:
        import torch  # type: ignore

        info["torch"] = getattr(torch, "__version__", None)
        info["cuda"] = bool(torch.cuda.is_available())
        if info["cuda"]:
            info["device"] = "0"
    except Exception as e:  # pragma: no cover - defensive
        info["torch"] = "error: %s" % e
    try:
        import ultralytics  # type: ignore

        info["ultralytics"] = getattr(ultralytics, "__version__", "installed")
    except Exception:
        info["ultralytics"] = None
    info["ready"] = bool(info["cuda"]) and info["ultralytics"] is not None
    if not info["ready"]:
        info["hint"] = PIP_HINT
    return info


def build_dataset_yaml(dataset_dir: str, out_path: Optional[str] = None) -> str:
    """Write a YOLO pose dataset YAML and return its path."""
    dataset_dir = os.path.abspath(dataset_dir)
    images_dir = os.path.join(dataset_dir, "images")
    labels_dir = os.path.join(dataset_dir, "labels")
    if not os.path.isdir(images_dir):
        raise ValueError("dataset/images directory not found: %s" % images_dir)
    if not os.path.isdir(labels_dir):
        raise ValueError("dataset/labels directory not found: %s" % labels_dir)

    # Validate at least one label with enough tokens (class + bbox + 6*3).
    valid = False
    for name in os.listdir(labels_dir):
        if not name.endswith(".txt"):
            continue
        path = os.path.join(labels_dir, name)
        try:
            with open(path, "r", encoding="utf-8") as f:
                for line in f:
                    toks = line.split()
                    if len(toks) >= 22:
                        valid = True
                        break
        except Exception:
            continue
        if valid:
            break
    if not valid:
        raise ValueError(
            "no valid label found in %s (expected >= 22 tokens: "
            "class + bbox + 6 keypoints)" % labels_dir
        )

    if out_path is None:
        out_path = os.path.join(dataset_dir, "dataset.yaml")
    out_path = os.path.abspath(out_path)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    lines = []
    lines.append("# Keypoints in order: base, shoulder, elbow, wrist_pitch, wrist_roll, gripper_tip")
    lines.append("path: %s" % dataset_dir)
    lines.append("train: images")
    lines.append("val: images")
    lines.append("kpt_shape: [6, 3]")
    lines.append("names: {0: arm}")
    lines.append("flip_idx: [0, 1, 2, 3, 4, 5]")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return out_path


def train(
    data_yaml: str,
    model: str = "yolo11n-pose.pt",
    epochs: int = 100,
    imgsz: int = 640,
    device: str = "0",
    project: str = "runs/arm_pose",
) -> dict:
    env = check_environment()
    # lazy import ultralytics
    try:
        import ultralytics  # type: ignore  # noqa: F401
    except Exception:
        return {"ok": False, "error": "ultralytics not installed", "hint": PIP_HINT}

    # CUDA fallback
    device_used = device
    try:
        import torch  # type: ignore

        if device == "0" and not torch.cuda.is_available():
            device_used = "cpu"
            device = "cpu"
    except Exception:
        if device == "0":
            device_used = "cpu"
            device = "cpu"

    try:
        from ultralytics import YOLO  # type: ignore

        m = YOLO(model)
        results = m.train(data=data_yaml, epochs=int(epochs), imgsz=int(imgsz),
                          device=device, project=project)
        save_dir = getattr(results, "save_dir", None)
        if save_dir is None:
            save_dir = project
        save_dir = str(save_dir)
        weights = os.path.join(save_dir, "weights", "best.pt")
        if not os.path.exists(weights):
            # fall back to last.pt
            alt = os.path.join(save_dir, "weights", "last.pt")
            if os.path.exists(alt):
                weights = alt
        return {
            "ok": True,
            "weights": weights,
            "save_dir": save_dir,
            "device_used": device_used,
        }
    except Exception as e:
        return {"ok": False, "error": str(e), "hint": PIP_HINT, "device_used": device_used}


def export_onnx(weights: str, out_path: str = "models/arm_keypoints.onnx") -> dict:
    try:
        import ultralytics  # type: ignore  # noqa: F401
    except Exception:
        return {"ok": False, "error": "ultralytics not installed", "hint": PIP_HINT}
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    try:
        from ultralytics import YOLO  # type: ignore

        model = YOLO(weights)
        exported = model.export(format="onnx", imgsz=640, simplify=True)
        # move/copy to out_path
        if exported and os.path.exists(str(exported)) and str(exported) != out_path:
            try:
                import shutil

                shutil.copy2(str(exported), out_path)
            except Exception:
                # try rename
                try:
                    os.replace(str(exported), out_path)
                except Exception:
                    pass
        if not os.path.exists(out_path):
            # maybe exported path is out_path already
            if exported and os.path.exists(str(exported)):
                out_path = str(exported)
        return {"ok": os.path.exists(out_path), "path": out_path, "exported": str(exported) if exported else None}
    except Exception as e:
        return {"ok": False, "error": str(e), "hint": PIP_HINT}


def _main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(prog="train_arm_keypoints")
    parser.add_argument("--dataset", default="dataset")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--model", default="yolo11n-pose.pt")
    parser.add_argument("--device", default="0")
    parser.add_argument("--export-onnx", action="store_true")
    parser.add_argument("--weights", default=None)
    parser.add_argument("--out", default="models/arm_keypoints.onnx")
    args = parser.parse_args(argv)

    env = check_environment()
    print("environment:", env)
    if not env["ready"]:
        print(env["hint"])
        # still allow yaml build
    yaml_path = None
    try:
        yaml_path = build_dataset_yaml(args.dataset)
        print("dataset yaml:", yaml_path)
    except Exception as e:
        print("build_dataset_yaml failed:", e)

    if args.export_onnx:
        w = args.weights or "yolo11n-pose.pt"
        res = export_onnx(w, args.out)
        print("export:", res)
        return 0

    if not env["ready"]:
        print("training skipped: environment not ready")
        return 0
    if yaml_path is None:
        print("no dataset yaml; abort")
        return 1
    res = train(yaml_path, model=args.model, epochs=args.epochs, imgsz=args.imgsz,
                device=args.device)
    print("train:", res)
    if res.get("ok") and args.export_onnx:
        export_onnx(res.get("weights"), args.out)
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(_main())
