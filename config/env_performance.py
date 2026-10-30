import os


def apply_performance_env() -> dict[str, str]:
    env_vars = {
        "OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS": "0",
        "OPENCV_VIDEOIO_PRIORITY_DSHOW": "100",
        "OMP_NUM_THREADS": "4",
        "OPENBLAS_NUM_THREADS": "4",
        "CUDA_MODULE_LOADING": "LAZY",
        "YOLO_VERBOSE": "False",
    }
    result = {}
    for key, value in env_vars.items():
        if os.environ.get(key) is None:
            os.environ[key] = value
            result[key] = value
        else:
            result[key] = os.environ[key]
    return result


def configure_opencv() -> None:
    try:
        import cv2

        if hasattr(cv2, "setNumThreads"):
            cv2.setNumThreads(4)
        if hasattr(cv2, "setLogLevel"):
            cv2.setLogLevel(0)
    except Exception:
        pass


# Apply env vars before NumPy/OpenCV are imported when imported first
apply_performance_env()
