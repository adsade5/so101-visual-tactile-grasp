#!/usr/bin/env python
"""OpenCV build and optional short camera probe.

Camera probing is opt-in. When enabled it reads at most a few frames,
does not show GUI windows, and releases every capture immediately.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path


def backend_name(cv2, backend: int | None) -> str:
    if backend is None:
        return "DEFAULT"
    if backend == cv2.CAP_DSHOW:
        return "DSHOW"
    if backend == cv2.CAP_MSMF:
        return "MSMF"
    return str(backend)


def black_frame_ratio(frame) -> float:
    try:
        import numpy as np

        gray_mean = float(np.mean(frame))
        return 1.0 if gray_mean < 1.0 else 0.0
    except Exception:
        return -1.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default="audit_results")
    parser.add_argument("--test-camera", action="store_true")
    parser.add_argument("--preferred-index", type=int, default=1)
    parser.add_argument("--max-index", type=int, default=5)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--per-open-timeout-s", type=float, default=4.0)
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    data = {
        "python_executable": sys.executable,
        "python_version": sys.version,
        "opencv_import": {"ok": False},
        "camera_test_enabled": args.test_camera,
        "camera_results": [],
    }

    try:
        import cv2

        build = cv2.getBuildInformation()
        data["opencv_import"] = {
            "ok": True,
            "version": cv2.__version__,
            "path": getattr(cv2, "__file__", None),
            "has_aruco": hasattr(cv2, "aruco"),
            "build_has_msmf": "Media Foundation" in build or "MSMF" in build,
            "build_has_dshow": "DirectShow" in build,
            "build_has_gui": any(token in build for token in ("GUI:", "Win32 UI", "QT:", "GTK")),
            "build_information": build,
        }
    except Exception:
        data["opencv_import"]["error"] = traceback.format_exc()
        text = json.dumps(data, indent=2, ensure_ascii=False)
        print(text)
        (results_dir / "opencv_camera.json").write_text(text, encoding="utf-8")
        return 2

    if args.test_camera:
        import cv2

        index_order = [args.preferred_index] + [i for i in range(args.max_index + 1) if i != args.preferred_index]
        backends: list[int | None] = [None, cv2.CAP_DSHOW, cv2.CAP_MSMF]

        for index in index_order:
            for backend in backends:
                name = backend_name(cv2, backend)
                item = {"index": index, "backend": name, "ok": False}
                cap = None
                start = time.monotonic()
                try:
                    cap = cv2.VideoCapture(index) if backend is None else cv2.VideoCapture(index, backend)
                    item["opened"] = bool(cap.isOpened())
                    frames_read = 0
                    saved = None
                    frame_shape = None
                    black = None
                    while cap.isOpened() and frames_read < args.frames:
                        if time.monotonic() - start > args.per_open_timeout_s:
                            item["timeout"] = True
                            break
                        ok, frame = cap.read()
                        if not ok or frame is None:
                            time.sleep(0.05)
                            continue
                        frames_read += 1
                        frame_shape = list(frame.shape)
                        black = black_frame_ratio(frame)
                        if saved is None:
                            resized = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
                            filename = f"camera_index{index}_{name.lower()}_test.png"
                            out_path = results_dir / filename
                            cv2.imwrite(str(out_path), resized)
                            saved = str(out_path.resolve())
                    item.update(
                        {
                            "frames_read": frames_read,
                            "shape": frame_shape,
                            "black_frame_flag": black,
                            "saved_image": saved,
                            "ok": frames_read > 0,
                        }
                    )
                except Exception:
                    item["error"] = traceback.format_exc()
                finally:
                    if cap is not None:
                        cap.release()
                data["camera_results"].append(item)

    text = json.dumps(data, indent=2, ensure_ascii=False)
    print(text)
    (results_dir / "opencv_camera.json").write_text(text, encoding="utf-8")
    return 0 if data["opencv_import"]["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
