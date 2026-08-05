from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = PROJECT_ROOT / "data" / "camera_probe"

BACKENDS = {
    "default": cv2.CAP_ANY,
    "dshow": cv2.CAP_DSHOW,
    "msmf": cv2.CAP_MSMF,
}


@dataclass
class ProbeResult:
    index: int
    backend_name: str
    opened: bool
    valid_frames: int
    attempted_frames: int
    width: int
    height: int
    measured_fps: float
    reported_fps: float
    saved_frame: Path | None
    error: str | None


def probe_camera(
    index: int,
    backend_name: str,
    *,
    requested_width: int,
    requested_height: int,
    requested_fps: float,
    frame_count: int,
) -> ProbeResult:
    backend = BACKENDS[backend_name]
    capture = cv2.VideoCapture(index, backend)

    if not capture.isOpened():
        capture.release()
        return ProbeResult(
            index=index,
            backend_name=backend_name,
            opened=False,
            valid_frames=0,
            attempted_frames=frame_count,
            width=0,
            height=0,
            measured_fps=0.0,
            reported_fps=0.0,
            saved_frame=None,
            error="VideoCapture could not be opened",
        )

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, requested_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, requested_height)
    capture.set(cv2.CAP_PROP_FPS, requested_fps)

    valid_frames = 0
    last_valid_frame = None
    start_time = time.monotonic()

    try:
        # 丢弃最开始的若干帧，给相机自动曝光留时间。
        for _ in range(10):
            capture.read()

        start_time = time.monotonic()

        for _ in range(frame_count):
            ok, frame = capture.read()

            if not ok or frame is None or frame.size == 0:
                continue

            valid_frames += 1
            last_valid_frame = frame

        elapsed = time.monotonic() - start_time

        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        reported_fps = float(capture.get(cv2.CAP_PROP_FPS))

    finally:
        capture.release()

    measured_fps = (
        valid_frames / elapsed
        if elapsed > 0
        else 0.0
    )

    saved_frame = None

    if last_valid_frame is not None:
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        saved_frame = (
            OUTPUT_DIR
            / f"camera_{index}_{backend_name}.png"
        )

        saved_ok = cv2.imwrite(
            str(saved_frame),
            last_valid_frame,
        )

        if not saved_ok:
            saved_frame = None

    return ProbeResult(
        index=index,
        backend_name=backend_name,
        opened=True,
        valid_frames=valid_frames,
        attempted_frames=frame_count,
        width=width,
        height=height,
        measured_fps=measured_fps,
        reported_fps=reported_fps,
        saved_frame=saved_frame,
        error=None,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Safely probe Windows OpenCV camera indices "
            "and capture backends."
        )
    )

    parser.add_argument(
        "--indices",
        nargs="+",
        type=int,
        default=[0, 1, 2, 3, 4, 5],
    )
    parser.add_argument(
        "--backends",
        nargs="+",
        choices=sorted(BACKENDS),
        default=["dshow", "msmf", "default"],
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=100,
    )

    args = parser.parse_args()

    if args.frames <= 0:
        parser.error("--frames must be positive")

    successful_results: list[ProbeResult] = []

    print("OpenCV version:", cv2.__version__)
    print("Output directory:", OUTPUT_DIR)
    print()

    for index in args.indices:
        for backend_name in args.backends:
            print(
                f"Testing index={index}, "
                f"backend={backend_name}..."
            )

            result = probe_camera(
                index,
                backend_name,
                requested_width=args.width,
                requested_height=args.height,
                requested_fps=args.fps,
                frame_count=args.frames,
            )

            if not result.opened:
                print(f"  FAIL: {result.error}")
                continue

            frame_ratio = (
                result.valid_frames
                / result.attempted_frames
            )

            print(
                "  OPENED: "
                f"{result.width}x{result.height}, "
                f"valid={result.valid_frames}/"
                f"{result.attempted_frames}, "
                f"ratio={frame_ratio:.1%}, "
                f"measured_fps="
                f"{result.measured_fps:.2f}, "
                f"reported_fps="
                f"{result.reported_fps:.2f}"
            )

            if result.saved_frame is not None:
                print(
                    f"  FRAME: {result.saved_frame}"
                )

            if frame_ratio >= 0.95:
                successful_results.append(result)

    print()
    print("=== SUMMARY ===")

    if not successful_results:
        print(
            "FAIL: no camera configuration achieved "
            "at least 95% valid frames"
        )
        return 1

    successful_results.sort(
        key=lambda item: (
            item.valid_frames,
            item.measured_fps,
        ),
        reverse=True,
    )

    best = successful_results[0]

    print("PASS: at least one stable camera configuration")
    print(f"BEST_INDEX={best.index}")
    print(f"BEST_BACKEND={best.backend_name}")
    print(f"BEST_RESOLUTION={best.width}x{best.height}")
    print(f"BEST_MEASURED_FPS={best.measured_fps:.2f}")
    print(f"BEST_REPORTED_FPS={best.reported_fps:.2f}")
    print(f"BEST_FRAME={best.saved_frame}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())