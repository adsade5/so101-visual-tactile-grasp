from __future__ import annotations

import json
from pathlib import Path

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIRECTORY = PROJECT_ROOT / "data" / "calibration"

BOARD_IMAGE_PATH = OUTPUT_DIRECTORY / "charuco_board_5x7.png"
BOARD_METADATA_PATH = OUTPUT_DIRECTORY / "charuco_board_5x7.json"

SQUARES_X = 7
SQUARES_Y = 5

SQUARE_LENGTH_M = 0.027
MARKER_LENGTH_M = 0.020

DICTIONARY_NAME = "DICT_4X4_50"

# 图案比例为5:7。这个分辨率适合打印。
IMAGE_WIDTH_PX = 2000
IMAGE_HEIGHT_PX = 2800


def main() -> int:
    if not hasattr(cv2, "aruco"):
        print("FAIL: this OpenCV build does not contain cv2.aruco")
        return 1

    dictionary_id = cv2.aruco.DICT_4X4_50
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_M,
        MARKER_LENGTH_M,
        dictionary,
    )

    board_image = board.generateImage(
        (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX),
        marginSize=0,
        borderBits=1,
    )

    OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)

    image_saved = cv2.imwrite(
        str(BOARD_IMAGE_PATH),
        board_image,
    )

    if not image_saved:
        print(f"FAIL: could not save board image: {BOARD_IMAGE_PATH}")
        return 1

    metadata = {
        "board_type": "charuco",
        "dictionary": DICTIONARY_NAME,
        "squares_x": SQUARES_X,
        "squares_y": SQUARES_Y,
        "square_length_m": SQUARE_LENGTH_M,
        "marker_length_m": MARKER_LENGTH_M,
        "pattern_width_m": SQUARES_X * SQUARE_LENGTH_M,
        "pattern_height_m": SQUARES_Y * SQUARE_LENGTH_M,
        "image_width_px": IMAGE_WIDTH_PX,
        "image_height_px": IMAGE_HEIGHT_PX,
    }

    with BOARD_METADATA_PATH.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            metadata,
            file,
            indent=2,
            ensure_ascii=False,
        )

    print("PASS: generated ChArUco calibration board")
    print(f"IMAGE={BOARD_IMAGE_PATH}")
    print(f"METADATA={BOARD_METADATA_PATH}")
    print(
        "PATTERN_SIZE="
        f"{metadata['pattern_width_m'] * 1000:.0f}x"
        f"{metadata['pattern_height_m'] * 1000:.0f} mm"
    )
    print(
        f"SQUARE_LENGTH={SQUARE_LENGTH_M * 1000:.0f} mm"
    )
    print(
        f"MARKER_LENGTH={MARKER_LENGTH_M * 1000:.0f} mm"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())