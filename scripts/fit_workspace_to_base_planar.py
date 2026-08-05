from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import shutil
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_CSV_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "workspace_to_base_correspondences.csv"
)

CONFIG_PATH = (
    PROJECT_ROOT
    / "config"
    / "workspace_to_base.json"
)

REPORT_PATH = (
    PROJECT_ROOT
    / "data"
    / "calibration"
    / "workspace_to_base_calibration_report.json"
)

FIT_RMS_LIMIT_MM = 3.0
FIT_MAX_LIMIT_MM = 5.0
HOLDOUT_MAX_LIMIT_MM = 5.0


def timestamp() -> str:
    return dt.datetime.now(
        dt.timezone.utc
    ).isoformat()


def load_rows(
    path: Path,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []

    with path.open(
        "r",
        encoding="utf-8-sig",
        newline="",
    ) as file:
        reader = csv.DictReader(file)

        required = {
            "id",
            "workspace_x_m",
            "workspace_y_m",
            "base_x_m",
            "base_y_m",
            "set",
        }

        if (
            reader.fieldnames is None
            or not required.issubset(
                set(reader.fieldnames)
            )
        ):
            raise ValueError(
                "CSV columns are incomplete"
            )

        for raw in reader:
            row = {
                "id": str(raw["id"]),
                "workspace": np.asarray(
                    [
                        float(
                            raw[
                                "workspace_x_m"
                            ]
                        ),
                        float(
                            raw[
                                "workspace_y_m"
                            ]
                        ),
                    ],
                    dtype=np.float64,
                ),
                "base": np.asarray(
                    [
                        float(
                            raw["base_x_m"]
                        ),
                        float(
                            raw["base_y_m"]
                        ),
                    ],
                    dtype=np.float64,
                ),
                "set": str(
                    raw["set"]
                )
                .strip()
                .lower(),
            }

            if not np.all(
                np.isfinite(
                    row["workspace"]
                )
            ):
                raise ValueError(
                    f"{row['id']} has "
                    "non-finite workspace data"
                )

            if not np.all(
                np.isfinite(
                    row["base"]
                )
            ):
                raise ValueError(
                    f"{row['id']} has "
                    "non-finite base data"
                )

            if row["set"] not in {
                "fit",
                "holdout",
            }:
                raise ValueError(
                    f"{row['id']} has "
                    "invalid set value"
                )

            rows.append(row)

    return rows


def fit_rigid_transform(
    workspace_points: np.ndarray,
    base_points: np.ndarray,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    if (
        workspace_points.shape
        != base_points.shape
        or workspace_points.ndim != 2
        or workspace_points.shape[1] != 2
    ):
        raise ValueError(
            "Point arrays must have "
            "matching shape (N, 2)"
        )

    if workspace_points.shape[0] < 3:
        raise ValueError(
            "At least three fit points "
            "are required"
        )

    workspace_center = np.mean(
        workspace_points,
        axis=0,
    )

    base_center = np.mean(
        base_points,
        axis=0,
    )

    workspace_zero = (
        workspace_points
        - workspace_center
    )

    base_zero = (
        base_points
        - base_center
    )

    covariance = (
        workspace_zero.T
        @ base_zero
    )

    u, singular_values, vt = (
        np.linalg.svd(
            covariance
        )
    )

    rotation = vt.T @ u.T

    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    translation = (
        base_center
        - rotation @ workspace_center
    )

    return (
        rotation,
        translation,
        singular_values,
    )


def evaluate_rows(
    rows: list[dict[str, Any]],
    rotation: np.ndarray,
    translation: np.ndarray,
) -> list[dict[str, Any]]:
    results = []

    for row in rows:
        predicted = (
            rotation
            @ row["workspace"]
            + translation
        )

        error_mm = float(
            np.linalg.norm(
                predicted - row["base"]
            )
            * 1000.0
        )

        results.append(
            {
                "id": row["id"],
                "set": row["set"],
                "workspace_xy_m": (
                    row["workspace"].tolist()
                ),
                "expected_base_xy_m": (
                    row["base"].tolist()
                ),
                "predicted_base_xy_m": (
                    predicted.tolist()
                ),
                "error_mm": error_mm,
            }
        )

    return results


def main() -> int:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_CSV_PATH,
    )

    parser.add_argument(
        "--tz-m",
        type=float,
        default=0.0,
        help=(
            "Provisional workspace-plane "
            "height in base_link."
        ),
    )

    parser.add_argument(
        "--write-config",
        action="store_true",
    )

    args = parser.parse_args()

    rows = load_rows(
        args.csv.resolve()
    )

    fit_rows = [
        row
        for row in rows
        if row["set"] == "fit"
    ]

    holdout_rows = [
        row
        for row in rows
        if row["set"] == "holdout"
    ]

    if len(fit_rows) < 6:
        raise ValueError(
            "At least six fit points "
            "are required"
        )

    if len(holdout_rows) < 2:
        raise ValueError(
            "At least two holdout points "
            "are required"
        )

    workspace_fit = np.stack(
        [
            row["workspace"]
            for row in fit_rows
        ]
    )

    base_fit = np.stack(
        [
            row["base"]
            for row in fit_rows
        ]
    )

    (
        rotation,
        translation,
        singular_values,
    ) = fit_rigid_transform(
        workspace_fit,
        base_fit,
    )

    yaw_rad = math.atan2(
        float(rotation[1, 0]),
        float(rotation[0, 0]),
    )

    yaw_deg = math.degrees(
        yaw_rad
    )

    results = evaluate_rows(
        rows,
        rotation,
        translation,
    )

    fit_errors = [
        item["error_mm"]
        for item in results
        if item["set"] == "fit"
    ]

    holdout_errors = [
        item["error_mm"]
        for item in results
        if item["set"] == "holdout"
    ]

    fit_rms_mm = math.sqrt(
        sum(
            error**2
            for error in fit_errors
        )
        / len(fit_errors)
    )

    fit_max_mm = max(
        fit_errors
    )

    holdout_max_mm = max(
        holdout_errors
    )

    passed = bool(
        fit_rms_mm
        <= FIT_RMS_LIMIT_MM
        and fit_max_mm
        <= FIT_MAX_LIMIT_MM
        and holdout_max_mm
        <= HOLDOUT_MAX_LIMIT_MM
    )

    report = {
        "stage": "2C-1B",
        "status": (
            "PASS"
            if passed
            else "FAIL"
        ),
        "timestamp": timestamp(),
        "description": (
            "Planar rigid calibration from "
            "workspace_plane to base_link"
        ),
        "input_csv": str(
            args.csv.resolve()
        ),
        "fit_point_count": len(
            fit_rows
        ),
        "holdout_point_count": len(
            holdout_rows
        ),
        "rotation_matrix_2d": (
            rotation.tolist()
        ),
        "translation_xy_m": (
            translation.tolist()
        ),
        "provisional_translation_z_m": (
            float(args.tz_m)
        ),
        "yaw_deg": yaw_deg,
        "singular_values": (
            singular_values.tolist()
        ),
        "metrics": {
            "fit_rms_mm": fit_rms_mm,
            "fit_max_mm": fit_max_mm,
            "holdout_max_mm": (
                holdout_max_mm
            ),
        },
        "thresholds": {
            "fit_rms_limit_mm": (
                FIT_RMS_LIMIT_MM
            ),
            "fit_max_limit_mm": (
                FIT_MAX_LIMIT_MM
            ),
            "holdout_max_limit_mm": (
                HOLDOUT_MAX_LIMIT_MM
            ),
        },
        "points": results,
        "configuration_written": False,
        "limitations": [
            (
                "Only planar X, Y and yaw "
                "are calibrated."
            ),
            (
                "Translation Z remains "
                "provisional and must be "
                "validated separately."
            ),
            (
                "The calibration becomes "
                "invalid if the camera, "
                "workspace board or robot "
                "base moves."
            ),
        ],
    }

    if passed and args.write_config:
        CONFIG_PATH.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        if CONFIG_PATH.exists():
            backup_path = (
                CONFIG_PATH.with_name(
                    "workspace_to_base_"
                    "synthetic_backup.json"
                )
            )

            shutil.copy2(
                CONFIG_PATH,
                backup_path,
            )

            report[
                "previous_config_backup"
            ] = str(backup_path)

        config = {
            "version": "1.0",
            "calibration_status": (
                "real_planar_calibrated_"
                "z_pending"
            ),
            "source_frame": (
                "workspace_plane"
            ),
            "target_frame": (
                "base_link"
            ),
            "translation_m": [
                float(translation[0]),
                float(translation[1]),
                float(args.tz_m),
            ],
            "yaw_deg": float(
                yaw_deg
            ),
            "stale_timeout_s": 0.5,
            "calibration_report": str(
                REPORT_PATH
            ),
            "fit_rms_mm": float(
                fit_rms_mm
            ),
            "fit_max_mm": float(
                fit_max_mm
            ),
            "holdout_max_mm": float(
                holdout_max_mm
            ),
        }

        CONFIG_PATH.write_text(
            json.dumps(
                config,
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )

        report[
            "configuration_written"
        ] = True

        report[
            "configuration_path"
        ] = str(CONFIG_PATH)

    REPORT_PATH.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    REPORT_PATH.write_text(
        json.dumps(
            report,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        ),
        encoding="utf-8",
    )

    print(
        json.dumps(
            {
                "status": report["status"],
                "yaw_deg": yaw_deg,
                "translation_xy_m": (
                    translation.tolist()
                ),
                "fit_rms_mm": fit_rms_mm,
                "fit_max_mm": fit_max_mm,
                "holdout_max_mm": (
                    holdout_max_mm
                ),
                "configuration_written": (
                    report[
                        "configuration_written"
                    ]
                ),
                "report": str(
                    REPORT_PATH
                ),
            },
            indent=2,
        )
    )

    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())