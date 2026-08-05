from __future__ import annotations

import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]

SOURCE_ROOT = (
    PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
)

SOURCE_URDF = (
    SOURCE_ROOT
    / "so101_new_calib.urdf"
)

SOURCE_ASSETS = SOURCE_ROOT / "assets"

PACKAGE_ROOT = (
    PROJECT_ROOT
    / "ros2_ws"
    / "src"
    / "so101_description"
)

OUTPUT_URDF = (
    PACKAGE_ROOT
    / "urdf"
    / "so101_visualization.urdf"
)

OUTPUT_ASSETS = PACKAGE_ROOT / "assets"

MANIFEST_PATH = (
    PACKAGE_ROOT
    / "model_preparation_manifest.json"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as file:
        while True:
            chunk = file.read(1024 * 1024)

            if not chunk:
                break

            digest.update(chunk)

    return digest.hexdigest()


def main() -> int:
    if not SOURCE_URDF.is_file():
        print(f"FAIL: URDF not found: {SOURCE_URDF}")
        return 1

    if not SOURCE_ASSETS.is_dir():
        print(
            f"FAIL: asset directory not found: "
            f"{SOURCE_ASSETS}"
        )
        return 1

    source_hash_before = sha256_file(
        SOURCE_URDF
    )

    OUTPUT_URDF.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    if OUTPUT_ASSETS.exists():
        shutil.rmtree(OUTPUT_ASSETS)

    shutil.copytree(
        SOURCE_ASSETS,
        OUTPUT_ASSETS,
    )

    tree = ET.parse(SOURCE_URDF)
    root = tree.getroot()

    replacements = []

    for mesh in root.iter("mesh"):
        original_filename = mesh.attrib.get(
            "filename"
        )

        if not original_filename:
            continue

        normalized = original_filename.replace(
            "\\",
            "/",
        )

        while normalized.startswith("./"):
            normalized = normalized[2:]

        replacement = None

        if normalized.startswith("assets/"):
            replacement = (
                "package://so101_description/"
                f"{normalized}"
            )

        elif normalized.startswith(
            "package://"
        ):
            replacement = None

        elif normalized.startswith("file://"):
            replacement = None

        else:
            local_candidate = (
                SOURCE_ROOT / normalized
            )

            if local_candidate.is_file():
                relative_path = (
                    local_candidate
                    .relative_to(SOURCE_ROOT)
                    .as_posix()
                )

                replacement = (
                    "package://so101_description/"
                    f"{relative_path}"
                )

        if replacement is not None:
            mesh.set(
                "filename",
                replacement,
            )

            replacements.append(
                {
                    "old": original_filename,
                    "new": replacement,
                }
            )

    try:
        ET.indent(
            tree,
            space="  ",
        )
    except AttributeError:
        pass

    tree.write(
        OUTPUT_URDF,
        encoding="utf-8",
        xml_declaration=True,
    )

    source_hash_after = sha256_file(
        SOURCE_URDF
    )

    if source_hash_before != source_hash_after:
        print(
            "FAIL: frozen source URDF was modified"
        )
        return 1

    asset_files = sorted(
        path
        for path in OUTPUT_ASSETS.rglob("*")
        if path.is_file()
    )

    manifest = {
        "status": "PASS",
        "source_urdf": str(SOURCE_URDF),
        "source_sha256": source_hash_before,
        "output_urdf": str(OUTPUT_URDF),
        "output_sha256": sha256_file(
            OUTPUT_URDF
        ),
        "mesh_path_replacement_count": (
            len(replacements)
        ),
        "asset_file_count": len(asset_files),
        "mesh_path_replacements": (
            replacements
        ),
        "notes": [
            (
                "The frozen kinematics URDF was not "
                "modified."
            ),
            (
                "This generated copy only changes mesh "
                "resource paths for ROS2/RViz."
            ),
        ],
    }

    MANIFEST_PATH.write_text(
        json.dumps(
            manifest,
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    print("=== SO-101 DESCRIPTION PREPARATION ===")
    print(f"SOURCE_URDF={SOURCE_URDF}")
    print(f"SOURCE_SHA256={source_hash_before}")
    print(f"OUTPUT_URDF={OUTPUT_URDF}")
    print(
        "MESH_PATH_REPLACEMENTS="
        f"{len(replacements)}"
    )
    print(f"ASSET_FILES={len(asset_files)}")
    print(f"MANIFEST={MANIFEST_PATH}")
    print(
        "PASS: visualization model prepared "
        "without modifying the frozen URDF"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())