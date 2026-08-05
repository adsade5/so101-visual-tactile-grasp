from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"File not found: {path}")

    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)

    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object: {path}")

    return value


def invert_rigid_transform(
    transform: np.ndarray,
) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation

    return inverse


def recover_camera_from_plane(
    camera_matrix: np.ndarray,
    plane_to_pixel: np.ndarray,
) -> np.ndarray:
    normalized = (
        np.linalg.inv(camera_matrix)
        @ plane_to_pixel
    )

    column_1 = normalized[:, 0]
    column_2 = normalized[:, 1]
    column_3 = normalized[:, 2]

    scale = 2.0 / (
        np.linalg.norm(column_1)
        + np.linalg.norm(column_2)
    )

    rotation_1 = scale * column_1
    rotation_2 = scale * column_2
    translation = scale * column_3

    if translation[2] < 0:
        rotation_1 = -rotation_1
        rotation_2 = -rotation_2
        translation = -translation

    rotation_3 = np.cross(
        rotation_1,
        rotation_2,
    )

    approximate_rotation = np.column_stack(
        [
            rotation_1,
            rotation_2,
            rotation_3,
        ]
    )

    u_matrix, _, vt_matrix = np.linalg.svd(
        approximate_rotation
    )

    rotation = u_matrix @ vt_matrix

    if np.linalg.det(rotation) < 0:
        u_matrix[:, -1] *= -1
        rotation = u_matrix @ vt_matrix

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    return transform


def pixels_to_camera_rays(
    pixels: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    normalized = cv2.undistortPoints(
        np.asarray(
            pixels,
            dtype=np.float64,
        ).reshape(-1, 1, 2),
        camera_matrix,
        distortion,
    ).reshape(-1, 2)

    rays = np.column_stack(
        [
            normalized[:, 0],
            normalized[:, 1],
            np.ones(len(normalized)),
        ]
    )

    norms = np.linalg.norm(
        rays,
        axis=1,
        keepdims=True,
    )

    if np.any(norms <= 1e-12):
        raise ValueError("Invalid camera ray")

    return rays / norms


def intersect_rays_with_height_plane(
    rays_camera: np.ndarray,
    plane_from_camera: np.ndarray,
    plane_z_mm: float,
) -> np.ndarray:
    camera_origin = plane_from_camera[:3, 3]

    directions = (
        plane_from_camera[:3, :3]
        @ rays_camera.T
    ).T

    denominators = directions[:, 2]

    if np.any(np.abs(denominators) < 1e-9):
        raise ValueError(
            "A camera ray is parallel to the object plane"
        )

    distances = (
        plane_z_mm
        - camera_origin[2]
    ) / denominators

    if np.any(distances <= 0):
        raise ValueError(
            "Object plane is behind the camera"
        )

    return (
        camera_origin.reshape(1, 3)
        + distances.reshape(-1, 1)
        * directions
    )


def wrap_degrees(angle: float) -> float:
    return (
        angle + 180.0
    ) % 360.0 - 180.0


def circular_mean_degrees(
    values: list[float],
) -> float:
    radians = np.radians(
        np.asarray(values, dtype=np.float64)
    )

    mean_sine = float(
        np.mean(np.sin(radians))
    )

    mean_cosine = float(
        np.mean(np.cos(radians))
    )

    return wrap_degrees(
        math.degrees(
            math.atan2(
                mean_sine,
                mean_cosine,
            )
        )
    )


def circular_std_degrees(
    values: list[float],
    mean_degrees: float,
) -> float:
    differences = [
        wrap_degrees(value - mean_degrees)
        for value in values
    ]

    return float(
        np.std(
            np.asarray(
                differences,
                dtype=np.float64,
            )
        )
    )


def quaternion_from_yaw(
    yaw_degrees: float,
) -> tuple[float, float, float, float]:
    half_yaw = math.radians(
        yaw_degrees
    ) / 2.0

    return (
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
    )


def find_target_index(
    marker_ids: np.ndarray,
    target_id: int,
) -> int | None:
    ids = marker_ids.reshape(-1)

    matches = np.where(
        ids == target_id
    )[0]

    if len(matches) == 0:
        return None

    return int(matches[0])


def marker_geometry(
    corners_plane: np.ndarray,
) -> tuple[np.ndarray, float, float]:
    center = np.mean(
        corners_plane,
        axis=0,
    )

    left_midpoint = 0.5 * (
        corners_plane[0]
        + corners_plane[3]
    )

    right_midpoint = 0.5 * (
        corners_plane[1]
        + corners_plane[2]
    )

    marker_x_axis = (
        right_midpoint
        - left_midpoint
    )

    yaw_degrees = wrap_degrees(
        math.degrees(
            math.atan2(
                float(marker_x_axis[1]),
                float(marker_x_axis[0]),
            )
        )
    )

    side_lengths = []

    for index in range(4):
        first = corners_plane[index]
        second = corners_plane[
            (index + 1) % 4
        ]

        side_lengths.append(
            float(
                np.linalg.norm(
                    second[:2] - first[:2]
                )
            )
        )

    return (
        center,
        yaw_degrees,
        float(np.mean(side_lengths)),
    )


def apply_center_offset(
    marker_center_xy: np.ndarray,
    yaw_degrees: float,
    offset_x_mm: float,
    offset_y_mm: float,
) -> np.ndarray:
    yaw_radians = math.radians(
        yaw_degrees
    )

    rotation = np.asarray(
        [
            [
                math.cos(yaw_radians),
                -math.sin(yaw_radians),
            ],
            [
                math.sin(yaw_radians),
                math.cos(yaw_radians),
            ],
        ],
        dtype=np.float64,
    )

    local_offset = np.asarray(
        [
            offset_x_mm,
            offset_y_mm,
        ],
        dtype=np.float64,
    )

    return (
        marker_center_xy
        + rotation @ local_offset
    )


class ObjectPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("object_pose_node")

        self.declare_parameter(
            "project_root",
            (
                "E:/PycharmProjects/Embodied_AI/"
                "LeRobot_Project/"
                "so101_visual_tactile_grasp"
            ),
        )

        self.declare_parameter(
            "show_debug_window",
            True,
        )

        self.declare_parameter(
            "publish_rate_hz",
            20.0,
        )

        self.declare_parameter(
            "frame_id",
            "workspace_plane",
        )

        project_root = Path(
            str(
                self.get_parameter(
                    "project_root"
                ).value
            )
        )

        self.show_debug_window = bool(
            self.get_parameter(
                "show_debug_window"
            ).value
        )

        publish_rate_hz = float(
            self.get_parameter(
                "publish_rate_hz"
            ).value
        )

        self.frame_id = str(
            self.get_parameter(
                "frame_id"
            ).value
        )

        marker_config_path = (
            project_root
            / "config"
            / "object_marker.json"
        )

        intrinsics_path = (
            project_root
            / "data"
            / "calibration"
            / "camera_calibration_report.json"
        )

        plane_calibration_path = (
            project_root
            / "data"
            / "calibration"
            / "workspace_plane_calibration.json"
        )

        marker_config = load_json(
            marker_config_path
        )

        intrinsics = load_json(
            intrinsics_path
        )

        plane_calibration = load_json(
            plane_calibration_path
        )

        if plane_calibration.get("quality") != "PASS":
            raise ValueError(
                "Workspace calibration is not PASS"
            )

        self.camera_matrix = np.asarray(
            intrinsics["camera_matrix"],
            dtype=np.float64,
        )

        self.distortion = np.asarray(
            intrinsics[
                "distortion_coefficients"
            ],
            dtype=np.float64,
        ).reshape(1, -1)

        dictionary_name = str(
            marker_config["dictionary"]
        )

        if dictionary_name not in DICTIONARIES:
            raise ValueError(
                f"Unsupported dictionary: "
                f"{dictionary_name}"
            )

        self.target_id = int(
            marker_config["target_id"]
        )

        if self.target_id < 0:
            raise ValueError(
                "object_marker.json target_id "
                "must be fixed"
            )

        self.marker_size_mm = (
            float(
                marker_config["marker_size_m"]
            )
            * 1000.0
        )

        self.object_height_mm = float(
            marker_config[
                "object_top_height_mm"
            ]
        )

        offset = marker_config.get(
            "marker_center_offset_mm",
            {},
        )

        self.offset_x_mm = float(
            offset.get("x", 0.0)
        )

        self.offset_y_mm = float(
            offset.get("y", 0.0)
        )

        camera_config = marker_config["camera"]

        self.camera_index = int(
            camera_config["index"]
        )

        self.image_width = int(
            camera_config["width"]
        )

        self.image_height = int(
            camera_config["height"]
        )

        plane_to_pixel = np.asarray(
            plane_calibration[
                "homography_plane_mm_to_undistorted_pixel"
            ],
            dtype=np.float64,
        )

        camera_from_plane = (
            recover_camera_from_plane(
                self.camera_matrix,
                plane_to_pixel,
            )
        )

        self.plane_from_camera = (
            invert_rigid_transform(
                camera_from_plane
            )
        )

        camera_position = (
            self.plane_from_camera[:3, 3]
        )

        self.internal_top_plane_z_mm = (
            math.copysign(
                self.object_height_mm,
                float(camera_position[2]),
            )
        )

        dictionary = (
            cv2.aruco.getPredefinedDictionary(
                DICTIONARIES[dictionary_name]
            )
        )

        detector_parameters = (
            cv2.aruco.DetectorParameters()
        )

        detector_parameters.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
        )

        self.detector = cv2.aruco.ArucoDetector(
            dictionary,
            detector_parameters,
        )

        self.capture = cv2.VideoCapture(
            self.camera_index,
            cv2.CAP_DSHOW,
        )

        if not self.capture.isOpened():
            raise RuntimeError(
                "Could not open workspace camera"
            )

        self.capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.image_width,
        )

        self.capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.image_height,
        )

        self.capture.set(
            cv2.CAP_PROP_FPS,
            30.0,
        )

        for _ in range(20):
            self.capture.read()

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/object_pose",
            10,
        )

        self.detected_publisher = (
            self.create_publisher(
                Bool,
                "/object_detected",
                10,
            )
        )

        self.stable_publisher = (
            self.create_publisher(
                Bool,
                "/object_pose_stable",
                10,
            )
        )

        self.samples: deque[
            tuple[float, float, float, float]
        ] = deque(maxlen=15)

        self.last_detection_time = 0.0
        self.last_log_time = 0.0

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.process_frame,
        )

        self.get_logger().info(
            "Object perception node started"
        )

        self.get_logger().info(
            f"target_id={self.target_id}, "
            f"marker_size={self.marker_size_mm:.1f} mm, "
            f"known_height={self.object_height_mm:.1f} mm"
        )

    def publish_boolean(
        self,
        publisher: Any,
        value: bool,
    ) -> None:
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    def process_frame(self) -> None:
        ok, frame = self.capture.read()

        if (
            not ok
            or frame is None
            or frame.size == 0
        ):
            self.publish_boolean(
                self.detected_publisher,
                False,
            )

            self.publish_boolean(
                self.stable_publisher,
                False,
            )

            return

        display = frame.copy()

        marker_corners, marker_ids, _ = (
            self.detector.detectMarkers(frame)
        )

        target_index = None

        if marker_ids is not None:
            target_index = find_target_index(
                marker_ids,
                self.target_id,
            )

            if self.show_debug_window:
                cv2.aruco.drawDetectedMarkers(
                    display,
                    marker_corners,
                    marker_ids,
                )

        now_monotonic = time.monotonic()

        if target_index is None:
            if (
                now_monotonic
                - self.last_detection_time
                > 0.3
            ):
                self.samples.clear()

            self.publish_boolean(
                self.detected_publisher,
                False,
            )

            self.publish_boolean(
                self.stable_publisher,
                False,
            )

            if self.show_debug_window:
                cv2.putText(
                    display,
                    "OBJECT NOT DETECTED",
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

                self.show_window(display)

            return

        raw_corners = np.asarray(
            marker_corners[target_index],
            dtype=np.float64,
        ).reshape(4, 2)

        try:
            rays = pixels_to_camera_rays(
                raw_corners,
                self.camera_matrix,
                self.distortion,
            )

            corners_plane = (
                intersect_rays_with_height_plane(
                    rays,
                    self.plane_from_camera,
                    self.internal_top_plane_z_mm,
                )
            )

            (
                marker_center,
                yaw_degrees,
                measured_marker_size_mm,
            ) = marker_geometry(
                corners_plane
            )

            object_center = apply_center_offset(
                marker_center[:2],
                yaw_degrees,
                self.offset_x_mm,
                self.offset_y_mm,
            )

        except ValueError as error:
            self.samples.clear()

            self.publish_boolean(
                self.detected_publisher,
                False,
            )

            self.publish_boolean(
                self.stable_publisher,
                False,
            )

            self.get_logger().warning(
                f"Geometry rejected: {error}"
            )

            return

        x_mm = float(object_center[0])
        y_mm = float(object_center[1])

        self.samples.append(
            (
                x_mm,
                y_mm,
                yaw_degrees,
                measured_marker_size_mm,
            )
        )

        self.last_detection_time = (
            now_monotonic
        )

        samples = np.asarray(
            self.samples,
            dtype=np.float64,
        )

        smooth_x_mm = float(
            np.median(samples[:, 0])
        )

        smooth_y_mm = float(
            np.median(samples[:, 1])
        )

        yaw_values = samples[:, 2].tolist()

        smooth_yaw_degrees = (
            circular_mean_degrees(
                yaw_values
            )
        )

        smooth_marker_size_mm = float(
            np.median(samples[:, 3])
        )

        x_std_mm = float(
            np.std(samples[:, 0])
        )

        y_std_mm = float(
            np.std(samples[:, 1])
        )

        yaw_std_degrees = (
            circular_std_degrees(
                yaw_values,
                smooth_yaw_degrees,
            )
        )

        marker_size_error_mm = abs(
            smooth_marker_size_mm
            - self.marker_size_mm
        )

        stable = (
            len(self.samples) >= 10
            and x_std_mm <= 1.5
            and y_std_mm <= 1.5
            and yaw_std_degrees <= 2.0
            and marker_size_error_mm <= 2.5
        )

        pose = PoseStamped()

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.header.frame_id = self.frame_id

        pose.pose.position.x = (
            smooth_x_mm / 1000.0
        )

        pose.pose.position.y = (
            smooth_y_mm / 1000.0
        )

        # ROS层统一规定工作平面+Z朝向相机。
        pose.pose.position.z = (
            self.object_height_mm / 1000.0
        )

        qx, qy, qz, qw = quaternion_from_yaw(
            smooth_yaw_degrees
        )

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pose_publisher.publish(pose)

        self.publish_boolean(
            self.detected_publisher,
            True,
        )

        self.publish_boolean(
            self.stable_publisher,
            stable,
        )

        if (
            now_monotonic
            - self.last_log_time
            >= 1.0
        ):
            self.get_logger().info(
                f"x={smooth_x_mm:.2f} mm, "
                f"y={smooth_y_mm:.2f} mm, "
                f"z={self.object_height_mm:.2f} mm, "
                f"yaw={smooth_yaw_degrees:.2f} deg, "
                f"stable={stable}"
            )

            self.last_log_time = (
                now_monotonic
            )

        if self.show_debug_window:
            center_pixel = tuple(
                np.round(
                    np.mean(
                        raw_corners,
                        axis=0,
                    )
                ).astype(int)
            )

            color = (
                (0, 255, 0)
                if stable
                else (0, 165, 255)
            )

            cv2.circle(
                display,
                center_pixel,
                6,
                (0, 0, 255),
                -1,
            )

            cv2.putText(
                display,
                (
                    f"X={smooth_x_mm:.1f} "
                    f"Y={smooth_y_mm:.1f} "
                    f"Z={self.object_height_mm:.1f} mm"
                ),
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                color,
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                display,
                (
                    f"yaw={smooth_yaw_degrees:.1f} deg "
                    f"stable={'YES' if stable else 'NO'}"
                ),
                (12, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                color,
                2,
                cv2.LINE_AA,
            )

            self.show_window(display)

    def show_window(
        self,
        display: np.ndarray,
    ) -> None:
        cv2.imshow(
            "SO-101 ROS2 Object Perception",
            display,
        )

        key = cv2.waitKey(1) & 0xFF

        if key in (
            ord("q"),
            ord("Q"),
            27,
        ):
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        if hasattr(self, "capture"):
            self.capture.release()

        cv2.destroyAllWindows()

        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node: ObjectPoseNode | None = None

    try:
        node = ObjectPoseNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()