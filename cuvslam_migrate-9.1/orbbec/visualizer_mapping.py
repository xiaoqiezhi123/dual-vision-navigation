# Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA software released under the NVIDIA Community License is intended to be used to enable
# the further development of AI and robotics technologies. Such software has been designed, tested,
# and optimized for use with NVIDIA hardware, and this License grants permission to use the software
# solely with such hardware.
# Subject to the terms of this License, NVIDIA confirms that you are free to commercially use,
# modify, and distribute the software with NVIDIA hardware. NVIDIA does not claim ownership of any
# outputs generated using the software or derivative works thereof. Any code contributions that you
# share with NVIDIA are licensed to NVIDIA as feedback under this License and may be incorporated
# in future releases without notice or attribution.
# By using, reproducing, modifying, distributing, performing, or displaying any portion or element
# of the software or derivative works thereof, you agree to be bound by this License.

import os
import sys
from typing import List, Optional, Tuple

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

import cuvslam as vslam

# Constants
DEFAULT_NUM_VIZ_CAMERAS = 1
POINT_RADIUS = 5.0
ARROW_SCALE = 0.1
GRAVITY_ARROW_SCALE = 0.2
GRAVITY_ARROW_RADIUS = 0.005

MAP_LANDMARK_COLOR = [128, 128, 255]
LOOP_CLOSURE_COLOR = [255, 0, 0]
TRAJECTORY_COLOR = [0, 64, 255]  # 轨迹线颜色（蓝色）


def _find_rerun_viewer_path() -> Optional[str]:
    """定位 rerun 内置的 Rerun Viewer 可执行文件。

    rerun-sdk 的 wheel 内置了原生 viewer（``rerun_sdk/rerun_cli/rerun``），但 rerun 0.33 的
    ``spawn()`` 自动查找在部分机器上只搜 ``PATH``、找不到这个内置二进制，报
    "Failed to find Rerun Viewer executable in PATH"。这里显式算出路径交给
    ``spawn(executable_path=...)``。找不到时返回 None（退回 rerun 默认的 PATH 查找）。
    """
    try:
        import rerun_cli  # noqa: PLC0415 — 延迟导入，避免 rerun 尚未初始化时报错

        path = os.path.join(os.path.dirname(rerun_cli.__file__), "rerun")
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    except Exception:  # noqa: BLE001 — 定位失败就退回默认行为
        pass
    return None


class MappingVisualizer:
    """Rerun-based visualizer for cuVSLAM tracking results, with optional mapping display.

    Compared to the base ``RerunVisualizer``, this class can additionally show the SLAM
    map being built in real time: sparse map landmarks, the pose graph (keyframes + edges),
    and loop-closure positions. Toggle via ``show_mapping``.
    """

    def __init__(
        self,
        num_viz_cameras: int = DEFAULT_NUM_VIZ_CAMERAS,
        image_size: Optional[Tuple[int, int]] = None,
        show_gravity: bool = False,
        show_mapping: bool = False,
        spawn: bool = True,
        save_path: Optional[str] = None,
    ) -> None:
        """Initialize rerun visualizer.

        Args:
            num_viz_cameras: Number of cameras to visualize
            image_size: Optional image size as (width, height) for fixed 2D view bounds
            show_gravity: Whether to show the estimated gravity direction
            show_mapping: Whether to show the SLAM map (landmarks / pose graph / loop closures)
            spawn: Whether to spawn the rerun viewer on startup
            save_path: Optional .rrd path to continuously record to (enables offline replay).
                Without it, data only streams to the spawned viewer and ``rr.save()`` writes an empty file.
        """
        self.num_viz_cameras = num_viz_cameras
        self.image_size = image_size
        self.show_gravity = show_gravity
        self.show_mapping = show_mapping
        self.save_path = save_path

        if save_path is not None:
            # FileSink 让数据持续落盘；spawn 时数据同时流向 viewer 与 .rrd 文件。
            self._recording = rr.RecordingStream("cuVSLAM Visualizer", make_default=True)
            self._recording.set_sinks(rr.FileSink(save_path))
            if spawn:
                try:
                    rr.spawn(executable_path=_find_rerun_viewer_path(), recording=self._recording)
                    print("[visualizer] Rerun Viewer 已启动（实时可视化 + 落盘 .rrd）")
                except Exception as e:  # noqa: BLE001 — 无显示器 / 找不到 viewer 时降级为仅落盘
                    print(f"[visualizer] 无法启动 Rerun Viewer GUI：{e}", file=sys.stderr)
                    print("[visualizer] 仅落盘 .rrd；稍后可用 ./venv/bin/python -m rerun_cli <文件> 离线查看",
                          file=sys.stderr)
        else:
            self._recording = None
            rr.init("cuVSLAM Visualizer", spawn=spawn)

        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

        # Set up the visualization layout
        self._setup_blueprint()
        self.track_colors = {}

    def _setup_blueprint(self) -> None:
        """Set up the Rerun blueprint for visualization layout."""
        rr.send_blueprint(
            rrb.Blueprint(
                rrb.TimePanel(state="collapsed"),
                rrb.Horizontal(
                    column_shares=[0.5, 0.5],
                    contents=[
                        rrb.Vertical(contents=[
                            rrb.Spatial2DView(
                                origin=f'world/camera_{i}',
                                visual_bounds=rrb.VisualBounds2D(
                                    x_range=[0, self.image_size[0]],
                                    y_range=[0, self.image_size[1]]
                                ) if self.image_size is not None else None
                            )
                            for i in range(self.num_viz_cameras)
                        ]),
                        rrb.Spatial3DView(origin='world')
                    ]
                )
            ),
            make_active=True
        )

    def _log_rig_pose(
        self, rotation_quat: np.ndarray, translation: np.ndarray
    ) -> None:
        """Log rig pose to Rerun.

        Args:
            rotation_quat: Rotation quaternion
            translation: Translation vector
        """
        rr.log(
            "world/rig",
            rr.Transform3D(translation=translation, quaternion=rotation_quat),
            rr.Arrows3D(
                vectors=np.eye(3) * ARROW_SCALE,
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]  # RGB for XYZ
            )
        )

    def _log_observations(
        self,
        observations_main_cam: List[vslam.Observation],
        image: np.ndarray,
        camera_name: str
    ) -> None:
        """Log 2D observations for a specific camera with consistent colors.

        Args:
            observations_main_cam: List of observations
            image: Camera image
            camera_name: Name of the camera for logging
        """
        # Handle different image datatypes for compression
        if image.dtype == np.uint8:
            image_log = rr.Image(image).compress()
        else:
            # For other datatypes, don't compress to avoid issues
            image_log = rr.Image(image)

        image_path = f"world/{camera_name}/image"
        if not observations_main_cam:
            rr.log(image_path, image_log)
            return

        # Assign random color to new tracks
        for obs in observations_main_cam:
            if obs.id not in self.track_colors:
                self.track_colors[obs.id] = np.random.randint(0, 256, size=3)

        points = np.array([[obs.u, obs.v] for obs in observations_main_cam])
        colors = np.array([
            self.track_colors[obs.id] for obs in observations_main_cam
        ])

        rr.log(
            image_path,
            image_log
        )
        rr.log(
            f"world/{camera_name}/observations",
            rr.Points2D(positions=points, colors=colors, radii=POINT_RADIUS)
        )

    def _log_gravity(self, gravity: np.ndarray) -> None:
        """Log gravity direction to Rerun."""
        gravity_norm = np.linalg.norm(gravity)
        if gravity_norm == 0:
            return

        rr.log(
            "world/rig/gravity",
            rr.Arrows3D(
                vectors=gravity / gravity_norm * GRAVITY_ARROW_SCALE,
                colors=[[255, 0, 0]],
                radii=GRAVITY_ARROW_RADIUS
            )
        )

    def _log_map_landmarks(self, map_landmarks: Optional[vslam.Tracker.SlamLandmarks]) -> None:
        """Log SLAM map landmarks as a sparse 3D point cloud."""
        if map_landmarks is None or not map_landmarks.landmarks:
            return

        rr.log(
            "world/map_landmarks",
            rr.Points3D(
                [l.coords for l in map_landmarks.landmarks],
                colors=[MAP_LANDMARK_COLOR]
            )
        )

    def _log_loop_closure_poses(
        self, loop_closure_poses: Optional[List[vslam.PoseStamped]]
    ) -> None:
        """Log loop-closure positions."""
        if not loop_closure_poses:
            return

        rr.log(
            "world/loop_closure_poses",
            rr.Points3D(
                [p.pose.translation for p in loop_closure_poses],
                colors=[LOOP_CLOSURE_COLOR]
            )
        )

    def visualize_frame(
        self,
        frame_id: int,
        images: List[np.ndarray],
        pose: vslam.Pose,
        observations_main_cam: List[List[vslam.Observation]],
        trajectory: List[np.ndarray],
        timestamp: int,
        gravity: Optional[np.ndarray] = None,
        trajectory_slam: Optional[List[np.ndarray]] = None,
        loop_closure_poses: Optional[List[vslam.PoseStamped]] = None,
        map_landmarks: Optional[vslam.Tracker.SlamLandmarks] = None,
    ) -> None:
        """Visualize current frame state using Rerun.

        Args:
            frame_id: Current frame ID
            images: List of camera images
            pose: Current pose estimate
            observations_main_cam: List of observations for each camera
            trajectory: List of trajectory points
            timestamp: Current timestamp
            gravity: Optional gravity vector
            trajectory_slam: Optional SLAM-optimized trajectory points
            loop_closure_poses: Optional positions of detected loop closures
            map_landmarks: Optional SLAM map landmarks (shown when ``show_mapping``)
        """
        # rerun >=0.23 移除了 set_time_sequence；用 set_time(..., sequence=...) 代替。
        rr.set_time("frame", sequence=frame_id)
        rr.log(
            "world/trajectory",
            rr.LineStrips3D(trajectory, colors=[TRAJECTORY_COLOR]),
            static=True,
        )
        if trajectory_slam:
            rr.log("world/trajectory_slam", rr.LineStrips3D(trajectory_slam), static=True)

        self._log_rig_pose(pose.rotation, pose.translation)

        for i in range(self.num_viz_cameras):
            self._log_observations(
                observations_main_cam[i], images[i], f"camera_{i}"
            )

        if self.show_gravity and gravity is not None:
            self._log_gravity(gravity)

        if self.show_mapping:
            self._log_map_landmarks(map_landmarks)
            self._log_loop_closure_poses(loop_closure_poses)

        rr.log("world/timestamp", rr.TextLog(str(timestamp)))

    def save(self, path: str) -> None:
        """Save the recorded rerun data to a .rrd file.

        Args:
            path: Output .rrd file path
        """
        if self.save_path is not None:
            # FileSink 已实时写盘；这里仅 flush 一次，footer 由进程退出时自动补写。
            if self._recording is not None:
                self._recording.flush()
            return
        rr.save(path)
