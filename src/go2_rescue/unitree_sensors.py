"""Optional native SDK state/camera adapter, with a hardware-specific lidar plugin.

There is deliberately no guessed DDS lidar topic or binary point layout. Supply
the actual scanner driver as module:factory(config) -> iterable[PointCloudPacket].
The driver must run on this computer and timestamp acquisition in time.monotonic.
"""
from __future__ import annotations

from dataclasses import dataclass
import importlib
import threading
import time

import cv2
import numpy as np

from .config import Config
from .geometry import rot2
from .types import Frame, Velocity


@dataclass
class PointCloudPacket:
    """Acquisition stamp and scanner-native XYZ before body_from_lidar."""
    timestamp: float
    points: np.ndarray
    point_times: np.ndarray | None = None


def native_frames(config: Config, provider: str, velocity_frame: str):
    """SDK and scanner access is lazy and optional; no motion commands here.

    velocity_frame must be explicitly chosen after inspecting the firmware's
    SportModeState velocity contract. 'imu_world' rotates XY with IMU yaw.
    """
    if not config.network_interface:
        raise ValueError("Native SDK sensor access requires network_interface")
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.go2.video.video_client import VideoClient
    from .sdk import initialize
    initialize(config.network_interface)
    module, factory = provider.split(":", 1)
    lidar = getattr(importlib.import_module(module), factory)(config)
    state, lock = {}, threading.Lock()

    def callback(message):
        with lock:
            state.update(timestamp=time.monotonic(), quaternion=np.asarray(message.imu_state.quaternion, float),
                         velocity=np.asarray(message.velocity, float), yaw_speed=float(message.yaw_speed))

    subscription = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    subscription.Init(callback, 1)
    camera = VideoClient()
    camera.SetTimeout(.15)
    camera.Init()
    try:
        for packet in lidar:
            if not isinstance(packet, PointCloudPacket):
                raise ValueError("Lidar driver must yield PointCloudPacket")
            with lock:
                snapshot = dict(state)
            if not snapshot:
                raise RuntimeError("No SportModeState received; verify model, firmware, topic and network")
            delta = abs(packet.timestamp - snapshot["timestamp"])
            if delta > config.max_sensor_age:
                raise RuntimeError("Native IMU/state and lidar are not synchronized")
            velocity = snapshot["velocity"][:2]
            if velocity_frame == "imu_world":
                from scipy.spatial.transform import Rotation
                yaw = Rotation.from_quat(np.roll(snapshot["quaternion"], -1)).as_euler("xyz")[2]
                velocity = rot2(-yaw) @ velocity
            began = time.monotonic()
            code, data = camera.GetImageSample()
            image = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR) if code == 0 else None
            # No exposure timestamp is exposed by GetImageSample. Timestamp the
            # request conservatively, and require measured latency at deployment.
            image_age = max(0., time.monotonic() - began)
            transform = np.asarray(config.body_from_lidar)
            points = np.asarray(packet.points) @ transform[:3, :3].T + transform[:3, 3]
            yield Frame(packet.timestamp, points, snapshot["quaternion"],
                        Velocity(float(velocity[0]), float(velocity[1]), snapshot["yaw_speed"]), image,
                        packet.point_times, delta, image_age)
    finally:
        if hasattr(lidar, "close"):
            lidar.close()
        subscription.Close()
