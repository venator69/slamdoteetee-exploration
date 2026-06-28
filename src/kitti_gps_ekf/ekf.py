"""Extended Kalman Filter for GPS + ORB-SLAM3 + IMU fusion."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


def _normalize_quaternion(quat: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quat)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


def _quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float64,
    )


def _quaternion_from_gyro(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    omega_quat = np.array([omega[0], omega[1], omega[2], 0.0], dtype=np.float64)
    q_dot = 0.5 * _quaternion_multiply(q, omega_quat)
    return _normalize_quaternion(q + dt * q_dot)


@dataclass
class EkfConfig:
    process_position_std: float = 0.05
    process_velocity_std: float = 0.5
    process_orientation_std: float = 0.01
    gps_position_std: float = 2.0
    slam_position_std: float = 0.1
    slam_orientation_std: float = 0.05
    initial_position_std: float = 1.0
    initial_velocity_std: float = 1.0
    initial_orientation_std: float = 0.1
    imu_enabled: bool = True


class PoseEkf:
    """
    EKF with state x = [px, py, pz, vx, vy, vz, qx, qy, qz, qw].

    Prediction uses constant-velocity or IMU propagation.
    Updates: GPS position (3D) and ORB-SLAM3 pose (7D).
    """

    STATE_DIM = 10

    def __init__(self, config: EkfConfig):
        self.config = config
        self.x = np.zeros(self.STATE_DIM, dtype=np.float64)
        self.x[6:10] = np.array([0.0, 0.0, 0.0, 1.0])
        self.P = np.eye(self.STATE_DIM, dtype=np.float64)
        self._set_initial_covariance()

    def _set_initial_covariance(self) -> None:
        cfg = self.config
        diag = np.array(
            [
                cfg.initial_position_std ** 2,
                cfg.initial_position_std ** 2,
                cfg.initial_position_std ** 2,
                cfg.initial_velocity_std ** 2,
                cfg.initial_velocity_std ** 2,
                cfg.initial_velocity_std ** 2,
                cfg.initial_orientation_std ** 2,
                cfg.initial_orientation_std ** 2,
                cfg.initial_orientation_std ** 2,
                cfg.initial_orientation_std ** 2,
            ],
            dtype=np.float64,
        )
        self.P = np.diag(diag)

    def initialize(self, position: np.ndarray, quaternion: np.ndarray, velocity: np.ndarray | None = None) -> None:
        self.x[0:3] = position
        if velocity is not None:
            self.x[3:6] = velocity
        else:
            self.x[3:6] = 0.0
        self.x[6:10] = _normalize_quaternion(quaternion)
        self._set_initial_covariance()

    def _process_noise(self, dt: float) -> np.ndarray:
        cfg = self.config
        q_pos = (cfg.process_position_std ** 2) * dt
        q_vel = (cfg.process_velocity_std ** 2) * dt
        q_ori = (cfg.process_orientation_std ** 2) * dt
        return np.diag(
            [q_pos, q_pos, q_pos, q_vel, q_vel, q_vel, q_ori, q_ori, q_ori, q_ori]
        )

    def predict(self, dt: float, accel_body: np.ndarray | None = None, gyro: np.ndarray | None = None) -> None:
        if dt <= 0.0:
            return

        position = self.x[0:3]
        velocity = self.x[3:6]
        quaternion = _normalize_quaternion(self.x[6:10])

        F = np.eye(self.STATE_DIM, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        if self.config.imu_enabled and accel_body is not None:
            rotation = Rotation.from_quat(quaternion).as_matrix()
            accel_world = rotation @ accel_body
            velocity = velocity + accel_world * dt
            position = position + velocity * dt + 0.5 * accel_world * dt * dt
        else:
            position = position + velocity * dt

        if self.config.imu_enabled and gyro is not None:
            quaternion = _quaternion_from_gyro(quaternion, gyro, dt)

        self.x[0:3] = position
        self.x[3:6] = velocity
        self.x[6:10] = quaternion
        self.P = F @ self.P @ F.T + self._process_noise(dt)

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        innovation = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[6:10] = _normalize_quaternion(self.x[6:10])
        I = np.eye(self.STATE_DIM)
        self.P = (I - K @ H) @ self.P

    def update_gps(self, position: np.ndarray) -> None:
        H = np.zeros((3, self.STATE_DIM), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        std = self.config.gps_position_std
        R = np.eye(3) * (std ** 2)
        self._update(position, H, R)

    def update_slam(self, position: np.ndarray, quaternion: np.ndarray) -> None:
        H = np.zeros((7, self.STATE_DIM), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        H[3, 6] = 1.0
        H[4, 7] = 1.0
        H[5, 8] = 1.0
        H[6, 9] = 1.0

        z = np.concatenate([position, _normalize_quaternion(quaternion)])
        pos_std = self.config.slam_position_std
        ori_std = self.config.slam_orientation_std
        R = np.diag(
            [pos_std ** 2] * 3 + [ori_std ** 2] * 4
        )
        self._update(z, H, R)

    def get_state(self) -> dict[str, np.ndarray]:
        return {
            "position": self.x[0:3].copy(),
            "velocity": self.x[3:6].copy(),
            "quaternion": _normalize_quaternion(self.x[6:10].copy()),
        }


def run_ekf_fusion(
    timestamps: np.ndarray,
    slam_positions: np.ndarray,
    slam_quaternions: np.ndarray,
    gps_positions: np.ndarray,
    gps_mask: np.ndarray,
    imu_accel: np.ndarray | None = None,
    imu_gyro: np.ndarray | None = None,
    config: EkfConfig | None = None,
) -> dict[str, np.ndarray]:
    """Run EKF fusion over synchronized SLAM and sparse GPS measurements."""
    if config is None:
        config = EkfConfig()

    ekf = PoseEkf(config)
    ekf.initialize(slam_positions[0], slam_quaternions[0])

    fused_positions = []
    fused_quaternions = []
    fused_velocities = []

    for idx in range(len(timestamps)):
        if idx > 0:
            dt = timestamps[idx] - timestamps[idx - 1]
            accel = imu_accel[idx] if imu_accel is not None else None
            gyro = imu_gyro[idx] if imu_gyro is not None else None
            ekf.predict(dt, accel_body=accel, gyro=gyro)

        ekf.update_slam(slam_positions[idx], slam_quaternions[idx])
        if gps_mask[idx]:
            ekf.update_gps(gps_positions[idx])

        state = ekf.get_state()
        fused_positions.append(state["position"])
        fused_quaternions.append(state["quaternion"])
        fused_velocities.append(state["velocity"])

    return {
        "timestamp": timestamps,
        "position": np.asarray(fused_positions),
        "quaternion": np.asarray(fused_quaternions),
        "velocity": np.asarray(fused_velocities),
    }
