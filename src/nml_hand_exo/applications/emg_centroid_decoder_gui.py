from __future__ import annotations

import sys
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from nml_hand_exo.applications._emg_lsl_helpers import (
    DARK_STYLE,
    ChunkBuffer,
    EmgStreamWorker,
    _adaptive_bandpass_limits,
    _parse_int_list,
    _preprocess_window,
    _select_channels,
)
from nml_hand_exo.processing._features import compute_rms

BIN_SIZE_DEG = 5.0
N_BINS = int(360 / BIN_SIZE_DEG)  # 72 bins


def _angle_to_bin(angle_deg: float) -> int:
    """Map any angle (−180 … +180 or 0 … 360) to a bin index 0 … N_BINS−1."""
    return int(angle_deg % 360.0 / BIN_SIZE_DEG) % N_BINS


def _bin_center_deg(bin_idx: int) -> float:
    return bin_idx * BIN_SIZE_DEG - 180.0


def _compute_roll_deg(accel_xyz: np.ndarray) -> float:
    """Roll around the forearm long axis from a 3-element accelerometer vector.

    Returns angle in [−180, +180] degrees representing supination/pronation.
    Works even during slow movement because gravity dominates.
    """
    ax, ay, az = float(accel_xyz[0]), float(accel_xyz[1]), float(accel_xyz[2])
    return float(np.degrees(np.arctan2(ay, az)))


def _compute_roll_from_mag_deg(mag_xyz: np.ndarray) -> float:
    """Approximate forearm roll from magnetometer Y/Z projection."""
    mx, my, mz = float(mag_xyz[0]), float(mag_xyz[1]), float(mag_xyz[2])
    del mx  # not needed for this simple roll proxy
    return float(np.degrees(np.arctan2(my, mz)))


def _blend_angles_deg(a_deg: float, b_deg: float, w_b: float) -> float:
    """Circular blend of two angles in degrees."""
    w_b = float(np.clip(w_b, 0.0, 1.0))
    w_a = 1.0 - w_b
    a = np.deg2rad(a_deg)
    b = np.deg2rad(b_deg)
    x = w_a * np.cos(a) + w_b * np.cos(b)
    y = w_a * np.sin(a) + w_b * np.sin(b)
    return float(np.degrees(np.arctan2(y, x)))


CLASS_ORDER = ("rest", "flex", "extend")
CLASS_COLORS = {
    "rest": "#aaaaaa",
    "flex": "#27ae60",
    "extend": "#c0392b",
}
CLASS_LABELS = {
    "rest": "rest",
    "flex": "close",
    "extend": "open",
}
PLOT_TICK_SEC = 0.05
PLOT_HISTORY_LIMIT = 200


def _common_mode_remove(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    return x - float(np.mean(x))


# ── Riemannian geometry helpers ──────────────────────────────────────────────

def _compute_covariance(window: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Regularized spatial covariance from (n_channels, n_samples) window."""
    n_ch, n_s = window.shape
    if n_s < 2:
        return np.eye(n_ch, dtype=np.float64)
    xc = window - window.mean(axis=1, keepdims=True)
    C = (xc @ xc.T) / (n_s - 1)
    C = (C + C.T) / 2  # symmetrize
    C += (eps * np.trace(C) / n_ch) * np.eye(n_ch)
    return C


def _riemann_mean(covs: list[np.ndarray], max_iter: int = 50, tol: float = 1e-8) -> np.ndarray:
    """Fréchet (Riemannian) mean of SPD matrices via gradient descent on the manifold."""
    from scipy.linalg import expm, logm, sqrtm, inv as la_inv  # lazy import
    M = np.mean(covs, axis=0)
    for _ in range(max_iter):
        M_sqrt = np.real(sqrtm(M))
        M_sqrt_inv = np.real(la_inv(M_sqrt))
        J = np.mean(
            [np.real(logm(M_sqrt_inv @ C @ M_sqrt_inv)) for C in covs], axis=0
        )
        if np.linalg.norm(J, "fro") < tol:
            break
        M = M_sqrt @ np.real(expm(J)) @ M_sqrt
    return (M + M.T) / 2


def _tangent_project(C: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Project SPD matrix C to tangent space at ref, return vectorized upper triangle.

    Uses the map: S = ref^{1/2} log(ref^{-1/2} C ref^{-1/2}) ref^{1/2}
    Off-diagonal elements scaled by √2 so the inner product is preserved.
    For 8 channels → 36-dimensional feature vector.
    """
    from scipy.linalg import logm, sqrtm, inv as la_inv
    ref_sqrt = np.real(sqrtm(ref))
    ref_sqrt_inv = np.real(la_inv(ref_sqrt))
    S = ref_sqrt @ np.real(logm(ref_sqrt_inv @ C @ ref_sqrt_inv)) @ ref_sqrt
    n = S.shape[0]
    rows, cols = np.triu_indices(n)
    vec = S[rows, cols].copy()
    vec[rows != cols] *= np.sqrt(2)
    return vec


@dataclass
class RiemannianFeatureExtractor:
    """Fits a Riemannian reference point and projects covariance matrices to tangent space.

    The reference point is the Riemannian mean of all training covariance matrices
    (pooled across classes).  At runtime, new covariance matrices are projected onto
    the tangent plane at this reference, giving a Euclidean feature vector that is
    invariant to electrode shift and robust to fatigue drift.
    """
    ref_mean: np.ndarray | None = None

    def fit(self, cov_list: list[np.ndarray]) -> None:
        self.ref_mean = _riemann_mean(cov_list)

    def transform(self, cov: np.ndarray) -> np.ndarray:
        if self.ref_mean is None:
            raise RuntimeError("RiemannianFeatureExtractor has not been fit")
        return _tangent_project(cov, self.ref_mean)

    @property
    def feature_dim(self) -> int:
        if self.ref_mean is None:
            return 0
        n = self.ref_mean.shape[0]
        return n * (n + 1) // 2


def _padded_2d(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    if x.ndim == 1:
        x = x.reshape(1, -1)
    if x.shape[1] >= 2:
        return x[:, :2]
    return np.hstack([x, np.zeros((x.shape[0], 1), dtype=np.float64)])


@dataclass
class CentroidDirectionDecoder:
    rest_centroid: np.ndarray | None = None
    flex_centroid: np.ndarray | None = None
    extend_centroid: np.ndarray | None = None
    direction: np.ndarray | None = None
    scale: float = 1.0
    rest_gate: float = 0.0
    pca_mean: np.ndarray | None = None
    pca_basis: np.ndarray | None = None
    class_counts: dict[str, int] = field(default_factory=dict)
    pairwise_distances: dict[str, float] = field(default_factory=dict)
    class_projection_means: dict[str, float] = field(default_factory=dict)
    class_projection_stds: dict[str, float] = field(default_factory=dict)
    fisher_ratio: float = 0.0

    def fit(self, samples_by_class: dict[str, list[np.ndarray]]):
        arrays: dict[str, np.ndarray] = {}
        for name in CLASS_ORDER:
            samples = samples_by_class.get(name, [])
            if len(samples) < 3:
                raise ValueError(f"Need at least 3 samples for {name}")
            arrays[name] = np.vstack(samples).astype(np.float64)

        self.class_counts = {name: int(arrays[name].shape[0]) for name in CLASS_ORDER}
        self.rest_centroid = np.mean(arrays["rest"], axis=0)
        self.flex_centroid = np.mean(arrays["flex"], axis=0)
        self.extend_centroid = np.mean(arrays["extend"], axis=0)

        raw_direction = self.flex_centroid - self.extend_centroid
        direction_norm = float(np.linalg.norm(raw_direction))
        if direction_norm < 1e-8:
            raise ValueError("Close and open centroids are too similar to define a direction")
        self.direction = raw_direction / direction_norm

        flex_proj = self.project_signed(arrays["flex"], apply_gate=False)
        extend_proj = self.project_signed(arrays["extend"], apply_gate=False)
        rest_proj = self.project_signed(arrays["rest"], apply_gate=False)
        scale_ref = max(
            float(np.percentile(np.abs(np.concatenate([flex_proj, extend_proj])), 95)),
            1e-6,
        )
        self.scale = 1.0 / scale_ref

        rest_residual = np.linalg.norm(arrays["rest"] - self.rest_centroid, axis=1)
        self.rest_gate = max(float(np.percentile(rest_residual, 95)), 1e-6)

        self.class_projection_means = {
            "rest": float(np.mean(rest_proj)),
            "flex": float(np.mean(flex_proj)),
            "extend": float(np.mean(extend_proj)),
        }
        self.class_projection_stds = {
            "rest": float(np.std(rest_proj)),
            "flex": float(np.std(flex_proj)),
            "extend": float(np.std(extend_proj)),
        }
        self.pairwise_distances = {
            "rest-flex": float(np.linalg.norm(self.rest_centroid - self.flex_centroid)),
            "rest-extend": float(np.linalg.norm(self.rest_centroid - self.extend_centroid)),
            "flex-extend": float(np.linalg.norm(self.flex_centroid - self.extend_centroid)),
        }
        denom = self.class_projection_stds["flex"] + self.class_projection_stds["extend"] + 1e-6
        self.fisher_ratio = abs(self.class_projection_means["flex"] - self.class_projection_means["extend"]) / denom

        all_samples = np.vstack([arrays[name] for name in CLASS_ORDER])
        self.pca_mean = np.mean(all_samples, axis=0)
        xc = all_samples - self.pca_mean
        _, _, vt = np.linalg.svd(xc, full_matrices=False)
        k = max(1, min(2, vt.shape[0]))
        self.pca_basis = vt[:k]

    def project_signed(self, x: np.ndarray, apply_gate: bool = True) -> np.ndarray:
        if self.direction is None or self.rest_centroid is None:
            raise RuntimeError("Decoder has not been fit")
        x = np.asarray(x, dtype=np.float64)
        if x.ndim == 1:
            x = x.reshape(1, -1)
        signed = (x - self.rest_centroid) @ self.direction
        signed = signed * self.scale
        if apply_gate and self.rest_gate > 1e-6:
            residual = np.linalg.norm(x - self.rest_centroid, axis=1)
            gate = np.clip(residual / self.rest_gate, 0.0, 1.0)
            signed = signed * gate
        return np.clip(signed, -1.0, 1.0)

    def project_2d(self, x: np.ndarray) -> np.ndarray:
        if self.pca_mean is None or self.pca_basis is None:
            raise RuntimeError("PCA view has not been fit")
        x = np.asarray(x, dtype=np.float64)
        if x.ndim == 1:
            x = x.reshape(1, -1)
        proj = (x - self.pca_mean) @ self.pca_basis.T
        return _padded_2d(proj)

    def centroid_2d(self, name: str) -> np.ndarray:
        lookup = {
            "rest": self.rest_centroid,
            "flex": self.flex_centroid,
            "extend": self.extend_centroid,
        }
        centroid = lookup[name]
        if centroid is None:
            raise RuntimeError("Centroid is unavailable")
        return self.project_2d(centroid.reshape(1, -1))[0]


@dataclass
class OrientationGatedDecoder:
    """Wraps per-orientation-bin CentroidDirectionDecoders.

    During calibration, each EMG feature window is tagged with a forearm roll
    angle from the IMU.  After fitting, one decoder is trained per 5-degree bin
    that has enough data.  At runtime the nearest populated bin is selected.
    A global fallback decoder (trained on all samples) is always available.
    """

    bin_size_deg: float = BIN_SIZE_DEG
    global_decoder: CentroidDirectionDecoder | None = None
    bin_decoders: dict[int, CentroidDirectionDecoder] = field(default_factory=dict)
    bin_counts: dict[int, dict[str, int]] = field(default_factory=dict)
    n_fitted_bins: int = 0

    def fit(
        self,
        class_samples: dict[str, list[np.ndarray]],
        class_orientations: dict[str, list[float | None]],
    ) -> None:
        # Always fit the global decoder first (orientation-agnostic fallback)
        self.global_decoder = CentroidDirectionDecoder()
        self.global_decoder.fit(class_samples)

        # Group samples by orientation bin
        bins_data: dict[int, dict[str, list[np.ndarray]]] = {}
        for name in CLASS_ORDER:
            for feat, angle in zip(class_samples[name], class_orientations[name]):
                if angle is None:
                    continue
                b = _angle_to_bin(angle)
                if b not in bins_data:
                    bins_data[b] = {n: [] for n in CLASS_ORDER}
                bins_data[b][name].append(feat)

        self.bin_decoders = {}
        self.bin_counts = {}
        for b, data in bins_data.items():
            counts = {name: len(data[name]) for name in CLASS_ORDER}
            self.bin_counts[b] = counts
            if all(counts[name] >= 3 for name in CLASS_ORDER):
                try:
                    dec = CentroidDirectionDecoder()
                    dec.fit(data)
                    self.bin_decoders[b] = dec
                except Exception:
                    pass
        self.n_fitted_bins = len(self.bin_decoders)

    def get_decoder(self, angle_deg: float | None) -> CentroidDirectionDecoder:
        """Return the best decoder for the current orientation angle."""
        if not self.bin_decoders or angle_deg is None:
            return self.global_decoder  # type: ignore[return-value]
        target = _angle_to_bin(angle_deg)
        if target in self.bin_decoders:
            return self.bin_decoders[target]
        # circular nearest-bin search
        best = min(
            self.bin_decoders.keys(),
            key=lambda b: min(abs(b - target), N_BINS - abs(b - target)),
        )
        return self.bin_decoders[best]

    def active_bin_label(self, angle_deg: float | None) -> str:
        if angle_deg is None:
            return "global (no IMU)"
        target = _angle_to_bin(angle_deg)
        if target in self.bin_decoders:
            return f"bin {target} ({_bin_center_deg(target):.0f}°)"
        if not self.bin_decoders:
            return "global (no bins)"
        best = min(
            self.bin_decoders.keys(),
            key=lambda b: min(abs(b - target), N_BINS - abs(b - target)),
        )
        return f"nearest bin {best} ({_bin_center_deg(best):.0f}°)"


class DecoderFitWorker(QThread):
    """Background worker for decoder fitting to keep the UI responsive."""

    fit_ok = pyqtSignal(object)
    fit_failed = pyqtSignal(str)

    def __init__(
        self,
        class_samples: dict[str, list[np.ndarray]],
        class_covs: dict[str, list[np.ndarray]],
        class_orientations: dict[str, list[float | None]],
        use_riemann: bool,
        max_cov_per_class: int,
    ):
        super().__init__()
        self._class_samples = class_samples
        self._class_covs = class_covs
        self._class_orientations = class_orientations
        self._use_riemann = use_riemann
        self._max_cov_per_class = max(0, int(max_cov_per_class))

    @staticmethod
    def _subsample_covs(covs: list[np.ndarray], limit: int) -> list[np.ndarray]:
        if limit <= 0 or len(covs) <= limit:
            return covs
        idx = np.linspace(0, len(covs) - 1, num=limit, dtype=np.int32)
        return [covs[int(i)] for i in idx]

    @staticmethod
    def _subsample_indices(n: int, limit: int) -> list[int]:
        if limit <= 0 or n <= limit:
            return list(range(n))
        idx = np.linspace(0, n - 1, num=limit, dtype=np.int32)
        return [int(i) for i in idx.tolist()]

    def run(self):
        try:
            samples = self._class_samples
            extractor = None
            n_cov_used = 0

            if self._use_riemann:
                covs_by_class: dict[str, list[np.ndarray]] = {}
                orients_by_class: dict[str, list[float | None]] = {name: [] for name in CLASS_ORDER}
                for name in CLASS_ORDER:
                    src_covs = self._class_covs[name]
                    src_orients = self._class_orientations[name]
                    keep = self._subsample_indices(len(src_covs), self._max_cov_per_class)
                    covs_by_class[name] = [src_covs[i] for i in keep]
                    orients_by_class[name] = [src_orients[i] for i in keep if i < len(src_orients)]
                all_covs: list[np.ndarray] = []
                for name in CLASS_ORDER:
                    all_covs.extend(covs_by_class[name])
                n_cov_used = len(all_covs)
                if n_cov_used < 6:
                    raise RuntimeError(
                        "Need at least 6 covariance matrices across all classes. Please capture more data first."
                    )
                extractor = RiemannianFeatureExtractor()
                extractor.fit(all_covs)
                samples = {name: [] for name in CLASS_ORDER}
                for name in CLASS_ORDER:
                    samples[name] = [extractor.transform(c) for c in covs_by_class[name]]
                orientations = orients_by_class
            else:
                orientations = self._class_orientations

            gated = OrientationGatedDecoder()
            gated.fit(samples, orientations)
            self.fit_ok.emit(
                {
                    "gated": gated,
                    "extractor": extractor,
                    "samples": samples,
                    "n_cov_used": n_cov_used,
                }
            )
        except Exception as exc:
            self.fit_failed.emit(str(exc))


class EmgCentroidDecoderGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NML EMG Centroid Direction Decoder")

        # EMG stream
        self._lsl_worker: EmgStreamWorker | None = None
        self._buffer = ChunkBuffer(2000)
        self._stream_meta: dict[str, object] = {}

        # IMU stream
        self._imu_worker: EmgStreamWorker | None = None
        self._imu_buffer = ChunkBuffer(500)
        self._imu_meta: dict[str, object] = {}
        self._current_roll_deg: float | None = None  # live forearm roll angle

        # Captured class data (feature vectors), raw covariances, per-sample orientation,
        # and optional per-sample 9-axis IMU vectors (ax,ay,az,gx,gy,gz,mx,my,mz).
        self._class_samples: dict[str, list[np.ndarray]] = {name: [] for name in CLASS_ORDER}
        self._class_covs: dict[str, list[np.ndarray]] = {name: [] for name in CLASS_ORDER}
        self._class_orientations: dict[str, list[float | None]] = {name: [] for name in CLASS_ORDER}
        self._class_imu9: dict[str, list[np.ndarray | None]] = {name: [] for name in CLASS_ORDER}

        self._capture_state: str | None = None
        self._capture_started_at: float | None = None
        self._fit_worker: DecoderFitWorker | None = None

        # Decoder: global or orientation-gated
        self._gated_decoder: OrientationGatedDecoder | None = None
        self._riemann_extractor: RiemannianFeatureExtractor | None = None
        self._live_feature: np.ndarray | None = None
        self._live_signed_history: deque[float] = deque(maxlen=PLOT_HISTORY_LIMIT)
        self._live_time_history: deque[float] = deque(maxlen=PLOT_HISTORY_LIMIT)

        # Adaptive rest centroid update state
        self._adaptive_rest_idle_since: float | None = None

        # LSL outlet (User Intent publisher)
        self._lsl_outlet: object | None = None  # pylsl.StreamOutlet or None
        self._publish_count: int = 0
        self._publish_start_time: float | None = None
        self._last_published_value: float = 0.0

        pg.setConfigOptions(antialias=True)
        pg.setConfigOption("background", "#111111")
        pg.setConfigOption("foreground", "#e0e0e0")

        self._build_ui()
        self._tick_timer = QTimer(self)
        self._tick_timer.setInterval(int(PLOT_TICK_SEC * 1000))
        self._tick_timer.timeout.connect(self._tick)

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        # Header bar
        header = QWidget()
        header.setStyleSheet("background-color: #0d0d0d; border-bottom: 2px solid #c0392b;")
        header_layout = QVBoxLayout(header)
        header_layout.setContentsMargins(20, 10, 20, 10)
        header_layout.setSpacing(2)
        title = QLabel("NML  ·  EMG Intent Decoder")
        title.setStyleSheet("font-size: 20px; font-weight: bold; color: #e0e0e0; background: transparent; border: none;")
        note = QLabel(
            "Centroid-direction decoder  ·  Common-mode-removed RMS  ·  Orientation-gated  ·  LSL publisher"
        )
        note.setStyleSheet("color: #555555; font-size: 11px; background: transparent; border: none;")
        header_layout.addWidget(title)
        header_layout.addWidget(note)
        outer.addWidget(header)

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        outer.addWidget(self.tabs)

        setup = QWidget()
        setup_layout = QVBoxLayout(setup)
        setup_layout.setContentsMargins(16, 12, 16, 12)
        setup_layout.setSpacing(8)
        self._build_connection_box(setup_layout)
        self._build_imu_box(setup_layout)
        self._build_capture_box(setup_layout)
        setup_layout.addStretch()
        self.tabs.addTab(setup, "Setup")

        viz = QWidget()
        viz_layout = QVBoxLayout(viz)
        viz_layout.setContentsMargins(16, 12, 16, 12)
        viz_layout.setSpacing(8)
        self._build_visualization_box(viz_layout)
        viz_layout.addStretch()
        self.tabs.addTab(viz, "Visualization")

        decoder = QWidget()
        decoder_layout = QVBoxLayout(decoder)
        decoder_layout.setContentsMargins(16, 12, 16, 12)
        decoder_layout.setSpacing(8)
        self._build_decoder_box(decoder_layout)
        decoder_layout.addStretch()
        self.tabs.addTab(decoder, "Decoder")

        intent = QWidget()
        intent_layout = QVBoxLayout(intent)
        intent_layout.setContentsMargins(16, 12, 16, 12)
        intent_layout.setSpacing(8)
        self._build_user_intent_box(intent_layout)
        intent_layout.addStretch()
        self.tabs.addTab(intent, "User Intent")

        log = QWidget()
        log_layout = QVBoxLayout(log)
        log_layout.setContentsMargins(16, 12, 16, 12)
        self._build_log_box(log_layout)
        self.tabs.addTab(log, "Log")

    def _build_connection_box(self, parent: QVBoxLayout):
        box = QGroupBox("Connection")
        grid = QGridLayout(box)

        self.stream_type_combo = QComboBox()
        self.stream_type_combo.addItems(["EMG", "Custom"])
        self.stream_name_edit = QLineEdit()
        self.stream_name_edit.setPlaceholderText("Optional stream name filter")
        self.stream_combo = QComboBox()
        self.stream_combo.setMinimumWidth(240)
        self.stream_combo.currentIndexChanged.connect(self._sync_stream_selection)
        self.refresh_btn = QPushButton("Refresh Streams")
        self.refresh_btn.clicked.connect(self._refresh_streams)
        self.connect_btn = QPushButton("Connect LSL")
        self.connect_btn.setProperty("accent", True)
        self.connect_btn.clicked.connect(self._toggle_lsl)
        self.status_label = QLabel("LSL disconnected")
        self.status_label.setStyleSheet("color: #888888;")

        grid.addWidget(QLabel("Stream type"), 0, 0)
        grid.addWidget(self.stream_type_combo, 0, 1)
        grid.addWidget(QLabel("Stream name"), 0, 2)
        grid.addWidget(self.stream_name_edit, 0, 3)
        grid.addWidget(QLabel("Available streams"), 1, 0)
        grid.addWidget(self.stream_combo, 1, 1, 1, 2)
        grid.addWidget(self.refresh_btn, 1, 3)
        grid.addWidget(self.connect_btn, 2, 0)
        grid.addWidget(self.status_label, 2, 1, 1, 3)
        parent.addWidget(box)

    def _build_imu_box(self, parent: QVBoxLayout):
        box = QGroupBox("IMU — forearm orientation (optional, for orientation-gated decoder)")
        grid = QGridLayout(box)

        self.imu_stream_type_combo = QComboBox()
        self.imu_stream_type_combo.addItems(["IMU", "Custom"])
        self.imu_stream_name_edit = QLineEdit()
        self.imu_stream_name_edit.setPlaceholderText("Optional IMU stream name filter")
        self.imu_stream_combo = QComboBox()
        self.imu_stream_combo.setMinimumWidth(200)
        self.imu_stream_combo.currentIndexChanged.connect(self._sync_imu_stream_selection)
        self.imu_refresh_btn = QPushButton("Refresh IMU")
        self.imu_refresh_btn.clicked.connect(self._refresh_imu_streams)
        self.imu_connect_btn = QPushButton("Connect IMU")
        self.imu_connect_btn.clicked.connect(self._toggle_imu)
        self.imu_status_label = QLabel("IMU disconnected")
        self.imu_status_label.setStyleSheet("color: #888888;")

        # IMU channel mappings (MindRove 9-axis default: acc=0,1,2 gyro=3,4,5 mag=6,7,8)
        self.imu_accel_channels_edit = QLineEdit("0,1,2")
        self.imu_accel_channels_edit.setToolTip(
            "Comma-separated channel indices for ax, ay, az in the IMU stream.\n"
            "MindRove default: 0=ax, 1=ay, 2=az"
        )
        self.imu_gyro_channels_edit = QLineEdit("3,4,5")
        self.imu_gyro_channels_edit.setToolTip(
            "Comma-separated channel indices for gx, gy, gz.\n"
            "MindRove default: 3=gx, 4=gy, 5=gz"
        )
        self.imu_mag_channels_edit = QLineEdit("6,7,8")
        self.imu_mag_channels_edit.setToolTip(
            "Comma-separated channel indices for mx, my, mz.\n"
            "MindRove 9-axis default: 6=mx, 7=my, 8=mz"
        )
        self.imu_roll_source_combo = QComboBox()
        self.imu_roll_source_combo.addItems(["Accel only", "Accel + Mag (fused)", "Mag only"])
        self.imu_roll_source_combo.setCurrentText("Accel only")
        self.imu_roll_source_combo.setToolTip(
            "Roll estimator for orientation bins:\n"
            "Accel only: robust and drift-free for slow movements.\n"
            "Accel + Mag: circular blend using magnetometer if available.\n"
            "Mag only: useful for testing full 9-axis streams.\n"
            "Note: all mapped 9-axis IMU values are saved per captured sample."
        )
        self.imu_angle_label = QLabel("Roll: — °  |  Bin: —  |  Source: —")
        self.imu_angle_label.setStyleSheet("color: #f1c40f; font-weight: bold;")

        grid.addWidget(QLabel("Stream type"), 0, 0)
        grid.addWidget(self.imu_stream_type_combo, 0, 1)
        grid.addWidget(QLabel("Stream name"), 0, 2)
        grid.addWidget(self.imu_stream_name_edit, 0, 3)
        grid.addWidget(QLabel("Available"), 1, 0)
        grid.addWidget(self.imu_stream_combo, 1, 1, 1, 2)
        grid.addWidget(self.imu_refresh_btn, 1, 3)
        grid.addWidget(self.imu_connect_btn, 2, 0)
        grid.addWidget(self.imu_status_label, 2, 1, 1, 3)
        grid.addWidget(QLabel("Accel channels"), 3, 0)
        grid.addWidget(self.imu_accel_channels_edit, 3, 1)
        grid.addWidget(QLabel("Gyro channels"), 3, 2)
        grid.addWidget(self.imu_gyro_channels_edit, 3, 3)
        grid.addWidget(QLabel("Mag channels"), 4, 0)
        grid.addWidget(self.imu_mag_channels_edit, 4, 1)
        grid.addWidget(QLabel("Roll source"), 4, 2)
        grid.addWidget(self.imu_roll_source_combo, 4, 3)
        grid.addWidget(self.imu_angle_label, 5, 0, 1, 4)
        parent.addWidget(box)

    def _build_capture_box(self, parent: QVBoxLayout):
        box = QGroupBox("Class capture and decoder fit")
        grid = QGridLayout(box)

        self.channels_edit = QLineEdit("all")
        self.channels_edit.setPlaceholderText("all or 0,1,2")
        self.window_spin = QDoubleSpinBox()
        self.window_spin.setRange(0.05, 2.0)
        self.window_spin.setSingleStep(0.05)
        self.window_spin.setValue(0.25)
        self.capture_sec_spin = QDoubleSpinBox()
        self.capture_sec_spin.setRange(1.0, 120.0)
        self.capture_sec_spin.setDecimals(1)
        self.capture_sec_spin.setSingleStep(5.0)
        self.capture_sec_spin.setValue(10.0)
        self.capture_sec_spin.setToolTip(
            "How long to record each class capture.\n"
            "Use longer (30–60 s) sweeping captures when IMU orientation gating is enabled\n"
            "so you cover many wrist orientations per class."
        )
        self.preprocessing_combo = QComboBox()
        self.preprocessing_combo.addItems(
            ["Raw", "Bandpass + notch", "Rectified envelope", "RMS envelope", "Z-score normalized"]
        )
        self.preprocessing_combo.setCurrentText("Bandpass + notch")

        self.feature_mode_combo = QComboBox()
        self.feature_mode_combo.addItems([
            "RMS + common-mode removal",
            "Riemannian (covariance → tangent space)",
        ])
        self.feature_mode_combo.setToolTip(
            "RMS+CMR: fast, interpretable — good starting point.\n"
            "Riemannian: covariance matrix projected to tangent space (36D for 8 ch).\n"
            "  More robust to electrode shift, fatigue, and wrist orientation.\n"
            "  Both covariance and RMS are stored during capture — you can switch and re-fit."
        )
        self.capture_append_chk = QCheckBox("Append captures")
        self.capture_append_chk.setChecked(True)
        self.capture_append_chk.setToolTip(
            "When enabled, new captures are appended to existing class data.\n"
            "Disable to replace that class with a fresh capture."
        )
        self.riemann_max_cov_spin = QSpinBox()
        self.riemann_max_cov_spin.setRange(50, 5000)
        self.riemann_max_cov_spin.setSingleStep(50)
        self.riemann_max_cov_spin.setValue(400)
        self.riemann_max_cov_spin.setToolTip(
            "Maximum covariance windows per class used for Riemannian fit.\n"
            "Lower values fit faster and reduce UI wait time."
        )

        self.capture_rest_btn = QPushButton("Capture Rest")
        self.capture_rest_btn.clicked.connect(lambda: self._start_capture("rest"))
        self.capture_flex_btn = QPushButton("Capture Close")
        self.capture_flex_btn.clicked.connect(lambda: self._start_capture("flex"))
        self.capture_extend_btn = QPushButton("Capture Open")
        self.capture_extend_btn.clicked.connect(lambda: self._start_capture("extend"))
        self.clear_btn = QPushButton("Clear Classes")
        self.clear_btn.clicked.connect(self._clear_classes)
        self.fit_btn = QPushButton("Fit Centroid Decoder")
        self.fit_btn.setProperty("accent", True)
        self.fit_btn.clicked.connect(self._fit_decoder)
        self.save_btn = QPushButton("Save Session…")
        self.save_btn.clicked.connect(self._save_session)
        self.save_btn.setToolTip("Save captured class data and fitted decoder to a .npz file")
        self.load_btn = QPushButton("Load Session…")
        self.load_btn.clicked.connect(self._load_session)
        self.load_btn.setToolTip("Load a previously saved .npz session (restores samples and decoder)")

        self.capture_status_label = QLabel("Capture: idle")
        self.capture_status_label.setWordWrap(True)
        self.capture_status_label.setStyleSheet("color: #aaaaaa;")
        self.class_count_label = QLabel("Samples | rest=0 close=0 open=0")
        self.class_count_label.setStyleSheet("color: #aaaaaa;")
        self.orientation_coverage_label = QLabel("Orientation bins: no IMU connected")
        self.orientation_coverage_label.setWordWrap(True)
        self.orientation_coverage_label.setStyleSheet("color: #4da3ff;")
        self.decoder_status_label = QLabel("Decoder: not fit")
        self.decoder_status_label.setWordWrap(True)
        self.decoder_status_label.setStyleSheet("color: #888888; font-weight: bold;")
        self.fit_progress_label = QLabel("Fit progress: idle")
        self.fit_progress_label.setWordWrap(True)
        self.fit_progress_label.setStyleSheet("color: #888888;")
        self.filter_label = QLabel("Adaptive bandpass: waiting for stream")
        self.filter_label.setStyleSheet("color: #aaaaaa;")

        grid.addWidget(QLabel("Channels"), 0, 0)
        grid.addWidget(self.channels_edit, 0, 1)
        grid.addWidget(QLabel("Window (s)"), 0, 2)
        grid.addWidget(self.window_spin, 0, 3)
        grid.addWidget(QLabel("Capture (s)"), 1, 0)
        grid.addWidget(self.capture_sec_spin, 1, 1)
        grid.addWidget(QLabel("Preprocessing"), 1, 2)
        grid.addWidget(self.preprocessing_combo, 1, 3)
        grid.addWidget(QLabel("Feature mode"), 2, 0)
        grid.addWidget(self.feature_mode_combo, 2, 1, 1, 3)
        grid.addWidget(self.capture_append_chk, 3, 0, 1, 2)
        grid.addWidget(QLabel("Riemannian max cov/class"), 3, 2)
        grid.addWidget(self.riemann_max_cov_spin, 3, 3)
        grid.addWidget(self.capture_rest_btn, 4, 0)
        grid.addWidget(self.capture_flex_btn, 4, 1)
        grid.addWidget(self.capture_extend_btn, 4, 2)
        grid.addWidget(self.fit_btn, 4, 3)
        grid.addWidget(self.clear_btn, 5, 0)
        grid.addWidget(self.save_btn, 5, 1)
        grid.addWidget(self.load_btn, 5, 2)
        grid.addWidget(self.capture_status_label, 5, 3)
        grid.addWidget(self.class_count_label, 6, 0, 1, 4)
        grid.addWidget(self.orientation_coverage_label, 7, 0, 1, 4)
        grid.addWidget(self.decoder_status_label, 8, 0, 1, 4)
        grid.addWidget(self.fit_progress_label, 9, 0, 1, 4)
        grid.addWidget(self.filter_label, 10, 0, 1, 4)
        parent.addWidget(box)

    def _build_visualization_box(self, parent: QVBoxLayout):
        box = QGroupBox("Intent geometry (rest / close / open)")
        outer = QVBoxLayout(box)

        # ── Top row: 2-column grid (scatter | feature) ──────────────────────
        top_row = QHBoxLayout()

        # Left: PCA scatter
        scatter_col = QVBoxLayout()
        self.scatter_plot = pg.PlotWidget()
        self.scatter_plot.showGrid(x=True, y=True, alpha=0.25)
        self.scatter_plot.setLabel("bottom", "PC1")
        self.scatter_plot.setLabel("left", "PC2")
        self.scatter_plot.setMinimumHeight(200)
        self.class_scatter_items: dict[str, pg.PlotDataItem] = {}
        self.class_centroid_items: dict[str, pg.PlotDataItem] = {}
        for name in CLASS_ORDER:
            color = CLASS_COLORS[name]
            self.class_scatter_items[name] = self.scatter_plot.plot(
                pen=None, symbol="o", symbolSize=6,
                symbolBrush=color, symbolPen=pg.mkPen(color), name=name,
            )
            self.class_centroid_items[name] = self.scatter_plot.plot(
                pen=None, symbol="s", symbolSize=12,
                symbolBrush=color, symbolPen=pg.mkPen("#ffffff"),
            )
        self.live_scatter_item = self.scatter_plot.plot(
            pen=None, symbol="o", symbolSize=12,
            symbolBrush="#f1c40f", symbolPen=pg.mkPen("#ffffff"),
        )
        self.axis_line_item = self.scatter_plot.plot(
            pen=pg.mkPen("#4da3ff", width=2, style=Qt.DashLine)
        )
        scatter_col.addWidget(self.scatter_plot)
        top_row.addLayout(scatter_col)

        # Right: per-channel feature bar chart
        feature_col = QVBoxLayout()
        self.feature_plot = pg.PlotWidget()
        self.feature_plot.showGrid(x=True, y=True, alpha=0.25)
        self.feature_plot.setLabel("bottom", "Channel")
        self.feature_plot.setLabel("left", "CMR-RMS")
        self.feature_plot.setMinimumHeight(200)
        self.feature_curve_live = self.feature_plot.plot(
            pen=pg.mkPen("#f1c40f", width=3), symbol="o", symbolSize=7, symbolBrush="#f1c40f"
        )
        self.feature_curve_rest = self.feature_plot.plot(pen=pg.mkPen(CLASS_COLORS["rest"], width=2))
        self.feature_curve_flex = self.feature_plot.plot(pen=pg.mkPen(CLASS_COLORS["flex"], width=2))
        self.feature_curve_extend = self.feature_plot.plot(pen=pg.mkPen(CLASS_COLORS["extend"], width=2))
        feature_col.addWidget(self.feature_plot)
        top_row.addLayout(feature_col)

        outer.addLayout(top_row)

        # ── Bottom row: 2-column grid (separation | coverage) ───────────────
        bot_row = QHBoxLayout()

        sep_col = QVBoxLayout()
        self.separation_plot = pg.PlotWidget()
        self.separation_plot.showGrid(x=True, y=True, alpha=0.25)
        self.separation_plot.setLabel("bottom", "Pair")
        self.separation_plot.setLabel("left", "Centroid dist.")
        self.separation_plot.setMinimumHeight(150)
        self.separation_label = QLabel("Separability: -")
        self.separation_label.setWordWrap(True)
        self.separation_label.setStyleSheet("color: #aaaaaa;")
        sep_col.addWidget(self.separation_plot)
        sep_col.addWidget(self.separation_label)
        bot_row.addLayout(sep_col)

        cov_col = QVBoxLayout()
        self.coverage_plot = pg.PlotWidget()
        self.coverage_plot.showGrid(x=True, y=True, alpha=0.25)
        self.coverage_plot.setLabel("bottom", "Roll angle (°)")
        self.coverage_plot.setLabel("left", "Samples")
        self.coverage_plot.setMinimumHeight(150)
        self.coverage_plot.setXRange(-185, 185)
        self.coverage_live_line = pg.InfiniteLine(
            pos=0.0, angle=90, pen=pg.mkPen("#f1c40f", width=2)
        )
        self.coverage_plot.addItem(self.coverage_live_line)
        self.coverage_label = QLabel("Orientation coverage: no IMU data")
        self.coverage_label.setWordWrap(True)
        self.coverage_label.setStyleSheet("color: #4da3ff;")
        cov_col.addWidget(self.coverage_plot)
        cov_col.addWidget(self.coverage_label)
        bot_row.addLayout(cov_col)

        outer.addLayout(bot_row)
        parent.addWidget(box)

    def _build_decoder_box(self, parent: QVBoxLayout):
        box = QGroupBox("Signed 1D decoder")
        layout = QVBoxLayout(box)

        self.decoder_plot = pg.PlotWidget()
        self.decoder_plot.showGrid(x=True, y=True, alpha=0.25)
        self.decoder_plot.setLabel("bottom", "Time", units="s")
        self.decoder_plot.setLabel("left", "Decoder output")
        self.decoder_plot.setYRange(-1.05, 1.05)
        self.decoder_plot.addLine(y=0.0, pen=pg.mkPen("#555555", style=Qt.DashLine))
        self.decoder_curve = self.decoder_plot.plot(pen=pg.mkPen("#27ae60", width=3))
        self.decoder_plot.setMinimumHeight(220)
        layout.addWidget(self.decoder_plot)

        # 1-D strip / number-line projection plot
        self.strip_plot = pg.PlotWidget()
        self.strip_plot.showGrid(x=True, y=False, alpha=0.25)
        self.strip_plot.setLabel("bottom", "Decoder axis (−open  …  +close)")
        self.strip_plot.setXRange(-1.25, 1.25)
        self.strip_plot.setYRange(-1.75, 1.75)
        self.strip_plot.setMinimumHeight(220)
        # labelled y-ticks for each class row
        left_axis = self.strip_plot.getAxis("left")
        left_axis.setTicks([[(1.0, "close"), (0.0, "rest"), (-1.0, "open")]])
        # horizontal guide lines for each class row
        for y_val in (-1.0, 0.0, 1.0):
            self.strip_plot.addLine(y=y_val, pen=pg.mkPen("#333333", style=Qt.DashLine))
        # zero column
        self.strip_plot.addLine(x=0.0, pen=pg.mkPen("#555555", style=Qt.DashLine))
        # per-class scatter items (dots at jittered y within each row)
        _y_centers = {"rest": 0.0, "flex": 1.0, "extend": -1.0}
        self._strip_y_centers = _y_centers
        self.strip_scatter: dict[str, pg.ScatterPlotItem] = {}
        for name in CLASS_ORDER:
            color = CLASS_COLORS[name]
            item = pg.ScatterPlotItem(
                pen=pg.mkPen(None),
                brush=pg.mkBrush(color + "99"),  # semi-transparent
                size=7,
            )
            self.strip_plot.addItem(item)
            self.strip_scatter[name] = item
        # class mean markers (diamond symbols, full opacity)
        self.strip_mean_items: dict[str, pg.ScatterPlotItem] = {}
        for name in CLASS_ORDER:
            color = CLASS_COLORS[name]
            item = pg.ScatterPlotItem(
                pen=pg.mkPen("#ffffff", width=1),
                brush=pg.mkBrush(color),
                size=16,
                symbol="d",
            )
            self.strip_plot.addItem(item)
            self.strip_mean_items[name] = item
        # ±1 std error bar items (drawn as thin vertical line segments)
        self.strip_std_items: dict[str, pg.PlotDataItem] = {}
        for name in CLASS_ORDER:
            color = CLASS_COLORS[name]
            item = self.strip_plot.plot(pen=pg.mkPen(color, width=3))
            self.strip_std_items[name] = item
        # live yellow vertical line
        self.strip_live_line = pg.InfiniteLine(
            pos=0.0, angle=90, pen=pg.mkPen("#f1c40f", width=3)
        )
        self.strip_plot.addItem(self.strip_live_line)
        layout.addWidget(self.strip_plot)

        # Output tuning controls
        tuning_box = QGroupBox("Output tuning")
        tuning_grid = QGridLayout(tuning_box)

        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setRange(0.001, 10.0)
        self.gain_spin.setDecimals(3)
        self.gain_spin.setSingleStep(0.001)
        self.gain_spin.setValue(1.0)
        self.gain_spin.setToolTip(
            "Multiplies the final decoder output before clipping to ±1.\n"
            "< 1 = less sensitive (more effort to reach ±1)\n"
            "> 1 = more sensitive (small movements fill the range)"
        )

        self.rest_gate_scale_spin = QDoubleSpinBox()
        self.rest_gate_scale_spin.setRange(0.1, 5.0)
        self.rest_gate_scale_spin.setSingleStep(0.05)
        self.rest_gate_scale_spin.setValue(1.0)
        self.rest_gate_scale_spin.setToolTip(
            "Scales the fitted rest dead-zone radius.\n"
            "< 1 = smaller dead zone (output activates with less movement from rest)\n"
            "> 1 = larger dead zone (need more effort before output leaves zero)"
        )

        self.reset_tuning_btn = QPushButton("Reset")
        self.reset_tuning_btn.setFixedWidth(70)
        self.reset_tuning_btn.clicked.connect(self._reset_tuning)

        # Adaptive rest centroid controls
        self.adaptive_rest_chk = QCheckBox("Adaptive rest update")
        self.adaptive_rest_chk.setChecked(False)
        self.adaptive_rest_chk.setToolTip(
            "When enabled, the rest centroid slowly follows the current EMG signal\n"
            "while output is near zero (EMA update).  Adapts to electrode drift and\n"
            "forearm fatigue without re-capturing classes."
        )
        self.adaptive_rest_tau_spin = QDoubleSpinBox()
        self.adaptive_rest_tau_spin.setRange(5.0, 300.0)
        self.adaptive_rest_tau_spin.setSingleStep(5.0)
        self.adaptive_rest_tau_spin.setValue(60.0)
        self.adaptive_rest_tau_spin.setSuffix(" s")
        self.adaptive_rest_tau_spin.setToolTip(
            "Time constant (τ) for the exponential moving average update of the rest centroid.\n"
            "Larger τ → slower adaptation.  Default 60 s gives a ~1 min settling time."
        )

        tuning_grid.addWidget(QLabel("Output gain"), 0, 0)
        tuning_grid.addWidget(self.gain_spin, 0, 1)
        tuning_grid.addWidget(QLabel("Rest gate scale"), 0, 2)
        tuning_grid.addWidget(self.rest_gate_scale_spin, 0, 3)
        tuning_grid.addWidget(self.reset_tuning_btn, 0, 4)
        tuning_grid.addWidget(self.adaptive_rest_chk, 1, 0, 1, 2)
        tuning_grid.addWidget(QLabel("τ"), 1, 2)
        tuning_grid.addWidget(self.adaptive_rest_tau_spin, 1, 3)
        tuning_grid.addWidget(
            QLabel(
                "Gain < 1 → less sensitive   |   Gain > 1 → more sensitive   |   "
                "Gate scale > 1 → larger dead zone at rest"
            ),
            2, 0, 1, 5,
        )
        layout.addWidget(tuning_box)

        self.active_bin_label = QLabel("Active decoder: not fit")
        self.active_bin_label.setStyleSheet("color: #4da3ff; font-weight: bold;")
        layout.addWidget(self.active_bin_label)

        self.decoder_value_label = QLabel("Live decoder: -")
        self.decoder_value_label.setStyleSheet("color: #aaaaaa; font-weight: bold;")
        layout.addWidget(self.decoder_value_label)
        parent.addWidget(box)

    def _build_user_intent_box(self, parent: QVBoxLayout):
        # ── LSL outlet config ───────────────────────────────────────────
        stream_box = QGroupBox("LSL Output Stream")
        sgrid = QGridLayout(stream_box)

        self.intent_stream_name_edit = QLineEdit("UserIntent")
        self.intent_stream_type_edit = QLineEdit("UserIntent")
        self.intent_source_id_edit = QLineEdit("nml-emg-centroid-decoder")
        self.intent_source_id_edit.setToolTip("Unique source ID embedded in the stream info")

        self.intent_publish_btn = QPushButton("Start Publishing")
        self.intent_publish_btn.setProperty("accent", True)
        self.intent_publish_btn.clicked.connect(self._toggle_publish)

        self.intent_stream_status_label = QLabel("Not publishing")
        self.intent_stream_status_label.setStyleSheet("color: #666666; font-weight: bold;")

        sgrid.addWidget(QLabel("Stream name"), 0, 0)
        sgrid.addWidget(self.intent_stream_name_edit, 0, 1)
        sgrid.addWidget(QLabel("Stream type"), 0, 2)
        sgrid.addWidget(self.intent_stream_type_edit, 0, 3)
        sgrid.addWidget(QLabel("Source ID"), 1, 0)
        sgrid.addWidget(self.intent_source_id_edit, 1, 1)
        sgrid.addWidget(self.intent_publish_btn, 1, 2)
        sgrid.addWidget(self.intent_stream_status_label, 1, 3)
        sgrid.addWidget(
            QLabel("1 channel · float32 · rate follows decoder tick (~20 Hz)"),
            2, 0, 1, 4
        )
        parent.addWidget(stream_box)

        # ── Live gauge ──────────────────────────────────────────────────
        gauge_box = QGroupBox("Live User Intent")
        gauge_layout = QVBoxLayout(gauge_box)

        # Big numeric readout
        self.intent_big_label = QLabel("—")
        self.intent_big_label.setAlignment(Qt.AlignCenter)
        self.intent_big_label.setStyleSheet(
            "font-size: 72px; font-weight: bold; color: #e0e0e0; "
            "font-family: 'Cascadia Code', 'Consolas', monospace; letter-spacing: 4px;"
        )
        self.intent_big_label.setMinimumHeight(110)
        gauge_layout.addWidget(self.intent_big_label)

        # Horizontal VU-meter gauge
        self.intent_gauge = pg.PlotWidget()
        self.intent_gauge.setFixedHeight(90)
        self.intent_gauge.showGrid(x=False, y=False)
        self.intent_gauge.setXRange(-1.1, 1.1)
        self.intent_gauge.setYRange(0, 1)
        self.intent_gauge.hideAxis("left")
        ax = self.intent_gauge.getAxis("bottom")
        ax.setTicks([[(-1.0, "−1  open"), (0.0, "  0  rest  "), (1.0, "close  +1")]])
        ax.setStyle(tickFont=pg.Qt.QtGui.QFont("Segoe UI", 9))

        # track background
        self.intent_gauge.addItem(pg.BarGraphItem(
            x=[0], height=[0.5], width=[2.0],
            brush=pg.mkBrush("#1e1e1e"), pen=pg.mkPen("#333333"),
            y0=[0.25],
        ))
        # zero line
        self.intent_gauge.addLine(x=0.0, pen=pg.mkPen("#444444", width=1))

        # Filled bar from 0 → value (two items, one each side, only one visible at a time)
        self._gauge_pos_bar = pg.BarGraphItem(
            x=[0], height=[0.44], width=[0.001],
            brush=pg.mkBrush("#27ae60"), pen=pg.mkPen(None), y0=[0.28]
        )
        self._gauge_neg_bar = pg.BarGraphItem(
            x=[0], height=[0.44], width=[0.001],
            brush=pg.mkBrush("#c0392b"), pen=pg.mkPen(None), y0=[0.28]
        )
        self.intent_gauge.addItem(self._gauge_pos_bar)
        self.intent_gauge.addItem(self._gauge_neg_bar)

        # Needle
        self._gauge_needle = pg.InfiniteLine(
            pos=0.0, angle=90, pen=pg.mkPen("#f1c40f", width=3)
        )
        self.intent_gauge.addItem(self._gauge_needle)
        gauge_layout.addWidget(self.intent_gauge)

        # Stats row
        self.intent_stats_label = QLabel("Samples published: 0  |  Rate: — Hz  |  Elapsed: —")
        self.intent_stats_label.setAlignment(Qt.AlignCenter)
        self.intent_stats_label.setStyleSheet("color: #555555; font-size: 11px;")
        gauge_layout.addWidget(self.intent_stats_label)

        # Active decoder label (mirrored here so user doesn't have to switch tabs)
        self.intent_bin_label = QLabel("Active decoder: not fit")
        self.intent_bin_label.setAlignment(Qt.AlignCenter)
        self.intent_bin_label.setStyleSheet("color: #4da3ff; font-size: 11px;")
        gauge_layout.addWidget(self.intent_bin_label)

        parent.addWidget(gauge_box)

    def _build_log_box(self, parent: QVBoxLayout):
        box = QGroupBox("Log")
        layout = QVBoxLayout(box)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(240)
        layout.addWidget(self.log)
        parent.addWidget(box)

    def _log_line(self, text: str):
        self.log.append(text)

    def _refresh_streams(self):
        self.stream_combo.clear()
        stream_type = self.stream_type_combo.currentText().strip() or "EMG"
        stream_name = self.stream_name_edit.text().strip()
        try:
            candidates = pg.Qt.QtCore.QCoreApplication.instance()  # keep Qt loaded
            del candidates
            from pylsl import resolve_byprop

            streams = resolve_byprop("type", stream_type, timeout=1)
        except Exception as exc:
            self._log_line(f"LSL refresh failed: {exc}")
            return
        if stream_name:
            streams = [s for s in streams if s.name() == stream_name]
        for stream in streams:
            self.stream_combo.addItem(f"{stream.name()} ({stream.type()})", stream.name())
        if self.stream_combo.count() == 0:
            self.stream_combo.addItem("No streams found", "")
        self._log_line(f"Found {self.stream_combo.count()} EMG stream(s)")

    def _refresh_imu_streams(self):
        self.imu_stream_combo.clear()
        stream_type = self.imu_stream_type_combo.currentText().strip() or "IMU"
        stream_name = self.imu_stream_name_edit.text().strip()
        try:
            from pylsl import resolve_byprop
            streams = resolve_byprop("type", stream_type, timeout=1)
        except Exception as exc:
            self._log_line(f"IMU LSL refresh failed: {exc}")
            return
        if stream_name:
            streams = [s for s in streams if s.name() == stream_name]
        for stream in streams:
            self.imu_stream_combo.addItem(f"{stream.name()} ({stream.type()})", stream.name())
        if self.imu_stream_combo.count() == 0:
            self.imu_stream_combo.addItem("No IMU streams found", "")
        self._log_line(f"Found {self.imu_stream_combo.count()} IMU stream(s)")

    def _sync_stream_selection(self, index: int = 0):
        del index
        name = self.stream_combo.currentData()
        if name:
            self.stream_name_edit.setText(str(name))

    def _sync_imu_stream_selection(self, index: int = 0):
        del index
        name = self.imu_stream_combo.currentData()
        if name:
            self.imu_stream_name_edit.setText(str(name))

    def _toggle_lsl(self):
        if self._lsl_worker is not None:
            self._stop_lsl()
            return
        if self.stream_combo.currentData():
            self.stream_name_edit.setText(str(self.stream_combo.currentData()))
        self._start_lsl()

    def _toggle_imu(self):
        if self._imu_worker is not None:
            self._stop_imu()
            return
        if self.imu_stream_combo.currentData():
            self.imu_stream_name_edit.setText(str(self.imu_stream_combo.currentData()))
        self._start_imu()

    def _start_lsl(self):
        stream_type = self.stream_type_combo.currentText().strip() or "EMG"
        stream_name = self.stream_name_edit.text().strip()
        self._lsl_worker = EmgStreamWorker(stream_type, stream_name)
        self._lsl_worker.status_changed.connect(self._on_status)
        self._lsl_worker.stream_ready.connect(self._on_stream_ready)
        self._lsl_worker.chunk_received.connect(self._on_chunk)
        self._lsl_worker.start()
        self.connect_btn.setText("Disconnect LSL")

    def _start_imu(self):
        stream_type = self.imu_stream_type_combo.currentText().strip() or "IMU"
        stream_name = self.imu_stream_name_edit.text().strip()
        self._imu_worker = EmgStreamWorker(stream_type, stream_name)
        self._imu_worker.status_changed.connect(self._on_imu_status)
        self._imu_worker.stream_ready.connect(self._on_imu_stream_ready)
        self._imu_worker.chunk_received.connect(self._on_imu_chunk)
        self._imu_worker.start()
        self.imu_connect_btn.setText("Disconnect IMU")

    def _stop_lsl(self):
        worker = self._lsl_worker
        if worker is None:
            return
        worker.stop()
        worker.wait(2000)
        self._lsl_worker = None
        self.connect_btn.setText("Connect LSL")
        self.status_label.setText("LSL disconnected")
        self.status_label.setStyleSheet("color: #888888;")
        self._tick_timer.stop()

    def _stop_imu(self):
        worker = self._imu_worker
        if worker is None:
            return
        worker.stop()
        worker.wait(2000)
        self._imu_worker = None
        self.imu_connect_btn.setText("Connect IMU")
        self.imu_status_label.setText("IMU disconnected")
        self.imu_status_label.setStyleSheet("color: #888888;")
        self._current_roll_deg = None
        self.imu_angle_label.setText("Roll: — °  |  Bin: —  |  Source: —")
        self.orientation_coverage_label.setText("Orientation bins: IMU disconnected")

    def _on_status(self, message: str, color: str):
        self.status_label.setText(message)
        self.status_label.setStyleSheet(f"color: {color};")
        self._log_line(message)

    def _on_imu_status(self, message: str, color: str):
        self.imu_status_label.setText(message)
        self.imu_status_label.setStyleSheet(f"color: {color};")
        self._log_line(f"IMU: {message}")

    def _on_stream_ready(self, info: object):
        self._stream_meta = dict(info)
        fs = int(self._stream_meta.get("sample_rate", 1000))
        self._buffer.set_max_samples(max(1, int(fs * 8)))
        lowcut, highcut = self._adaptive_bandpass_label(fs)
        self.filter_label.setText(
            f"Adaptive bandpass: {lowcut:.1f} Hz to {highcut:.1f} Hz at {fs} Hz sample rate"
        )
        self._log_line(
            f"Stream ready: {self._stream_meta.get('name')} | {self._stream_meta.get('channel_count')} channels | {fs} Hz"
        )
        if not self._tick_timer.isActive():
            self._tick_timer.start()

    def _on_imu_stream_ready(self, info: object):
        self._imu_meta = dict(info)
        fs = int(self._imu_meta.get("sample_rate", 100))
        self._imu_buffer.set_max_samples(max(1, int(fs * 2)))
        self._log_line(
            f"IMU stream ready: {self._imu_meta.get('name')} | {self._imu_meta.get('channel_count')} channels | {fs} Hz"
        )
        self.orientation_coverage_label.setText("Orientation bins: IMU connected — start capturing")

    def _adaptive_bandpass_label(self, fs: int) -> tuple[float, float]:
        return _adaptive_bandpass_limits(fs)

    def _on_chunk(self, data: object, timestamps: object):
        del timestamps
        chunk = np.asarray(data, dtype=np.float64)
        if chunk.ndim != 2 or chunk.shape[1] == 0:
            return
        self._buffer.append(chunk)

    def _on_imu_chunk(self, data: object, timestamps: object):
        del timestamps
        chunk = np.asarray(data, dtype=np.float64)
        if chunk.ndim != 2 or chunk.shape[1] == 0:
            return
        self._imu_buffer.append(chunk)

    def _latest_imu_vector(self, channels_text: str, snapshot: np.ndarray) -> np.ndarray | None:
        indices = _parse_int_list(channels_text.strip())
        if len(indices) != 3:
            return None
        if min(indices) < 0 or max(indices) >= snapshot.shape[0]:
            return None
        return np.mean(snapshot[indices, :], axis=1)

    def _latest_imu_vectors(self) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        """Return mean accel/gyro/mag vectors over the last ~100 ms of IMU data."""
        snap = self._imu_buffer.snapshot()
        if snap is None:
            return None, None, None
        fs = int(self._imu_meta.get("sample_rate", 100))
        win = max(1, int(fs * 0.1))
        snap = snap[:, -win:]
        accel = self._latest_imu_vector(self.imu_accel_channels_edit.text(), snap)
        gyro = self._latest_imu_vector(self.imu_gyro_channels_edit.text(), snap)
        mag = self._latest_imu_vector(self.imu_mag_channels_edit.text(), snap)
        return accel, gyro, mag

    @staticmethod
    def _compose_imu9_vector(
        accel: np.ndarray | None,
        gyro: np.ndarray | None,
        mag: np.ndarray | None,
    ) -> np.ndarray | None:
        if accel is None and gyro is None and mag is None:
            return None
        out = np.full(9, np.nan, dtype=np.float64)
        if accel is not None and accel.shape[0] == 3:
            out[0:3] = accel
        if gyro is not None and gyro.shape[0] == 3:
            out[3:6] = gyro
        if mag is not None and mag.shape[0] == 3:
            out[6:9] = mag
        return out

    def _estimate_roll_deg(
        self,
        accel: np.ndarray | None,
        mag: np.ndarray | None,
    ) -> tuple[float | None, str]:
        source = self.imu_roll_source_combo.currentText()
        if source == "Mag only":
            if mag is None:
                return None, "mag unavailable"
            return _compute_roll_from_mag_deg(mag), "mag"
        if source == "Accel + Mag (fused)":
            if accel is None and mag is None:
                return None, "accel+mag unavailable"
            if accel is None:
                return _compute_roll_from_mag_deg(mag), "mag fallback"
            if mag is None:
                return _compute_roll_deg(accel), "accel fallback"
            roll_acc = _compute_roll_deg(accel)
            roll_mag = _compute_roll_from_mag_deg(mag)
            return _blend_angles_deg(roll_acc, roll_mag, w_b=0.2), "fused"
        if accel is None:
            return None, "accel unavailable"
        return _compute_roll_deg(accel), "accel"

    def _latest_window(self) -> np.ndarray | None:
        snapshot = self._buffer.snapshot()
        if snapshot is None:
            return None
        fs = int(self._stream_meta.get("sample_rate", 1000))
        win = max(8, int(fs * float(self.window_spin.value())))
        if snapshot.shape[1] > win:
            snapshot = snapshot[:, -win:]
        channel_indices = _parse_int_list(self.channels_edit.text())
        if channel_indices:
            snapshot = _select_channels(snapshot, channel_indices)
        return snapshot

    def _start_capture(self, state: str):
        if self._buffer.snapshot() is None:
            QMessageBox.information(self, "No data", "Connect an LSL stream before capturing a class.")
            return
        append_mode = bool(self.capture_append_chk.isChecked())
        if not append_mode:
            self._class_samples[state] = []
            self._class_covs[state] = []
            self._class_orientations[state] = []
            self._class_imu9[state] = []
        self._capture_state = state
        self._capture_started_at = time.time()
        if not self._tick_timer.isActive():
            self._tick_timer.start()
        imu_note = " (+ orientation)" if self._imu_worker is not None else ""
        mode_note = "append" if append_mode else "replace"
        intent = CLASS_LABELS.get(state, state)
        self.capture_status_label.setText(f"Capture: recording {intent} [{mode_note}]{imu_note}")
        self._log_line(f"Capture started: {intent} [{mode_note}]{imu_note}")

    def _save_session(self):
        """Save class samples, orientations, and fitted decoder to a .npz file."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Session", "", "NumPy session (*.npz)"
        )
        if not path:
            return
        if not path.endswith(".npz"):
            path += ".npz"

        arrays: dict[str, object] = {}

        # Class samples and orientation tags
        for name in CLASS_ORDER:
            samples = self._class_samples[name]
            orientations = self._class_orientations[name]
            imu9_list = self._class_imu9[name]
            if samples:
                arrays[f"samples_{name}"] = np.vstack(samples)
            # Store orientations as float array; NaN where None
            angles = np.array(
                [a if a is not None else np.nan for a in orientations], dtype=np.float64
            )
            arrays[f"orientations_{name}"] = angles
            if imu9_list:
                imu9_arr = np.vstack([
                    np.full(9, np.nan, dtype=np.float64) if v is None else np.asarray(v, dtype=np.float64)
                    for v in imu9_list
                ])
                arrays[f"imu9_{name}"] = imu9_arr

        # Fitted decoder parameters
        if self._gated_decoder is not None:
            dec = self._gated_decoder.global_decoder
            if dec is not None:
                arrays["dec_rest_centroid"] = dec.rest_centroid
                arrays["dec_flex_centroid"] = dec.flex_centroid
                arrays["dec_extend_centroid"] = dec.extend_centroid
                arrays["dec_direction"] = dec.direction
                arrays["dec_scale"] = np.array([dec.scale])
                arrays["dec_rest_gate"] = np.array([dec.rest_gate])
                arrays["dec_fisher"] = np.array([dec.fisher_ratio])
                if dec.pca_mean is not None:
                    arrays["dec_pca_mean"] = dec.pca_mean
                if dec.pca_basis is not None:
                    arrays["dec_pca_basis"] = dec.pca_basis

            # Per-bin decoders: save each bin's centroids and direction
            bin_ids = np.array(list(self._gated_decoder.bin_decoders.keys()), dtype=np.int32)
            arrays["bin_ids"] = bin_ids
            for b, bdec in self._gated_decoder.bin_decoders.items():
                arrays[f"bin_{b}_rest"] = bdec.rest_centroid
                arrays[f"bin_{b}_flex"] = bdec.flex_centroid
                arrays[f"bin_{b}_extend"] = bdec.extend_centroid
                arrays[f"bin_{b}_direction"] = bdec.direction
                arrays[f"bin_{b}_scale"] = np.array([bdec.scale])
                arrays[f"bin_{b}_rest_gate"] = np.array([bdec.rest_gate])

        # Riemannian reference mean (if used)
        if self._riemann_extractor is not None and self._riemann_extractor.ref_mean is not None:
            arrays["dec_riemann_ref_mean"] = self._riemann_extractor.ref_mean

        try:
            np.savez_compressed(path, **arrays)
        except Exception as exc:
            QMessageBox.warning(self, "Save failed", str(exc))
            return

        n_samples = {name: len(self._class_samples[name]) for name in CLASS_ORDER}
        n_imu9 = {
            name: sum(1 for v in self._class_imu9[name] if v is not None)
            for name in CLASS_ORDER
        }
        n_bins = self._gated_decoder.n_fitted_bins if self._gated_decoder else 0
        self._log_line(
            f"Session saved → {path} | "
            + " ".join(f"{CLASS_LABELS.get(n, n)}={n_samples[n]}" for n in CLASS_ORDER)
            + " | imu9 "
            + " ".join(f"{CLASS_LABELS.get(n, n)}={n_imu9[n]}" for n in CLASS_ORDER)
            + f" | {n_bins} orientation bins"
        )

    def _load_session(self):
        """Load a previously saved .npz session file."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Session", "", "NumPy session (*.npz)"
        )
        if not path:
            return

        try:
            data = np.load(path, allow_pickle=False)
        except Exception as exc:
            QMessageBox.warning(self, "Load failed", str(exc))
            return

        # Restore class samples
        new_samples: dict[str, list[np.ndarray]] = {name: [] for name in CLASS_ORDER}
        new_orientations: dict[str, list[float | None]] = {name: [] for name in CLASS_ORDER}
        new_imu9: dict[str, list[np.ndarray | None]] = {name: [] for name in CLASS_ORDER}
        for name in CLASS_ORDER:
            key = f"samples_{name}"
            if key in data:
                arr = data[key]
                new_samples[name] = [arr[i] for i in range(arr.shape[0])]
            okey = f"orientations_{name}"
            if okey in data:
                angles = data[okey]
                new_orientations[name] = [
                    None if np.isnan(a) else float(a) for a in angles
                ]
            ikey = f"imu9_{name}"
            if ikey in data:
                imu_arr = np.asarray(data[ikey], dtype=np.float64)
                if imu_arr.ndim == 2 and imu_arr.shape[1] == 9:
                    new_imu9[name] = [
                        None if np.all(np.isnan(imu_arr[i])) else imu_arr[i].copy()
                        for i in range(imu_arr.shape[0])
                    ]

        self._class_samples = new_samples
        self._class_orientations = new_orientations
        self._class_imu9 = new_imu9
        for name in CLASS_ORDER:
            n_s = len(self._class_samples[name])
            n_i = len(self._class_imu9[name])
            if n_i < n_s:
                self._class_imu9[name].extend([None] * (n_s - n_i))

        # Restore decoder if parameters were saved
        gated = None
        if "dec_rest_centroid" in data:
            try:
                global_dec = CentroidDirectionDecoder(
                    rest_centroid=data["dec_rest_centroid"],
                    flex_centroid=data["dec_flex_centroid"],
                    extend_centroid=data["dec_extend_centroid"],
                    direction=data["dec_direction"],
                    scale=float(data["dec_scale"][0]),
                    rest_gate=float(data["dec_rest_gate"][0]),
                    fisher_ratio=float(data["dec_fisher"][0]),
                    pca_mean=data["dec_pca_mean"] if "dec_pca_mean" in data else None,
                    pca_basis=data["dec_pca_basis"] if "dec_pca_basis" in data else None,
                )
                # Recompute class projection means/stds from restored samples
                for class_name in CLASS_ORDER:
                    samples = new_samples[class_name]
                    if samples:
                        arr = np.vstack(samples)
                        proj = global_dec.project_signed(arr, apply_gate=False)
                        global_dec.class_projection_means[class_name] = float(np.mean(proj))
                        global_dec.class_projection_stds[class_name] = float(np.std(proj))
                        global_dec.pairwise_distances[f"rest-flex"] = float(
                            np.linalg.norm(global_dec.rest_centroid - global_dec.flex_centroid)
                        )
                        global_dec.pairwise_distances[f"rest-extend"] = float(
                            np.linalg.norm(global_dec.rest_centroid - global_dec.extend_centroid)
                        )
                        global_dec.pairwise_distances[f"flex-extend"] = float(
                            np.linalg.norm(global_dec.flex_centroid - global_dec.extend_centroid)
                        )

                gated = OrientationGatedDecoder(global_decoder=global_dec)

                # Restore per-bin decoders
                if "bin_ids" in data:
                    for b in data["bin_ids"].tolist():
                        bk = f"bin_{b}_rest"
                        if bk not in data:
                            continue
                        bdec = CentroidDirectionDecoder(
                            rest_centroid=data[f"bin_{b}_rest"],
                            flex_centroid=data[f"bin_{b}_flex"],
                            extend_centroid=data[f"bin_{b}_extend"],
                            direction=data[f"bin_{b}_direction"],
                            scale=float(data[f"bin_{b}_scale"][0]),
                            rest_gate=float(data[f"bin_{b}_rest_gate"][0]),
                        )
                        gated.bin_decoders[int(b)] = bdec
                    gated.n_fitted_bins = len(gated.bin_decoders)

            except Exception as exc:
                QMessageBox.warning(self, "Decoder restore failed", str(exc))
                gated = None

        self._gated_decoder = gated
        self._live_signed_history.clear()
        self._live_time_history.clear()

        # Restore Riemannian extractor if present
        self._riemann_extractor = None
        if "dec_riemann_ref_mean" in data:
            extractor = RiemannianFeatureExtractor(ref_mean=data["dec_riemann_ref_mean"].copy())
            self._riemann_extractor = extractor
            self._log_line(f"Riemannian reference mean restored ({extractor.ref_mean.shape})")

        # Refresh UI
        self._update_class_count_label()
        self._clear_visuals()
        if self._gated_decoder is not None:
            dec = self._gated_decoder.global_decoder
            n_bins = self._gated_decoder.n_fitted_bins
            self.decoder_status_label.setText(
                f"Decoder: loaded | fisher={dec.fisher_ratio:.2f} | rest gate={dec.rest_gate:.3f}"
                + (f" | {n_bins} orientation bins" if n_bins > 0 else "")
            )
            self.decoder_status_label.setStyleSheet("color: #4da3ff; font-weight: bold;")
            self.fit_progress_label.setText("Fit progress: loaded from session")
            self.fit_progress_label.setStyleSheet("color: #4da3ff;")
            self._update_geometry_plots()
            self._update_projection_plot()
            self._update_coverage_plot()
        else:
            self.decoder_status_label.setText("Session loaded (samples only — re-fit to use decoder)")
            self.decoder_status_label.setStyleSheet("color: #f1c40f; font-weight: bold;")
            self.fit_progress_label.setText("Fit progress: session has samples only")
            self.fit_progress_label.setStyleSheet("color: #f1c40f;")

        n_samples = {name: len(self._class_samples[name]) for name in CLASS_ORDER}
        n_imu9 = {
            name: sum(1 for v in self._class_imu9[name] if v is not None)
            for name in CLASS_ORDER
        }
        n_bins = self._gated_decoder.n_fitted_bins if self._gated_decoder else 0
        self._log_line(
            f"Session loaded ← {path} | "
            + " ".join(f"{CLASS_LABELS.get(n, n)}={n_samples[n]}" for n in CLASS_ORDER)
            + " | imu9 "
            + " ".join(f"{CLASS_LABELS.get(n, n)}={n_imu9[n]}" for n in CLASS_ORDER)
            + (f" | {n_bins} orientation bins restored" if n_bins > 0 else " | no decoder")
        )

    def _clear_classes(self):
        self._class_samples = {name: [] for name in CLASS_ORDER}
        self._class_covs = {name: [] for name in CLASS_ORDER}
        self._class_orientations = {name: [] for name in CLASS_ORDER}
        self._class_imu9 = {name: [] for name in CLASS_ORDER}
        self._capture_state = None
        self._capture_started_at = None
        self._gated_decoder = None
        self._riemann_extractor = None
        self._adaptive_rest_idle_since = None
        self._live_signed_history.clear()
        self._live_time_history.clear()
        self.capture_status_label.setText("Capture: idle")
        self.decoder_status_label.setText("Decoder: not fit")
        self.fit_progress_label.setText("Fit progress: idle")
        self.fit_progress_label.setStyleSheet("color: #888888;")
        self.active_bin_label.setText("Active decoder: not fit")
        self._update_class_count_label()
        self._clear_visuals()
        self._log_line("Cleared all class captures")

    def _clear_visuals(self):
        for name in CLASS_ORDER:
            self.class_scatter_items[name].setData([], [])
            self.class_centroid_items[name].setData([], [])
        self.live_scatter_item.setData([], [])
        self.axis_line_item.setData([], [])
        self.feature_curve_live.setData([], [])
        self.feature_curve_rest.setData([], [])
        self.feature_curve_flex.setData([], [])
        self.feature_curve_extend.setData([], [])
        self.separation_plot.clear()
        self.separation_plot.showGrid(x=True, y=True, alpha=0.25)
        self.separation_plot.setLabel("bottom", "Pair")
        self.separation_plot.setLabel("left", "Centroid distance")
        self.decoder_curve.setData([], [])
        for name in CLASS_ORDER:
            self.strip_scatter[name].setData([], [])
            self.strip_mean_items[name].setData([], [])
            self.strip_std_items[name].setData([], [])
        self.strip_live_line.setValue(0.0)
        self.coverage_plot.clear()
        self.coverage_plot.showGrid(x=True, y=True, alpha=0.25)
        self.coverage_plot.setLabel("bottom", "Forearm roll angle (°)")
        self.coverage_plot.setLabel("left", "Samples in bin")
        self.coverage_plot.setXRange(-185, 185)
        self.coverage_live_line = pg.InfiniteLine(
            pos=0.0, angle=90, pen=pg.mkPen("#f1c40f", width=2)
        )
        self.coverage_plot.addItem(self.coverage_live_line)
        self.coverage_label.setText("Orientation coverage: no IMU data")
        self.separation_label.setText("Separability: -")
        self.active_bin_label.setText("Active decoder: not fit")
        self.decoder_value_label.setText("Live decoder: -")

    def _update_class_count_label(self):
        self.class_count_label.setText(
            "Samples | "
            + " ".join(
                f"{CLASS_LABELS.get(name, name)}={len(self._class_samples[name])}"
                for name in CLASS_ORDER
            )
        )

    def _fit_decoder(self):
        if self._fit_worker is not None and self._fit_worker.isRunning():
            QMessageBox.information(self, "Fit in progress", "Decoder fitting is already running.")
            return

        use_riemann = self.feature_mode_combo.currentText().startswith("Riemannian")
        samples_copy = {
            name: [np.array(v, copy=True) for v in self._class_samples[name]]
            for name in CLASS_ORDER
        }
        covs_copy = {
            name: [np.array(c, copy=True) for c in self._class_covs[name]]
            for name in CLASS_ORDER
        }
        orientations_copy = {
            name: list(self._class_orientations[name])
            for name in CLASS_ORDER
        }

        self.fit_btn.setEnabled(False)
        self.fit_btn.setText("Fitting…")
        self.decoder_status_label.setText("Decoder: fitting in background…")
        self.decoder_status_label.setStyleSheet("color: #f1c40f; font-weight: bold;")
        if use_riemann:
            self.fit_progress_label.setText(
                f"Fit progress: Fitting (Riemannian, cap={int(self.riemann_max_cov_spin.value())}/class)…"
            )
        else:
            self.fit_progress_label.setText("Fit progress: Fitting (RMS)…")
        self.fit_progress_label.setStyleSheet("color: #f1c40f;")
        self.capture_status_label.setText("Capture: idle (fit running)")
        self._log_line("Decoder fit started (background worker)")

        self._fit_worker = DecoderFitWorker(
            class_samples=samples_copy,
            class_covs=covs_copy,
            class_orientations=orientations_copy,
            use_riemann=use_riemann,
            max_cov_per_class=int(self.riemann_max_cov_spin.value()),
        )
        self._fit_worker.fit_ok.connect(self._on_fit_ok)
        self._fit_worker.fit_failed.connect(self._on_fit_failed)
        self._fit_worker.finished.connect(self._on_fit_worker_finished)
        self._fit_worker.start()

    def _on_fit_ok(self, payload: object):
        result = payload  # dict payload from DecoderFitWorker
        if not isinstance(result, dict):
            self._on_fit_failed("Invalid fit worker result")
            return
        gated = result.get("gated")
        extractor = result.get("extractor")
        fit_samples = result.get("samples")
        n_cov_used = int(result.get("n_cov_used", 0))
        if gated is None or fit_samples is None:
            self._on_fit_failed("Fit worker returned incomplete results")
            return

        self._gated_decoder = gated
        self._riemann_extractor = extractor
        self._class_samples = fit_samples

        global_dec = gated.global_decoder
        n_bins = gated.n_fitted_bins
        use_riemann = extractor is not None
        mode_tag = "Riemannian+CMR-dir" if use_riemann else "RMS+CMR-dir"
        extra = f" | cov used={n_cov_used}" if use_riemann else ""
        self.decoder_status_label.setText(
            f"Decoder: fit [{mode_tag}] | fisher={global_dec.fisher_ratio:.2f} | rest gate={global_dec.rest_gate:.3f}"
            + (f" | orientation bins={n_bins}" if n_bins > 0 else " | no orientation bins (IMU not used)")
            + extra
        )
        self.decoder_status_label.setStyleSheet("color: #27ae60; font-weight: bold;")
        self.fit_progress_label.setText(f"Fit progress: Complete [{mode_tag}]")
        self.fit_progress_label.setStyleSheet("color: #27ae60;")
        self._update_geometry_plots()
        self._update_projection_plot()
        self._update_coverage_plot()
        self._log_line(
            f"Fitted orientation-gated decoder [{mode_tag}] | global fisher={global_dec.fisher_ratio:.2f} | "
            f"orientation bins fitted={n_bins}{extra}"
        )

    def _on_fit_failed(self, message: str):
        QMessageBox.information(self, "Cannot fit decoder", message)
        self.decoder_status_label.setText("Decoder: fit failed")
        self.decoder_status_label.setStyleSheet("color: #c0392b; font-weight: bold;")
        self.fit_progress_label.setText("Fit progress: Failed")
        self.fit_progress_label.setStyleSheet("color: #c0392b;")
        self._log_line(f"Decoder fit failed: {message}")

    def _on_fit_worker_finished(self):
        self.fit_btn.setEnabled(True)
        self.fit_btn.setText("Fit Centroid Decoder")
        if self._fit_worker is not None:
            self._fit_worker.deleteLater()
            self._fit_worker = None

    def _update_geometry_plots(self):
        if self._gated_decoder is None:
            return
        dec = self._gated_decoder.global_decoder
        for name in CLASS_ORDER:
            samples = self._class_samples[name]
            if not samples:
                continue
            arr = np.vstack(samples)
            proj = dec.project_2d(arr)
            self.class_scatter_items[name].setData(proj[:, 0], proj[:, 1])
            centroid = dec.centroid_2d(name)
            self.class_centroid_items[name].setData([centroid[0]], [centroid[1]])

        rest2 = dec.centroid_2d("rest")
        flex2 = dec.centroid_2d("flex")
        extend2 = dec.centroid_2d("extend")
        self.axis_line_item.setData([extend2[0], flex2[0]], [extend2[1], flex2[1]])

        x_axis = np.arange(1, len(dec.rest_centroid) + 1)
        self.feature_curve_rest.setData(x_axis, dec.rest_centroid)
        self.feature_curve_flex.setData(x_axis, dec.flex_centroid)
        self.feature_curve_extend.setData(x_axis, dec.extend_centroid)

        self.separation_plot.clear()
        self.separation_plot.showGrid(x=True, y=True, alpha=0.25)
        self.separation_plot.setLabel("bottom", "Pair")
        self.separation_plot.setLabel("left", "Centroid distance")
        self.separation_plot.addItem(
            pg.BarGraphItem(
                x=np.arange(1, 4),
                height=[
                    dec.pairwise_distances["rest-flex"],
                    dec.pairwise_distances["rest-extend"],
                    dec.pairwise_distances["flex-extend"],
                ],
                width=0.65,
                brush=pg.mkBrush("#4da3ff"),
            )
        )
        self.separation_label.setText(
            "Separability: "
            f"rest-close={dec.pairwise_distances['rest-flex']:.3f}, "
            f"rest-open={dec.pairwise_distances['rest-extend']:.3f}, "
            f"close-open={dec.pairwise_distances['flex-extend']:.3f}, "
            f"fisher={dec.fisher_ratio:.2f}"
        )

    def _update_projection_plot(self):
        if self._gated_decoder is None:
            return
        dec = self._gated_decoder.global_decoder
        rng = np.random.default_rng(0)
        for name in CLASS_ORDER:
            samples = self._class_samples[name]
            if not samples:
                self.strip_scatter[name].setData([], [])
                self.strip_mean_items[name].setData([], [])
                self.strip_std_items[name].setData([], [])
                continue
            arr = np.vstack(samples)
            proj = dec.project_signed(arr, apply_gate=False)
            y_center = self._strip_y_centers[name]
            jitter = rng.uniform(-0.3, 0.3, len(proj))
            self.strip_scatter[name].setData(x=proj, y=y_center + jitter)
            mean = float(dec.class_projection_means[name])
            std = float(dec.class_projection_stds[name])
            self.strip_mean_items[name].setData(x=[mean], y=[y_center])
            self.strip_std_items[name].setData(
                x=[mean - std, mean - std, mean - std, mean + std, mean + std, mean + std],
                y=[y_center - 0.35, y_center + 0.35, y_center, y_center, y_center - 0.35, y_center + 0.35],
            )

    def _update_coverage_plot(self):
        """Draw stacked bars showing sample counts per orientation bin per class."""
        self.coverage_plot.clear()
        self.coverage_plot.showGrid(x=True, y=True, alpha=0.25)
        self.coverage_plot.setLabel("bottom", "Forearm roll angle (°)")
        self.coverage_plot.setLabel("left", "Samples in bin")
        self.coverage_plot.setXRange(-185, 185)

        # Rebuild live line (clear() removed it)
        self.coverage_live_line = pg.InfiniteLine(
            pos=self._current_roll_deg if self._current_roll_deg is not None else 0.0,
            angle=90, pen=pg.mkPen("#f1c40f", width=2)
        )
        self.coverage_plot.addItem(self.coverage_live_line)

        # Gather per-bin, per-class counts from captured data
        bins_counts: dict[int, dict[str, int]] = {}
        for name in CLASS_ORDER:
            for angle in self._class_orientations[name]:
                if angle is None:
                    continue
                b = _angle_to_bin(angle)
                if b not in bins_counts:
                    bins_counts[b] = {n: 0 for n in CLASS_ORDER}
                bins_counts[b][name] += 1

        if not bins_counts:
            self.coverage_label.setText("Orientation coverage: no IMU data in captures")
            return

        bar_width = BIN_SIZE_DEG * 0.8
        bottom: dict[int, float] = {b: 0.0 for b in bins_counts}
        for name in CLASS_ORDER:
            color = CLASS_COLORS[name]
            xs, hs = [], []
            for b, counts in bins_counts.items():
                h = counts[name]
                xs.append(_bin_center_deg(b))
                hs.append(h)
            # Need stacked bottom values
            bottoms_list = [bottom[b] for b in bins_counts]
            self.coverage_plot.addItem(
                pg.BarGraphItem(
                    x=xs, height=hs, width=bar_width,
                    brush=pg.mkBrush(color + "cc"),
                    pen=pg.mkPen(None),
                    y0=bottoms_list,
                )
            )
            for i, b in enumerate(bins_counts):
                bottom[b] += bins_counts[b][name]

        # Mark fitted bins with a green tick at top
        if self._gated_decoder and self._gated_decoder.bin_decoders:
            fitted_x = [_bin_center_deg(b) for b in self._gated_decoder.bin_decoders]
            fitted_y = [bottom.get(b, 0) + 0.5 for b in self._gated_decoder.bin_decoders]
            self.coverage_plot.plot(
                fitted_x, fitted_y,
                pen=None, symbol="t1", symbolSize=8,
                symbolBrush="#27ae60", symbolPen=pg.mkPen(None),
            )

        n_bins_with_data = len(bins_counts)
        n_bins_fitted = self._gated_decoder.n_fitted_bins if self._gated_decoder else 0
        span_deg = len(bins_counts) * BIN_SIZE_DEG
        self.coverage_label.setText(
            f"Orientation coverage: {n_bins_with_data} bins with data  |  "
            f"{n_bins_fitted} bins with fitted decoders  |  "
            f"~{span_deg:.0f}° span covered"
        )

    def _reset_tuning(self):
        self.gain_spin.setValue(1.0)
        self.rest_gate_scale_spin.setValue(1.0)

    def _toggle_publish(self):
        if self._lsl_outlet is not None:
            self._stop_publish()
        else:
            self._start_publish()

    def _start_publish(self):
        if self._gated_decoder is None:
            QMessageBox.information(self, "No decoder", "Fit a decoder before publishing.")
            return
        try:
            from pylsl import StreamInfo, StreamOutlet
            name = self.intent_stream_name_edit.text().strip() or "UserIntent"
            stype = self.intent_stream_type_edit.text().strip() or "UserIntent"
            source_id = self.intent_source_id_edit.text().strip() or "nml-emg-intent"
            info = StreamInfo(name, stype, 1, int(1.0 / PLOT_TICK_SEC), "float32", source_id)
            self._lsl_outlet = StreamOutlet(info)
        except Exception as exc:
            QMessageBox.warning(self, "LSL outlet error", str(exc))
            return
        self._publish_count = 0
        self._publish_start_time = time.time()
        self.intent_publish_btn.setText("Stop Publishing")
        self.intent_stream_status_label.setText("Publishing ●")
        self.intent_stream_status_label.setStyleSheet("color: #27ae60; font-weight: bold;")
        name = self.intent_stream_name_edit.text().strip() or "UserIntent"
        self._log_line(f"LSL outlet started: '{name}'")

    def _stop_publish(self):
        self._lsl_outlet = None  # StreamOutlet closes on GC
        self.intent_publish_btn.setText("Start Publishing")
        self.intent_stream_status_label.setText("Not publishing")
        self.intent_stream_status_label.setStyleSheet("color: #666666; font-weight: bold;")
        self._log_line("LSL outlet stopped")

    def _push_intent_sample(self, value: float):
        """Push one sample and update gauge + stats."""
        self._last_published_value = value

        # Gauge needle + filled bar
        self._gauge_needle.setValue(value)
        if value >= 0.0:
            self._gauge_pos_bar.setOpts(x=[value / 2.0], width=[max(value, 0.001)])
            self._gauge_neg_bar.setOpts(x=[0], width=[0.001])
            color = "#27ae60"
        else:
            self._gauge_neg_bar.setOpts(x=[value / 2.0], width=[max(-value, 0.001)])
            self._gauge_pos_bar.setOpts(x=[0], width=[0.001])
            color = "#c0392b"

        # Big readout — colour shifts with sign
        sign_str = f"{value:+.3f}"
        self.intent_big_label.setText(sign_str)
        self.intent_big_label.setStyleSheet(
            f"font-size: 72px; font-weight: bold; color: {color}; "
            "font-family: 'Cascadia Code', 'Consolas', monospace; letter-spacing: 4px;"
        )

        # Publish over LSL if outlet open
        if self._lsl_outlet is not None:
            try:
                self._lsl_outlet.push_sample([float(value)])
                self._publish_count += 1
            except Exception:
                pass

        # Stats
        if self._publish_start_time is not None:
            elapsed = time.time() - self._publish_start_time
            rate = self._publish_count / max(elapsed, 0.001)
            elapsed_str = f"{elapsed:.0f}s" if elapsed < 60 else f"{elapsed/60:.1f}min"
            self.intent_stats_label.setText(
                f"Samples published: {self._publish_count:,}  "
                f"|  Rate: {rate:.1f} Hz  |  Elapsed: {elapsed_str}"
            )

    def _project_tuned(self, feature: np.ndarray, decoder: CentroidDirectionDecoder) -> float:
        """Project feature onto signed axis, applying user gain and rest gate scale."""
        x = np.asarray(feature, dtype=np.float64).reshape(1, -1)
        raw = float(((x - decoder.rest_centroid) @ decoder.direction)[0]) * decoder.scale
        effective_gate = decoder.rest_gate * float(self.rest_gate_scale_spin.value())
        if effective_gate > 1e-6:
            residual = float(np.linalg.norm(x - decoder.rest_centroid))
            gate = min(residual / effective_gate, 1.0)
            raw *= gate
        return float(np.clip(raw * float(self.gain_spin.value()), -1.0, 1.0))

    def _tick(self):
        # --- Read IMU ---
        accel, gyro, mag = self._latest_imu_vectors()
        imu9 = self._compose_imu9_vector(accel, gyro, mag)
        roll, source_tag = self._estimate_roll_deg(accel, mag)
        if roll is not None:
            self._current_roll_deg = roll
            b = _angle_to_bin(roll)
            self.imu_angle_label.setText(f"Roll: {roll:+.1f}°  |  Bin: {b}  |  Source: {source_tag}")
            self.coverage_live_line.setValue(roll)
            self._update_orientation_coverage_label()
        elif self._imu_worker is not None:
            self.imu_angle_label.setText(f"Roll: — °  |  Bin: —  |  Source: {source_tag}")

        # --- Read EMG window ---
        window = self._latest_window()
        if window is None:
            return
        fs = int(self._stream_meta.get("sample_rate", 1000))
        processed = _preprocess_window(window, fs, self.preprocessing_combo.currentText())
        rms = compute_rms(processed, axis=1)
        rms_feature = _common_mode_remove(rms)

        # Always compute covariance matrix for Riemannian path (stored during capture,
        # used for live decode if Riemannian extractor is fitted)
        cov_matrix = _compute_covariance(processed)

        # Derive the live decode feature: Riemannian tangent if extractor available, else RMS+CMR
        if self._riemann_extractor is not None:
            try:
                feature = self._riemann_extractor.transform(cov_matrix)
            except Exception:
                feature = rms_feature
        else:
            feature = rms_feature

        self._live_feature = feature

        x_axis = np.arange(1, len(rms_feature) + 1)
        self.feature_curve_live.setData(x_axis, rms_feature)  # always plot RMS for interpretability

        # --- Accumulate capture ---
        if self._capture_state is not None:
            # Store both feature vectors (RMS+CMR) and covariance matrices simultaneously
            self._class_samples[self._capture_state].append(np.array(rms_feature, copy=True))
            self._class_covs[self._capture_state].append(cov_matrix.copy())
            self._class_orientations[self._capture_state].append(self._current_roll_deg)
            self._class_imu9[self._capture_state].append(None if imu9 is None else imu9.copy())
            duration = float(self.capture_sec_spin.value())
            elapsed = 0.0 if self._capture_started_at is None else time.time() - self._capture_started_at
            n_tagged = sum(
                1 for a in self._class_orientations[self._capture_state] if a is not None
            )
            n_total = len(self._class_samples[self._capture_state])
            n_imu9 = sum(1 for v in self._class_imu9[self._capture_state] if v is not None)
            imu_note = (
                f" ({n_tagged}/{n_total} orient, {n_imu9}/{n_total} imu9)"
                if self._imu_worker else ""
            )
            intent = CLASS_LABELS.get(self._capture_state, self._capture_state)
            self.capture_status_label.setText(
                f"Capture: recording {intent} {elapsed:.1f}/{duration:.1f}s{imu_note}"
            )
            if elapsed >= duration:
                intent = CLASS_LABELS.get(self._capture_state, self._capture_state)
                self._log_line(
                    f"Capture complete: {intent} "
                    f"({n_total} samples, {n_tagged} with orientation, {n_imu9} with IMU9)"
                )
                self._capture_state = None
                self._capture_started_at = None
                self.capture_status_label.setText("Capture: idle")
                self._update_class_count_label()

        # --- Decode ---
        if self._gated_decoder is not None:
            active_dec = self._gated_decoder.get_decoder(self._current_roll_deg)
            signed = self._project_tuned(feature, active_dec)
            now = time.time()
            self._live_time_history.append(now)
            self._live_signed_history.append(signed)
            self.decoder_curve.setData(
                [t - self._live_time_history[0] for t in self._live_time_history],
                list(self._live_signed_history),
            )
            # 2D scatter always uses rms_feature projected via PCA for visual consistency
            live_2d = self._gated_decoder.global_decoder.project_2d(rms_feature.reshape(1, -1))[0]
            self.live_scatter_item.setData([live_2d[0]], [live_2d[1]])
            self.strip_live_line.setValue(signed)
            bin_label = self._gated_decoder.active_bin_label(self._current_roll_deg)
            self.active_bin_label.setText(f"Active decoder: {bin_label}")
            self.intent_bin_label.setText(f"Active decoder: {bin_label}")
            g_dec = self._gated_decoder.global_decoder
            self.decoder_value_label.setText(
                f"Live decoder: {signed:+.3f}  [gain={self.gain_spin.value():.3f}  gate×{self.rest_gate_scale_spin.value():.2f}]"
                f"  | close≈{g_dec.class_projection_means['flex']:+.3f}"
                f"  | open≈{g_dec.class_projection_means['extend']:+.3f}"
            )
            self._push_intent_sample(signed)

            # --- Adaptive rest centroid update ---
            if self.adaptive_rest_chk.isChecked():
                rest_threshold = 0.15
                tau = float(self.adaptive_rest_tau_spin.value())
                if abs(signed) < rest_threshold:
                    if self._adaptive_rest_idle_since is None:
                        self._adaptive_rest_idle_since = now
                    idle_duration = now - self._adaptive_rest_idle_since
                    if idle_duration > 2.0:
                        # EMA update: α per tick ≈ PLOT_TICK_SEC / τ
                        alpha = PLOT_TICK_SEC / max(tau, 0.1)
                        alpha = min(alpha, 0.05)  # cap to avoid instability
                        # Update rest centroid in all fitted decoders (global + bins)
                        for dec in [self._gated_decoder.global_decoder] + list(
                            self._gated_decoder.bin_decoders.values()
                        ):
                            dec.rest_centroid = (
                                (1.0 - alpha) * dec.rest_centroid + alpha * feature
                            )
                else:
                    self._adaptive_rest_idle_since = None

        self._update_class_count_label()

    def _update_orientation_coverage_label(self):
        """Update the live orientation coverage summary in the capture box."""
        if self._imu_worker is None:
            return
        total_tagged = sum(
            sum(1 for a in self._class_orientations[name] if a is not None)
            for name in CLASS_ORDER
        )
        occupied_bins: set[int] = set()
        for name in CLASS_ORDER:
            for a in self._class_orientations[name]:
                if a is not None:
                    occupied_bins.add(_angle_to_bin(a))
        roll_str = f"{self._current_roll_deg:+.1f}°" if self._current_roll_deg is not None else "—"
        self.orientation_coverage_label.setText(
            f"Orientation bins: current={roll_str}  |  {len(occupied_bins)} bins occupied  |  "
            f"{total_tagged} orientation-tagged samples across all classes"
        )

    def closeEvent(self, event):
        if self._fit_worker is not None and self._fit_worker.isRunning():
            self._fit_worker.wait(2000)
        self._stop_lsl()
        self._stop_imu()
        self._stop_publish()
        super().closeEvent(event)


def main(argv: list[str] | None = None) -> int:
    app = QApplication(argv or sys.argv)
    app.setStyleSheet(DARK_STYLE)
    window = EmgCentroidDecoderGUI()
    window.resize(1300, 980)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
