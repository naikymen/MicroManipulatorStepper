"""Headless Raspberry Pi camera service: MJPEG preview plus a JSON control API.

This module drives a Raspberry Pi HQ Camera (Sony IMX477) through Picamera2 and
publishes it over plain HTTP so that any client on the network can watch the
preview and, more importantly, take real measurements from it.

Two independent capabilities are exposed on purpose:

* the **preview path** (``GET /stream.mjpg``) is a low bitrate MJPEG stream that
  is meant to be displayed. It is deliberately lossy and never used for
  measurement.
* the **measurement path** (``POST /capture``) stops the preview, reconfigures
  the sensor for a full resolution *still* capture, writes a JPEG plus an
  optional DNG raw file, and returns the full libcamera metadata as JSON. This
  is the path that other scientific software should use, either by reading the
  files straight off the Pi or by downloading them through ``GET /captures``.

The service is pure userspace: no kernel modules, no UVC gadget mode, and it
never requires exclusive ownership of a video device node. That also means the
libcamera pipeline is only ever opened by this process -- see
``documentation/camera/pi_zero_hq_microscope_setup.md`` for why UVC gadget mode
and Picamera2 cannot be used at the same time.

The module is importable without Picamera2/libcamera installed so that the HTTP
layer and the MJPEG fan-out can be exercised by offline unit tests.
"""

from __future__ import annotations

import csv
import json
import logging
import math
import queue
import re
import socket
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from itertools import count
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

SERVICE_NAME = "PiCameraService"
SERVICE_VERSION = "0.2.0"
API_VERSION = 1

DEFAULT_MJPEG_BOUNDARY = "picamera-frame"
# The port every client assumes. Keep this in step with the GUI's
# DEFAULT_PI_CAMERA_PORT in software/OpenMicroManipulatorGUI/source/hardware/camera_pi.py.
DEFAULT_PORT = 8000
DEFAULT_CLIENT_QUEUE_DEPTH = 2
DEFAULT_STREAM_POLL_S = 1.0
DEFAULT_STREAM_STALL_TIMEOUT_S = 5.0
DEFAULT_CAPTURE_TIMEOUT_S = 60.0
DEFAULT_STILL_QUALITY = 95
DEFAULT_PREVIEW_QUALITY = 70


# --------------------------------------------------------------------------- #
# Picamera2 dependency handling
# --------------------------------------------------------------------------- #

# ``picamera2`` only exists on a Raspberry Pi with the Raspberry Pi OS camera
# stack installed. Everything above this line is portable, and `MjpegFanoutOutput`
# below degrades to a plain stand-in base class so that offline tests can run.
try:  # pragma: no cover - depends on the host
    from picamera2.outputs import Output as _Picamera2Output
    from picamera2 import Picamera2 as _Picamera2

    PICAMERA2_AVAILABLE = True
except Exception:  # pragma: no cover - depends on the host
    _Picamera2Output = None
    _Picamera2 = None
    PICAMERA2_AVAILABLE = False

try:  # pragma: no cover - depends on the host
    import libcamera
except Exception:  # pragma: no cover - depends on the host
    libcamera = None


if _Picamera2Output is None:

    class _OutputBase:
        """Minimal stand-in for ``picamera2.outputs.Output``.

        Only the members that this module actually relies on are implemented.
        Keeping the signature identical means ``MjpegFanoutOutput`` behaves the
        same whether or not Picamera2 is present.
        """

        def __init__(self, pts=None):
            self._pts_output = None
            self.recording = False
            self.needs_pacing = False
            self.needs_add_stream = False
            self.ptsoutput = pts

        def start(self):
            self.recording = True

        def stop(self):
            self.recording = False

        def outputframe(self, frame, keyframe=True, timestamp=None, packet=None, audio=False):
            raise NotImplementedError

        def outputtimestamp(self, timestamp):
            return None

        def _add_stream(self, encoder_stream, *args, **kwargs):
            # Picamera2 calls this when the encoder is configured. Outputs that do
            # not care about individual streams simply ignore it.
            return None

        @property
        def ptsoutput(self):
            return self._pts_output

        @ptsoutput.setter
        def ptsoutput(self, value):
            self._pts_output = value

else:
    _OutputBase = _Picamera2Output


class CameraUnavailable(RuntimeError):
    """Raised when an operation needs a working camera and there is none."""


# --------------------------------------------------------------------------- #
# MJPEG fan-out
# --------------------------------------------------------------------------- #


class MjpegFanoutOutput(_OutputBase):
    """Picamera2 output that hands every encoded JPEG to all connected clients.

    Picamera2 invokes :meth:`outputframe` from the encoder's own thread, so the
    fan-out has to be thread-safe and must never block: a slow HTTP client would
    otherwise stall the camera pipeline. Each client therefore gets a small
    bounded queue and the *oldest* frame is dropped when it is full. That trades
    an occasional skipped frame for bounded memory and low latency, which is the
    right trade for a live preview.

    The most recent frame is also retained so ``GET /snapshot.jpg`` can answer
    instantly without touching the camera.
    """

    def __init__(self, pts=None, queue_depth: int = DEFAULT_CLIENT_QUEUE_DEPTH):
        super().__init__(pts=pts)
        self._queue_depth = max(1, int(queue_depth))
        self._lock = threading.Lock()
        self._clients: dict[int, queue.Queue] = {}
        self._client_ids = count(1)
        self._latest_lock = threading.Lock()
        self._latest: bytes | None = None

        self.frames_sent = 0
        self.bytes_sent = 0
        self.frames_dropped = 0
        self.clients_seen = 0
        self.last_frame_time: float | None = None

    # -- client bookkeeping ------------------------------------------------- #

    def register_client(self) -> tuple[int, "queue.Queue[bytes]"]:
        client_id = next(self._client_ids)
        client_queue: queue.Queue = queue.Queue(maxsize=self._queue_depth)
        with self._lock:
            self._clients[client_id] = client_queue
            self.clients_seen += 1
        return client_id, client_queue

    def unregister_client(self, client_id: int) -> None:
        with self._lock:
            self._clients.pop(client_id, None)

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)

    def next_frame(self, client_queue: "queue.Queue[bytes]", timeout: float) -> bytes | None:
        """Return the next frame for a client, or ``None`` if none arrived in time."""
        try:
            return client_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    # -- latest frame ------------------------------------------------------- #

    def latest_frame(self) -> tuple[bytes | None, int]:
        """Return ``(jpeg_bytes, sequence)`` for the most recent encoded frame."""
        with self._latest_lock:
            return self._latest, self.frames_sent

    def seconds_since_last_frame(self) -> float:
        """Age of the newest frame, or ``inf`` if none has arrived yet."""
        with self._latest_lock:
            last = self.last_frame_time
        if last is None:
            return float("inf")
        return time.monotonic() - last

    # -- picamera2 entry point ---------------------------------------------- #

    def outputframe(self, frame, keyframe=True, timestamp=None, packet=None, audio=False):
        if audio or not frame:
            return

        data = bytes(frame)

        with self._latest_lock:
            self._latest = data
            self.frames_sent += 1
            self.bytes_sent += len(data)
            self.last_frame_time = time.monotonic()

        with self._lock:
            clients = list(self._clients.values())

        for client_queue in clients:
            self._offer(client_queue, data)

    def _offer(self, client_queue: "queue.Queue[bytes]", data: bytes) -> None:
        try:
            client_queue.put_nowait(data)
            return
        except queue.Full:
            pass

        # Drop the oldest frame so the client catches up to the live edge.
        try:
            client_queue.get_nowait()
            self.frames_dropped += 1
        except queue.Empty:  # pragma: no cover - racing consumer
            pass

        try:
            client_queue.put_nowait(data)
        except queue.Full:  # pragma: no cover - racing consumer
            self.frames_dropped += 1


# --------------------------------------------------------------------------- #
# Control mapping
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class ControlSpec:
    """Description of one settable libcamera control.

    ``requires`` lists the ``Picamera2.camera_controls`` entries that must exist
    for the control to be offered at all. Ranges are taken from the camera where
    possible; ``lo``/``hi`` are only used as a fallback and to advertise sane
    bounds for controls libcamera reports as unbounded.

    ``kind`` is one of ``int``, ``float``, ``bool``, ``intpair``, ``floatpair``
    or ``enum``.

    ``auto_value`` marks a value that means "let the algorithm decide" rather
    than a measurement. libcamera advertises ``ExposureTime`` and
    ``AnalogueGain`` with a zero minimum for exactly this reason, so such a
    value must bypass range clamping.
    """

    name: str
    kind: str
    unit: str | None = None
    default: object | None = None
    lo: float | None = None
    hi: float | None = None
    requires: tuple[str, ...] = ()
    description: str = ""
    auto_value: object | None = None


CONTROL_SPECS: dict[str, ControlSpec] = {
    "ExposureTime": ControlSpec(
        "ExposureTime", "int", unit="us",
        requires=("ExposureTime",),
        description="Shutter time in microseconds. 0 hands control back to the AE algorithm.",
        auto_value=0,
    ),
    "AnalogueGain": ControlSpec(
        "AnalogueGain", "float",
        requires=("AnalogueGain",),
        description=(
            "Sensor analogue gain as a linear factor (1.0 = unity). "
            "The GUI sends decibels; the mapping is gain = 10 ** (dB / 20)."
        ),
        auto_value=0.0,
    ),
    "AeEnable": ControlSpec(
        "AeEnable", "bool", requires=("ExposureTime",),
        description="Enable the automatic exposure algorithm. Overrides ExposureTime/AnalogueGain.",
    ),
    "ExposureValue": ControlSpec(
        "ExposureValue", "float", lo=-8.0, hi=8.0, default=0.0,
        requires=("ExposureValue",),
        description="Exposure compensation in stops, only meaningful while AE is enabled.",
    ),
    "AwbEnable": ControlSpec(
        "AwbEnable", "bool", requires=("ColourGains",),
        description="Enable the automatic white balance algorithm.",
    ),
    "ColourGains": ControlSpec(
        "ColourGains", "floatpair", requires=("ColourGains",),
        description="Manual red/blue gains as [red, blue].",
    ),
    "ColourTemperature": ControlSpec(
        "ColourTemperature", "int", unit="K", requires=("ColourTemperature",),
        description="Correlated colour temperature hint for the AWB algorithm.",
    ),
    "FrameDurationLimits": ControlSpec(
        "FrameDurationLimits", "intpair", unit="us",
        requires=("FrameDurationLimits",),
        description="[min, max] frame duration in microseconds. Equal values lock the frame rate.",
    ),
    "NoiseReductionMode": ControlSpec(
        "NoiseReductionMode", "enum", default=0,
        requires=("NoiseReductionMode",),
        description="For quantitative work select 'off' or 'minimal'.",
    ),
    "AeExposureMode": ControlSpec("AeExposureMode", "enum", default=0, requires=("AeExposureMode",)),
    "AeMeteringMode": ControlSpec("AeMeteringMode", "enum", default=0, requires=("AeMeteringMode",)),
    "AwbMode": ControlSpec("AwbMode", "enum", default=0, requires=("AwbMode",)),
    "AfMode": ControlSpec("AfMode", "enum", default=0, requires=("AfMode",)),
    "AfTrigger": ControlSpec("AfTrigger", "enum", default=0, requires=("AfTrigger",)),
    "AfRange": ControlSpec("AfRange", "enum", default=0, requires=("AfRange",)),
    "AfSpeed": ControlSpec("AfSpeed", "enum", default=0, requires=("AfSpeed",)),
    "Brightness": ControlSpec(
        "Brightness", "float", lo=-1.0, hi=1.0, default=0.0,
        requires=("Brightness",),
        description="Applied after colour processing; changes pixel values, avoid for measurement.",
    ),
    "Contrast": ControlSpec("Contrast", "float", lo=0.0, hi=4.0, default=1.0, requires=("Contrast",)),
    "Saturation": ControlSpec("Saturation", "float", lo=0.0, hi=4.0, default=1.0, requires=("Saturation",)),
    "Sharpness": ControlSpec("Sharpness", "float", lo=0.0, hi=4.0, default=1.0, requires=("Sharpness",)),
    "ScalerCrop": ControlSpec(
        "ScalerCrop", "int4", requires=("ScalerCrop",),
        description="Sensor readout window [x, y, width, height] used to implement digital zoom.",
    ),
}

# Symbolic names for the enum controls. The value is the attribute name on the
# matching ``libcamera.controls`` enum class.
ENUM_OPTIONS: dict[str, dict[str, str]] = {
    "NoiseReductionMode": {
        "off": "Off", "fast": "Fast", "high_quality": "HighQuality",
        "minimal": "Minimal", "zsl": "ZSL",
    },
    "AeExposureMode": {"normal": "Normal", "short": "Short", "long": "Long", "custom": "Custom"},
    "AeMeteringMode": {"centre": "Centre", "spot": "Spot", "matrix": "Matrix", "custom": "Custom"},
    "AwbMode": {
        "auto": "Auto", "incandescent": "Incandescent", "tungsten": "Tungsten",
        "fluorescent": "Fluorescent", "indoor": "Indoor", "daylight": "Daylight",
        "cloudy": "Cloudy", "custom": "Custom",
    },
    "AfMode": {"manual": "Manual", "auto": "Auto", "continuous": "Continuous"},
    "AfTrigger": {"start": "Start", "cancel": "Cancel"},
    "AfRange": {"normal": "Normal", "macro": "Macro", "full": "Full"},
    "AfSpeed": {"normal": "Normal", "fast": "Fast"},
}

# Controls that only exist for autofocus-capable modules (the HQ Camera is not
# one of them), plus the special case of NoiseReductionMode living in
# ``libcamera.controls.draft``.
_LIBCAMERA_ENUM_MODULES = ("controls", "controls.draft")


def _enum_class(name: str):
    """Return the libcamera enum class backing a control, or ``None``."""
    if libcamera is None:
        return None
    for module_name in _LIBCAMERA_ENUM_MODULES:
        module = libcamera
        for part in module_name.split("."):
            module = getattr(module, part, None)
            if module is None:
                break
        if module is None:
            continue
        enum_class = getattr(module, f"{name}Enum", None)
        if enum_class is not None:
            return enum_class
    return None


def resolve_enum(name: str, value) -> int:
    """Turn ``"off"`` / ``3`` into the integer libcamera expects."""
    enum_class = _enum_class(name)
    if isinstance(value, str):
        options = ENUM_OPTIONS.get(name, {})
        attribute = options.get(value.strip().lower())
        if attribute is None:
            raise ValueError(
                f"{name} does not accept {value!r}; "
                f"known names: {', '.join(sorted(options)) or 'none'}"
            )
        if enum_class is None:
            raise ValueError(f"{name} symbolic values are unavailable without libcamera")
        try:
            return int(getattr(enum_class, attribute))
        except AttributeError as exc:
            raise ValueError(f"libcamera has no {name}.{attribute}") from exc
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    raise ValueError(f"{name} expects an integer or a symbolic name, got {type(value).__name__}")


def _as_pair(value, cast):
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return tuple(cast(v) for v in value)
    raise ValueError(f"expected a two element list, got {value!r}")


def _as_quad(value, cast):
    if isinstance(value, (list, tuple)) and len(value) == 4:
        return tuple(cast(v) for v in value)
    raise ValueError(f"expected a four element list, got {value!r}")


def _cast_scalar(kind: str, value):
    if kind == "bool":
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in {"true", "yes", "on", "1"}:
                return True
            if lowered in {"false", "no", "off", "0"}:
                return False
        raise ValueError(f"expected a boolean, got {value!r}")
    if kind == "int":
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, float):
            if not float(value).is_integer():
                raise ValueError(f"expected a whole number, got {value!r}")
            return int(value)
        if isinstance(value, int):
            return value
        if isinstance(value, str):
            return int(value, 10)
        raise ValueError(f"expected an integer, got {value!r}")
    if kind == "float":
        if isinstance(value, bool):
            return float(value)
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, str):
            return float(value)
        raise ValueError(f"expected a number, got {value!r}")
    raise ValueError(f"unsupported control kind {kind!r}")


def cast_control_value(spec: ControlSpec, value):
    """Coerce an incoming JSON value to the type libcamera expects."""
    if spec.kind == "enum":
        return resolve_enum(spec.name, value)
    if spec.kind == "floatpair":
        return _as_pair(value, float)
    if spec.kind == "intpair":
        return _as_pair(value, int)
    if spec.kind == "int4":
        return _as_quad(value, int)
    return _cast_scalar(spec.kind, value)


def control_limits(spec: ControlSpec, camera_controls: dict) -> tuple:
    """Best known ``(minimum, maximum, default)`` for a control."""
    limits = camera_controls.get(spec.name)
    if limits:
        minimum, maximum, default = limits[0], limits[1], limits[2]
    else:
        minimum, maximum, default = spec.lo, spec.hi, spec.default

    if minimum is None:
        minimum = spec.lo
    if maximum is None:
        maximum = spec.hi
    if default is None:
        default = spec.default

    # libcamera reports a 0 minimum for exposure/gain because 0 is the "auto"
    # sentinel, which is not a useful lower bound to advertise.
    if spec.name in {"ExposureTime", "AnalogueGain"} and minimum == 0 and maximum:
        minimum = maximum[0] if isinstance(maximum, (list, tuple)) else 1

    return minimum, maximum, default


def _clamp(value, minimum, maximum):
    if isinstance(value, (list, tuple)):
        return type(value)(
            _clamp(item, _item_limit(minimum, index), _item_limit(maximum, index))
            for index, item in enumerate(value)
        )
    if minimum is not None and value < minimum:
        return minimum
    if maximum is not None and value > maximum:
        return maximum
    return value


def _item_limit(limit, index):
    if isinstance(limit, (list, tuple)):
        if index < len(limit):
            return limit[index]
        return None
    return limit


def sanitize_controls(
    payload: dict,
    camera_controls: dict,
    *,
    clamp: bool = True,
) -> tuple[dict, dict, list[str]]:
    """Validate a ``{name: value}`` patch against the camera's advertised controls.

    Returns ``(applyable, normalized, warnings)`` where *applyable* is what gets
    handed to ``Picamera2.set_controls`` and *normalized* is what the service
    reports back as the current value.
    """
    applyable: dict = {}
    normalized: dict = {}
    warnings: list[str] = []

    if not isinstance(payload, dict):
        raise ValueError("controls payload must be a JSON object")

    for raw_name, raw_value in payload.items():
        name = str(raw_name)
        if name == "FrameRate":  # convenience alias handled by picamera2's Controls class
            spec = CONTROL_SPECS["FrameDurationLimits"]
            applyable[name] = raw_value
            normalized["FrameDurationLimits"] = raw_value
            warnings.append("FrameRate is stored as FrameDurationLimits and reported there")
            continue

        spec = CONTROL_SPECS.get(name)
        if spec is None:
            warnings.append(f"unknown control {name!r} ignored")
            continue

        missing = [entry for entry in spec.requires if entry not in camera_controls]
        if camera_controls and missing:
            warnings.append(f"{name} is not supported by this sensor")
            continue

        try:
            value = cast_control_value(spec, raw_value)
        except (ValueError, TypeError) as exc:
            warnings.append(f"{name}: {exc}")
            continue

        minimum, maximum, _ = control_limits(spec, camera_controls)
        if spec.auto_value is not None and value == spec.auto_value:
            # 0 for exposure/gain means "auto": a real mode, not a short exposure.
            pass
        elif minimum is not None or maximum is not None:
            clipped = _clamp(value, minimum, maximum)
            if clipped != value:
                if not clamp:
                    warnings.append(f"{name}={value} is outside {minimum}..{maximum}")
                    continue
                warnings.append(f"{name}={value} clamped to {clipped} (range {minimum}..{maximum})")
                value = clipped

        applyable[name] = value
        normalized[name] = value

    return applyable, normalized, warnings


def decibels_to_gain(decibels: float) -> float:
    """Convert an amplitude (power) ratio in dB to a linear gain factor."""
    return float(10.0 ** (float(decibels) / 20.0))


def gain_to_decibels(gain: float) -> float:
    """Inverse of :func:`decibels_to_gain`, guarding against a zero gain."""
    gain = float(gain)
    if gain <= 0.0:
        return float("-inf")
    return 20.0 * math.log10(gain)


# --------------------------------------------------------------------------- #
# Backend
# --------------------------------------------------------------------------- #


@dataclass
class StreamSettings:
    """Preview stream geometry and quality."""

    width: int = 1024
    height: int = 768
    fps: int = 25
    quality: str = "medium"

    def frame_duration_us(self) -> int:
        return int(round(1_000_000 / max(1, self.fps)))

    def as_dict(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "quality": self.quality,
            "frame_duration_us": self.frame_duration_us(),
        }


def _iso_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _safe_slug(text: str | None) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "-", (text or "").strip()).strip("-._")
    return slug[:60]


class PicameraBackend:
    """Owns the single :class:`Picamera2` instance and serialises access to it.

    ``picam2_factory`` is injected so that offline tests can supply a fake. Every
    public method is safe to call from an HTTP handler thread.

    Picamera2 quirk worth knowing: controls are attached to the *next* request
    that is submitted and the internal ``Controls`` object is then reset, so
    ``picam2.controls`` never reflects what is actually in effect. The set of
    controls this service applied is therefore tracked in :attr:`user_controls`
    and re-applied after every reconfigure (a reconfigure rebuilds ``Controls``
    from the configuration dictionary, dropping anything set later).
    """

    def __init__(
        self,
        stream: StreamSettings | None = None,
        *,
        capture_dir: str | Path = "captures",
        mjpeg_quality: str | None = None,
        bitrate: int | None = None,
        buffer_count: int = 3,
        picam2_factory=None,
        logger: logging.Logger | None = None,
    ):
        self.log = logger or logging.getLogger(__name__)
        self.stream = stream or StreamSettings()
        self.capture_dir = Path(capture_dir).expanduser()
        self.bitrate = bitrate
        self.buffer_count = max(1, int(buffer_count))
        self._picam2_factory = picam2_factory or self._default_factory

        self.mjpeg = MjpegFanoutOutput()
        self.user_controls: dict = {}
        self.open_warning: str | None = None

        self._lock = threading.RLock()
        self._capture_lock = threading.Lock()
        self._picam2 = None
        self._capture_count = 0
        self._closed = False
        self.encoder_kind: str | None = None
        self.sensor_modes_cache: list[dict] | None = None

    # -- lifecycle ---------------------------------------------------------- #

    @staticmethod
    def _default_factory():  # pragma: no cover - requires a Raspberry Pi
        from picamera2 import Picamera2

        return Picamera2()

    def open(self) -> "PicameraBackend":
        """Open the camera and start the preview stream.

        A missing camera is not fatal: :attr:`open_warning` is set, ``/info``
        keeps answering, and the failure is reported to clients instead.
        """
        with self._lock:
            if self._picam2 is not None:
                return self
            try:
                picam2 = self._picam2_factory()
            except Exception as exc:
                self.open_warning = f"camera unavailable: {exc}"
                self.log.warning("Could not open camera: %s", exc)
                return self

            self._picam2 = picam2
            try:
                self._configure_locked()
                # Read the sensor modes while the camera is configured but not
                # yet started: libcamera refuses the query on a running camera,
                # and no app has told us to keep the sensor busy.
                self._read_sensor_modes_locked()
                self._start_locked()
            except Exception as exc:
                self.open_warning = f"camera failed to start: {exc}"
                self.log.error("Could not start the camera: %s", exc)
                try:
                    picam2.close()
                except Exception:
                    pass
                self._picam2 = None
            return self

    def close(self) -> None:
        with self._lock:
            self._closed = True
            picam2 = self._picam2
            self._picam2 = None
        if picam2 is None:
            return
        try:
            picam2.stop()
        except Exception:
            pass
        try:
            picam2.close()
        except Exception:
            pass

    def is_open(self) -> bool:
        with self._lock:
            return self._picam2 is not None

    # -- camera configuration ---------------------------------------------- #

    def _video_configuration_locked(self):
        picam2 = self._require_camera()
        controls: dict = {}
        if "FrameDurationLimits" in picam2.camera_controls:
            duration = self.stream.frame_duration_us()
            controls["FrameDurationLimits"] = (duration, duration)
        if "NoiseReductionMode" in picam2.camera_controls:
            controls["NoiseReductionMode"] = resolve_enum("NoiseReductionMode", "fast")

        colour_space = None
        if libcamera is not None:
            # Rec709 is the wrong space for a microscope: the sensor is not
            # looking at a scene lit to broadcast primaries, and Sycc keeps the
            # transfer curve closer to linear-ish sRGB.
            colour_space = libcamera.ColorSpace.Sycc()

        return picam2.create_video_configuration(
            main={"size": (self.stream.width, self.stream.height), "format": "RGB888"},
            lores=None,
            raw=None,
            colour_space=colour_space,
            buffer_count=self.buffer_count,
            controls=controls,
            encode="main",
        )

    def _configure_locked(self) -> None:
        picam2 = self._require_camera()
        config = self._video_configuration_locked()
        picam2.configure(config)
        self._encoder_kind = None

    def _ensure_encoder_locked(self) -> None:
        """Start the hardware MJPEG encoder, falling back to software if needed."""
        if self._encoder_kind is not None:
            return
        picam2 = self._require_camera()

        candidates = ["mjpeg", "jpeg"]
        last_error: Exception | None = None
        for kind in candidates:
            try:
                if kind == "mjpeg":
                    from picamera2.encoders import MJPEGEncoder

                    encoder = MJPEGEncoder(bitrate=self.bitrate) if self.bitrate else MJPEGEncoder()
                else:
                    from picamera2.encoders import JpegEncoder

                    encoder = JpegEncoder(q=self.stream.quality)
            except Exception as exc:  # pragma: no cover - depends on the host
                last_error = exc
                continue

            try:
                picam2.start_encoder(encoder, self.mjpeg)
            except Exception as exc:  # pragma: no cover - depends on the host
                last_error = exc
                self.log.info("Encoder %s unavailable: %s", kind, exc)
                continue

            self._encoder_kind = kind
            self.log.info("MJPEG preview encoded with the %s encoder", kind)
            return

        raise CameraUnavailable(f"no MJPEG encoder available: {last_error}")

    def _start_locked(self) -> None:
        picam2 = self._require_camera()
        # Controls are only honoured when present in the configuration that the
        # camera was started with, and `start` wipes picam2.controls, so re-apply
        # the user's settings right after starting.
        if not picam2.started:
            picam2.start()
        if self.user_controls:
            picam2.set_controls(dict(self.user_controls))
        self._ensure_encoder_locked()

    def reconfigure(self, stream: StreamSettings) -> None:
        """Resize the preview stream, keeping the current controls."""
        with self._lock:
            picam2 = self._require_camera()
            self.stream = stream
            if self._encoder_kind is not None:
                try:
                    picam2.stop_encoder()
                except Exception:
                    pass
                self._encoder_kind = None
            picam2.stop()
            self._configure_locked()
            self._start_locked()

    def restart_preview(self) -> bool:
        """Tear the encoder and camera down and bring the preview back up.

        The v4l2 MJPEG encoder can stop emitting buffers without reporting an
        error -- observed on a Zero 2 W when memory ran short, and when the
        camera was restarted without stopping the encoder first. The encoder
        must be stopped *before* the camera or it keeps stale buffer file
        descriptors and dies with ``VIDIOC_QBUF: Invalid argument``. Returns
        ``True`` when the preview was restarted.
        """
        with self._lock:
            if self._picam2 is None:
                return False
            picam2 = self._picam2
            self.log.warning("Restarting the preview to recover the encoder")
            try:
                if self._encoder_kind is not None:
                    try:
                        picam2.stop_encoder()
                    except Exception:
                        pass
                    self._encoder_kind = None
                try:
                    picam2.stop()
                except Exception:
                    pass
                self._configure_locked()
                self._start_locked()
                return True
            except Exception as exc:
                self.open_warning = f"preview restart failed: {exc}"
                self.log.error("Could not restart the preview: %s", exc)
                return False

    # -- controls ----------------------------------------------------------- #

    def camera_controls(self) -> dict:
        with self._lock:
            if self._picam2 is None:
                return {}
            return dict(getattr(self._picam2, "camera_controls", {}) or {})

    def supported_controls(self) -> dict:
        """Describe every control this sensor and service accept."""
        available = self.camera_controls()
        result: dict = {}
        for name, spec in CONTROL_SPECS.items():
            if available and any(entry not in available for entry in spec.requires):
                continue
            minimum, maximum, default = control_limits(spec, available)
            entry = {
                "type": spec.kind,
                "min": minimum,
                "max": maximum,
                "default": default,
            }
            if spec.unit:
                entry["unit"] = spec.unit
            if spec.description:
                entry["description"] = spec.description
            if spec.kind == "enum":
                entry["options"] = ENUM_OPTIONS.get(name, {})
            result[name] = entry
        return result

    def apply_controls(self, payload: dict) -> dict:
        """Validate and apply a control patch, returning the effective values."""
        available = self.camera_controls()
        applyable, normalized, warnings = sanitize_controls(payload, available)

        with self._lock:
            if self._picam2 is not None:
                if applyable:
                    self._picam2.set_controls(applyable)
                self.user_controls.update(normalized)
            else:
                warnings.append("camera is not open; settings were not applied")

        return {
            "applied": normalized,
            "controls": self.current_controls(),
            "warnings": warnings,
        }

    def current_controls(self) -> dict:
        """Report the controls this service believes are in effect."""
        effective = dict(self.user_controls)
        available = self.camera_controls()
        for name, spec in CONTROL_SPECS.items():
            if any(entry not in available for entry in spec.requires):
                continue
            effective.setdefault(name, control_limits(spec, available)[2])
        return effective

    # -- frames and stills -------------------------------------------------- #

    def snapshot(self, timeout: float = 5.0) -> bytes | None:
        """Latest preview frame, waiting briefly if the encoder has not started."""
        frame, _ = self.mjpeg.latest_frame()
        if frame is not None:
            return frame
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            time.sleep(0.05)
            frame, _ = self.mjpeg.latest_frame()
            if frame is not None:
                return frame
        return None

    def still_configuration(self, raw: bool = True, size: tuple[int, int] | None = None):
        picam2 = self._require_camera()
        main: dict = {"format": "BGR888"}
        sensor_size = self._sensor_resolution()
        if size is not None:
            main["size"] = size
        elif sensor_size is not None:
            main["size"] = sensor_size
        return picam2.create_still_configuration(
            main=main,
            lores=None,
            raw={} if raw else None,
            buffer_count=1,
        )

    def capture_still(
        self,
        *,
        raw: bool = True,
        quality: int = DEFAULT_STILL_QUALITY,
        label: str | None = None,
        timeout: float = DEFAULT_CAPTURE_TIMEOUT_S,
        exif_data: dict | None = None,
    ) -> dict:
        """Full resolution still capture. Blocks, and interrupts the preview.

        The sensor has to be reconfigured for the still, so the MJPEG preview
        pauses for roughly a second. That is the price of a genuinely full
        resolution frame -- the preview cannot be cropped or upscaled into one.
        """
        if not self._capture_lock.acquire(timeout=max(1.0, timeout)):
            raise CameraUnavailable("another capture is already in progress")
        try:
            return self._capture_still_locked(
                raw=raw, quality=quality, label=label, timeout=timeout, exif_data=exif_data
            )
        finally:
            self._capture_lock.release()

    def _capture_still_locked(
        self,
        *,
        raw: bool,
        quality: int,
        label: str | None,
        timeout: float,
        exif_data: dict | None,
    ) -> dict:
        with self._lock:
            picam2 = self._require_camera()
            self.capture_dir.mkdir(parents=True, exist_ok=True)

            self._capture_count += 1
            stamp = time.strftime("%Y%m%d-%H%M%S")
            slug = _safe_slug(label)
            stem = f"{stamp}-{self._capture_count:04d}" + (f"-{slug}" if slug else "")

            jpeg_path = self.capture_dir / f"{stem}.jpg"
            dng_path = self.capture_dir / f"{stem}.dng"
            metadata: dict = {}

            try:
                try:
                    picam2.stop_encoder()
                except Exception:
                    pass
                self._encoder_kind = None
                picam2.stop()

                picam2.configure(self.still_configuration(raw=raw))
                picam2.start()

                if self.user_controls:
                    picam2.set_controls(dict(self.user_controls))

                original_quality = picam2.options.get("quality", DEFAULT_STILL_QUALITY)
                picam2.options["quality"] = int(quality)
                request = picam2.capture_request(wait=max(1.0, float(timeout)))
                try:
                    metadata = dict(request.get_metadata())
                    request.save("main", str(jpeg_path), exif_data=exif_data)
                    if raw:
                        try:
                            request.save_dng(str(dng_path))
                        except Exception as exc:
                            self.log.warning("DNG write failed: %s", exc)
                finally:
                    request.release()
                    picam2.options["quality"] = original_quality
            finally:
                # Put the preview back the way it was, whatever happened. An
                # aborted capture must not leave the sensor stuck in still mode.
                try:
                    picam2.stop()
                    self._configure_locked()
                    picam2.start()
                    if self.user_controls:
                        picam2.set_controls(dict(self.user_controls))
                    self._ensure_encoder_locked()
                except Exception as exc:  # pragma: no cover - recovery path
                    self.log.error("Could not restore the preview after capture: %s", exc)

        files: dict[str, dict] = {}
        if jpeg_path.exists():
            files["jpeg"] = {
                "name": jpeg_path.name,
                "url": f"/captures/{jpeg_path.name}",
                "bytes": jpeg_path.stat().st_size,
            }
        if dng_path.exists():
            files["raw"] = {
                "name": dng_path.name,
                "url": f"/captures/{dng_path.name}",
                "bytes": dng_path.stat().st_size,
            }

        result = {
            "ok": True,
            "timestamp": _iso_now(),
            "label": label,
            "files": files,
            "controls": dict(self.user_controls),
            "metadata": _json_safe(metadata),
            "stream_configuration": {
                "main": self._stream_config_dict("main"),
                "raw": self._stream_config_dict("raw"),
            },
        }
        self._append_index(stem, result)
        return result

    def _stream_config_dict(self, name: str) -> dict | None:
        with self._lock:
            if self._picam2 is None:
                return None
            config = getattr(self._picam2, "camera_config", None) or {}
            stream = config.get(name)
        if not stream:
            return None
        return _json_safe(
            {
                "format": stream.get("format"),
                "size": stream.get("size"),
                "stride": stream.get("stride"),
            }
        )

    def _append_index(self, stem: str, result: dict) -> None:
        """Append one line per capture so measurements can be traced back to settings."""
        index_path = self.capture_dir / "shots.csv"
        fields = [
            "stem", "timestamp", "label", "jpeg", "dng",
            "ExposureTime", "AnalogueGain", "ColourGains", "ColourTemperature",
            "SensorTimestamp", "Lux", "FrameDuration",
        ]
        metadata = result.get("metadata") or {}
        controls = result.get("controls") or {}
        row = {
            "stem": stem,
            "timestamp": result.get("timestamp"),
            "label": result.get("label") or "",
            "jpeg": result["files"].get("jpeg", {}).get("name", ""),
            "dng": result["files"].get("raw", {}).get("name", ""),
            "ExposureTime": controls.get("ExposureTime", ""),
            "AnalogueGain": controls.get("AnalogueGain", ""),
            "ColourGains": _format_cell(controls.get("ColourGains", "")),
            "ColourTemperature": metadata.get("ColourTemperature", ""),
            "SensorTimestamp": metadata.get("SensorTimestamp", ""),
            "Lux": metadata.get("Lux", ""),
            "FrameDuration": metadata.get("FrameDuration", ""),
        }
        try:
            new_file = not index_path.exists()
            with index_path.open("a", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                if new_file:
                    writer.writeheader()
                writer.writerow(row)
        except Exception as exc:  # pragma: no cover - best effort
            self.log.warning("Could not append to %s: %s", index_path, exc)

    # -- introspection ------------------------------------------------------ #

    def _require_camera(self):
        if self._picam2 is None:
            raise CameraUnavailable(self.open_warning or "camera is not open")
        return self._picam2

    def _sensor_resolution(self) -> tuple[int, int] | None:
        with self._lock:
            if self._picam2 is None:
                return None
            properties = getattr(self._picam2, "camera_properties", None) or {}
            resolution = properties.get("PixelArraySize")
        if resolution and len(resolution) == 2:
            return (int(resolution[0]), int(resolution[1]))
        return None

    def device_id(self) -> str | None:
        """A stable identifier for this sensor.

        ``camera_properties`` covers most keys libcamera reports, but the unique
        camera ``Id`` (for example ``/base/soc/i2c0mux/i2c@1/imx477@1a``) only
        appears in the global camera info, so it is looked up there as well.
        Two identical sensors on one Pi are otherwise indistinguishable, since
        both report ``Model`` as ``imx477``.
        """
        with self._lock:
            if self._picam2 is None:
                return None
            properties = dict(getattr(self._picam2, "camera_properties", None) or {})
            try:
                index = self._picam2.camera_idx
            except Exception:
                index = None

        if "Id" not in properties and index is not None and _Picamera2 is not None:
            try:
                for entry in _Picamera2.global_camera_info():
                    if entry.get("Num") == index and entry.get("Id"):
                        properties["Id"] = entry["Id"]
                        break
            except Exception:
                pass

        for key in ("Id", "SensorModel", "Model"):
            value = properties.get(key)
            if value:
                return str(value)
        return None

    def _read_sensor_modes_locked(self) -> None:
        """Populate the sensor mode cache. The camera must not be running.

        ``Picamera2.sensor_modes`` enumerates modes by calling ``configure()``
        once per raw format with its own ``create_preview_configuration()`` and
        never restores the caller's configuration. That leaves the camera set up
        for picamera2's default 640x480 preview, so the preview after this call
        must be reconfigured or it silently encodes at 640x480 instead of the
        requested size.

        The query has to happen while the camera is not running, because
        libcamera refuses the reconfigure once frames are flowing. Restarting a
        running preview afterwards is *not* safe either: the v4l2 MJPEG encoder
        keeps its file descriptors to buffers that no longer exist and dies with
        ``VIDIOC_QBUF: [Errno 22]``, leaving clients waiting on a stream that
        never produces another frame. So the query happens once, before the
        first ``start()``, and its result is cached.
        """
        if self.sensor_modes_cache is not None or self._picam2 is None:
            return
        try:
            modes = list(getattr(self._picam2, "sensor_modes", []) or [])
        except Exception as exc:
            self.log.warning("Could not list sensor modes: %s", exc)
            return

        # Undo the preview configuration left behind by the mode probe.
        try:
            self._configure_locked()
        except Exception as exc:
            self.log.warning("Could not restore the preview configuration: %s", exc)

        self.sensor_modes_cache = [
            {
                "index": index,
                "size": _json_safe(mode.get("size")),
                "bit_depth": mode.get("bit_depth"),
                "format": _json_safe(mode.get("format")),
                "unpacked": _json_safe(mode.get("unpacked")),
                "fps": mode.get("fps"),
                "crop_limits": _json_safe(mode.get("crop_limits")),
                "exposure_limits": _json_safe(mode.get("exposure_limits")),
            }
            for index, mode in enumerate(modes)
        ]
        self.log.info("Sensor offers %d modes", len(self.sensor_modes_cache))

    def sensor_modes(self) -> list[dict]:
        """Cached sensor modes, or an empty list if they could not be read."""
        if self.sensor_modes_cache is None:
            with self._lock:
                self._read_sensor_modes_locked()
        return self.sensor_modes_cache or []

    def info(self) -> dict:
        with self._lock:
            properties = {}
            if self._picam2 is not None:
                properties = dict(getattr(self._picam2, "camera_properties", None) or {})

        return {
            "service": SERVICE_NAME,
            "version": SERVICE_VERSION,
            "api_version": API_VERSION,
            "hostname": socket.gethostname(),
            "timestamp": _iso_now(),
            "camera": {
                "open": self.is_open(),
                "model": properties.get("Model"),
                "device_id": self.device_id(),
                "sensor_resolution": self._sensor_resolution(),
                "warning": self.open_warning,
            },
            "stream": {
                **self.stream.as_dict(),
                # The size libcamera actually negotiated, which can differ from
                # the requested size after the pipeline handler adjusts it.
                "main": self._stream_config_dict("main"),
                "encoder": self._encoder_kind,
                "bitrate_bps": self.bitrate,
                "mjpeg_clients": self.mjpeg.client_count,
                "mjpeg_frames": self.mjpeg.frames_sent,
                "mjpeg_bytes": self.mjpeg.bytes_sent,
                "mjpeg_frames_dropped": self.mjpeg.frames_dropped,
                "last_frame_age_s": (
                    None
                    if self.mjpeg.last_frame_time is None
                    else round(time.monotonic() - self.mjpeg.last_frame_time, 3)
                ),
                "boundary": DEFAULT_MJPEG_BOUNDARY,
            },
            "sensor_modes": self.sensor_modes(),
            "controls": self.supported_controls(),
            "current_controls": self.current_controls(),
            "capture_dir": str(self.capture_dir),
            "captures": self.capture_count(),
            "capabilities": {
                # The preview is for looking at; it is lossy and must not be used
                # for photometric measurements.
                "preview_mjpeg": True,
                "snapshot_jpeg": True,
                # This is the measurement path.
                "still_jpeg_full_resolution": True,
                "still_dng_raw": True,
                "libcamera_metadata": True,
                "capture_index_csv": True,
                "measurement_path": "POST /capture (optionally raw=1) then GET /captures/<name>",
                "note": (
                    "Still captures pause the preview while the sensor is reconfigured; "
                    "use the DNG plus the returned metadata for quantitative work."
                ),
            },
        }

    def capture_count(self) -> int:
        try:
            names = [p.name for p in self.capture_dir.iterdir() if p.suffix in {".jpg", ".dng"}]
        except FileNotFoundError:
            return 0
        except OSError:
            return 0
        return len(names)

    def list_captures(self) -> list[dict]:
        try:
            entries = sorted(
                (p for p in self.capture_dir.iterdir() if p.suffix in {".jpg", ".dng"}),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
        except FileNotFoundError:
            return []
        result = []
        for path in entries:
            stat = path.stat()
            result.append(
                {
                    "name": path.name,
                    "bytes": stat.st_size,
                    "modified": datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat(
                        timespec="seconds"
                    ),
                    "url": f"/captures/{path.name}",
                }
            )
        return result

    def resolve_capture(self, name: str) -> Path | None:
        """Map a client supplied name to a file inside the capture directory."""
        candidate = Path(unquote(name)).name
        if not candidate or candidate in {".", ".."}:
            return None
        if candidate != name.split("/")[-1]:
            return None
        path = (self.capture_dir / candidate).resolve()
        try:
            root = self.capture_dir.resolve()
        except OSError:
            return None
        if root not in path.parents:
            return None
        if not path.is_file():
            return None
        if path.suffix.lower() not in {".jpg", ".jpeg", ".dng", ".png", ".csv"}:
            return None
        return path


def _format_cell(value) -> str:
    if isinstance(value, (list, tuple)):
        return ";".join(_format_cell(item) for item in value)
    return str(value)


def _json_safe(value):
    """Best effort conversion of libcamera/metadata values into JSON types."""
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    try:
        return [_json_safe(item) for item in value]
    except TypeError:
        return str(value)


# --------------------------------------------------------------------------- #
# HTTP layer
# --------------------------------------------------------------------------- #


@dataclass
class ServiceConfig:
    """Everything the HTTP layer needs to know."""

    host: str = "0.0.0.0"
    port: int = DEFAULT_PORT
    device_name: str = "Pi HQ Camera"
    allow_stream_clients: int = 4
    advertise: bool = True
    capture_timeout_s: float = DEFAULT_CAPTURE_TIMEOUT_S
    stream_stall_timeout_s: float = DEFAULT_STREAM_STALL_TIMEOUT_S
    logger: logging.Logger = field(default_factory=lambda: logging.getLogger("picamera-service"))


class CameraRequestHandler(BaseHTTPRequestHandler):
    """Request handler for the camera service.

    The backend lives on the server object (``self.server.backend``) so that the
    handler stays stateless and is easy to reason about.
    """

    protocol_version = "HTTP/1.1"
    server_version = f"{SERVICE_NAME}/{SERVICE_VERSION}"
    sys_version = ""

    # -- plumbing ----------------------------------------------------------- #

    @property
    def backend(self) -> PicameraBackend:
        return self.server.backend  # type: ignore[attr-defined]

    @property
    def config(self) -> ServiceConfig:
        return self.server.config  # type: ignore[attr-defined]

    def log_message(self, fmt, *args):  # noqa: D102 - quieter than the default
        self.config.logger.debug("%s - %s", self.address_string(), fmt % args)

    def log_error(self, fmt, *args):  # noqa: D102
        self.config.logger.warning("%s - %s", self.address_string(), fmt % args)

    # -- responses ---------------------------------------------------------- #

    def _send_json(self, payload, status: int = 200) -> None:
        body = json.dumps(payload, default=str).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):  # pragma: no cover
            pass

    def _send_error_json(self, message: str, status: int) -> None:
        self._send_json({"ok": False, "error": message}, status=status)

    def _read_body(self, limit: int = 1_000_000) -> bytes:
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        if length <= 0:
            return b""
        if length > limit:
            raise ValueError(f"request body too large ({length} bytes)")
        return self.rfile.read(length)

    def _read_json_body(self) -> dict:
        raw = self._read_body()
        if not raw:
            return {}
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid JSON body: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    # -- routing ------------------------------------------------------------ #

    def do_GET(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        parsed = urlparse(self.path)
        route = parsed.path.rstrip("/") or "/"
        query = parse_qs(parsed.query)

        try:
            if route == "/":
                self._handle_index()
            elif route == "/info":
                self._send_json(self.backend.info())
            elif route == "/status":
                self._handle_status()
            elif route == "/control":
                self._send_json(
                    {
                        "controls": self.backend.supported_controls(),
                        "current": self.backend.current_controls(),
                    }
                )
            elif route == "/stream.mjpg" or route == "/stream":
                self._handle_stream()
            elif route == "/snapshot.jpg" or route == "/snapshot":
                self._handle_snapshot()
            elif route == "/captures":
                self._send_json({"captures": self.backend.list_captures()})
            elif route.startswith("/captures/"):
                self._handle_capture_download(parsed.path[len("/captures/"):])
            else:
                self._send_error_json(f"unknown endpoint {route}", status=404)
        except CameraUnavailable as exc:
            self._send_error_json(str(exc), status=503)
        except ValueError as exc:
            self._send_error_json(str(exc), status=400)
        except (BrokenPipeError, ConnectionResetError):  # pragma: no cover
            pass
        except Exception as exc:  # pragma: no cover - defensive
            self.log_error("Unhandled error on %s: %r", self.path, exc)
            self._send_error_json(f"internal error: {exc}", status=500)

    def do_POST(self):  # noqa: N802
        parsed = urlparse(self.path)
        route = parsed.path.rstrip("/") or "/"
        query = parse_qs(parsed.query)

        try:
            if route == "/control":
                payload = self._read_json_body()
                if "controls" in payload and isinstance(payload["controls"], dict):
                    payload = payload["controls"]
                self._send_json(self.backend.apply_controls(payload))
            elif route == "/capture":
                raw = _query_flag(query, "raw", default=True)
                label = _query_first(query, "label")
                quality = _query_int(query, "quality", DEFAULT_STILL_QUALITY)
                if not self.backend.is_open():
                    raise CameraUnavailable(self.backend.open_warning or "camera is not open")
                result = self.backend.capture_still(
                    raw=raw,
                    quality=quality,
                    label=label,
                    timeout=self.config.capture_timeout_s,
                )
                self._send_json(result)
            elif route == "/reconfigure":
                payload = self._read_json_body()
                stream = StreamSettings(
                    width=int(payload.get("width", self.backend.stream.width)),
                    height=int(payload.get("height", self.backend.stream.height)),
                    fps=int(payload.get("fps", self.backend.stream.fps)),
                    quality=str(payload.get("quality", self.backend.stream.quality)),
                )
                self.backend.reconfigure(stream)
                self._send_json({"ok": True, "stream": stream.as_dict()})
            else:
                self._send_error_json(f"unknown endpoint {route}", status=404)
        except CameraUnavailable as exc:
            self._send_error_json(str(exc), status=503)
        except ValueError as exc:
            self._send_error_json(str(exc), status=400)
        except (BrokenPipeError, ConnectionResetError):  # pragma: no cover
            pass
        except Exception as exc:  # pragma: no cover - defensive
            self.log_error("Unhandled error on %s: %r", self.path, exc)
            self._send_error_json(f"internal error: {exc}", status=500)

    # -- individual endpoints ---------------------------------------------- #

    def _handle_index(self) -> None:
        info = self.backend.info()
        camera = info["camera"]
        stream = info["stream"]
        warning = ""
        if camera.get("warning"):
            warning = f"<em> - {escape_html(str(camera['warning']))}</em>"
        html = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>{SERVICE_NAME} - {escape_html(self.config.device_name)}</title>
<style>
 body {{ font-family: system-ui, sans-serif; margin: 1.5rem; background: #101418; color: #e6edf3; }}
 img {{ max-width: 100%; border: 1px solid #30363d; border-radius: 6px; background: #000; }}
 code {{ color: #7ee787; }} a {{ color: #79c0ff; }}
</style></head><body>
<h1>{SERVICE_NAME} <small>v{SERVICE_VERSION}</small></h1>
<p>Camera: <strong>{escape_html(str(camera.get("model")))}</strong>
   (device id <code>{escape_html(str(camera.get("device_id")))}</code>,
   sensor {escape_html(str(camera.get("sensor_resolution")))}){warning}</p>
<img src="/stream.mjpg" alt="live preview">
<p>Preview {stream["width"]}x{stream["height"]} @ {stream["fps"]} fps,
   encoder <code>{escape_html(str(stream["encoder"]))}</code>,
   {stream["mjpeg_clients"]} viewer(s).</p>
<ul>
 <li><a href="/info">/info</a> - full configuration, sensor modes and control ranges (JSON)</li>
 <li><code>GET /stream.mjpg</code> - MJPEG preview, for display only</li>
 <li><code>POST /capture?raw=1&amp;label=name</code> - full resolution still, this is the measurement path</li>
 <li><a href="/captures">/captures</a> - list and download full resolution images</li>
</ul>
</body></html>
"""
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):  # pragma: no cover
            pass

    def _handle_status(self) -> None:
        info = self.backend.info()
        self._send_json(
            {
                "ok": True,
                "camera_open": info["camera"]["open"],
                "warning": info["camera"]["warning"],
                "stream": info["stream"],
                "captures": info["captures"],
                "timestamp": info["timestamp"],
            }
        )

    def _handle_stream(self) -> None:
        if not self.backend.is_open():
            raise CameraUnavailable(self.backend.open_warning or "camera is not open")

        client_count = self.backend.mjpeg.client_count
        if client_count >= self.config.allow_stream_clients:
            raise CameraUnavailable(
                f"too many preview viewers ({client_count}); limit is {self.config.allow_stream_clients}"
            )

        boundary = DEFAULT_MJPEG_BOUNDARY
        client_id, client_queue = self.backend.mjpeg.register_client()
        self.log_message("MJPEG viewer connected (%d total)", self.backend.mjpeg.client_count)

        try:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.end_headers()

            stalls = 0
            while True:
                frame = self.backend.mjpeg.next_frame(client_queue, DEFAULT_STREAM_POLL_S)
                if frame is None:
                    if not self.backend.is_open():
                        break
                    # A dead encoder never produces another frame, so waiting
                    # forever would leave the client hanging on a frozen image.
                    timeout = self.config.stream_stall_timeout_s
                    if timeout > 0 and self.backend.mjpeg.seconds_since_last_frame() > timeout:
                        stalls += 1
                        if stalls > 1 or not self.backend.restart_preview():
                            self.log_error(
                                "preview produced no frames for %.1fs; closing the stream",
                                timeout,
                            )
                            break
                        self.log_message("preview stalled; restarted the encoder, still streaming")
                    continue
                stalls = 0
                header = (
                    f"--{boundary}\r\n"
                    "Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n\r\n"
                ).encode("ascii")
                self.wfile.write(header)
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.backend.mjpeg.unregister_client(client_id)
            self.close_connection = True
            self.log_message("MJPEG viewer disconnected (%d left)", self.backend.mjpeg.client_count)

    def _handle_snapshot(self) -> None:
        if not self.backend.is_open():
            raise CameraUnavailable(self.backend.open_warning or "camera is not open")
        frame = self.backend.snapshot(timeout=5.0)
        if frame is None:
            self._send_error_json("no preview frame available yet", status=503)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(frame)
        except (BrokenPipeError, ConnectionResetError):  # pragma: no cover
            pass

    def _handle_capture_download(self, raw_name: str) -> None:
        path = self.backend.resolve_capture(raw_name)
        if path is None:
            self._send_error_json("capture not found", status=404)
            return
        data = path.read_bytes()
        content_type = {
            ".jpg": "image/jpeg",
            ".jpeg": "image/jpeg",
            ".png": "image/png",
            ".dng": "image/x-adobe-dng",
            ".csv": "text/csv",
        }.get(path.suffix.lower(), "application/octet-stream")
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):  # pragma: no cover
            pass


def _query_first(query: dict, name: str, default=None):
    values = query.get(name)
    if not values:
        return default
    return values[0]


def _query_int(query: dict, name: str, default: int) -> int:
    raw = _query_first(query, name)
    if raw is None or raw == "":
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer") from exc


def _query_flag(query: dict, name: str, default: bool = False) -> bool:
    raw = _query_first(query, name)
    if raw is None:
        return default
    return str(raw).strip().lower() in {"1", "true", "yes", "on", ""}


def escape_html(text: str) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


class CameraHTTPServer(ThreadingHTTPServer):
    """Threaded HTTP server that owns the camera backend."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, backend: PicameraBackend, config: ServiceConfig):
        super().__init__(address, CameraRequestHandler)
        self.backend = backend
        self.config = config


def build_server(backend: PicameraBackend, config: ServiceConfig) -> CameraHTTPServer:
    return CameraHTTPServer((config.host, config.port), backend, config)


def serve_forever(backend: PicameraBackend, config: ServiceConfig) -> None:
    server = build_server(backend, config)
    advertiser = None
    if config.advertise:
        advertiser = advertise(backend, config)
    try:
        config.logger.info(
            "Serving %s v%s on http://%s:%d/",
            SERVICE_NAME,
            SERVICE_VERSION,
            config.host,
            config.port,
        )
        server.serve_forever()
    finally:
        server.server_close()
        if advertiser is not None:
            try:
                advertiser.close()
            except Exception:  # pragma: no cover - best effort
                pass


# --------------------------------------------------------------------------- #
# mDNS advertisement
# --------------------------------------------------------------------------- #


def advertise(backend: PicameraBackend, config: ServiceConfig):
    """Announce the service over mDNS so clients never need to know the IP.

    Discovery is the only sane answer to a Pi whose address comes from DHCP or
    from USB gadget mode: the client browses for ``_picamera._tcp.local.``
    instead of being configured with a hostname. Missing ``zeroconf`` is not an
    error -- the service still works, users just have to type the address.
    """
    try:  # pragma: no cover - depends on the host
        from zeroconf import ServiceInfo, Zeroconf
    except Exception:
        config.logger.warning(
            "zeroconf is not installed; skipping mDNS advertisement "
            "(clients must be given the IP address by hand)"
        )
        return None

    try:  # pragma: no cover - depends on the host
        address = socket.inet_aton(_primary_ipv4())
        info = ServiceInfo(
            "_picamera._tcp.local.",
            f"{config.device_name}._picamera._tcp.local.",
            addresses=[address],
            port=config.port,
            properties={
                "version": SERVICE_VERSION,
                "api": str(API_VERSION),
                "model": str((backend.info()["camera"].get("model")) or ""),
                "device_id": str(backend.device_id() or ""),
                "path": "/",
            },
            server=f"{socket.gethostname()}.local.",
        )
        zeroconf = Zeroconf()
        zeroconf.register_service(info)
        config.logger.info("Advertising %s over mDNS", info.name)
        return zeroconf
    except Exception as exc:  # pragma: no cover - depends on the host
        config.logger.warning("Could not advertise over mDNS: %s", exc)
        return None


def _primary_ipv4() -> str:  # pragma: no cover - depends on the host
    """Best guess at the interface clients can actually reach."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def discover_services(timeout: float = 3.0) -> list[dict]:
    """Browse the local network for running camera services.

    Used by the GUI when ``zeroconf`` is available, and by the CLI ``--scan``
    flag. Returns a list of ``{"host", "port", "name", "properties"}``.
    """
    try:  # pragma: no cover - depends on the host
        from zeroconf import ServiceBrowser, Zeroconf
    except Exception:
        return []

    found: list[dict] = []

    class _Listener:  # pragma: no cover - depends on the host
        def add_service(self, zeroconf, service_type, name):
            info = zeroconf.get_service_info(service_type, name, timeout=2000)
            if not info:
                return
            addresses = [
                socket.inet_ntoa(addr) for addr in info.addresses if len(addr) == 4
            ]
            found.append(
                {
                    "name": info.name,
                    "host": addresses[0] if addresses else info.server,
                    "port": info.port,
                    "properties": {
                        key.decode("utf-8", "replace") if isinstance(key, bytes) else str(key):
                        (value.decode("utf-8", "replace") if isinstance(value, bytes) else value)
                        for key, value in (info.properties or {}).items()
                    },
                }
            )

        def update_service(self, *args, **kwargs):
            return None

        def remove_service(self, *args, **kwargs):
            return None

    zeroconf = Zeroconf()
    try:  # pragma: no cover - depends on the host
        ServiceBrowser(zeroconf, "_picamera._tcp.local.", _Listener())
        time.sleep(timeout)
    finally:
        zeroconf.close()
    return found


__all__ = [
    "API_VERSION",
    "CameraHTTPServer",
    "CameraRequestHandler",
    "CameraUnavailable",
    "CONTROL_SPECS",
    "ControlSpec",
    "DEFAULT_MJPEG_BOUNDARY",
    "MjpegFanoutOutput",
    "PicameraBackend",
    "SERVICE_NAME",
    "SERVICE_VERSION",
    "ServiceConfig",
    "StreamSettings",
    "advertise",
    "build_server",
    "cast_control_value",
    "control_limits",
    "decibels_to_gain",
    "discover_services",
    "gain_to_decibels",
    "resolve_enum",
    "sanitize_controls",
    "serve_forever",
]
