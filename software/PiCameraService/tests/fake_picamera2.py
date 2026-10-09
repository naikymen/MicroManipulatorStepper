"""A stand-in for ``picamera2`` and ``libcamera`` so the service can be tested off-Pi.

The real service imports ``picamera2.outputs`` at import time and re-exports
hardware encoders lazily, so installing these fakes into :data:`sys.modules`
before importing ``picamera_service`` exercises the same code paths that run on
a Raspberry Pi -- including ``MjpegFanoutOutput`` subclassing the real
``Output`` base class.

Everything here is intentionally dumb: the fakes record what was asked of them
so the tests can assert on it, and never touch hardware.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

# --------------------------------------------------------------------------- #
# Encoders
# --------------------------------------------------------------------------- #


class MJPEGEncoder:
    def __init__(self, bitrate=None, **kwargs):
        self.bitrate = bitrate
        self.extra = kwargs
        self.started = False
        self.quality = None

    def start(self, quality=None):
        self.started = True
        self.quality = quality

    def stop(self):
        self.started = False


class JpegEncoder:
    def __init__(self, q=None, **kwargs):
        self.q = q
        self.extra = kwargs
        self.started = False


class H264Encoder:
    def __init__(self, bitrate=None, **kwargs):
        self.bitrate = bitrate
        self.started = False


class MJPEGEncoderUnavailable(MJPEGEncoder):
    """Encoder stand-in that fails at construction, mimicking a software-only platform."""


# --------------------------------------------------------------------------- #
# Outputs
# --------------------------------------------------------------------------- #


class FakeOutput:
    """Mirror of ``picamera2.outputs.Output`` with the members the service uses."""

    def __init__(self, pts=None):
        self._pts_output = None
        self.recording = False
        self.needs_pacing = False
        self.needs_add_stream = False
        self.ptsoutput = pts
        self.added_streams = []

    def start(self):
        self.recording = True

    def stop(self):
        self.recording = False

    def outputframe(self, frame, keyframe=True, timestamp=None, packet=None, audio=False):
        return None

    def outputtimestamp(self, timestamp):
        return None

    def _add_stream(self, encoder_stream, *args, **kwargs):
        self.added_streams.append((encoder_stream, args, kwargs))

    @property
    def ptsoutput(self):
        return self._pts_output

    @ptsoutput.setter
    def ptsoutput(self, value):
        self._pts_output = value


FileOutput = FakeOutput


# --------------------------------------------------------------------------- #
# libcamera fakes
# --------------------------------------------------------------------------- #


class _NoiseReductionModeEnum:
    Off = 0
    Fast = 1
    HighQuality = 2
    Minimal = 3
    ZSL = 4


class _AeExposureModeEnum:
    Normal = 0
    Short = 1
    Long = 2
    Custom = 3


class _AeMeteringModeEnum:
    Centre = 0
    Spot = 1
    Matrix = 2
    Custom = 3


class _AwbModeEnum:
    Auto = 0
    Incandescent = 1
    Tungsten = 2
    Fluorescent = 3
    Indoor = 4
    Daylight = 5
    Cloudy = 6
    Custom = 7


class _ColorSpace:
    @staticmethod
    def Sycc():
        return "Sycc"

    @staticmethod
    def Smpte170m():
        return "Smpte170m"

    @staticmethod
    def Rec709():
        return "Rec709"


# --------------------------------------------------------------------------- #
# Picamera2 fake
# --------------------------------------------------------------------------- #


def default_camera_controls() -> dict:
    """A plausible IMX477 control set, with the AF controls deliberately absent."""
    return {
        "ExposureTime": (30, 67000000, 30000),
        "AnalogueGain": (1.0, 22.26, 1.0),
        "AeEnable": (False, True, True),
        "ExposureValue": (-8.0, 8.0, 0.0),
        "AwbEnable": (False, True, True),
        "ColourGains": ((0.0, 0.0), (32.0, 32.0), (1.0, 1.0)),
        "ColourTemperature": (100, 100000, 4500),
        "FrameDurationLimits": (1000, 1000000, 33333),
        "NoiseReductionMode": (0, 4, 1),
        "Brightness": (-1.0, 1.0, 0.0),
        "Contrast": (0.0, 4.0, 1.0),
        "Saturation": (0.0, 4.0, 1.0),
        "Sharpness": (0.0, 4.0, 1.0),
        "ScalerCrop": ((0, 0, 1, 1), (0, 0, 4056, 3040), (0, 0, 4056, 3040)),
    }


class FakeRequest:
    def __init__(self, picam2):
        self.picam2 = picam2
        self.released = False
        self.metadata = {
            "SensorTimestamp": 1234567890123,
            "ExposureTime": 12000,
            "AnalogueGain": 4.0,
            "ColourTemperature": 4600,
            "Lux": 12.5,
            "FrameDuration": 40000,
        }

    def get_metadata(self):
        return dict(self.metadata)

    def save(self, name, file_output, format=None, exif_data=None):  # noqa: A002
        Path(str(file_output)).write_bytes(b"\xff\xd8fake-jpeg-payload\xff\xd9")

    def save_dng(self, file_output, name="raw"):
        Path(str(file_output)).write_bytes(b"FAKEDNG" * 32)

    def release(self):
        self.released = True


class FakePicamera2:
    """Records every call so tests can assert on the resulting configuration."""

    instances: list["FakePicamera2"] = []

    def __init__(self, *args, **kwargs):
        self.camera_controls = default_camera_controls()
        # Real hardware does not expose "Id" here; it only appears in
        # global_camera_info(), which is why device_id() has a fallback.
        self.camera_properties = {
            "Model": "imx477",
            "PixelArraySize": (4056, 3040),
            "Rotation": 180,
            "PipelineHandler": "rpi/vc4",
        }
        self.camera_idx = 0
        self._sensor_modes = [
            {
                "size": (1332, 990),
                "bit_depth": 10,
                "format": "SBGGR10_CSI2P",
                "unpacked": "SBGGR10",
                "fps": 120.13,
                "crop_limits": (0, 0, 2664, 1980),
                "exposure_limits": (13, 6737430, None),
            },
            {
                "size": (2028, 1520),
                "bit_depth": 12,
                "format": "SBGGR12_CSI2P",
                "unpacked": "SBGGR12",
                "fps": 50.03,
                "crop_limits": (0, 0, 4056, 3040),
                "exposure_limits": (17, 6940285, None),
            },
            {
                "size": (4056, 3040),
                "bit_depth": 12,
                "format": "SBGGR12_CSI2P",
                "unpacked": "SBGGR12",
                "fps": 10.0,
                "crop_limits": (0, 0, 4056, 3040),
                "exposure_limits": (27, 11057975, None),
            },
        ]
        self.mode_probe_calls = 0
        self.options = {"quality": 90}
        self.camera_config = None
        self.started = False
        self.closed = False
        self.configure_calls: list[dict] = []
        self.started_encoders: list[str] = []
        self.encoder_stop_count = 0
        self.control_history: list[dict] = []
        self.applied_controls: dict = {}
        self.capture_count = 0
        self.requests: list[FakeRequest] = []
        self.fail_encoders: set[str] = set()
        self.video_kwargs: dict | None = None
        self.still_kwargs: dict | None = None
        FakePicamera2.instances.append(self)

    @staticmethod
    def global_camera_info():
        return [
            {
                "Model": "imx477",
                "Location": 2,
                "Rotation": 180,
                "Id": "/base/soc/i2c0mux/i2c@1/imx477@1a",
                "Num": 0,
            }
        ]

    @property
    def sensor_modes(self):
        """Mirror the real Picamera2.sensor_modes side effect.

        The real implementation enumerates modes by calling ``configure()`` once
        per raw format with its own ``create_preview_configuration()`` and never
        restores the caller's configuration, leaving the camera on picamera2's
        640x480 preview. Reproducing that here is what makes the regression test
        for preview sizing meaningful.
        """
        if self._sensor_modes is None:
            return None
        if self.mode_probe_calls == 0:
            self.mode_probe_calls += 1
            self.camera_config = {
                "main": {
                    "format": "XBGR8888",
                    "size": (640, 480),
                    "stride": 2560,
                    "framesize": 640 * 480 * 4,
                }
            }
        return self._sensor_modes

    # -- configuration ------------------------------------------------------ #

    def create_video_configuration(self, **kwargs):
        self.video_kwargs = kwargs
        main = dict(kwargs.get("main") or {})
        size = tuple(main.get("size") or (1280, 720))
        return {
            "use_case": "video",
            "colour_space": kwargs.get("colour_space"),
            "buffer_count": kwargs.get("buffer_count"),
            "main": {
                "format": main.get("format", "XBGR8888"),
                "size": size,
                "stride": size[0] * 3,
            },
            "lores": kwargs.get("lores"),
            "raw": None,
            "controls": dict(kwargs.get("controls") or {}),
            "display": kwargs.get("display"),
            "encode": kwargs.get("encode"),
            "sensor": kwargs.get("sensor"),
        }

    def create_still_configuration(self, **kwargs):
        self.still_kwargs = kwargs
        main = dict(kwargs.get("main") or {})
        size = tuple(main.get("size") or (4056, 3040))
        raw = None
        if kwargs.get("raw"):
            raw = {"format": "SBGGR12", "size": size, "stride": size[0] * 2}
        return {
            "use_case": "still",
            "colour_space": kwargs.get("colour_space"),
            "buffer_count": kwargs.get("buffer_count", 1),
            "main": {"format": main.get("format", "BGR888"), "size": size, "stride": size[0] * 3},
            "lores": kwargs.get("lores"),
            "raw": raw,
            "controls": dict(kwargs.get("controls") or {}),
            "display": None,
            "encode": None,
            "sensor": kwargs.get("sensor"),
        }

    def configure(self, config):
        self.camera_config = config
        self.configure_calls.append(config)

    def stream_configuration(self, name="main"):
        """Mirror Picamera2.stream_configuration so tests can inspect what is live."""
        if not self.camera_config:
            return None
        stream = self.camera_config.get(name)
        return dict(stream) if stream else None

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def start_encoder(self, encoder=None, output=None, pts=None, quality=None, name=None):
        encoder_name = type(encoder).__name__
        if encoder_name in self.fail_encoders:
            raise RuntimeError(f"{encoder_name} is not available on this platform")
        self.started_encoders.append(encoder_name)
        self.running_encoder = encoder
        self.encoder_output = output

    def stop_encoder(self):
        self.encoder_stop_count += 1

    def set_controls(self, controls):
        unknown = sorted(set(controls) - set(self.camera_controls))
        if unknown:
            raise RuntimeError(f"Control {unknown[0]} is not advertised by libcamera")
        self.control_history.append(dict(controls))
        self.applied_controls.update(controls)

    # -- capture ------------------------------------------------------------ #

    def capture_request(self, wait=None):
        self.capture_count += 1
        request = FakeRequest(self)
        self.requests.append(request)
        return request

    def close(self):
        self.closed = True
        self.started = False


# --------------------------------------------------------------------------- #
# Installation
# --------------------------------------------------------------------------- #


def install() -> None:
    """Insert the fakes into :data:`sys.modules`.

    Call this *before* importing ``picamera_service`` so that the module picks
    up the fakes as if it were running on a Raspberry Pi.
    """
    picamera2 = types.ModuleType("picamera2")
    picamera2.__path__ = []  # type: ignore[attr-defined]
    picamera2.Picamera2 = FakePicamera2

    outputs = types.ModuleType("picamera2.outputs")
    outputs.Output = FakeOutput
    outputs.FileOutput = FakeOutput
    outputs.CircularOutput = FakeOutput

    encoders = types.ModuleType("picamera2.encoders")
    encoders.MJPEGEncoder = MJPEGEncoder
    encoders.JpegEncoder = JpegEncoder
    encoders.H264Encoder = H264Encoder

    picamera2.outputs = outputs
    picamera2.encoders = encoders

    libcamera = types.ModuleType("libcamera")
    libcamera.__path__ = []  # type: ignore[attr-defined]
    controls = types.ModuleType("libcamera.controls")
    controls.NoiseReductionModeEnum = _NoiseReductionModeEnum
    controls.AeExposureModeEnum = _AeExposureModeEnum
    controls.AeMeteringModeEnum = _AeMeteringModeEnum
    controls.AwbModeEnum = _AwbModeEnum
    draft = types.ModuleType("libcamera.controls.draft")
    draft.NoiseReductionModeEnum = _NoiseReductionModeEnum
    controls.draft = draft
    libcamera.controls = controls
    libcamera.ColorSpace = _ColorSpace

    sys.modules.update(
        {
            "picamera2": picamera2,
            "picamera2.outputs": outputs,
            "picamera2.encoders": encoders,
            "libcamera": libcamera,
            "libcamera.controls": controls,
            "libcamera.controls.draft": draft,
        }
    )


def reset() -> None:
    """Drop recorded state so tests do not leak into each other."""
    FakePicamera2.instances.clear()
