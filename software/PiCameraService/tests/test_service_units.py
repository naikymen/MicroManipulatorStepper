"""Unit tests for the MJPEG fan-out and the control mapping helpers."""

from __future__ import annotations

import logging
import queue
import sys
import threading
import unittest
from pathlib import Path
from unittest import mock

SERVICE_DIR = Path(__file__).resolve().parents[1]
if str(SERVICE_DIR) not in sys.path:
    sys.path.insert(0, str(SERVICE_DIR))

from tests import fake_picamera2  # noqa: E402

fake_picamera2.install()

import picamera_service  # noqa: E402


class MjpegFanoutOutputTests(unittest.TestCase):
    def setUp(self):
        self.output = picamera_service.MjpegFanoutOutput(queue_depth=2)

    def test_frames_reach_every_registered_client(self):
        first_id, first_queue = self.output.register_client()
        second_id, second_queue = self.output.register_client()

        self.output.outputframe(b"jpeg-one")
        self.output.outputframe(b"jpeg-two")

        self.assertEqual(first_queue.get_nowait(), b"jpeg-one")
        self.assertEqual(first_queue.get_nowait(), b"jpeg-two")
        self.assertEqual(second_queue.get_nowait(), b"jpeg-one")
        self.assertEqual(second_queue.get_nowait(), b"jpeg-two")

        self.output.unregister_client(first_id)
        self.output.unregister_client(second_id)
        self.assertEqual(self.output.client_count, 0)

    def test_full_queue_drops_the_oldest_frame_instead_of_blocking(self):
        _client_id, client_queue = self.output.register_client()

        for index in range(5):
            self.output.outputframe(f"jpeg-{index}".encode())

        # The queue keeps two frames and both are the most recent ones.
        self.assertEqual(client_queue.get_nowait(), b"jpeg-3")
        self.assertEqual(client_queue.get_nowait(), b"jpeg-4")
        self.assertEqual(self.output.frames_dropped, 3)

    def test_latest_frame_is_retained_for_snapshots(self):
        frame, sequence = self.output.latest_frame()
        self.assertIsNone(frame)
        self.assertEqual(sequence, 0)

        self.output.outputframe(b"jpeg-latest")

        frame, sequence = self.output.latest_frame()
        self.assertEqual(frame, b"jpeg-latest")
        self.assertEqual(sequence, 1)
        self.assertEqual(self.output.bytes_sent, len(b"jpeg-latest"))

    def test_audio_and_empty_frames_are_ignored(self):
        _client_id, client_queue = self.output.register_client()
        self.output.outputframe(b"", keyframe=True)
        self.output.outputframe(b"audio", audio=True)

        with self.assertRaises(queue.Empty):
            client_queue.get_nowait()
        self.assertEqual(self.output.frames_sent, 0)

    def test_next_frame_times_out_without_blocking_forever(self):
        _client_id, client_queue = self.output.register_client()
        self.assertIsNone(self.output.next_frame(client_queue, timeout=0.01))
        self.output.outputframe(b"jpeg-1")
        self.assertEqual(self.output.next_frame(client_queue, timeout=0.01), b"jpeg-1")

    def test_concurrent_registration_is_thread_safe(self):
        registered: list[int] = []
        lock = threading.Lock()

        def worker():
            for _ in range(20):
                client_id, _client_queue = self.output.register_client()
                with lock:
                    registered.append(client_id)

        threads = [threading.Thread(target=worker) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        self.assertEqual(len(registered), 80)
        self.assertEqual(len(set(registered)), 80, "client ids must be unique")
        self.assertEqual(self.output.client_count, 80)


class ControlMappingTests(unittest.TestCase):
    def setUp(self):
        self.controls = fake_picamera2.default_camera_controls()

    def test_unknown_control_is_reported_and_skipped(self):
        applyable, normalized, warnings = picamera_service.sanitize_controls(
            {"Nonsense": 1}, self.controls
        )
        self.assertEqual(applyable, {})
        self.assertEqual(normalized, {})
        self.assertIn("unknown control", warnings[0])

    def test_out_of_range_values_are_clamped(self):
        applyable, _normalized, warnings = picamera_service.sanitize_controls(
            {"Contrast": 12.0}, self.controls
        )
        self.assertEqual(applyable["Contrast"], 4.0)
        self.assertTrue(any("clamped" in warning for warning in warnings))

    def test_clamping_can_be_disabled(self):
        applyable, _normalized, warnings = picamera_service.sanitize_controls(
            {"Contrast": 12.0}, self.controls, clamp=False
        )
        self.assertEqual(applyable, {})
        self.assertTrue(any("outside" in warning for warning in warnings))

    def test_pair_controls_are_validated(self):
        applyable, _normalized, _warnings = picamera_service.sanitize_controls(
            {"ColourGains": [1.4, 1.9]}, self.controls
        )
        self.assertEqual(applyable["ColourGains"], (1.4, 1.9))

    def test_pair_length_is_reported_as_a_warning(self):
        applyable, _normalized, warnings = picamera_service.sanitize_controls(
            {"ColourGains": [1.4]}, self.controls
        )
        self.assertEqual(applyable, {})
        self.assertTrue(any("two element list" in warning for warning in warnings))

    def test_pair_controls_clamp_each_element(self):
        applyable, _normalized, _warnings = picamera_service.sanitize_controls(
            {"ColourGains": [99.0, 0.1]}, self.controls
        )
        self.assertEqual(applyable["ColourGains"], (32.0, 0.1))

    def test_symbolic_enums_resolve_against_libcamera(self):
        applyable, _normalized, _warnings = picamera_service.sanitize_controls(
            {"NoiseReductionMode": "off"}, self.controls
        )
        self.assertEqual(applyable["NoiseReductionMode"], 0)

        applyable, _normalized, _warnings = picamera_service.sanitize_controls(
            {"NoiseReductionMode": "high_quality"}, self.controls
        )
        self.assertEqual(applyable["NoiseReductionMode"], 2)

    def test_bad_enum_name_is_rejected_with_the_valid_options(self):
        applyable, _normalized, warnings = picamera_service.sanitize_controls(
            {"NoiseReductionMode": "super"}, self.controls
        )
        self.assertEqual(applyable, {})
        self.assertIn("off", warnings[0])

    def test_controls_absent_from_the_sensor_are_rejected(self):
        # The HQ Camera has no autofocus, so AfMode is not in the control set.
        applyable, _normalized, warnings = picamera_service.sanitize_controls(
            {"AfMode": "auto"}, self.controls
        )
        self.assertEqual(applyable, {})
        self.assertTrue(any("not supported by this sensor" in warning for warning in warnings))

    def test_booleans_accept_strings(self):
        applyable, _normalized, _warnings = picamera_service.sanitize_controls(
            {"AeEnable": "false"}, self.controls
        )
        self.assertIs(applyable["AeEnable"], False)

    def test_fractional_integers_are_rejected(self):
        applyable, _normalized, warnings = picamera_service.sanitize_controls(
            {"ExposureTime": 12.5}, self.controls
        )
        self.assertEqual(applyable, {})
        self.assertTrue(warnings)

    def test_exposure_zero_is_kept_because_it_means_auto(self):
        applyable, normalized, _warnings = picamera_service.sanitize_controls(
            {"ExposureTime": 0}, self.controls
        )
        self.assertEqual(applyable["ExposureTime"], 0)
        self.assertEqual(normalized["ExposureTime"], 0)

    def test_non_dict_payload_is_rejected(self):
        with self.assertRaises(ValueError):
            picamera_service.sanitize_controls(["ExposureTime"], self.controls)

    def test_exposure_range_does_not_advertise_zero_as_the_minimum(self):
        minimum, maximum, default = picamera_service.control_limits(
            picamera_service.CONTROL_SPECS["ExposureTime"], self.controls
        )
        self.assertEqual(minimum, 30)
        self.assertEqual(maximum, 67000000)
        self.assertEqual(default, 30000)

    def test_unsupported_control_is_absent_from_ads(self):
        available = {}
        for name, spec in picamera_service.CONTROL_SPECS.items():
            if all(entry in self.controls for entry in spec.requires):
                available[name] = spec
        self.assertIn("ExposureTime", available)
        self.assertNotIn("AfMode", available)

    def test_decibel_helpers_round_trip(self):
        self.assertAlmostEqual(picamera_service.decibels_to_gain(0.0), 1.0)
        self.assertAlmostEqual(picamera_service.decibels_to_gain(6.0), 1.9952623, places=6)
        self.assertAlmostEqual(picamera_service.gain_to_decibels(1.0), 0.0)
        self.assertAlmostEqual(picamera_service.gain_to_decibels(10.0), 20.0)
        self.assertEqual(picamera_service.gain_to_decibels(0.0), float("-inf"))

    def test_frame_rate_alias_is_reported_as_duration_limits(self):
        applyable, normalized, warnings = picamera_service.sanitize_controls(
            {"FrameRate": 25}, self.controls
        )
        self.assertEqual(applyable, {"FrameRate": 25})
        self.assertEqual(normalized, {"FrameDurationLimits": 25})
        self.assertTrue(warnings)


class BackendPreviewSizingTests(unittest.TestCase):
    """Regression tests for preview sizing, driven by what the real Pi did.

    ``Picamera2.sensor_modes`` enumerates modes by reconfiguring the camera to
    its own 640x480 preview configuration and never restores the caller's. The
    first version of this service read the modes and then started the camera,
    which encoded 640x480 frames while ``/info`` still reported the requested
    1024x768 -- a silent resolution loss that is fatal for a 40x objective.
    """

    def setUp(self):
        fake_picamera2.reset()

    def _open_backend(self, **stream_kwargs):
        backend = picamera_service.PicameraBackend(
            picamera_service.StreamSettings(**stream_kwargs),
            logger=logging.getLogger("test"),
        )
        backend.open()
        self.addCleanup(backend.close)
        return backend

    def test_reading_sensor_modes_does_not_change_the_preview_size(self):
        backend = self._open_backend(width=1024, height=768, fps=25)
        picam2 = backend._picam2

        self.assertEqual(picam2.mode_probe_calls, 1, "sensor modes should be read exactly once")
        self.assertEqual(backend.sensor_modes_cache is not None, True)
        self.assertEqual(
            picam2.stream_configuration("main")["size"],
            (1024, 768),
            "the preview must be reconfigured after the sensor mode probe",
        )

    def test_info_reports_the_size_libcamera_negotiated(self):
        backend = self._open_backend(width=1024, height=768)
        self.assertEqual(backend.info()["stream"]["main"]["size"], [1024, 768])

    def test_sensor_modes_are_only_read_once(self):
        backend = self._open_backend()
        backend.sensor_modes()
        backend.sensor_modes()
        self.assertEqual(backend._picam2.mode_probe_calls, 1)

    def test_device_id_prefers_the_libcamera_camera_id(self):
        backend = self._open_backend()
        self.assertEqual(backend.device_id(), "/base/soc/i2c0mux/i2c@1/imx477@1a")

    def test_device_id_falls_back_to_the_model_without_a_camera_id(self):
        backend = self._open_backend()
        with mock.patch.object(fake_picamera2.FakePicamera2, "global_camera_info", staticmethod(lambda: [])):
            self.assertEqual(backend.device_id(), "imx477")

    def test_device_id_is_none_when_the_camera_is_closed(self):
        backend = self._open_backend()
        backend.close()
        self.assertIsNone(backend.device_id())


class StreamSettingsTests(unittest.TestCase):
    def test_frame_duration_is_derived_from_fps(self):
        stream = picamera_service.StreamSettings(width=800, height=600, fps=25)
        self.assertEqual(stream.frame_duration_us(), 40000)
        self.assertEqual(stream.as_dict()["frame_duration_us"], 40000)

    def test_zero_fps_does_not_divide_by_zero(self):
        stream = picamera_service.StreamSettings(fps=0)
        self.assertGreater(stream.frame_duration_us(), 0)

    def test_safe_slug_strips_dangerous_characters(self):
        self.assertEqual(picamera_service._safe_slug("../../etc/passwd"), "etc-passwd")
        self.assertEqual(picamera_service._safe_slug("  dark frame 1  "), "dark-frame-1")
        self.assertEqual(picamera_service._safe_slug(None), "")


class ServiceConfigDefaultsTests(unittest.TestCase):
    def test_the_default_port_matches_the_gui_client(self):
        """A drift here means the GUI never finds a default-configured service."""
        gui_source = (
            Path(__file__).resolve().parents[2]
            / "OpenMicroManipulatorGUI"
            / "source"
            / "hardware"
            / "camera_pi.py"
        )
        if not gui_source.exists():  # pragma: no cover - partial checkout
            self.skipTest("GUI client is not present in this checkout")

        gui_port = None
        for line in gui_source.read_text(encoding="utf-8").splitlines():
            if line.startswith("DEFAULT_PI_CAMERA_PORT"):
                gui_port = int(line.split("=", 1)[1].strip())
                break

        self.assertIsNotNone(gui_port, "could not read DEFAULT_PI_CAMERA_PORT")
        self.assertEqual(gui_port, picamera_service.DEFAULT_PORT)

    def test_the_daemon_and_service_agree_on_the_default_port(self):
        import picamera_daemon

        parsed = picamera_daemon.build_parser().parse_args([])
        self.assertEqual(parsed.port, picamera_service.DEFAULT_PORT)
        self.assertEqual(picamera_service.ServiceConfig().port, picamera_service.DEFAULT_PORT)


if __name__ == "__main__":
    unittest.main()
