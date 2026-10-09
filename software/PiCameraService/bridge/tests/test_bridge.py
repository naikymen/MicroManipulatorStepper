"""Offline tests for the workstation-side v4l2loopback bridge.

Nothing here loads a kernel module, opens a video device, starts ffmpeg, or
touches the network. The sysfs tree, the module check and the service probe are
all faked, so these tests run on any machine.

The safety property under test is the important one: the bridge must never
choose a real camera as its output device.
"""
import importlib.util
import io
import json
import sys
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import patch

BRIDGE_PATH = Path(__file__).resolve().parents[1] / "pi_camera_v4l2_bridge.py"


def load_bridge():
    """Import the bridge by path, since it is a standalone script."""
    spec = importlib.util.spec_from_file_location("pi_camera_v4l2_bridge", BRIDGE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


bridge = load_bridge()


def make_sysfs(tmp_path: Path, devices: dict, *, loopback_module: bool):
    """Build a fake /sys/class/video4linux tree.

    Each device entry maps a node name to (card name, driver, has_device_link).
    Like the real sysfs, a node in video4linux is a symlink into the device tree,
    so a real camera's resolved path has an ancestor carrying a `driver`
    symlink. A v4l2loopback device has no hardware parent at all.
    """
    root = tmp_path / "video4linux"
    root.mkdir(parents=True, exist_ok=True)
    devices_root = tmp_path / "devices"
    devices_root.mkdir()

    for node, (card, driver_name, has_parent) in devices.items():
        node_path = devices_root / node / "video4linux" / node
        node_path.mkdir(parents=True)
        (node_path / "name").write_text(card, encoding="utf-8")
        if has_parent:
            driver = devices_root / "drivers" / driver_name
            driver.mkdir(parents=True, exist_ok=True)
            (devices_root / node / "driver").symlink_to(driver)
            (node_path / "device").symlink_to(devices_root / node)
        (root / node).symlink_to(node_path)

    module_dir = tmp_path / "module-v4l2loopback"
    if loopback_module:
        module_dir.mkdir()
    return root, module_dir


class FakeSysfsMixin:
    @staticmethod
    def listed(devices: dict, *, loopback_module: bool):
        """Return the devices sysfs would report, using a temporary tree."""
        with tempfile.TemporaryDirectory() as tmp:
            root, module = make_sysfs(Path(tmp), devices, loopback_module=loopback_module)
            with patch.object(bridge, "SYSFS_VIDEO", root), \
                    patch.object(bridge, "SYSFS_LOOPBACK_MODULE", module):
                return bridge.list_video_devices()


class AddressParsingTests(unittest.TestCase):
    def test_bare_host_uses_the_default_port(self):
        self.assertEqual(bridge.normalize_address("192.168.1.39"), ("192.168.1.39", 8000))

    def test_explicit_port_is_kept(self):
        self.assertEqual(bridge.normalize_address("192.168.1.39:9000"),
                         ("192.168.1.39", 9000))

    def test_urls_are_accepted(self):
        for text in ("http://192.168.1.39:8000", "http://192.168.1.39",
                     "http://192.168.1.39/info", "http://192.168.1.39:8000/info"):
            with self.subTest(address=text):
                self.assertEqual(bridge.normalize_address(text), ("192.168.1.39", 8000))

    def test_mdns_names_are_accepted(self):
        self.assertEqual(bridge.normalize_address("raspberrypi.local"),
                         ("raspberrypi.local", 8000))

    def test_a_custom_default_port_is_used_when_none_is_given(self):
        self.assertEqual(bridge.normalize_address("pi", default_port=1234), ("pi", 1234))

    def test_a_missing_address_is_an_explained_error(self):
        for blank in ("", "   ", None):
            with self.subTest(address=blank):
                with self.assertRaises(bridge.BridgeError) as ctx:
                    bridge.normalize_address(blank)
                self.assertIn("192.168.1.39", str(ctx.exception))

    def test_a_port_beyond_the_valid_range_falls_back(self):
        self.assertEqual(bridge.normalize_address("pi:99999"), ("pi", 8000))


class DeviceClassificationTests(FakeSysfsMixin, unittest.TestCase):
    def test_a_usb_camera_is_reported_by_its_driver(self):
        devices = self.listed(
            {"video0": ("HP HD Camera", "uvcvideo", True)}, loopback_module=False
        )
        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0].driver, "uvcvideo")
        self.assertFalse(devices[0].is_loopback)

    def test_a_loopback_device_is_recognised_only_while_the_module_is_loaded(self):
        for module_loaded, expected in ((True, True), (False, False)):
            with self.subTest(module_loaded=module_loaded):
                devices = self.listed(
                    {"video10": ("Pi HQ Camera", "none", False)},
                    loopback_module=module_loaded,
                )
                self.assertEqual(devices[0].is_loopback, expected)

    def test_devices_are_ordered_by_number(self):
        devices = self.listed(
            {
                "video10": ("Loopback", "none", False),
                "video2": ("Webcam", "uvcvideo", True),
                "video0": ("Webcam", "uvcvideo", True),
            },
            loopback_module=True,
        )
        self.assertEqual([d.path for d in devices],
                         ["/dev/video0", "/dev/video2", "/dev/video10"])

    def test_an_unreadable_node_is_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            root, module = make_sysfs(
                Path(tmp), {"video7": ("Broken", "none", False)}, loopback_module=False
            )
            (root / "video7" / "name").unlink()
            with patch.object(bridge, "SYSFS_VIDEO", root), \
                    patch.object(bridge, "SYSFS_LOOPBACK_MODULE", module):
                self.assertEqual(bridge.list_video_devices(), [])

    def test_a_missing_sysfs_tree_yields_no_devices(self):
        with patch.object(bridge, "SYSFS_VIDEO", Path("/nonexistent/video4linux")):
            self.assertEqual(bridge.list_video_devices(), [])


class TargetSelectionTests(unittest.TestCase):
    """The safety property: a real camera must never be selected or written to."""

    REAL = bridge.VideoDevice("/dev/video0", "HP HD Camera", "uvcvideo")
    LOOPBACK = bridge.VideoDevice("/dev/video10", "Pi HQ Camera", bridge.LOOPBACK_DRIVER)

    def test_a_real_camera_is_never_selected_automatically(self):
        with self.assertRaises(bridge.BridgeError) as ctx:
            bridge.select_target_device([self.REAL])
        self.assertIn("v4l2loopback", str(ctx.exception))
        self.assertIn("modprobe v4l2loopback", str(ctx.exception))

    def test_an_explicitly_requested_real_camera_is_refused(self):
        with self.assertRaises(bridge.BridgeError) as ctx:
            bridge.select_target_device([self.REAL, self.LOOPBACK], "/dev/video0")
        message = str(ctx.exception)
        self.assertIn("Refusing to write to a real camera", message)
        self.assertIn("uvcvideo", message)

    def test_the_loopback_device_is_chosen_when_present(self):
        self.assertEqual(
            bridge.select_target_device([self.REAL, self.LOOPBACK]), self.LOOPBACK
        )

    def test_real_cameras_are_skipped_in_favour_of_a_loopback_device(self):
        second_real = bridge.VideoDevice("/dev/video1", "HP HD Camera", "uvcvideo")
        chosen = bridge.select_target_device([self.REAL, second_real, self.LOOPBACK])
        self.assertEqual(chosen, self.LOOPBACK)

    def test_a_bare_device_number_is_accepted(self):
        self.assertEqual(bridge.select_target_device([self.LOOPBACK], "10"),
                         self.LOOPBACK)

    def test_an_unknown_device_lists_the_known_ones(self):
        with self.assertRaises(bridge.BridgeError) as ctx:
            bridge.select_target_device([self.LOOPBACK], "/dev/video99")
        message = str(ctx.exception)
        self.assertIn("/dev/video99", message)
        self.assertIn("/dev/video10", message)

    def test_no_devices_at_all_is_explained(self):
        with self.assertRaises(bridge.BridgeError) as ctx:
            bridge.select_target_device([])
        self.assertIn("modprobe", str(ctx.exception))


class InstallHintTests(unittest.TestCase):
    def test_the_install_command_matches_the_distribution(self):
        cases = {
            'ID=endeavouros\nNAME="EndeavourOS"': "pacman -S v4l2loopback-dkms",
            'ID=debian\nPRETTY_NAME="Debian GNU/Linux 13"': "apt install v4l2loopback-dkms",
            'ID=fedora\nNAME="Fedora Linux"': "dnf install kmod-v4l2loopback",
            "ID=gentoo": "your package manager",
        }
        for content, expected in cases.items():
            with self.subTest(content=content), tempfile.TemporaryDirectory() as tmp:
                release = Path(tmp) / "os-release"
                release.write_text(content, encoding="utf-8")
                self.assertIn(expected, bridge.install_command(release))

    def test_a_missing_os_release_does_not_crash(self):
        command = bridge.install_command(Path("/nonexistent/os-release"))
        self.assertIn("package manager", command)

    def test_the_setup_hint_explains_both_steps(self):
        hint = bridge.setup_hint()
        self.assertIn("modprobe v4l2loopback", hint)
        self.assertIn("video_nr=10", hint)
        self.assertIn("exclusive_caps=1", hint)


class FfmpegCommandTests(unittest.TestCase):
    def _command(self, **kwargs):
        return bridge.build_ffmpeg_command(
            "http://192.168.1.39:8000/stream.mjpg",
            bridge.VideoDevice("/dev/video10", "Pi HQ Camera", bridge.LOOPBACK_DRIVER),
            **kwargs,
        )

    def test_the_stream_url_is_the_input(self):
        command = self._command()
        self.assertEqual(command[command.index("-i") + 1],
                         "http://192.168.1.39:8000/stream.mjpg")

    def test_the_target_device_is_the_output(self):
        command = self._command()
        self.assertEqual(command[-1], "/dev/video10")
        self.assertEqual(command[command.index("-f") + 1], "v4l2")

    def test_frames_are_written_as_yuv420p(self):
        command = self._command()
        self.assertEqual(command[command.index("-pix_fmt") + 1], "yuv420p")

    def test_low_latency_flags_are_present(self):
        command = self._command()
        self.assertIn("nobuffer", command)
        self.assertIn("low_delay", command)

    def test_a_frame_rate_is_added_only_when_requested(self):
        self.assertNotIn("-r", self._command())
        command = self._command(fps=15)
        self.assertEqual(command[command.index("-r") + 1], "15")

    def test_an_alternate_ffmpeg_is_honoured(self):
        self.assertEqual(self._command(ffmpeg="/opt/ffmpeg")[0], "/opt/ffmpeg")


class ProbeTests(unittest.TestCase):
    @staticmethod
    def _response(payload):
        class Response(io.BytesIO):
            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

        return Response(json.dumps(payload).encode("utf-8"))

    def test_a_picamera_service_is_recognised(self):
        payload = {"service": "PiCameraService", "camera": {"model": "imx477"}}
        with patch("urllib.request.urlopen", return_value=self._response(payload)):
            result = bridge.probe_service("192.168.1.39")
        self.assertEqual(result["service"], "PiCameraService")
        self.assertEqual(result["_host"], "192.168.1.39")
        self.assertEqual(result["_port"], 8000)

    def test_the_effective_host_and_port_are_injected(self):
        payload = {"service": "PiCameraService"}
        with patch("urllib.request.urlopen", return_value=self._response(payload)):
            result = bridge.probe_service("http://pi.local:9999/info")
        self.assertEqual((result["_host"], result["_port"]), ("pi.local", 9999))

    def test_another_web_server_is_rejected(self):
        with patch("urllib.request.urlopen",
                   return_value=self._response({"service": "something-else"})):
            with self.assertRaises(bridge.BridgeError) as ctx:
                bridge.probe_service("192.168.1.39")
        self.assertIn("not a PiCameraService", str(ctx.exception))

    def test_a_plain_dict_without_the_marker_is_rejected(self):
        with patch("urllib.request.urlopen", return_value=self._response({})):
            with self.assertRaises(bridge.BridgeError):
                bridge.probe_service("192.168.1.39")

    def test_an_http_error_is_reported_with_its_code(self):
        error = urllib.error.HTTPError("http://pi/info", 503, "unavailable", None, None)
        with patch("urllib.request.urlopen", side_effect=error):
            with self.assertRaises(bridge.BridgeError) as ctx:
                bridge.probe_service("192.168.1.39")
        self.assertIn("503", str(ctx.exception))

    def test_an_unreachable_host_names_the_url_and_keeps_the_reason(self):
        error = urllib.error.URLError("Connection refused")
        with patch("urllib.request.urlopen", side_effect=error):
            with self.assertRaises(bridge.BridgeError) as ctx:
                bridge.probe_service("192.168.1.39")
        message = str(ctx.exception)
        self.assertIn("http://192.168.1.39:8000/info", message)
        self.assertIn("Connection refused", message)
        self.assertIn("daemon is running", message)

    def test_a_timeout_is_reported_as_such(self):
        with patch("urllib.request.urlopen", side_effect=TimeoutError("timed out")):
            with self.assertRaises(bridge.BridgeError) as ctx:
                bridge.probe_service("192.168.1.39")
        self.assertIn("Timed out", str(ctx.exception))

    def test_a_non_json_response_is_reported(self):
        class Response(io.BytesIO):
            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

        with patch("urllib.request.urlopen",
                   return_value=Response(b"<html>hello</html>")):
            with self.assertRaises(bridge.BridgeError) as ctx:
                bridge.probe_service("192.168.1.39")
        self.assertIn("JSON", str(ctx.exception))


class MissingDependencyTests(unittest.TestCase):
    def test_a_missing_ffmpeg_is_explained_before_anything_starts(self):
        with patch("shutil.which", return_value=None):
            with self.assertRaises(bridge.BridgeError) as ctx:
                bridge.require_ffmpeg()
        message = str(ctx.exception)
        self.assertIn("ffmpeg is required", message)
        self.assertIn("Install it", message)

    def test_ffmpeg_is_returned_when_present(self):
        with patch("shutil.which", return_value="/usr/bin/ffmpeg"):
            self.assertEqual(bridge.require_ffmpeg(), "/usr/bin/ffmpeg")


class DescribeTargetTests(unittest.TestCase):
    DEVICE = bridge.VideoDevice("/dev/video10", "Pi HQ Camera", bridge.LOOPBACK_DRIVER)

    def test_the_device_and_camera_are_named(self):
        text = bridge.describe_target(
            self.DEVICE,
            {"camera": {"model": "imx477"},
             "stream": {"fps": 25, "main": {"size": [1024, 768]}}},
        )
        self.assertIn("/dev/video10", text)
        self.assertIn("imx477", text)
        self.assertIn("1024x768", text)
        self.assertIn("25 fps", text)

    def test_a_thin_info_payload_does_not_crash(self):
        self.assertIn("/dev/video10", bridge.describe_target(self.DEVICE, {}))


class ListDevicesOutputTests(unittest.TestCase):
    def test_real_cameras_are_marked_as_such(self):
        lines = []
        bridge.run_list_devices(
            [bridge.VideoDevice("/dev/video0", "HP HD Camera", "uvcvideo")],
            out=lines.append,
        )
        text = "\n".join(lines)
        self.assertIn("real (uvcvideo)", text)
        self.assertIn("no v4l2loopback device", text.lower())

    def test_a_loopback_device_is_offered(self):
        lines = []
        bridge.run_list_devices(
            [bridge.VideoDevice("/dev/video10", "Pi HQ Camera", bridge.LOOPBACK_DRIVER)],
            out=lines.append,
        )
        self.assertIn("/dev/video10", "\n".join(lines))

    def test_no_devices_is_reported_without_crashing(self):
        lines = []
        bridge.run_list_devices([], out=lines.append)
        self.assertIn("none found", "\n".join(lines))


class MainCommandTests(unittest.TestCase):
    """End-to-end argument handling, with every side effect patched out."""

    def test_no_address_without_a_mode_is_a_usage_error(self):
        with patch.object(bridge, "list_video_devices", return_value=[]), \
                patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as ctx:
                bridge.main([])
        self.assertEqual(ctx.exception.code, 2)

    def test_list_devices_does_not_need_an_address(self):
        with patch.object(bridge, "list_video_devices", return_value=[]), \
                patch("sys.stdout", new_callable=io.StringIO) as out:
            self.assertEqual(bridge.main(["--list-devices"]), 0)
        self.assertIn("Local video devices", out.getvalue())

    def test_print_setup_does_not_need_an_address(self):
        with patch("sys.stdout", new_callable=io.StringIO) as out:
            self.assertEqual(bridge.main(["--print-setup"]), 0)
        self.assertIn("modprobe v4l2loopback", out.getvalue())

    def test_a_missing_loopback_device_exits_with_an_error(self):
        with patch.object(bridge, "list_video_devices", return_value=[]), \
                patch.object(bridge, "probe_service",
                             return_value={"service": "PiCameraService"}), \
                patch("sys.stderr", new_callable=io.StringIO) as err:
            self.assertEqual(bridge.main(["192.168.1.39"]), 1)
        self.assertIn("No v4l2loopback device", err.getvalue())

    def test_a_real_camera_is_refused_even_when_explicitly_requested(self):
        real = bridge.VideoDevice("/dev/video0", "HP HD Camera", "uvcvideo")
        with patch.object(bridge, "list_video_devices", return_value=[real]), \
                patch.object(bridge, "probe_service", return_value={"service": "PiCameraService"}), \
                patch("sys.stderr", new_callable=io.StringIO) as err:
            self.assertEqual(bridge.main(["192.168.1.39", "--device", "/dev/video0"]), 1)
        self.assertIn("Refusing to write to a real camera", err.getvalue())

    def test_the_bridge_never_starts_ffmpeg_without_a_service(self):
        loopback = bridge.VideoDevice("/dev/video10", "Pi HQ", bridge.LOOPBACK_DRIVER)
        error = urllib.error.URLError("Connection refused")
        with patch.object(bridge, "list_video_devices", return_value=[loopback]), \
                patch("urllib.request.urlopen", side_effect=error), \
                patch.object(bridge, "PreviewBridge") as preview, \
                patch("sys.stderr", new_callable=io.StringIO) as err:
            self.assertEqual(bridge.main(["192.168.1.39"]), 1)
        preview.assert_not_called()
        self.assertIn("No PiCameraService", err.getvalue())


class PreviewBridgeTests(unittest.TestCase):
    DEVICE = bridge.VideoDevice("/dev/video10", "Pi HQ Camera", bridge.LOOPBACK_DRIVER)

    def test_stop_is_safe_when_nothing_was_started(self):
        bridge.PreviewBridge(self.DEVICE).stop()

    def test_an_ffmpeg_failure_is_reported_with_its_output(self):
        instance = bridge.PreviewBridge(self.DEVICE, log=lambda *_: None)
        with patch.object(bridge.subprocess, "Popen") as popen:
            process = popen.return_value
            process.communicate.return_value = ("", "Cannot open /dev/video10")
            process.returncode = 1
            with self.assertRaises(bridge.BridgeError) as ctx:
                instance.run(["ffmpeg"])
        message = str(ctx.exception)
        self.assertIn("status 1", message)
        self.assertIn("Cannot open /dev/video10", message)

    def test_a_signal_stop_is_not_reported_as_a_failure(self):
        instance = bridge.PreviewBridge(self.DEVICE, log=lambda *_: None)

        def communicate_then_stop():
            instance.stopping = True
            return "", ""

        with patch.object(bridge.subprocess, "Popen") as popen:
            process = popen.return_value
            process.communicate.side_effect = communicate_then_stop
            process.returncode = -15
            self.assertEqual(instance.run(["ffmpeg"]), 0)

    def test_stop_terminates_a_running_ffmpeg(self):
        instance = bridge.PreviewBridge(self.DEVICE, log=lambda *_: None)
        with patch.object(bridge.subprocess, "Popen") as popen:
            process = popen.return_value
            process.poll.return_value = None
            instance.process = process
            instance.stop()
        process.terminate.assert_called_once()

    def test_stop_kills_an_ffmpeg_that_ignores_termination(self):
        instance = bridge.PreviewBridge(self.DEVICE, log=lambda *_: None)
        with patch.object(bridge.subprocess, "Popen") as popen:
            process = popen.return_value
            process.poll.return_value = None
            process.wait.side_effect = bridge.subprocess.TimeoutExpired("ffmpeg", 5)
            instance.process = process
            instance.stop()
        process.kill.assert_called_once()

    def test_start_reports_an_unlaunchable_ffmpeg(self):
        instance = bridge.PreviewBridge(self.DEVICE, log=lambda *_: None)
        with patch.object(bridge.subprocess, "Popen", side_effect=OSError("no exec")):
            with self.assertRaises(bridge.BridgeError) as ctx:
                instance.start(["ffmpeg"])
        self.assertIn("Could not start ffmpeg", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
