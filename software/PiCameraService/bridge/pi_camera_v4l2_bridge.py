#!/usr/bin/env python3
"""Expose a PiCameraService preview as a local Linux video device.

Runs on the **workstation**, not on the Pi. It relays the MJPEG preview from the
Pi into a `v4l2loopback` device so that anything on the workstation which expects
a normal webcam - including this project's GUI through its existing OpenCV path -
sees the Pi HQ Camera without any code change.

The relay itself is delegated to ffmpeg, which is the well-trodden path into
`v4l2loopback`. This script's job is the safety and discovery around it:

* it never writes to a real camera. Only devices the v4l2loopback module owns are
  ever offered as targets, so an existing webcam on /dev/video0 is never
  disturbed, and
* it explains exactly what is missing and how to install it, rather than failing
  with a permission error or an opaque ffmpeg message.

This path is a convenience, not the measurement path. Scientific control and
quantitative capture go through the PiCameraService HTTP API, which this script
does not replace, modify, or depend on: stopping the bridge leaves the daemon
and its measurement endpoints untouched.

Requires, on the workstation:

    ffmpeg                  present on most distributions
    v4l2loopback-dkms       Arch     sudo pacman -S v4l2loopback-dkms v4l-utils
    v4l2loopback-dkms       Debian   sudo apt install v4l2loopback-dkms v4l-utils

The kernel module needs to be loaded with a device number that does not collide
with an existing camera. `--print-setup` prints the command; nothing in this
script escalates privileges on its own.
"""
from __future__ import annotations

import argparse
import json
import shlex
import shutil
import signal
import subprocess
import sys
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlsplit

DEFAULT_PORT = 8000
SERVICE_MARKER = "PiCameraService"
SYSFS_VIDEO = Path("/sys/class/video4linux")
SYSFS_LOOPBACK_MODULE = Path("/sys/module/v4l2loopback")
PROBE_TIMEOUT_S = 3.0
STOP_GRACE_S = 5.0


class BridgeError(RuntimeError):
    """Something the user has to fix, reported without a traceback."""


@dataclass(frozen=True)
class VideoDevice:
    """One /dev/videoN device, as described by sysfs."""

    path: str
    name: str
    driver: str

    @property
    def is_loopback(self) -> bool:
        return self.driver == LOOPBACK_DRIVER

    @property
    def index(self) -> int:
        return int(self.path.rsplit("video", 1)[1])

    def describe(self) -> str:
        kind = "v4l2loopback" if self.is_loopback else f"real ({self.driver})"
        return f"{self.path:<14} {kind:<24} {self.name}"


# The pseudo-driver name reported for devices owned by the v4l2loopback module.
LOOPBACK_DRIVER = "v4l2 loopback"


def read_video_device(entry: Path) -> VideoDevice | None:
    """Describe one sysfs video node, or return None if it is unusable."""
    try:
        name = (entry / "name").read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    return VideoDevice(
        path=f"/dev/{entry.name}",
        name=name.strip() or "unnamed",
        driver=classify_device(entry),
    )


def classify_device(entry: Path) -> str:
    """Return the driver name for a sysfs video node.

    sysfs has no plain `driver` file for video devices. A real camera is a USB
    (or platform) device and therefore has a `device` symlink whose target is
    bound to a driver such as uvcvideo. A v4l2loopback device has no hardware
    parent at all, and the module's own sysfs directory exists while it is
    loaded, so the two cases are distinguishable without opening anything.
    """
    if not (entry / "device").exists():
        if SYSFS_LOOPBACK_MODULE.is_dir():
            return LOOPBACK_DRIVER
        return "virtual"

    for parent in entry.resolve().parents:
        candidate = parent / "driver"
        if candidate.exists():
            try:
                return candidate.resolve().name or "unknown"
            except OSError:
                return "unknown"
    return "unknown"


def list_video_devices(sysfs: Path | None = None) -> list[VideoDevice]:
    """Every video device the kernel currently exposes, ordered by number."""
    sysfs = SYSFS_VIDEO if sysfs is None else sysfs
    if not sysfs.is_dir():
        return []

    devices = []
    for entry in sysfs.iterdir():
        if not entry.name.startswith("video"):
            continue
        device = read_video_device(entry)
        if device is not None:
            devices.append(device)
    return sorted(devices, key=lambda device: device.index)


def loopback_devices(devices: list[VideoDevice] | None = None) -> list[VideoDevice]:
    return [d for d in (devices or list_video_devices()) if d.is_loopback]


def select_target_device(devices: list[VideoDevice], requested=None) -> VideoDevice:
    """Pick the loopback device to write to, refusing to touch a real camera.

    An explicit request is honoured only if it really is a loopback device:
    silently writing into a physical camera's node would be far worse than
    refusing to start.
    """
    if requested:
        wanted = str(requested)
        if not wanted.startswith("/dev/"):
            wanted = f"/dev/video{wanted}"
        for device in devices:
            if device.path == wanted:
                if not device.is_loopback:
                    raise BridgeError(
                        f"{wanted} is {device.driver!r} ({device.name}), not a "
                        "v4l2loopback device. Refusing to write to a real camera."
                    )
                return device
        known = ", ".join(device.path for device in devices) or "none"
        raise BridgeError(f"{wanted} does not exist. Known devices: {known}")

    candidates = loopback_devices(devices)
    if not candidates:
        raise BridgeError(
            "No v4l2loopback device found, so there is nothing to write the "
            "preview into.\n\nCreate one with:\n\n"
            f"{setup_hint()}\n\n"
            "Then run this script again; it will pick the device automatically. "
            "Run with --list-devices to see what is already present."
        )
    return candidates[0]


def setup_hint(video_nr: int = 10) -> str:
    return (
        f"  {install_command()}\n"
        f'  sudo modprobe v4l2loopback video_nr={video_nr} '
        f'card_label="Pi HQ Camera" exclusive_caps=1'
    )


def install_command(os_release: Path = Path("/etc/os-release")) -> str:
    """The package install command for this distribution, best effort."""
    try:
        text = os_release.read_text(encoding="utf-8", errors="replace").lower()
    except OSError:
        text = ""

    if any(token in text for token in ("arch", "endeavouros", "manjaro")):
        return "sudo pacman -S v4l2loopback-dkms v4l-utils"
    if any(token in text for token in ("debian", "ubuntu", "raspbian", "mint")):
        return "sudo apt install v4l2loopback-dkms v4l-utils"
    if any(token in text for token in ("fedora", "rhel", "centos")):
        return "sudo dnf install kmod-v4l2loopback v4l-utils"
    return "install v4l2loopback-dkms and v4l-utils with your package manager"


def normalize_address(address: str, default_port: int = DEFAULT_PORT) -> tuple[str, int]:
    """Turn host, host:port or a URL into (host, port).

    The same rules the GUI applies, so an address that works there works here.
    """
    text = str(address or "").strip()
    if not text:
        raise BridgeError("An address is required, for example 192.168.1.39")

    if "://" not in text:
        text = f"http://{text}"

    parsed = urlsplit(text)
    host = parsed.hostname
    if not host:
        raise BridgeError(f"Could not read a host from {address!r}")
    try:
        port = parsed.port or default_port
    except ValueError:
        port = default_port
    return host, int(port)


def probe_service(address: str, default_port: int = DEFAULT_PORT,
                  timeout: float = PROBE_TIMEOUT_S) -> dict:
    """Confirm that an address really is a PiCameraService and describe it."""
    host, port = normalize_address(address, default_port)
    url = f"http://{host}:{port}/info"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        raise BridgeError(f"{url} answered HTTP {exc.code}.") from exc
    except urllib.error.URLError as exc:
        raise BridgeError(
            f"No PiCameraService at {url} ({exc.reason}).\n"
            "Check the address and port, and that the daemon is running."
        ) from exc
    except OSError as exc:
        raise BridgeError(f"Timed out reaching {url} ({exc}).") from exc
    except json.JSONDecodeError as exc:
        raise BridgeError(f"{url} did not return JSON ({exc}).") from exc

    if not isinstance(payload, dict) or payload.get("service") != SERVICE_MARKER:
        raise BridgeError(
            f"{url} answered but is not a PiCameraService. "
            "Point this script at the camera daemon, not at some other server."
        )

    payload["_host"] = host
    payload["_port"] = port
    return payload


def build_ffmpeg_command(stream_url: str, device: VideoDevice, fps: int | None = None,
                         ffmpeg: str = "ffmpeg") -> list[str]:
    """Relay MJPEG into the loopback device as raw frames.

    The preview is already compressed MJPEG, so nothing is re-encoded here: the
    stream is decoded and written out as the YUV frames the device expects.
    """
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel", "warning",
        # Keep the preview close to live instead of letting ffmpeg build a
        # buffer while it decides what to do with the input.
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-i", stream_url,
    ]
    if fps:
        command += ["-r", str(int(fps))]
    command += ["-pix_fmt", "yuv420p", "-f", "v4l2", device.path]
    return command


def require_ffmpeg() -> str:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise BridgeError(
            "ffmpeg is required to relay the preview into the loopback device, "
            "and it was not found on PATH.\n"
            "Install it with your package manager (for example 'sudo pacman -S "
            "ffmpeg' or 'sudo apt install ffmpeg')."
        )
    return ffmpeg


class PreviewBridge:
    """Supervises the ffmpeg relay and reports why it stopped."""

    def __init__(self, device: VideoDevice, log=print):
        self.device = device
        self.log = log
        self.process: subprocess.Popen | None = None
        self.stopping = False

    def start(self, command: list[str]) -> subprocess.Popen:
        self.log(f"Relaying into {self.device.path} ({self.device.name})")
        self.log(f"  {shlex.join(command)}")
        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as exc:
            raise BridgeError(f"Could not start ffmpeg ({exc}).") from exc
        return self.process

    def run(self, command: list[str]) -> int:
        process = self.start(command)
        try:
            _, stderr = process.communicate()
        except KeyboardInterrupt:
            self.stop()
            self.log("Stopped.")
            return 0

        if self.stopping:
            return 0
        if process.returncode:
            raise BridgeError(
                f"ffmpeg exited with status {process.returncode}.\n"
                f"{(stderr or '').strip()[-2000:]}"
            )
        return 0

    def stop(self):
        self.stopping = True
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=STOP_GRACE_S)
            except subprocess.TimeoutExpired:
                self.process.kill()


def describe_target(device: VideoDevice, info: dict) -> str:
    """How to pick this device in the GUI once the bridge is running."""
    camera = info.get("camera") or {}
    stream = info.get("stream") or {}
    size = stream.get("main", {}).get("size") or [stream.get("width"), stream.get("height")]
    size_text = "x".join(str(part) for part in size if part)

    return (
        f"The preview is now available as {device.path}.\n"
        f"  Camera: {camera.get('model', 'unknown')} "
        f"{size_text} @ {stream.get('fps', '?')} fps\n\n"
        "Choose the matching OpenCV Camera entry in the GUI's camera dropdown. "
        "Prefer the\nPi Camera entry when you need full-resolution stills and "
        "photometric metadata."
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pi_camera_v4l2_bridge",
        description=(
            "Relay a PiCameraService MJPEG preview into a local v4l2loopback "
            "device so ordinary Linux applications see the Pi camera as a "
            "webcam. Runs on the workstation."
        ),
        epilog=(
            "Examples:\n"
            "  %(prog)s --list-devices\n"
            "  %(prog)s --print-setup\n"
            "  %(prog)s 192.168.1.39\n"
            "  %(prog)s 192.168.1.39:8000 --device /dev/video10\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "address", nargs="?",
        help=f"Pi address, as host, host:port or a URL (default port {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"port to use when the address has none (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--device", help="loopback device to write to, for example /dev/video10",
    )
    parser.add_argument(
        "--fps", type=int, help="force an output frame rate instead of the source's",
    )
    parser.add_argument(
        "--list-devices", action="store_true",
        help="list local video devices and which can be used, then exit",
    )
    parser.add_argument(
        "--print-setup", action="store_true",
        help="print the commands that create a loopback device, then exit",
    )
    parser.add_argument(
        "--ffmpeg", help="ffmpeg executable to use instead of the one on PATH",
    )
    return parser


def run_list_devices(devices: list[VideoDevice], out=print) -> int:
    out("Local video devices:\n")
    if devices:
        for device in devices:
            out(f"  {device.describe()}")
    else:
        out("  none found")

    usable = loopback_devices(devices)
    out("")
    if usable:
        out("Usable for the preview: " + ", ".join(d.path for d in usable))
    else:
        out("No v4l2loopback device is available, so there is nothing to write "
            "the preview into.")
        out("Run with --print-setup for the commands that create one.")
    return 0


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.print_setup:
        print("Create a v4l2loopback device on the workstation:\n")
        print(setup_hint())
        print(
            "\nPick a device number that does not collide with an existing "
            "camera; run with\n--list-devices to see what is there. v4l2loopback "
            "is only needed to expose the\npreview to generic applications. The "
            "micromanipulator GUI and the measurement\nAPI do not need it."
        )
        return 0

    devices = list_video_devices()

    if args.list_devices:
        return run_list_devices(devices)

    if not args.address:
        parser.error("an address is required (or use --list-devices / --print-setup)")

    try:
        info = probe_service(args.address, args.port)
        device = select_target_device(devices, args.device)
        ffmpeg = args.ffmpeg or require_ffmpeg()
    except BridgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    command = build_ffmpeg_command(
        f"http://{info['_host']}:{info['_port']}/stream.mjpg",
        device,
        fps=args.fps,
        ffmpeg=ffmpeg,
    )

    print(describe_target(device, info))
    print()

    bridge = PreviewBridge(device)
    previous = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda *_: bridge.stop())
    try:
        return bridge.run(command)
    except BridgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGINT, previous)


if __name__ == "__main__":
    sys.exit(main())
