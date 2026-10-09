#!/usr/bin/env python3
"""Command line entry point for the Raspberry Pi camera service.

Typical use on the Pi (or under the bundled systemd unit)::

    python3 picamera_daemon.py --width 1024 --height 768 --fps 25

Useful one-off commands::

    python3 picamera_daemon.py --list-controls      # what this sensor accepts
    python3 picamera_daemon.py --scan               # find services on the network
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import threading

from picamera_service import (
    API_VERSION,
    DEFAULT_PORT,
    SERVICE_NAME,
    SERVICE_VERSION,
    PicameraBackend,
    ServiceConfig,
    StreamSettings,
    advertise,
    build_server,
    discover_services,
)

PREVIEW_QUALITIES = ["very_low", "low", "medium", "high", "very_high"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="picamera_daemon",
        description="Serve a Raspberry Pi camera over HTTP: MJPEG preview plus a JSON control API.",
    )
    parser.add_argument("--host", default="0.0.0.0", help="interface to bind (default: %(default)s)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="port to bind (default: %(default)s)")
    parser.add_argument("--width", type=int, default=1024, help="preview width (default: %(default)s)")
    parser.add_argument("--height", type=int, default=768, help="preview height (default: %(default)s)")
    parser.add_argument("--fps", type=int, default=25, help="preview frame rate (default: %(default)s)")
    parser.add_argument(
        "--quality",
        default="medium",
        choices=PREVIEW_QUALITIES,
        help="MJPEG preview quality (default: %(default)s)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=None,
        help="explicit MJPEG bitrate in bits/s, overriding the derived default",
    )
    parser.add_argument(
        "--buffer-count",
        type=int,
        default=3,
        help="frames requested from libcamera, fewer means lower latency (default: %(default)s)",
    )
    parser.add_argument(
        "--capture-dir",
        default="captures",
        help="where still captures and shots.csv are written (default: %(default)s)",
    )
    parser.add_argument(
        "--device-name",
        default="Pi HQ Camera",
        help="name advertised over mDNS (default: %(default)s)",
    )
    parser.add_argument(
        "--max-viewers",
        type=int,
        default=4,
        help="maximum simultaneous MJPEG viewers (default: %(default)s)",
    )
    parser.add_argument("--no-advertise", action="store_true", help="do not announce over mDNS")
    parser.add_argument(
        "--log-level",
        default="info",
        choices=["debug", "info", "warning", "error"],
        help="logging verbosity (default: %(default)s)",
    )
    parser.add_argument("--list-controls", action="store_true", help="print control ranges and exit")
    parser.add_argument("--scan", action="store_true", help="scan the network for other services and exit")
    parser.add_argument("--scan-timeout", type=float, default=3.0, help="seconds to scan for mDNS services")
    parser.add_argument("--version", action="version", version=f"{SERVICE_NAME} {SERVICE_VERSION} (API {API_VERSION})")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )
    logger = logging.getLogger("picamera-service")

    if args.scan:
        services = discover_services(timeout=args.scan_timeout)
        if not services:
            print("No camera services found (is zeroconf installed and is the daemon advertising?)")
            return 1
        for service in services:
            print(f"{service['name']}  ->  http://{service['host']}:{service['port']}/  {service['properties']}")
        return 0

    stream = StreamSettings(
        width=args.width,
        height=args.height,
        fps=args.fps,
        quality=args.quality,
    )
    backend = PicameraBackend(
        stream,
        capture_dir=args.capture_dir,
        bitrate=args.bitrate,
        buffer_count=args.buffer_count,
        logger=logger,
    )

    if args.list_controls:
        backend.open()
        print(json.dumps(backend.supported_controls(), indent=2))
        print(json.dumps(backend.info()["camera"], indent=2))
        backend.close()
        return 0

    backend.open()
    if backend.open_warning:
        logger.warning("%s", backend.open_warning)

    config = ServiceConfig(
        host=args.host,
        port=args.port,
        device_name=args.device_name,
        allow_stream_clients=args.max_viewers,
        advertise=not args.no_advertise,
        logger=logger,
    )

    server = build_server(backend, config)
    advertiser = advertise(backend, config) if config.advertise else None
    shutdown_requested = threading.Event()

    def request_shutdown(signum, frame):  # noqa: ARG001
        logger.info("Received signal %s, shutting down", signum)
        shutdown_requested.set()
        # serve_forever() blocks in the accept loop; shutdown() from another
        # thread is the documented way out of it.
        threading.Thread(target=server.shutdown, daemon=True).start()

    for signal_name in ("SIGINT", "SIGTERM"):
        if hasattr(signal, signal_name):
            signal.signal(getattr(signal, signal_name), request_shutdown)

    try:
        logger.info(
            "Serving %s v%s on http://%s:%d/ (preview %dx%d @ %d fps, capture dir %s)",
            SERVICE_NAME,
            SERVICE_VERSION,
            config.host,
            config.port,
            stream.width,
            stream.height,
            stream.fps,
            backend.capture_dir,
        )
        logger.info("Measurement path: POST /capture?raw=1&label=<name> then GET /captures/<name>")
        server.serve_forever()
    finally:
        server.server_close()
        if advertiser is not None:
            try:
                advertiser.close()
            except Exception:
                pass
        backend.close()
        logger.info("Stopped")

    if shutdown_requested.is_set():
        # libcamera starts its CameraManager and IPA worker threads outside
        # Python's control and never joins them, so a normal interpreter exit
        # would hang here. Both the camera and the server are closed by now, so
        # skip the remaining atexit handlers and hand the exit code to systemd.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)

    return 0


if __name__ == "__main__":
    sys.exit(main())
