# Changelog

## Configure a persistent PlatformIO serial device

- Configure upload and serial monitoring to use the controller's stable Linux
  `/dev/serial/by-id` path at the firmware's 921600 baud rate.
- This avoids uploads and monitoring targeting the wrong device when transient
  `/dev/ttyACM*` numbers change after reconnecting or rebooting the controller.
