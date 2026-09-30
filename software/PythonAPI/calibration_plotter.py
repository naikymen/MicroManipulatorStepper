import argparse

import matplotlib.pyplot as plt

from open_micro_stage_api import OpenMicroStageInterface, SerialInterface

plt.rcParams['figure.dpi'] = 200


def plot_calibration_data(ax_encoder_counts, ax_field_angel, label, data):
    # Plot on the provided Axes object
    if ax_encoder_counts is not None:
        ax_encoder_counts.plot(data[0], data[2], label=label)
        ax_encoder_counts.set_xlabel('Motor Angle [rad]')
        ax_encoder_counts.set_ylabel('Encoder Counts Raw')
        ax_encoder_counts.set_title('Encoder Count Plot')
        ax_encoder_counts.legend()
        ax_encoder_counts.grid(True)

    # Plot on the provided Axes object
    if ax_field_angel is not None:
        ax_field_angel.plot(data[0], data[1], label=label)
        ax_field_angel.set_xlabel('Motor Angle [rad]')
        ax_field_angel.set_ylabel('Motor Field Angle [rad]')
        ax_field_angel.set_title('Field Angle Plot')
        ax_field_angel.legend()
        ax_field_angel.grid(True)


def list_available_ports():
    devices = OpenMicroStageInterface.enumerate_devices()
    if not devices:
        print('No serial devices detected.')
        return devices

    print('Available serial devices:')
    for device in devices:
        print(f"  {device['label']}")

    return devices


def resolve_port(port):
    if port:
        return port

    devices = OpenMicroStageInterface.enumerate_devices()
    pico_devices = [device for device in devices if 'pico' in device['label'].lower()]

    if len(pico_devices) == 1:
        print(f"Using detected Pico serial device: {pico_devices[0]['label']}")
        return pico_devices[0]['port']

    if len(pico_devices) > 1:
        print('Multiple Pico serial devices detected. Pass --port to choose one:')
        for device in pico_devices:
            print(f"  {device['label']}")
        raise SystemExit(1)

    if len(devices) == 1:
        print(f"Using detected serial device: {devices[0]['label']}")
        return devices[0]['port']

    if devices:
        print('Multiple serial devices detected. Pass --port to choose one:')
        for device in devices:
            print(f"  {device['label']}")
    else:
        print('No serial devices detected.')

    raise SystemExit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run joint calibration and plot the measured data.',
    )
    parser.add_argument(
        '--port',
        help='Serial port to use (for example /dev/ttyACM0 or COM3).',
    )
    parser.add_argument(
        '--list-ports',
        action='store_true',
        help='List detected serial devices and exit.',
    )
    parser.add_argument(
        '--save',
        action='store_true',
        help='Save each calibration to controller flash after validation.',
    )
    parser.add_argument(
        '--no-plot',
        action='store_true',
        help='Run calibration without opening the plot window.',
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Suppress the thousands of raw calibration rows in the terminal.',
    )
    parser.add_argument(
        '--stop-on-error',
        action='store_true',
        help='Stop instead of continuing when a joint calibration fails.',
    )
    parser.add_argument(
        '--disable-after',
        action='store_true',
        help='Send M18 after calibration and on exceptional exit.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.list_ports:
        list_available_ports()
        return

    port = resolve_port(args.port)
    oms = OpenMicroStageInterface(
        show_communication=not args.quiet,
        show_log_messages=True,
    )
    if not oms.connect(port):
        raise SystemExit(f'Could not connect to {port}.')

    try:
        # Create subplots
        fig, ax = plt.subplots(1, 1, figsize=(10, 7), sharex='all')

        for i in range(3):
            if args.quiet:
                print(f'Calibrating joint {i}...')
            res, data = oms.calibrate_joint(i, save_result=args.save)
            if args.stop_on_error:
                if res != SerialInterface.ReplyStatus.OK:
                    raise RuntimeError(f'Joint {i} calibration failed ({res.name}); stopping.')
                if len(data) != 3 or any(len(column) == 0 for column in data):
                    raise RuntimeError(f'Joint {i} returned no calibration samples; stopping.')

            if args.quiet and res == SerialInterface.ReplyStatus.OK:
                print(f'Joint {i}: OK ({len(data[0])} samples)'
                      + (' and saved' if args.save else ''))
            plot_calibration_data(ax, None, f'Actuator {i}', data)

        if args.disable_after:
            # Do not leave calibrated motors energized while the plot is open.
            status, _ = oms.serial.send_command('M18', timeout=10)
            if status != SerialInterface.ReplyStatus.OK:
                raise RuntimeError(f'Could not disable motors ({status.name}).')

        # Adjust layout and show
        plt.tight_layout()
        if not args.no_plot:
            plt.show()
    finally:
        if args.disable_after and oms.is_connected():
            # Best-effort shutdown also covers exceptions and interrupted runs.
            oms.serial.send_command('M18', timeout=10)
        oms.disconnect()


if __name__ == '__main__':
    main()
