# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import argparse
import importlib_metadata
import sys
import warnings

from . import candle_device
from . import fdcanusb_device
from . import pythoncan_device
from . import transport

class FdcanusbFactory:
    PRIORITY = 10

    name = 'fdcanusb'

    def add_args(self, parser):
        try:
            parser.add_argument('--can-disable-brs', action='store_true',
                                help='do not set BRS')
        except argparse.ArgumentError:
            # It must already be set.
            pass

        parser.add_argument('--fdcanusb', type=str, action='append',
                            metavar='FILE',
                            help='path to fdcanusb device')

    def is_args_set(self, args):
        return args and args.fdcanusb

    def __call__(self, args):
        kwargs = {}
        if args and args.can_disable_brs:
            kwargs['disable_brs'] = True

        if args and args.can_debug:
            kwargs['debug_log'] = args.can_debug

        if args and args.fdcanusb:
            return [fdcanusb_device.FdcanusbDevice(path, **kwargs)
                    for path in args.fdcanusb]

        results = [fdcanusb_device.FdcanusbDevice(x, **kwargs) for x in
                   fdcanusb_device.FdcanusbDevice.detect_fdcanusbs()]
        if not results:
            raise RuntimeError('No fdcanusb detected')

        return results


class PythonCanFactory:
    PRIORITY = 11

    name = 'pythoncan'

    def add_args(self, parser):
        try:
            parser.add_argument('--can-disable-brs', action='store_true',
                                help='do not set BRS')
        except argparse.ArgumentError:
            # It must already be set.
            pass
        parser.add_argument('--can-iface', type=str, metavar='IFACE',
                            help='pythoncan "interface" (default: socketcan)')
        parser.add_argument('--can-chan', type=str, action='append',
                            metavar='CHAN',
                            help='pythoncan "channel" (default: can0)')

    def is_args_set(self, args):
        return args and (args.can_iface or args.can_chan)

    def __call__(self, args):
        kwargs = {}
        if args:
            if args.can_iface:
                kwargs['interface'] = args.can_iface
            if args.can_disable_brs:
                kwargs['disable_brs'] = True
            if args.can_debug:
                kwargs['debug_log'] = args.can_debug

        if args and args.can_chan:
            return [
                pythoncan_device.PythonCanDevice(
                    channel=channel, **kwargs)
                for channel in args.can_chan
            ]

        return pythoncan_device.PythonCanDevice.enumerate_devices(**kwargs)


class CandleFactory:
    """Transport factory for candle CAN-FD USB adapters.

    Auto-activates on Windows and macOS where the python-can candle
    backend is the native interface for such devices.  On Linux,
    candle devices typically appear as socketcan interfaces and are
    handled by PythonCanFactory instead.
    """

    PRIORITY = 12  # after FdcanusbFactory(10) and PythonCanFactory(11)

    name = 'candle'

    def add_args(self, parser):
        parser.add_argument(
            '--candle-bitrate', type=int, default=None,
            metavar='RATE',
            help='candle nominal bitrate in bps (default: 1000000)')
        parser.add_argument(
            '--candle-data-bitrate', type=int, default=None,
            metavar='RATE',
            help='candle data phase bitrate in bps for FD (default: 5000000)')

    def is_args_set(self, args):
        return args and (
            getattr(args, 'candle_bitrate', None) is not None or
            getattr(args, 'candle_data_bitrate', None) is not None or
            getattr(args, 'candle_chan', None)
        )

    def __call__(self, args):
        if not sys.platform.startswith('linux'):
            # Only auto-enumerate on non-Linux; on Linux use socketcan.
            pass
        elif not self.is_args_set(args):
            raise RuntimeError(
                'candle transport requires --can-chan on Linux')

        kwargs = {}
        if args:
            if getattr(args, 'candle_bitrate', None):
                kwargs['bitrate'] = args.candle_bitrate
            if getattr(args, 'candle_data_bitrate', None):
                kwargs['data_bitrate'] = args.candle_data_bitrate
            if getattr(args, 'can_debug', None):
                kwargs['debug_log'] = args.can_debug

        all_devices = candle_device.CandleDevice.enumerate_devices(**kwargs)

        if not all_devices:
            raise RuntimeError('No candle devices detected')

        return all_devices


TRANSPORT_FACTORIES = []

_transports_initialized = False

def get_transport_factories():
    global _transports_initialized

    if not _transports_initialized:
        # We initialize these in a deferred manner so that transports
        # are able to import things from moteus as necessary.
        _transports_initialized = True
        TRANSPORT_FACTORIES.extend([
            FdcanusbFactory(),
            PythonCanFactory(),
            CandleFactory(),
        ] + [ep.load()() for ep in
             importlib_metadata.entry_points().select(
                 group='moteus.transports2')
             ])

    return TRANSPORT_FACTORIES

GLOBAL_TRANSPORT = None


def make_transport_args(parser):
    """Add transport specific arguments to an argparse.ArgumentParser

    Args:
        parser: the argparse.ArgumentParser instance
    """
    for factory in get_transport_factories():
        if hasattr(factory, 'add_args'):
            factory.add_args(parser)

    parser.add_argument(
        '--can-debug', type=str,
        metavar='FILE',
        help='write raw CAN log')
    parser.add_argument(
        '--force-transport', type=str,
        choices=[x.name for x in get_transport_factories()],
        help='Force the given transport type to be used exclusively')


def check_gui_compatibility():
    try:
        import moteus_gui
        from importlib.metadata import version, PackageNotFoundError
        from packaging.version import parse as parse_version

        try:
            gui_version = version('moteus-gui')
            if parse_version(gui_version) < parse_version('0.3.93'):
                warnings.warn(
                    "moteus-gui is outdated.  Please upgrade: python -m pip install -U moteus-gui",
                    UserWarning
                )
        except PackageNotFoundError:
            pass
    except ImportError:
        pass


def get_singleton_transport(args=None):
    """Return (and construct if necessary) a transport.Transport
    instance that uses either all available CAN-FD interfaces on the
    system, or those configured by args.

    Args:
        args: an argparse.Namespace object

    Returns:
        a moteus.Transport object
    """
    global GLOBAL_TRANSPORT

    if GLOBAL_TRANSPORT:
        return GLOBAL_TRANSPORT

    # We check this here because it will likely only be called once
    # per application instance, and will nearly always be called by
    # the GUI.
    check_gui_compatibility()

    if args and args.can_debug:
        args.can_debug = open(args.can_debug, 'wb')

    maybe_result = None
    to_try = sorted(get_transport_factories(), key=lambda x: x.PRIORITY)
    if args and args.force_transport:
        to_try = [x for x in to_try if x.name == args.force_transport]
    elif args:
        # See if any transports have options set.  If so, then limit
        # to just those that do.
        if any([x.is_args_set(args) for x in get_transport_factories()]):
            to_try = [x for x in to_try if x.is_args_set(args)]

    devices = []
    fdcanusb_serials = set()

    errors = []
    for factory in to_try:
        maybe_results = []
        try:
            maybe_results = factory(args)
        except Exception as e:
            # This can be useful when implementing factories, as the
            # str(e) representation can be a bit abbreviated to find
            # an actual problem in code.
            if False:
                import traceback
                traceback.print_exc()

            errors.append((factory, str(e)))
            pass

        # If this is fdcanusb factory, collect serial numbers for deduplication
        if factory.name == 'fdcanusb':
            for device in maybe_results:
                serial = getattr(device, 'serial_number', None)
                if serial:
                    fdcanusb_serials.add(serial)

        devices.extend(maybe_results)

    # Deduplicate: remove pythoncan devices that are fdcanusb duplicates
    filtered_devices = []
    for device in devices:
        # Check if this is a pythoncan device with fdcanusb serial
        fdcanusb_serial = getattr(device, 'fdcanusb_serial', None)

        if fdcanusb_serial and fdcanusb_serial in fdcanusb_serials:
            # Skip this device - already available as fdcanusb CDC
            import logging
            logging.info(f"Excluding {device} - already available as fdcanusb CDC")
        else:
            filtered_devices.append(device)

    devices = filtered_devices

    if not devices:
        raise RuntimeError("Unable to find a default transport, tried: {}".format(
            ','.join([str(x) for x in errors])))

    GLOBAL_TRANSPORT = transport.Transport(devices)
    return GLOBAL_TRANSPORT
