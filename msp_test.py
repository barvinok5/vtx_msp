#!/usr/bin/env python3
"""
MSP test console for the Barvinok VTX (pico_radio app_tx).

Sends Barvinok video MSP commands to the VTX CRSF UART exactly as the BLRS
receiver does it

  CRSF extended frame
    [0]     0xC8          sync (CRSF_ADDRESS_FLIGHT_CONTROLLER)
    [1]     len           bytes after this one, CRC included
    [2]     0x91          CRSF_FRAMETYPE_BARVINOK_VIDEO_MSP
    [3]     0xF0          destination: CRSF_ADDRESS_VIDEO_MODEM
    [4]     0xEA          origin: CRSF_ADDRESS_RADIO_TRANSMITTER
  Encapsulated MSP v1
    [5]     0x30          status byte (CRSF_MSP_STATUS_BYTE)
    [6]     size          payload size
    [7]     function      MSP_BLRS_* command
    [8..]   payload
    [n-2]   XOR of size, function and payload
  CRSF
    [n-1]   CRC8 (poly 0xD5) of bytes [2 .. n-2]


Usage:
  ./msp_test.py -p /dev/ttyUSB0              interactive console
  ./msp_test.py -p /dev/ttyUSB0 power 50     send one command and exit
  ./msp_test.py --dry-run                    console that only prints the frames
"""

import argparse
import cmd
import shlex
import sys

# MSP commands, lrs/elrs/src/lib/MSP/msptypes.h
MSP_BLRS_SET_FEC = 0x32
MSP_BLRS_SET_CAMERA_ID = 0x33
MSP_BLRS_SET_NIGH_MODE_ID = 0x34
MSP_BLRS_SET_VTX_POWER_ID = 0x35
MSP_BLRS_SET_VTX_FREQ_ID = 0x36

MSP_NAMES = {
    MSP_BLRS_SET_FEC: "MSP_BLRS_SET_FEC",
    MSP_BLRS_SET_CAMERA_ID: "MSP_BLRS_SET_CAMERA_ID",
    MSP_BLRS_SET_NIGH_MODE_ID: "MSP_BLRS_SET_NIGH_MODE_ID",
    MSP_BLRS_SET_VTX_POWER_ID: "MSP_BLRS_SET_VTX_POWER_ID",
    MSP_BLRS_SET_VTX_FREQ_ID: "MSP_BLRS_SET_VTX_FREQ_ID",
}

# CRSF, lrs/elrs/src/lib/CrsfProtocol/crsf_protocol.h
CRSF_SYNC_BYTE = 0xC8
CRSF_FRAMETYPE_BARVINOK_VIDEO_MSP = 0x91
CRSF_ADDRESS_VIDEO_MODEM = 0xF0
CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA
CRSF_MSP_STATUS_BYTE = 0x30
CRSF_CRC_POLY = 0xD5

# ELRS refuses bigger payloads (ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE in ELRS msptypes.h)
MSP_MAX_PAYLOAD_SIZE = 32

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 420000  # CRSF_BAUD_RATE in pico_radio/app/hw_config/hw_config.h

# VTX limits, pico_radio sources
FEC_IDS = {"none": 0, "1/2": 1}
NIGHT_MODES = {"day": 0, "night": 1}
VTX_POWER_MAX = 100
FREQ_MHZ_MAX = 2900


def crc8_d5(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ CRSF_CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def msp_xor(data):
    crc = 0
    for byte in data:
        crc ^= byte
    return crc


def build_frame(function, payload):
    """Builds the CRSF frame with the encapsulated MSP command, as ELRS sendPacketToFC() does."""
    payload = bytes(payload)
    if len(payload) > MSP_MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload is {len(payload)} bytes, max {MSP_MAX_PAYLOAD_SIZE}")

    msp = bytes([CRSF_MSP_STATUS_BYTE, len(payload), function]) + payload
    msp += bytes([msp_xor(msp[1:])])

    body = bytes([CRSF_FRAMETYPE_BARVINOK_VIDEO_MSP, CRSF_ADDRESS_VIDEO_MODEM, CRSF_ADDRESS_RADIO_TRANSMITTER]) + msp
    return bytes([CRSF_SYNC_BYTE, len(body) + 1]) + body + bytes([crc8_d5(body)])


def parse_int(text, name, lo, hi):
    try:
        value = int(text, 0)
    except ValueError:
        raise ValueError(f"{name}: '{text}' is not a number")
    if not lo <= value <= hi:
        raise ValueError(f"{name}: {value} is out of range {lo}..{hi}")
    return value


def warn(text):
    print(f"warning: {text}")


class MspConsole(cmd.Cmd):
    intro = "Barvinok VTX MSP test console. Type 'help' for commands, 'quit' to exit."
    prompt = "msp> "

    def __init__(self, port):
        super().__init__()
        self.port = port

    # ---- helpers ----

    def send(self, function, payload, description):
        frame = build_frame(function, payload)
        print(f"{MSP_NAMES.get(function, 'MSP')} (0x{function:02X}) {description}")
        print(f"  tx {len(frame):2d} bytes: {frame.hex(' ').upper()}")
        if self.port is not None:
            self.port.write(frame)
            self.port.flush()

    def run(self, func, arg):
        try:
            func(shlex.split(arg))
        except ValueError as err:
            print(f"error: {err}")
        return False

    def preloop(self):
        # '/' is a readline word delimiter, it breaks the completion of 'fec 1/2'
        try:
            import readline
            readline.set_completer_delims(readline.get_completer_delims().replace("/", ""))
        except ImportError:
            pass

    def emptyline(self):
        return False  # don't repeat the last command on Enter

    def default(self, line):
        print(f"unknown command: {line.split()[0]}, type 'help'")

    def print_topics(self, header, cmds, cmdlen, maxcol):
        if header != self.undoc_header:  # hides EOF (Ctrl+D) from help
            super().print_topics(header, cmds, cmdlen, maxcol)

    # ---- commands ----

    def do_fec(self, arg):
        """fec <none|1/2|id>
        MSP_BLRS_SET_FEC (0x32), payload: uint8 FEC id (0 none, 1 = 1/2).
        The VTX applies it and forwards it to the camera.
        Examples:
          fec none     FEC off (id 0)
          fec 1/2      FEC 1/2 (id 1)'"""
        def cmd_fec(args):
            if len(args) != 1:
                raise ValueError("usage: fec <none|1/2|id>")
            key = args[0].lower()
            fec_id = FEC_IDS[key] if key in FEC_IDS else parse_int(key, "fec id", 0, 255)
            if fec_id not in FEC_IDS.values():
                warn(f"FEC id {fec_id} is unknown to the VTX (fecId_t is 0..2)")
            self.send(MSP_BLRS_SET_FEC, [fec_id], f"fec id {fec_id}")
        return self.run(cmd_fec, arg)

    def complete_fec(self, text, *_):
        return [k for k in FEC_IDS if k.startswith(text)]

    def do_camera(self, arg):
        """camera <id>
        MSP_BLRS_SET_CAMERA_ID (0x33), payload: uint8 camera id.
        The VTX forwards it to the camera.
        Examples:
          camera 0     select camera 0
          camera 1     select camera 1"""
        def cmd_camera(args):
            if len(args) != 1:
                raise ValueError("usage: camera <id>")
            camera_id = parse_int(args[0], "camera id", 0, 255)
            self.send(MSP_BLRS_SET_CAMERA_ID, [camera_id], f"camera id {camera_id}")
        return self.run(cmd_camera, arg)

    def do_night(self, arg):
        """night <day|night|0|1>
        MSP_BLRS_SET_NIGH_MODE_ID (0x34), payload: uint8 mode (0 day, 1 night).
        The VTX forwards it to the camera.
        Examples:
          night day    day mode (0)
          night night  night mode (1)
          night 1      same as 'night night'"""
        def cmd_night(args):
            if len(args) != 1:
                raise ValueError("usage: night <day|night|0|1>")
            key = args[0].lower()
            mode = NIGHT_MODES[key] if key in NIGHT_MODES else parse_int(key, "night mode", 0, 255)
            if mode not in NIGHT_MODES.values():
                warn(f"night mode {mode} is unknown (0 day, 1 night)")
            name = {0: "day", 1: "night"}.get(mode, "?")
            self.send(MSP_BLRS_SET_NIGH_MODE_ID, [mode], f"mode {mode} ({name})")
        return self.run(cmd_night, arg)

    def complete_night(self, text, *_):
        return [k for k in NIGHT_MODES if k.startswith(text)]

    def do_power(self, arg):
        """power <0..100>
        MSP_BLRS_SET_VTX_POWER_ID (0x35), payload: uint8 power in percent.
        ELRS sends 0, 50 or 100. The VTX rejects values above 100.
        Examples:
          power 100    max power
          power 50     50% power
          power 0      min power"""
        def cmd_power(args):
            if len(args) != 1:
                raise ValueError("usage: power <0..100>")
            level = parse_int(args[0], "power", 0, 255)
            if level > VTX_POWER_MAX:
                warn(f"power {level} > {VTX_POWER_MAX}, the VTX rejects it")
            self.send(MSP_BLRS_SET_VTX_POWER_ID, [level], f"power {level}%")
        return self.run(cmd_power, arg)

    def do_freq(self, arg):
        """freq <begin_mhz> <end_mhz>
        MSP_BLRS_SET_VTX_FREQ_ID (0x36), payload: uint16 LE begin MHz, uint16 LE end MHz.
        'freq 0 0' removes the frequency override.
        A valid range is announced to the VRX, saved, and the VTX restarts.
        The VTX rejects frequencies above 4294 MHz.
        Examples:
          freq 2400 2480   set range 2400 - 2480 MHz
          freq 0 0         remove the override, the provisioned range is used"""
        def cmd_freq(args):
            if len(args) != 2:
                raise ValueError("usage: freq <begin_mhz> <end_mhz>")
            begin = parse_int(args[0], "begin MHz", 0, 0xFFFF)
            end = parse_int(args[1], "end MHz", 0, 0xFFFF)
            if begin > FREQ_MHZ_MAX or end > FREQ_MHZ_MAX:
                warn(f"frequency above {FREQ_MHZ_MAX} MHz, the VTX rejects it")
            elif begin > end:
                warn("begin is above end, the VTX is expected to reject the range")
            payload = [begin & 0xFF, begin >> 8, end & 0xFF, end >> 8]
            text = "remove override" if begin == 0 and end == 0 else f"{begin} - {end} MHz"
            self.send(MSP_BLRS_SET_VTX_FREQ_ID, payload, text)
        return self.run(cmd_freq, arg)

    def do_raw(self, arg):
        """raw <function> [byte ...]
        Sends any MSP function with the given payload bytes (decimal or 0x hex).
        Examples:
          raw 0x35 50                  same as 'power 50'
          raw 0x36 0x60 0x09 0xB0 0x09 same as 'freq 2400 2480'
          raw 0x37                     reserved command, no payload"""
        def cmd_raw(args):
            if not args:
                raise ValueError("usage: raw <function> [byte ...]")
            function = parse_int(args[0], "function", 0, 255)
            payload = [parse_int(b, "byte", 0, 255) for b in args[1:]]
            self.send(function, payload, f"payload [{bytes(payload).hex(' ').upper()}]")
        return self.run(cmd_raw, arg)

    def do_info(self, arg):
        """info
        Shows the serial port settings.
        Example:
          info"""
        if self.port is None:
            print("dry run: frames are printed only, nothing is sent")
        else:
            print(f"port {self.port.port}, {self.port.baudrate} baud, 8N1")

    def do_quit(self, arg):
        """quit
        Exits the console (also 'exit' or Ctrl+D).
        Example:
          quit"""
        return True

    do_exit = do_quit

    def do_EOF(self, arg):
        print()
        return True

EPILOG = """\
commands (without a command an interactive console is started, 'help <command>' there):
  fec <none|1/2|id>       MSP_BLRS_SET_FEC          0x32
  camera <id>                 MSP_BLRS_SET_CAMERA_ID    0x33
  night <day|night|0|1>       MSP_BLRS_SET_NIGH_MODE_ID 0x34
  power <0..100>              MSP_BLRS_SET_VTX_POWER_ID 0x35
  freq <begin_mhz> <end_mhz>  MSP_BLRS_SET_VTX_FREQ_ID  0x36
  raw <function> [byte ...]   any MSP function
  info, quit

examples:
  %(prog)s -p /dev/ttyUSB0                  interactive console
  %(prog)s -p /dev/ttyUSB0 fec none         FEC off
  %(prog)s -p /dev/ttyUSB0 fec 1/2          FEC 1/2
  %(prog)s -p /dev/ttyUSB0 camera 1         select camera 1
  %(prog)s -p /dev/ttyUSB0 night night      night mode
  %(prog)s -p /dev/ttyUSB0 night day        day mode
  %(prog)s -p /dev/ttyUSB0 power 50         50%% power
  %(prog)s -p /dev/ttyUSB0 freq 2400 2480   range 2400 - 2480 MHz
  %(prog)s -p /dev/ttyUSB0 freq 0 0         remove the frequency override
  %(prog)s -p /dev/ttyUSB0 raw 0x35 50      raw command, same as 'power 50'
  %(prog)s -n power 50                      dry run, print the frame only
"""


def main():
    parser = argparse.ArgumentParser(
        description="Sends Barvinok video MSP commands to the VTX over the CRSF UART.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=EPILOG)
    parser.add_argument("-p", "--port", default=DEFAULT_PORT, help=f"serial port (default {DEFAULT_PORT})")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate (default {DEFAULT_BAUD})")
    parser.add_argument("-n", "--dry-run", action="store_true", help="print the frames, don't open the serial port")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="command to send once, e.g. 'power 50'")
    args = parser.parse_args()

    port = None
    if not args.dry_run:
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is required: pip install pyserial")
        try:
            port = serial.Serial(args.port, args.baud, bytesize=8, parity="N", stopbits=1, timeout=0, write_timeout=1)
        except serial.SerialException as err:
            sys.exit(f"can't open {args.port}: {err}")

    console = MspConsole(port)
    try:
        if args.command:
            console.onecmd(" ".join(shlex.quote(a) for a in args.command))
        else:
            console.cmdloop()
    except KeyboardInterrupt:
        print()
    finally:
        if port is not None:
            port.close()


if __name__ == "__main__":
    main()
