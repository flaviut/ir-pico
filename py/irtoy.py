#!/usr/bin/env python3
#
# Class for simplifying the reading and transmitting of IR signals from the IR Toy.
# This only works for firmware revision 22 or greater.
# see https://github.com/crleblanc/PyIrToy and
# http://dangerousprototypes.com/docs/USB_Infrared_Toy for more info.
#
# Chris LeBlanc, 2012
#
# --
#
# This work is free: you can redistribute it and/or modify it under the terms
# of Creative Commons Attribution ShareAlike license v3.0
#
# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more details. You should have received a copy of the License along
# with this program. If not, see <http://creativecommons.org/licenses/by-sa/3.0/>.

import time
import binascii
import gc

__author__ = "Chris LeBlanc"
__version__ = "0.2.7"
__email__ = "crleblanc@gmail.com"


class FirmwareVersionError(Exception):
    pass


class IRTransmitError(Exception):
    pass


def sliding_window(iterable, n):
    "Collect data into overlapping fixed-length chunks or blocks."
    from collections import deque
    from itertools import islice

    # sliding_window('ABCDEFG', 4) → ABCD BCDE CDEF DEFG
    iterator = iter(iterable)
    window = deque(islice(iterator, n - 1), maxlen=n)
    for x in iterator:
        window.append(x)
        yield tuple(window)


class IrToy(object):
    def __init__(self, serialDevice):
        """Create a new IrToy instance using the serial device for the USB IR Toy"""
        self.toy = serialDevice

        self.required_version = 21
        revision = self.firmware_revision()[1]
        if revision < self.required_version:
            raise FirmwareVersionError(
                "pyirtoy will only work with firmware version %d or greater, current=%d"
                % (self.required_version, revision)
            )

        # always use sampling mode
        self._set_sampling_mode()

    def firmware_revision(self):
        """Return the hardware and firmware revision returned as a tuple"""
        self.toy.write(b"v")

        version_string = self.toy.read(4)
        hardware_version = int(version_string[1:2])
        firmware_version = int(version_string[2:4])

        return hardware_version, firmware_version

    def _set_sampling_mode(self):
        """set the IR Toy to use sampling mode, which we use exclusively"""
        self.toy.write(b"\0\0\0\0\0S")
        self.protocolVersion = self.toy.read(3)

    def receive(self):
        """Read a signal from the toy, returns a list of IR Codes converted from hex to ints.  See
        http://dangerousprototypes.com/docs/USB_IR_Toy:_Sampling_mode for more information on the
        sample format.  Reading starts when an IR signal is received and stops after 1.7 seconds of
        inactivity"""

        # reset and put back in receive mode in case it was in transmit mode or there was junk in the buffer
        self._set_sampling_mode()

        bytes_to_read = 1
        ir_code = []

        while True:
            val = ord(self.toy.read(bytes_to_read))

            ir_code.append(val)

            if len(ir_code) >= 2 and ir_code[-1] == 255 and ir_code[-2] == 255:
                break

        return ir_code

    def reset(self):
        """Reset the IR Toy to sampling mode and clear the Toy's 62 byte buffer"""

        time.sleep(0.5)
        self.toy.write(b"\0" * 5)

    def transmit(self, code):
        """switch to transmit mode and write the list (or list-like) set of ints to the toy for transmission,
        Must be an even number of elements.  The list created by read() can be used for transmitting.  If the
        last two elements of the list are not 0xff these will be appended."""

        if len(code) < 2:
            raise ValueError(
                "Length of code argument must be greater than or equal to two"
            )

        if len(code) % 2 != 0:
            raise ValueError("Length of code argument must be an even number")

        # ensure the last two codes are always 0xff (255) to tell the IR Toy it's the end of the signal
        if code[-2:] != [0xFF, 0xFF]:
            code.extend([0xFF, 0xFF])

        for window in sliding_window(code[:-2], 2):
            if window == (0xFF, 0xFF):
                raise ValueError("Unexpected 0xffff in middle of transmit code")

        total_bytes = len(code)

        try:
            # Enable transmit handshake, notify on complete, and byte count report, then start transmit mode
            self.toy.write(b"\x26\x25\x24\x03")

            try:
                gc.disable()
                self.toy.timeout = 1
                max_write_size = ord(self.toy.read(1))
                while code:
                    split = min(max_write_size, len(code))
                    segment = code[:split]
                    code = code[split:]

                    self.toy.write(segment)
                    max_write_size = ord(self.toy.read(1))
            finally:
                self.toy.timeout = None
                gc.enable()

            # get transmit report
            byte_count = int(binascii.b2a_hex(self.toy.read(3)[1:]), 16)
            complete = self.toy.read(1)

            if byte_count != total_bytes:
                raise IRTransmitError(
                    f"Failed to transmit IR code, only sent {byte_count}/{total_bytes}"
                )

            if complete not in [b"c", b"C"]:
                raise IRTransmitError(f"Failed to transmit IR code, report={complete}")
        except:
            # if anything went wrong then sheepishly try to reset state and raise the exception,
            # surprisingly common on a weak CPU like the raspberry pi
            # self.toy.flushOutput() # hmm, maybe this will help? Interesting: we get a crazy state until a new program is started, then fine.
            self.reset()
            self._set_sampling_mode()
            raise

        # experimentation shows that returning to sampling mode is needed to avoid dropping the serial connection on Linux
        self._set_sampling_mode()


if __name__ == "__main__":
    import serial

    with serial.Serial(
        "/dev/serial/by-id/usb-flaviutamas.com_p_ir-pico_RP2040_Infrared_remote_transceiver__IR_Toy__0000001-if00",
        timeout=2,
        write_timeout=2,
    ) as serial_device:
        connection = IrToy(serial_device)
        for _ in range(3):
            connection.transmit(
                [
                    1,
                    167,
                    0,
                    211,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    24,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    24,
                    0,
                    27,
                    0,
                    79,
                    0,
                    28,
                    0,
                    77,
                    0,
                    28,
                    0,
                    77,
                    0,
                    28,
                    6,
                    238,
                    1,
                    167,
                    0,
                    211,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    77,
                    0,
                    28,
                    0,
                    77,
                    0,
                    28,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    24,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    169,
                    96,
                    1,
                    167,
                    0,
                    212,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    77,
                    0,
                    28,
                    0,
                    76,
                    0,
                    29,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    76,
                    0,
                    29,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    78,
                    0,
                    29,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    6,
                    238,
                    1,
                    167,
                    0,
                    211,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    27,
                    0,
                    77,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    25,
                    0,
                    27,
                    0,
                    79,
                    0,
                    28,
                    0,
                    23,
                    0,
                    27,
                    0,
                    79,
                    0,
                    26,
                    0,
                    79,
                    0,
                    27,
                    0,
                    23,
                    0,
                    27,
                ]
            )
