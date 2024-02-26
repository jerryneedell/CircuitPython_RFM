# SPDX-FileCopyrightText: 2024 Jerry Needell for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""

* Author(s): Jerry Needell
"""

import time

try:
    from typing import Callable, Optional, Type, Union, Tuple, List, Any, ByteString
    from circuitpython_typing import WriteableBuffer, ReadableBuffer
    import digitalio
    import busio

except ImportError:
    pass

from micropython import const

HAS_SUPERVISOR = False

try:
    import supervisor

    if hasattr(supervisor, "ticks_ms"):
        HAS_SUPERVISOR = True
except ImportError:
    pass

from adafruit_bus_device import spi_device

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/jerryneedell/CircuitPython_RFM.git"

# supervisor.ticks_ms() contants
_TICKS_PERIOD = const(1 << 29)
_TICKS_MAX = const(_TICKS_PERIOD - 1)
_TICKS_HALFPERIOD = const(_TICKS_PERIOD // 2)

def ticks_diff(ticks1: int, ticks2: int) -> int:
    """Compute the signed difference between two ticks values
    assuming that they are within 2**28 ticks
    """
    diff = (ticks1 - ticks2) & _TICKS_MAX
    diff = ((diff + _TICKS_HALFPERIOD) & _TICKS_MAX) - _TICKS_HALFPERIOD
    return diff


def check_timeout(flag: Callable, limit: float) -> bool:
    """test for timeout waiting for specified flag"""
    timed_out = False
    if HAS_SUPERVISOR:
        start = supervisor.ticks_ms()
        while not timed_out and not flag():
            if ticks_diff(supervisor.ticks_ms(), start) >= limit * 1000:
                timed_out = True
    else:
        start = time.monotonic()
        while not timed_out and not flag():
            if time.monotonic() - start >= limit:
                timed_out = True
    return timed_out




class RFM:  # pylint: disable-msg=no-member
    """Base class for all RFM display devices
    """

    def __init__(self) -> None:
        self.init()

    def init(self) -> None:
        """Run the initialization commands."""
        pass

class RFMSPI(RFM):
    """Base class for SPI type devices"""

    class RegisterBits:
        # Class to simplify access to the many configuration bits avaialable
        # on the chip's registers.  This is a subclass here instead of using
        # a higher level module to increase the efficiency of memory usage
        # (all of the instances of this bit class will share the same buffer
        # used by the parent RFM69 class instance vs. each having their own
        # buffer and taking too much memory).

        # Quirk of pylint that it requires public methods for a class.  This
        # is a decorator class in Python and by design it has no public methods.
        # Instead it uses dunder accessors like get and set below.  For some
        # reason pylint can't figure this out so disable the check.
        # pylint: disable=too-few-public-methods

        # Again pylint fails to see the true intent of this code and warns
        # against private access by calling the write and read functions below.
        # This is by design as this is an internally used class.  Disable the
        # check from pylint.
        # pylint: disable=protected-access

        def __init__(self, address: int, *, offset: int = 0, bits: int = 1) -> None:
            assert 0 <= offset <= 7
            assert 1 <= bits <= 8
            assert (offset + bits) <= 8
            self._address = address
            self._mask = 0
            for _ in range(bits):
                self._mask <<= 1
                self._mask |= 1
            self._mask <<= offset
            self._offset = offset

        def __get__(self, obj: Optional["RFM"], objtype: Type["RFM"]) -> int:
            reg_value = obj.read_u8(self._address)
            return (reg_value & self._mask) >> self._offset

        def __set__(self, obj: Optional["RFM"], val: int) -> None:
            reg_value = obj.read_u8(self._address)
            reg_value &= ~self._mask
            reg_value |= (val & 0xFF) << self._offset
            obj.write_u8(self._address, reg_value)


    # pylint: disable-msg=too-many-arguments
    def __init__(
        self,
        spi: busio.SPI,
        cs: digitalio.DigitalInOut,
        rst: Optional[digitalio.DigitalInOut] = None,
        baudrate: int = 5000000,
        polarity: int = 0,
        phase: int = 0
    ):
        self.spi_device = spi_device.SPIDevice(
            spi, cs, baudrate=baudrate, polarity=polarity, phase=phase
        )
        #self.rst = rst
        #if self.rst:
        #    self.rst.switch_to_output(value=0)
        #    self.reset()
        super().__init__()

    # pylint: enable-msg=too-many-arguments

    # Global buffer for SPI commands
    _BUFFER = bytearray(4)


    # pylint: disable=no-member
    # Reconsider pylint: disable when this can be tested
    def read_into(
        self, address: int, buf: WriteableBuffer, length: Optional[int] = None
    ) -> None:
        # Read a number of bytes from the specified address into the provided
        # buffer.  If length is not specified (the default) the entire buffer
        # will be filled.
        if length is None:
            length = len(buf)
        with self.spi_device as device:
            self._BUFFER[0] = address & 0x7F  # Strip out top bit to set 0
            # value (read).
            device.write(self._BUFFER, end=1)
            device.readinto(buf, end=length)

    def read_u8(self, address: int) -> int:
        # Read a single byte from the provided address and return it.
        self.read_into(address, self._BUFFER, length=1)
        return self._BUFFER[0]

    def write_from(
        self, address: int, buf: ReadableBuffer, length: Optional[int] = None
    ) -> None:
        # Write a number of bytes to the provided address and taken from the
        # provided buffer.  If no length is specified (the default) the entire
        # buffer is written.
        if length is None:
            length = len(buf)
        with self.spi_device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
            # indicate a write.
            device.write(self._BUFFER, end=1)
            device.write(buf, end=length)

    def write_u8(self, address: int, val: int) -> None:
        # Write a byte register to the chip.  Specify the 7-bit address and the
        # 8-bit value to write to that address.
        with self.spi_device as device:
            self._BUFFER[0] = (
                address | 0x80
            ) & 0xFF  # Set top bit to 1 to indicate a write.
            self._BUFFER[1] = val & 0xFF
            device.write(self._BUFFER, end=2)

    #def reset(self) -> None:
    #    """Perform a reset of the chip."""
    #    # See section 7.2.2 of the datasheet for reset description.
    #    self.rst.value = True  # Set Reset Low
    #    time.sleep(0.0001)  # 100 us
    #    self.rst.value = False  # set Reset High
    #    time.sleep(0.005)  # 5 ms
