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


# Internal constants:
# Register names (FSK Mode even though we use LoRa instead, from table 85)
_RH_RF95_REG_00_FIFO = const(0x00)
_RH_RF95_REG_01_OP_MODE = const(0x01)
_RH_RF95_REG_06_FRF_MSB = const(0x06)
_RH_RF95_REG_07_FRF_MID = const(0x07)
_RH_RF95_REG_08_FRF_LSB = const(0x08)
_RH_RF95_REG_09_PA_CONFIG = const(0x09)
_RH_RF95_REG_0A_PA_RAMP = const(0x0A)
_RH_RF95_REG_0B_OCP = const(0x0B)
_RH_RF95_REG_0C_LNA = const(0x0C)
_RH_RF95_REG_0D_FIFO_ADDR_PTR = const(0x0D)
_RH_RF95_REG_0E_FIFO_TX_BASE_ADDR = const(0x0E)
_RH_RF95_REG_0F_FIFO_RX_BASE_ADDR = const(0x0F)
_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR = const(0x10)
_RH_RF95_REG_11_IRQ_FLAGS_MASK = const(0x11)
_RH_RF95_REG_12_IRQ_FLAGS = const(0x12)
_RH_RF95_REG_13_RX_NB_BYTES = const(0x13)
_RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB = const(0x14)
_RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB = const(0x15)
_RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB = const(0x16)
_RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB = const(0x17)
_RH_RF95_REG_18_MODEM_STAT = const(0x18)
_RH_RF95_REG_19_PKT_SNR_VALUE = const(0x19)
_RH_RF95_REG_1A_PKT_RSSI_VALUE = const(0x1A)
_RH_RF95_REG_1B_RSSI_VALUE = const(0x1B)
_RH_RF95_REG_1C_HOP_CHANNEL = const(0x1C)
_RH_RF95_REG_1D_MODEM_CONFIG1 = const(0x1D)
_RH_RF95_REG_1E_MODEM_CONFIG2 = const(0x1E)
_RH_RF95_REG_1F_SYMB_TIMEOUT_LSB = const(0x1F)
_RH_RF95_REG_20_PREAMBLE_MSB = const(0x20)
_RH_RF95_REG_21_PREAMBLE_LSB = const(0x21)
_RH_RF95_REG_22_PAYLOAD_LENGTH = const(0x22)
_RH_RF95_REG_23_MAX_PAYLOAD_LENGTH = const(0x23)
_RH_RF95_REG_24_HOP_PERIOD = const(0x24)
_RH_RF95_REG_25_FIFO_RX_BYTE_ADDR = const(0x25)
_RH_RF95_REG_26_MODEM_CONFIG3 = const(0x26)

_RH_RF95_REG_40_DIO_MAPPING1 = const(0x40)
_RH_RF95_REG_41_DIO_MAPPING2 = const(0x41)
_RH_RF95_REG_42_VERSION = const(0x42)

_RH_RF95_REG_4B_TCXO = const(0x4B)
_RH_RF95_REG_4D_PA_DAC = const(0x4D)
_RH_RF95_REG_5B_FORMER_TEMP = const(0x5B)
_RH_RF95_REG_61_AGC_REF = const(0x61)
_RH_RF95_REG_62_AGC_THRESH1 = const(0x62)
_RH_RF95_REG_63_AGC_THRESH2 = const(0x63)
_RH_RF95_REG_64_AGC_THRESH3 = const(0x64)

_RH_RF95_DETECTION_OPTIMIZE = const(0x31)
_RH_RF95_DETECTION_THRESHOLD = const(0x37)

_RH_RF95_PA_DAC_DISABLE = const(0x04)
_RH_RF95_PA_DAC_ENABLE = const(0x07)

# The crystal oscillator frequency of the module
_RH_RF95_FXOSC = 32000000.0

# Internal constants:
_REG_FIFO = const(0x00)
_REG_OP_MODE = const(0x01)
_REG_DATA_MOD = const(0x02)
_REG_BITRATE_MSB = const(0x03)
_REG_BITRATE_LSB = const(0x04)
_REG_FDEV_MSB = const(0x05)
_REG_FDEV_LSB = const(0x06)
_REG_FRF_MSB = const(0x07)
_REG_FRF_MID = const(0x08)
_REG_FRF_LSB = const(0x09)
_REG_VERSION = const(0x10)
_REG_PA_LEVEL = const(0x11)
_REG_OCP = const(0x13)
_REG_RX_BW = const(0x19)
_REG_AFC_BW = const(0x1A)
_REG_RSSI_VALUE = const(0x24)
_REG_DIO_MAPPING1 = const(0x25)
_REG_IRQ_FLAGS1 = const(0x27)
_REG_IRQ_FLAGS2 = const(0x28)
_REG_PREAMBLE_MSB = const(0x2C)
_REG_PREAMBLE_LSB = const(0x2D)
_REG_SYNC_CONFIG = const(0x2E)
_REG_SYNC_VALUE1 = const(0x2F)
_REG_PACKET_CONFIG1 = const(0x37)
_REG_FIFO_THRESH = const(0x3C)
_REG_PACKET_CONFIG2 = const(0x3D)
_REG_AES_KEY1 = const(0x3E)
_REG_TEMP1 = const(0x4E)
_REG_TEMP2 = const(0x4F)
_REG_TEST_PA1 = const(0x5A)
_REG_TEST_PA2 = const(0x5C)
_REG_TEST_DAGC = const(0x6F)

_TEST_PA1_NORMAL = const(0x55)
_TEST_PA1_BOOST = const(0x5D)
_TEST_PA2_NORMAL = const(0x70)
_TEST_PA2_BOOST = const(0x7C)
_OCP_NORMAL = const(0x1A)
_OCP_HIGH_POWER = const(0x0F)

# The crystal oscillator frequency and frequency synthesizer step size.
# See the datasheet for details of this calculation.
_FXOSC = 32000000.0
_FSTEP = _FXOSC / 524288


# The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
_RH_RF95_FSTEP = _RH_RF95_FXOSC / 524288

# RadioHead specific compatibility constants.
_RH_BROADCAST_ADDRESS = const(0xFF)

# The acknowledgement bit in the FLAGS
# The top 4 bits of the flags are reserved for RadioHead. The lower 4 bits are reserved
# for application layer use.
_RH_FLAGS_ACK = const(0x80)
_RH_FLAGS_RETRY = const(0x40)



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

    def receive(
        self,
        *,
        keep_listening: bool = True,
        with_header: bool = False,
        with_ack: bool = False,
        timeout: Optional[float] = None
    ) -> Optional[bytearray]:
        """Wait to receive a packet from the receiver. If a packet is found the payload bytes
        are returned, otherwise None is returned (which indicates the timeout elapsed with no
        reception).
        If keep_listening is True (the default) the chip will immediately enter listening mode
        after reception of a packet, otherwise it will fall back to idle mode and ignore any
        future reception.
        All packets must have a 4-byte header for compatibility with the
        RadioHead library.
        The header consists of 4 bytes (To,From,ID,Flags). The default setting will  strip
        the header before returning the packet to the caller.
        If with_header is True then the 4 byte header will be returned with the packet.
        The payload then begins at packet[4].
        If with_ack is True, send an ACK after receipt (Reliable Datagram mode)
        """
        timed_out = False
        if timeout is None:
            timeout = self.receive_timeout
        if timeout is not None:
            # Wait for the payloadready signal.  This is not ideal and will
            # surely miss or overflow the FIFO when packets aren't read fast
            # enough, however it's the best that can be done from Python without
            # interrupt supports.
            # Make sure we are listening for packets.
            self.listen()
            timed_out = check_timeout(self.payload_ready, timeout)
        # Payload ready is set, a packet is in the FIFO.
        packet = None
        # save last RSSI reading
        self.last_rssi = self.rssi
        if self.module == 'RFM9X':
            # save the last SNR reading
            self.last_snr = self.snr

        # Enter idle mode to stop receiving other packets.
        self.idle()
        if not timed_out:
            if self.enable_crc and self.crc_error():
                self.crc_error_count += 1
            else:
                # Read the data from the FIFO.
                # Read the length of the FIFO.
                if self.module == 'RFM9X':
                    fifo_length = self.read_u8(_RH_RF95_REG_13_RX_NB_BYTES)
                elif self.module == 'RFM69':
                    fifo_length = self.read_u8(_REG_FIFO)
                else:
                    raise RuntimeError("Unknown Module Type")
                # Handle if the received packet is too small to include the 4 byte
                # RadioHead header and at least one byte of data --reject this packet and ignore it.
                if fifo_length > 0:  # read and clear the FIFO if anything in it
                    packet = bytearray(fifo_length)
                    if self.module == 'RFM9X':
                        current_addr = self.read_u8(_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR)
                        self.write_u8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, current_addr)
                        # read the packet
                        self.read_into(_RH_RF95_REG_00_FIFO, packet)

                    elif self.module == 'RFM69':
                        # read the packet
                        self.read_into(_REG_FIFO, packet, fifo_length)
                    else:
                        raise RuntimeError("Unknown Module Type")
                    if self.module == 'RFM9X':
                        # clear interrupt
                        self.write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
                if fifo_length < 5:
                    packet = None
                else:
                    if (
                        self.node != _RH_BROADCAST_ADDRESS
                        and packet[0] != _RH_BROADCAST_ADDRESS
                        and packet[0] != self.node
                    ):
                        packet = None
                    # send ACK unless this was an ACK or a broadcast
                    elif (
                        with_ack
                        and ((packet[3] & _RH_FLAGS_ACK) == 0)
                        and (packet[0] != _RH_BROADCAST_ADDRESS)
                    ):
                        # delay before sending Ack to give receiver a chance to get ready
                        if self.ack_delay is not None:
                            time.sleep(self.ack_delay)
                        # send ACK packet to sender (data is b'!')
                        self.send(
                            b"!",
                            destination=packet[1],
                            node=packet[0],
                            identifier=packet[2],
                            flags=(packet[3] | _RH_FLAGS_ACK),
                        )
                        # reject Retries if we have seen this idetifier from this source before
                        if (self.seen_ids[packet[1]] == packet[2]) and (
                            packet[3] & _RH_FLAGS_RETRY
                        ):
                            packet = None
                        else:  # save the packet identifier for this source
                            self.seen_ids[packet[1]] = packet[2]
                    if (
                        not with_header and packet is not None
                    ):  # skip the header if not wanted
                        packet = packet[4:]
        # Listen again if necessary and return the result packet.
        if keep_listening:
            self.listen()
        else:
            # Enter idle mode to stop receiving other packets.
            self.idle()
        if self.module == 'RFM9x':
            # Clear interrupt.
            self.write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
        return packet
