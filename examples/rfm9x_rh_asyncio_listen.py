# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Example using Interrupts to send a message and then wait indefinitely for messages
# to be received. Interrupts are used only for receive. sending is done with polling.
# This example is for systems that support interrupts like the Raspberry Pi with "blinka"
# CircuitPython does not support interrupts so it will not work on  Circutpython boards
# Author: Tony DiCola, Jerry Needell
import asyncio
import board
import busio
import digitalio
from circuitpython_rfm import rfm9x

# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define pins connected to the chip, use these if wiring up the breakout according to the guide:
CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)

# Initialize SPI bus.
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialze RFM radio
rfm9x = rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# send startup message from my_node
# rfm9x.send(bytes("startup message from node {}".format(rfm9x.node), "UTF-8"))
rfm9x.listen()
# Wait to receive packets.
print("Waiting for packets...")
# initialize flag and timer


# pylint: disable=too-few-public-methods
class Packet:
    """Simple class to hold an  value. Use .value to to read or write."""

    def __init__(self):
        self.received = False


# setup interrupt callback function
async def wait_for_packets(packet_status):
    while True:
        if rfm9x.payload_ready():
            packet = await rfm9x.asyncio_receive(with_header=True, timeout=None)
            if packet is not None:
                packet_status.received = True
                # Received a packet!
                # Print out the raw bytes of the packet:
                print("Received (raw bytes): {0}".format(packet))
                print([hex(x) for x in packet])
                print("RSSI: {0}".format(rfm9x.last_rssi))
        await asyncio.sleep(0.001)


async def main():
    packet_status = Packet()
    task1 = asyncio.create_task(wait_for_packets(packet_status))

    await asyncio.gather(task1)  # Don't forget "await"!


asyncio.run(main())
