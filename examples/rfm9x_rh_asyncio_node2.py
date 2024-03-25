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

# set delay before sending ACK
# rfm9x.ack_delay = 0.25
# set node addresses
rfm9x.node = 2
rfm9x.destination = 1
# send startup message from my_node
# rfm9x.send_with_ack(bytes("startup message from node {}".format(rfm9x.node), "UTF-8"))
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
async def wait_for_packets(packet_status, lock):
    while True:
        if rfm9x.payload_ready():
            if lock.locked():
                print("locked waiting for receive")
            async with lock:
                packet = await rfm9x.asyncio_receive_with_ack(
                    with_header=True, timeout=None
                )
            if packet is not None:
                packet_status.received = True
                # Received a packet!
                # Print out the raw bytes of the packet:
                print("Received (raw bytes): {0}".format(packet))
                print([hex(x) for x in packet])
                print("RSSI: {0}".format(rfm9x.last_rssi))
        await asyncio.sleep(0.001)


async def send_packets(packet_status, lock):
    # initialize counter
    counter = 0
    ack_failed_counter = 0
    counter = 0
    while True:
        # If no packet was received during the timeout then None is returned.
        if packet_status.received:
            packet_status.received = False
            counter += 1
            # send a  mesage to destination_node from my_node
            if lock.locked():
                print("locked waiting for send")
            async with lock:
                if not await rfm9x.asyncio_send_with_ack(
                    bytes(
                        "message from node {} {} {}".format(
                            rfm9x.node, counter, ack_failed_counter
                        ),
                        "UTF-8",
                    )
                ):
                    ack_failed_counter += 1
                    print(" No Ack: ", counter, ack_failed_counter)
        await asyncio.sleep(0.001)


async def main():
    packet_status = Packet()
    lock = asyncio.Lock()
    task1 = asyncio.create_task(wait_for_packets(packet_status, lock))
    task2 = asyncio.create_task(send_packets(packet_status, lock))

    await asyncio.gather(task1, task2)  # Don't forget "await"!


asyncio.run(main())
