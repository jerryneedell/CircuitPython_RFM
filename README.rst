Introduction
============




.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/jerryneedell/CircuitPython_RFM/workflows/Build%20CI/badge.svg
    :target: https://github.com/jerryneedell/CircuitPython_RFM/actions
    :alt: Build Status


.. image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json
    :target: https://github.com/astral-sh/ruff
    :alt: Code Style: Ruff

CAUTION: This library is a "Work In Progress", Breaking changes may be made at any time.
CircuitPython support for RFM9x (LoRa,FSK/OOK) and RFM69 (FSK/OOK)  modules.
With FSK/OOK RFM9x and RFM69 modules may communicate with eachother.
LoRa is only supported on the RFM9x.

The default configuration is compatible with the Arduino RadioHead library and incorparates the required 4 byte RadioHead Header.
THe header amy be optionally disabled for transmission of "raw" packets for compatibility with other libraries such as RadioLib.

The library uses aysncio internally so it must be installed. The user code does not have to use asyncio, but it is still used internally.
Examples with and without asyncio are provided.


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

Installing from PyPI
=====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/circuitpython-rfm/>`_.
To install for current user:

.. code-block:: shell

    pip3 install circuitpython-rfm

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install circuitpython-rfm

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .env/bin/activate
    pip3 install circuitpython-rfm

Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install rfm

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============
Comming soon

Documentation
=============
API documentation for this library can be found on `Read the Docs <https://circuitpython-rfm.readthedocs.io/>`_.

For information on building library documentation, please check out
`this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/jerryneedell/CircuitPython_RFM/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
