# SAMC21 X-Pro EVK w/ QT8 X-Pro Touch Surface

## Overview
This repository contains the firmware for streaming 2D touch position data over UART to
[MPLAB Data Visualizer](https://www.microchip.com/en-us/development-tools-tools-and-software/embedded-software-center/mplab-data-visualizer)
from the [SAMC21 Xplained Pro](https://www.microchip.com/developmenttools/ProductDetails/ATSAMC21-XPRO)
with the [QT8 Xplained Pro Water-Tolerant Touch-Surface](https://www.microchip.com/developmenttools/ProductDetails/AC164161).

## Firmware Configurations
The included project is configured for self-capacitance sensing of the QT8.  The scan rate is set to 200Hz, so that
an updated touch status and position can be expected every 5ms.  In order to reconfigure, the touch parameters
can be adjusted within MHC or directly in {{touch.h}}.
