# 8x8x8 Volumetric LED Cube Display

## Project Overview

This repository contains STM32 firmware and Python host application for an 8x8x8 LED cube. The project aims to create an accessible yet high-performance true 3D display using 512 WS2812B Neopixel LEDs, controlled by an STM32F401RE microcontroller. It displays pre-programmed animations and renders 3D models from STL files received from the host application via serial communication.

A key feature is the high refresh rate (approx. 400 Hz) achieved through optimised firmware utilising Timer-synchronised DMA transfers to control 8 LED strips (planes) in parallel, leveraging the STM32's BSRR register and ARM bit-banding for efficient data preparation.


## Features

* **Hardware:** 8x8x8 matrix (512 WS2812B LEDs), custom 3D-printed base, STM32F401RE Nucleo board, Mean Well SMPS.
* **STM32 Firmware:**
    * Parallel driving of 8 LED strips/planes using TIM1 + DMA2 + GPIO BSRR.
    * High refresh rate (~400 Hz).
    * Efficient DMA buffer management using interrupts and bit-banding (based on Martin Hubacek's library).
    * Displays pre-programmed animations.
    * Receives 8x8x8 voxel data via UART.
    * Displays received voxel data (e.g., from STL files).
    * Real-time rotation of displayed voxel models via button input.
    * Software brightness control.
* **Python Host Application**
    * Graphical User Interface (PyQt5).
    * Loads standard STL files.
    * Processes STLs into an 8x8x8 voxel grid using a multi-resolution pipeline (Trimesh, NumPy, SciPy): Normalise -> 64x64x64 Voxelise -> Downsample (with threshold).
    * Visualises voxel grids using Matplotlib.
    * Handles serial port detection and connection (PySerial).
    * Packs 8x8x8 voxel data into 64 bytes with coordinate transformation.
    * Transmits data to STM32 using a start/end marker protocol.
    * Listens for the 'Ready' signal from STM32.
    * Uses background threading (QThread) for non-blocking STL processing.

## Hardware Requirements

* STM32F401RE Nucleo-64 Development Board (or similar STM32F4).
* 512 x WS2812B Individually Addressable RGB LEDs in 5mm through-hole package).
* Approx. 40-50 metres of Tinned Copper Wire (e.g., 18 SWG / 1.1-1.2mm diameter).
* 512 x 0.1µF Decoupling Capacitors (e.g., 0603 SMD package).
* Hookup Wire (e.g., 26 AWG) for data lines.
* 3D Printed Base Components (STL files would ideally be included in the repo).
* 3D Printed Jigs (for construction).
* Power Supply Unit (e.g., Mean Well LSP-160-5T - 5V, >25A recommended depending on brightness).
* Fused Mains Inlet Connector (e.g., C14 panel mount).
* Appropriate Mains Cable, Internal Wiring (e.g., 1.5mm²), Spade Terminals, Fuses.
* Connectors (e.g., Wago 221) for internal power distribution.
* Push Buttons (x2) for STM32 control.
* USB Cable (Type A to Mini-B for Nucleo board).

## Software Components

1.  **STM32 Firmware:** Located in the `LED_CUBE` directory. Developed using STM32CubeIDE and HAL libraries. Based on the WS2812B driver library by Martin Hubacek.
2.  **Python Host Application:** Located in the `Python` directory. Requires Python 3.x.

## Dependencies

**Python Host Application**

* PyQt5
* NumPy
* SciPy
* Trimesh (and its dependencies)
* Matplotlib
* PySerial

## Installation and Setup

**STM32 Firmware:**

1.  Open the project in STM32CubeIDE (or your preferred IDE).
2.  Ensure the correct target device (STM32F401RE) is selected.
3.  Verify pin assignments in the `.ioc` file or code match your hardware connections (especially TIM1, DMA2 streams/channels, GPIOC pins 0-7, UART2 pins, Button pins).
4.  Build the project.
5.  Flash the resulting binary (`.bin` or `.elf`) onto the Nucleo board using STM32CubeProgrammer or the IDE's built-in flashing tool via the ST-Link USB connection.


## Usage

1.  **Connect Hardware:** Connect the programmed STM32 Nucleo board to your computer via USB (provides power to MCU and serial connection). Connect the main 5V power supply to the cube base.
2.  **Run Python GUI:** Execute the main GUI script (e.g., `python gui.py`).
3.  **Select Serial Port:** Use the dropdown menu in the GUI to select the correct serial port corresponding to the STM32 Nucleo board (e.g., COMx on Windows, /dev/ttyACMx on Linux). Refresh the list if needed.
4.  **Connect:** Click the "Connect" button. The status should update to "Connected".
5.  **Select STL File:** Click "Select STL File" and choose a `.stl` model.
6.  **Process STL:** Click "Process STL File". A loading indicator should appear. Matplotlib windows will pop up showing the high-resolution and low-resolution voxelisations. Close these windows to continue. The GUI status should update to "Ready to send".
7.  **Send to Cube:** Wait for the status to indicate "Ready to send (STM32 is ready)" (meaning the STM32 sent the 0xAA signal). Click the "Send" button. The voxel data will be transmitted to the cube.
8.  **Interact with Cube:** Use the push buttons connected to the STM32 to cycle through animation patterns or display/rotate the loaded STL model.


The core WS2812B DMA driver implementation is based on work by Martin Hubacek and is typically licensed under the MIT License. Please refer to the specific source files for exact licensing terms.

## Acknowledgements

* Core WS2812B STM32 driver concepts adapted from Martin Hubacek (martinhubacek.cz).
* Guidance provided by Dr. Thomas Gilbert.
