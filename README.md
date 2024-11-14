# BMS (Battery Management System)

## Overview

The **BMS** project is a comprehensive Battery Management System designed to monitor and manage multiple battery packs.
It
leverages the LTC6812 battery monitoring ICs and an ESP32-based development board to provide real-time data on cell
voltages and temperatures. The system facilitates safe operation by ensuring cells remain within specified voltage
thresholds and offers scalability for complex battery configurations.

## Table of Contents

- [Features](#features)
- [What the Program Does](#what-the-program-does)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [License](#license)

## Features

- Real-Time Monitoring: Continuously tracks cell voltages and temperatures.
- SPI Communication: Interfaces with LTC6812 ICs using SPI for efficient data transfer.
- Multi-Pack Support: Capable of managing multiple battery packs with independent SPI channels.
- Fault Detection: Detects over-voltage and under-voltage conditions to prevent battery damage.
- Extensible Architecture: Designed to accommodate future enhancements like additional sensors or communication
  protocols.
- PlatformIO Integration: Utilizes PlatformIO for streamlined development and testing workflows.

## What the Program Does

The BMS program performs the following key functions to ensure effective battery management:

1. Initialization and Configuration:
    - SPI Setup: Configures two independent SPI channels on the ESP32 to communicate with multiple LTC6812 ICs, each
      managing a separate battery pack.
    - LTC6812 Configuration: Initializes and configures the LTC6812 ICs with specified voltage thresholds, GPIO
      settings,
      and discharge parameters to prepare for accurate monitoring.
    - TWAI (CAN) Bus Integration: Installs and initializes the TWAI driver to enable CAN bus communication for potential
      future enhancements and integration with other vehicle systems.

2. Voltage Monitoring:
    - Data Acquisition: Periodically wakes up the LTC6812 ICs to perform ADC voltage conversions, capturing real-time
      cell voltage data across all connected battery packs.
    - Data Processing: Reads and validates the voltage data, ensuring integrity through PEC (Packet Error Code) checks.
      If errors are detected, they are logged for diagnostic purposes.
    - Voltage Timestamping: Records the exact time each voltage snapshot is taken, facilitating accurate time-based data
      analysis.

3. Temperature Monitoring (Planned):
    - Data Acquisition: Although currently marked as TODO, the program is structured to incorporate temperature sensor
      data in the future.
    - Data Processing: Intended to process and integrate temperature readings alongside voltage data to provide
      comprehensive battery health metrics.

4. Data Output:
    - Serial Communication: Formats the collected voltage (and future temperature) data into a structured table and
      outputs it to the serial monitor for real-time observation and debugging.
    - Task Management: Utilizes FreeRTOS tasks to handle different aspects of data acquisition and output concurrently,
      ensuring efficient and non-blocking operations.

5. SPI Channel Management:
    - Dynamic Switching: Alternates between SPI channels to manage multiple battery packs, ensuring balanced
      communication and preventing data collision.
    - Error Handling: Detects and recovers from invalid SPI states, ensuring continuous and reliable communication with
      all connected battery packs.

6. Fault Detection and Safety:
    - Voltage Threshold Enforcement: Monitors cell voltages against predefined over-voltage and under-voltage thresholds
      to prevent battery damage and ensure safe operation.
    - Discharge Control: Manages discharge switches to balance cell voltages and maintain overall battery health.

7. Extensibility:
    - CAN Bus Communication: Prepares the system for integration with other vehicle systems via the TWAI (CAN) bus,
      enabling future enhancements like remote monitoring or control.
    - Modular Architecture: Designed to easily incorporate additional sensors or communication protocols as needed.

By executing these functions, the BMS program ensures that battery packs are efficiently monitored, managed, and
protected, thereby enhancing the reliability and longevity of the power system it oversees.

## Hardware Requirements

- ESP32 Development Board:
    - Recommended: AZ-Delivery ESP32-DevKitC V4
- LTC6812 Battery Monitoring ICs:
    - Quantity based on the number of battery packs.
- Power Supply:
    - Suitable for the ESP32 and connected battery packs.
- Connecting Wires and SPI Components:
    - For establishing SPI communication between the ESP32 and LTC6812 ICs.
- Additional Components:
    - Depending on specific use cases (e.g., CAN transceivers for TWAI communication).

## Software Requirements

- PlatformIO:
    - Integrated Development Environment (IDE) for embedded development.
- Arduino Framework:
    - Provides the necessary libraries and tools for programming the ESP32.
- Linduino Libraries:
    - `LTC6812.h` and `LTC681x.h` for interfacing with the LTC6812 ICs.
    - Available at Analog Devices Linduino Libraries.
- CMake:
    - For managing the build process of the C components.
- FreeRTOS:
    - Embedded operating system for task management on the ESP32.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
