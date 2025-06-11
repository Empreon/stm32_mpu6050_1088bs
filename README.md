# 3D Angle Visualization with MPU6050 and LED Matrices

A course project for EE 2004 – Microprocessor Systems at Marmara University. This project captures real-time 3D orientation data from an MPU6050 IMU and visualizes the roll, pitch, and yaw angles on a pair of 8x8 LED matrices, controlled by an STM32 Nucleo-L053R8 microcontroller.

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware](#hardware)
  - [Components](#components)
  - [Wiring](#wiring)
- [Software & Libraries](#software--libraries)
- [How It Works](#how-it-works)
  - [Data Acquisition with DMP](#data-acquisition-with-dmp)
  - [Interrupt-Driven Architecture](#interrupt-driven-architecture)
  - [Display Logic](#display-logic)
- [Setup and Installation](#setup-and-installation)
- [Code Structure](#code-structure)
- [Development Challenge & Solution](#development-challenge--solution)
- [Authors](#authors)

## Project Overview

The goal of this project is to process sensor data from an MPU6050 to calculate Euler angles (yaw, pitch, and roll) and display them dynamically. The visualization is achieved using two 8x8 LED matrices arranged perpendicularly to represent a 3D coordinate space:

- **Matrix 1 (Roll vs. Pitch):** The horizontal axis represents the roll angle (x-axis), and the vertical axis represents the pitch angle (y-axis).
- **Matrix 2 (Yaw vs. Pitch):** The horizontal axis represents the yaw angle (z-axis), and the vertical axis again represents the pitch angle (y-axis).

To ensure high performance and accuracy, the project leverages the MPU6050's integrated **Digital Motion Processor (DMP)**, which offloads the complex sensor fusion algorithms from the STM32 microcontroller.

## Features

- Real-time 3D orientation tracking (Roll, Pitch, Yaw).
- Visualizes angles on two perpendicular 8x8 LED matrices.
- Utilizes the MPU6050's onboard Digital Motion Processor (DMP) for efficient, high-performance angle calculation.
- Employs an interrupt-driven, non-blocking architecture for a highly responsive system.
- Built on the STM32 Nucleo-L053R8 platform and developed in the Arduino IDE.

## Hardware

### Components

| Component              | Description                                      |
| ---------------------- | ------------------------------------------------ |
| STM32 Nucleo-L053R8    | Microcontroller board                            |
| MPU6050                | Inertial Measurement Unit (Accelerometer + Gyro) |
| 2x 8x8 LED Matrix      | Display modules                                  |
| 2x MAX7219             | LED matrix driver ICs                            |
| Breadboard & Wires     | For connections                                  |

### Wiring

The components are connected using I2C for the sensor and SPI for the displays.

**MPU6050 (I2C Connection)**

| MPU6050 Pin | STM32 Pin | Description        |
| ----------- | --------- | ------------------ |
| VCC         | 5V        | Power              |
| GND         | GND       | Ground             |
| SCL         | D15 (SCL) | I2C Clock          |
| SDA         | D14 (SDA) | I2C Data           |
| INT         | D3        | Interrupt Signal   |

**MAX7219 LED Matrices (SPI Daisy-Chained)**

The two MAX7219 drivers are daisy-chained. The `DOUT` of the first driver is connected to the `DIN` of the second driver.

| MAX7219 (First Driver) Pin | STM32 Pin | Description        |
| -------------------------- | --------- | ------------------ |
| VCC                        | 5V        | Power              |
| GND                        | GND       | Ground             |
| DIN                        | D11 (MOSI)| SPI Data In        |
| CS (LOAD)                  | D10 (SS)  | SPI Chip Select    |
| CLK                        | D13 (SCK) | SPI Clock          |

## Software & Libraries

The firmware is developed using the **Arduino IDE** with the STM32Duino core. The following libraries are required:

- **`Wire.h`**: Standard library for I2C communication.
- **`I2Cdev.h`** and **`MPU6050_6Axis_MotionApps20.h`**: From Jeff Rowberg's i2cdevlib, for interfacing with the MPU6050 and its DMP.
- **`LedControl.h`**: For controlling the MAX7219 display drivers.

## How It Works

### Data Acquisition with DMP

Instead of running a sensor fusion algorithm like the Madgwick filter on the STM32, this project offloads the task to the MPU6050's onboard Digital Motion Processor (DMP). The DMP continuously processes raw accelerometer and gyroscope data, performs sensor fusion, and places the resulting orientation data (in the form of quaternions) into a FIFO buffer, ready for the microcontroller to read.

### Interrupt-Driven Architecture

The system is designed to be highly efficient and non-blocking.
1. The MPU6050's `INT` pin is connected to an interrupt-capable pin on the STM32.
2. When the DMP has a new data packet ready in the FIFO buffer, it generates a RISING edge signal on the `INT` pin.
3. This signal triggers an Interrupt Service Routine (ISR), `DMPDataReady()`, on the STM32.
4. The ISR's only job is to set a boolean flag, `MPUInterrupt = true;`.
5. The main `loop()` function constantly checks this flag. If it's `false`, the CPU does nothing, saving cycles. If it's `true`, it proceeds to read the data from the MPU.

### Display Logic

1. When new data is available, the main loop retrieves the data packet from the MPU's FIFO buffer.
2. It uses the library functions to convert the quaternion data into Euler angles (yaw, pitch, roll) in degrees.
3. The pitch angle is mapped from a range of `[-90, 90]` to a row index `[0, 7]` for the LED matrices.
4. The roll and yaw angles are similarly mapped to a column index `[0, 7]`. This index is then converted into a byte where only one bit is set (e.g., index 2 becomes `B00100000`) using the `mapAngle2LedBit()` helper function.
5. The displays are cleared, and `lc.setRow()` is used to light up the single LED on each matrix that corresponds to the current orientation.

## Setup and Installation

1.  **Hardware Assembly**: Connect the components as described in the [Wiring](#wiring) section. Ensure the MAX7219 drivers are correctly daisy-chained.
2.  **Software Setup**:
    - Install the [Arduino IDE](https://www.arduino.cc/en/software).
    - Install the [STM32Duino core](https://github.com/stm32duino/Arduino_Core_STM32) via the Board Manager.
    - Install the required libraries using the Arduino Library Manager:
      - `LedControl` by Eberhard Fahle
      - `MPU6050` by Jeff Rowberg (this will also install `I2Cdev`)
3.  **Upload Code**:
    - Open the `stm32_mpu6050_1088bs.ino` file in the Arduino IDE.
    - Select your STM32 Nucleo board from the `Tools > Board` menu.
    - Select the correct COM port.
    - Upload the sketch to the board.

## Code Structure

- **`setup()`**: Initializes Serial communication (for debugging), I2C, the MPU6050, and the DMP. It performs an initial calibration and enables the DMP interrupt. It also initializes the two MAX7219 drivers.
- **`loop()`**: The main non-blocking loop. It waits for the `MPUInterrupt` flag to be set, checks for FIFO overflow, reads the data packet, processes it into yaw, pitch, and roll, and updates the two LED matrices.
- **`DMPDataReady()`**: The Interrupt Service Routine (ISR) that sets the `MPUInterrupt` flag when new data is available from the sensor.
- **`mapAngle2LedBit()`**: A helper function to map an angle to a single bit in a byte, used for setting the correct column LED.

## Development Challenge & Solution

A key challenge was performance. An initial attempt using a software-based Madgwick sensor fusion filter on the STM32 was too computationally intensive, resulting in a low update frequency (400-500 Hz) and inaccurate angle calculations. This also led to frequent FIFO buffer overflows on the MPU6050, as the STM32 could not read the data fast enough.

The decisive solution was to switch to the **MPU6050's onboard DMP**. By offloading the complex fusion calculations to the sensor itself, the STM32 was freed to focus solely on data retrieval and display updates. This architectural change, combined with the interrupt-driven approach, resolved the performance bottleneck and eliminated the FIFO overflow issue, resulting in a highly responsive and stable system.

## Authors

- **Ahmet Utku YILMAZ** ([150723841](mailto:150723841@marun.edu.tr))
- **Feyzullah OĞUZ** ([150720035](mailto:150720035@marun.edu.tr))
