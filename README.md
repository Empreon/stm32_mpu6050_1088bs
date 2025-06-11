3D Angle Visualization with MPU6050 and LED Matrices

This project captures real-time 3D orientation data (yaw, pitch, and roll) from an MPU6050 Inertial Measurement Unit (IMU) and visualizes it on a pair of 8x8 LED matrices. The system is controlled by an STM32 Nucleo-L053R8 microcontroller and leverages the MPU6050's onboard Digital Motion Processor (DMP) for efficient, high-performance angle calculations.

Features

Real-time Orientation Tracking: Accurately tracks yaw, pitch, and roll angles.

Efficient DMP Processing: Offloads complex sensor fusion algorithms from the microcontroller to the MPU6050's DMP, ensuring high responsiveness and stability.

Interrupt-Driven Architecture: A non-blocking main loop processes data only when signaled by an interrupt from the MPU6050, minimizing CPU load and preventing data loss (FIFO overflow).

3D Coordinate Visualization: Two 8x8 LED matrices are arranged to represent a 3D space:

Matrix 0: Displays Roll (X-axis) vs. Pitch (Y-axis).

Matrix 1: Displays Yaw (Z-axis) vs. Pitch (Y-axis).

Arduino IDE Compatible: The firmware is developed in the Arduino environment for ease of use and portability.

Hardware Requirements

STM32 Nucleo-L053R8 (or a compatible STM32 board)

MPU6050 IMU Sensor Module

2 x MAX7219 8x8 Dot Matrix LED Display Modules

Breadboard and Jumper Wires

Software & Libraries

Arduino IDE

STM32 Core for Arduino

i2cdevlib by Jeff Rowberg (specifically the I2Cdev and MPU6050_6Axis_MotionApps20 libraries)

LedControl by Eberhard Fahle

Wiring and Connections

Connect the components as described in the tables below.

MPU6050 (I2C)
MPU6050 Pin	STM32 Nucleo Pin	Description
VCC	3.3V	Power
GND	GND	Ground
SCL	D15 (SCL)	I2C Clock
SDA	D14 (SDA)	I2C Data
INT	D3	Interrupt Signal
MAX7219 LED Matrices (SPI - Daisy Chained)

The two MAX7219 modules are daisy-chained. The DOUT of the first matrix connects to the DIN of the second.

MAX7219 Pin	STM32 Nucleo Pin	Description
VCC	5V	Power
GND	GND	Ground
DIN	D11 (MOSI)	SPI Data In (to Matrix 1)
CS / LOAD	D10	SPI Chip Select
CLK	D13 (SCK)	SPI Clock
How It Works

Initialization: The setup() function initializes serial communication, the I2C bus, the MPU6050 sensor, and the MAX7219 display drivers.

DMP Firmware: It then loads the Digital Motion Processor firmware onto the MPU6050. This allows the sensor itself to perform the complex sensor fusion calculations, converting raw accelerometer and gyroscope data into stable quaternion values.

Calibration: A live calibration of the accelerometer and gyroscope is performed to ensure accuracy.

Interrupt-Driven Data Acquisition: The MPU6050 is configured to issue a hardware interrupt on its INT pin whenever a new packet of orientation data is available in its FIFO (First-In, First-Out) buffer. The STM32 listens for this interrupt.

Non-Blocking Loop: The main loop() does nothing until the interrupt flag is set. This is highly efficient.

Data Processing: Once an interrupt is detected, the STM32 reads the data packet from the MPU's FIFO buffer. It extracts the quaternion and calculates the final yaw, pitch, and roll angles in degrees.

Visualization: The yaw, pitch, and roll angles (ranging from -90 to +90 degrees) are mapped to the 8x8 grid of the LED matrices. A single LED is lit on each matrix to represent the device's current orientation in the conceptual 3D space.

Installation and Usage

Hardware: Assemble the circuit according to the Wiring and Connections section.

IDE Setup: Install the Arduino IDE and add support for STM32 boards using the Boards Manager.

Install Libraries: Install the i2cdevlib and LedControl libraries. You can do this via the Arduino Library Manager or by downloading the repositories and adding them to your libraries folder.

Upload Code: Open the stm32_mpu6050_1088bs.ino file in the Arduino IDE. Select your STM32 board and the correct COM port.

Run: Upload the code. Open the Serial Monitor at 115200 baud. You will be prompted to send any character to begin the DMP calibration and start the program.

Visualize: Tilt the MPU6050 sensor and observe the corresponding LEDs light up on the two matrices.

Authors

Ahmet Utku YILMAZ

Feyzullah OĞUZ

License

This project is licensed under the MIT License - see the LICENSE.md file for details.
