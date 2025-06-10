/*
  Project: MPU6050 3D Angle Visualization on LED Matrices
  This code obtains raw data from an MPU6050 inertial measurement unit (IMU)
  using the I2C protocol for communication with an STM32 Nucleo-L053R8 board.
  The raw data is processed using the MPU6050's onboard Digital Motion Processor (DMP)
  to calculate orientation angles (e.g., roll, pitch, yaw).

  These angles are then visualized on two LD1088BS 8x8 LED matrices,
  driven by MAX7219 display driver ICs. Communication with the MAX7219 drivers
  is handled via the SPI protocol from the STM32. The LED matrices aim to
  represent the device's orientation on a conceptual 3D coordinate plane.
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Wire.h>
#include "LedControl.h"

/*-----Global Constants-----*/
#define SPI_DATA_PIN 11
#define SPI_CLK_PIN 13
#define SPI_LOAD_PIN 10
#define NUM_MAX7219_DEVICES 2

#define ANGLE_MIN -90
#define ANGLE_MAX 90
#define LED_MATRIX_MIN 0
#define LED_MATRIX_MAX 7

MPU6050 mpu; // MPU6050 default I2C address is 0x68

int const INTERRUPT_PIN = 3; // Define the MPU interrupt pin (D3)
bool blinkState = false;     // For LED blinking

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;        // [w, x, y, z]         Quaternion container
VectorFloat gravity; // [x, y, z]            Gravity vector
float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// Print control variables
unsigned long lastPrintTime = 0;
const unsigned int PRINT_INTERVAL = 100; // Print every 250ms

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false; // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

/*-----MAX7219 pins-----*/
/*
  pin 11 is connected to the DataIn
  pin 13 is connected to the CLK
  pin 10 is connected to LOAD
*/
LedControl lc = LedControl(SPI_DATA_PIN, SPI_CLK_PIN, SPI_LOAD_PIN, NUM_MAX7219_DEVICES);

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. (Maximum limit of MPU6050)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  while (!Serial); // Wait for Serial port to connect.

  /*Initialize MPU6050 device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT); // MPU interrupt pin

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println(F("MPU6050 connection failed"));
    while (true); // Halt on failure
  } else {
    Serial.println(F("MPU6050 connection successful"));
  }

  /*Wait for Serial input to proceed. (Also, used for stopping the code)*/
  Serial.println(F("\nSend any character to begin DMP programming: "));
  while (Serial.available() && Serial.read()); // Empty buffer
  while (!Serial.available());                 // Wait for data
  while (Serial.available() && Serial.read()); // Empty buffer again

  /* Initialize and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    Serial.println(F("Performing live calibration (ensure MPU is stationary and flat)..."));
    mpu.CalibrateAccel(6); // Calibrate accelerometer
    mpu.CalibrateGyro(6);  // Calibrate gyroscope
    Serial.println(F("Live calibration finished."));

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true); // Turn on the DMP

    /*Enable interrupt detection*/
    Serial.print(F("Enabling interrupt detection (STM32 Nucleo Board external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus(); // Read initial interrupt status

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(LED_BUILTIN, OUTPUT);

  /*
  The MAX72XX is in power-saving mode on startup,
  we have to do a wakeup call
  */
  lc.shutdown(0,false);
  lc.setIntensity(0,8);
  lc.clearDisplay(0);

  lc.shutdown(1,false);
  lc.setIntensity(1,8);
  lc.clearDisplay(1);
}

void loop() {
  if (!DMPReady || !MPUInterrupt) {
    return; // If DMP setup failed or no interrupt, do nothing
  }

  // We have an interrupt, so let's process the FIFO
  MPUInterrupt = false; // Reset our software interrupt flag
  MPUIntStatus = mpu.getIntStatus();

  // Check for FIFO overflow
  if (MPUIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) {
    mpu.resetFIFO(); // Reset the FIFO buffer if it has overflowed
    Serial.println(F("FIFO overflow detected!"));
    return; // Skip processing this iteration
  }

  bool new_data_processed = false;
  // Use a while loop to drain the FIFO.
  while (mpu.getFIFOCount() >= packetSize) {
    // Try to get a packet from FIFO.
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      // Get Yaw, Pitch, Roll values
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      new_data_processed = true;
    }
  }

  // If we processed any new data, update the display
  if (new_data_processed) {
    float roll_deg = ypr[2] * 180.0f / M_PI;
    float pitch_deg = ypr[1] * 180.0f / M_PI;
    float yaw_deg = ypr[0] * 180.0f / M_PI;

    // Clear both displays once
    lc.clearDisplay(0);
    lc.clearDisplay(1);

    // Map angles to LED coordinates
    byte x_bits = mapAngle2LedBit(roll_deg);
    byte z_bits = mapAngle2LedBit(yaw_deg);
    long y_index = map(pitch_deg, ANGLE_MIN, ANGLE_MAX, LED_MATRIX_MIN, LED_MATRIX_MAX);

    // Draw the points on the matrices
    // Matrix 0: Roll (X) vs Pitch (Y)
    lc.setRow(0, y_index, x_bits);
    // Matrix 1: Yaw (Z) vs Pitch (Y)
    lc.setRow(1, y_index, z_bits);

    // Blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }

  /*
  // Only print the latest data at a fixed interval
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime > PRINT_INTERVAL) {
    lastPrintTime = currentTime; // Update the last print time
    printRollPitchYaw();
  }
  */
}

void printRollPitchYaw() {
  Serial.print("Yaw: ");
  Serial.print(ypr[0] * 180.0f / M_PI);
  Serial.print("\t Pitch: ");
  Serial.print(ypr[1] * 180.0f / M_PI);
  Serial.print("\t Roll: ");
  Serial.println(ypr[2] * 180.0f / M_PI);
}

byte mapAngle2LedBit(float angle) {
  long index = map(angle, ANGLE_MIN, ANGLE_MAX, LED_MATRIX_MIN, LED_MATRIX_MAX);

  // Byte values produced by shifting B10000000 (128) value to right
  // index=0 -> 128 >> 0 -> B10000000
  // index=1 -> 128 >> 1 -> B01000000
  // ...
  // index=7 -> 128 >> 7 -> B00000001
  return (byte)(128 >> index);
}