/*
  Project: MPU6050 3D Angle Visualization on LED Matrices
  This code obtains raw data from an MPU6050 inertial measurement unit (IMU)
  using the I2C protocol for communication with an STM32 Nucleo-L053R8 board.
  The raw data is processed using the MPU6050's onboard Digital Motion Processor (DMP)
  to calculate orientation angles (e.g., roll, pitch, yaw).

  These angles are then visualized on two LD1088BS 8x8 LED matrices
  (code for LED matrices to be added later),
  driven by MAX7219 display driver ICs. Communication with the MAX7219 drivers
  is handled via the SPI protocol from the STM32. The LED matrices aim to
  represent the device's orientation on a conceptual 3D coordinate plane.
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Wire.h>
#include "LedControl.h"

MPU6050 mpu; // MPU6050 default I2C address is 0x68

int const INTERRUPT_PIN = 3; // Define the MPU interrupt pin (e.g., D3)
bool blinkState = false;     // For LED blinking

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false; // Set true if DMP init was successful
uint8_t MPUIntStatus;  // Holds actual interrupt status byte from MPU
uint8_t devStatus; // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;   // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;        // [w, x, y, z]         Quaternion container
VectorFloat gravity; // [x, y, z]            Gravity vector
float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// FIX: Add variables for timed printing
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
  We have only a single MAX72XX.
*/
LedControl lc = LedControl(11,13,10,1);

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial); // Wait for Serial port to connect. Needed for native USB port boards only.

  /*Initialize MPU6050 device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT); // MPU interrupt pin

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println(F("MPU6050 connection failed"));
    while (true)
      ; // Halt on failure
  } else {
    Serial.println(F("MPU6050 connection successful"));
  }

  /*Wait for Serial input to proceed (optional debug step)*/
  Serial.println(F("\nSend any character to begin DMP programming: "));
  while (Serial.available() && Serial.read())
    ; // Empty buffer
  while (!Serial.available())
    ; // Wait for data
  while (Serial.available() && Serial.read())
    ; // Empty buffer again

  /* Initialize and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    Serial.println(
      F("Performing live calibration (ensure MPU is stationary and flat)...")
    );
    mpu.CalibrateAccel(6); // Calibrate accelerometer
    mpu.CalibrateGyro(6);  // Calibrate gyroscope
    Serial.println(F("Live calibration finished. Active offsets:"));
    mpu.PrintActiveOffsets(); // Print the offsets

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true); // Turn on the DMP

    /*Enable interrupt detection*/
    Serial.print(
      F("Enabling interrupt detection (Arduino external interrupt ")
    );
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
  // Set the brightness to a medium values and clear the display
  lc.setIntensity(0,8);
  lc.clearDisplay(0);
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

  // Use a while loop to drain the FIFO.
  while (mpu.getFIFOCount() >= packetSize) {
    // Try to get a packet from FIFO.
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      // Get Yaw, Pitch, Roll values
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      float roll_deg = ypr[2] * 180.0f / M_PI;
      float pitch_deg = ypr[1] * 180.0f / M_PI;

      lc.clearDisplay(0);

      byte x_bits = mapAngle2LedBit(roll_deg);

      long y_index = map(pitch_deg, -90, 90, 0, 8);
      y_index = constrain(y_index, 0, 7);

      lc.setRow(0, y_index, x_bits);

      // Blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_BUILTIN, blinkState);
    }
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
  long index = map(angle, -90, 90, 0, 8);
  index = constrain(index, 0, 7);

  // Byte values produced by shifting B10000000 (128) value to right
  // index=0 -> 128 >> 0 -> B10000000
  // index=1 -> 128 >> 1 -> B01000000
  // ...
  // index=7 -> 128 >> 7 -> B00000001
  return (byte)(128 >> index);
}