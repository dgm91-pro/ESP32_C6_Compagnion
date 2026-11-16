>[!WARNING]
>No hardware test was performed while building this library;
>all register sequences were implemented directly against the datasheet,
>but only a real test on-board can check everything out.
>A compile check with the ESP32-C6 Dev Module in Arduino IDE was made without errors.

# QMI8658A Arduino Library

Arduino IDE library for the **QMI8658A**, written specifically for the **Waveshare ESP32-C6
Touch LCD 1.47** board and my pcb which require **GPIO6 ( INT2 )** to always be in **High-Z** mode

All registers and settings come from this datasheet : https://files.waveshare.com/upload/5/5f/QMI8658A_Datasheet_Rev_A.pdf

## Software Tweaks

**INT2 is permanently kept in High-Z mode.** 

This library never configures INT2 as an active output, never routes any interrupt source to it, and explicitely disable it in the `applyBaseConfiguration()` function ( QMI8658A.cpp )

Every interrupt-capable feature (FIFO, Any-Motion, No-Motion, Wake on Motion, pedometer, CTRL9 handshakes) is routed
exclusively to **INT1**.

## Installation

* Copy the **QMI8658A** folder into your Arduino `libraries`
   folder (`Documents/Arduino/libraries/QMI8658A`), or use Arduino
   IDE's "Add .ZIP Library..." with the provided ZIP archive.
* Restart the Arduino IDE.
* Open `File > Examples > QMI8658A` to try the example sketches.

## Hardware configuration

All pins and defaults are at the top of
`src/QMI8658A.h`:

```cpp
#define QMI8658A_DEFAULT_SDA_PIN         18
#define QMI8658A_DEFAULT_SCL_PIN         19
#define QMI8658A_DEFAULT_INT1_PIN        5
#define QMI8658A_DEFAULT_I2C_ADDRESS     0x6B
#define QMI8658A_DEFAULT_I2C_FREQUENCY   400000UL
```

These match the **Waveshare ESP32-C6 Touch LCD 1.47** documentation : 
https://docs.waveshare.com/ESP32-C6-Touch-LCD-1.47

>[!NOTE]
>The I2C address depends on the sensor SA0 pin `begin()` automatically tries the alternate valid address
>(0x6A) if the default (0x6B) does not respond, so this should work out-of-the-box.

## Features

- **Accelerometer**: X/Y/Z axies in g, configurable range (±2/4/8/16g) and ODR, commutable Low-Pass Filter.
- **Gyroscope**: X/Y/Z axies in °/s, configurable range (±16 to ±2048 dps) and ODR, commutable Low-Pass Filter.
- **Temperature**: internal temperature sensor of the chip, in °C.
- **Motion detection**: Any-Motion and No-Motion, with per-axis enable/disable thresholds and duration. ( no S-Motion implemented )
- **Wake on Motion**: ultra-low-power mode with configurable threshold, blanking time and interrupt polarity.
- **Pedometer**: enable/disable, step count read, step count reset.
- **Software Reset**: full chip reset following the official sequence, with the library's internal state brought back to a consistent default.
- **Calibration**:
  
  **`calibrate()`** : computes and applies gyroscope zero-rate offsets (no external reference needed - a still gyroscope must read ~0 dps),
   plus an on-chip gain refinement (Calibration-On-Demand) for gyroscope X/Y.
  
  **`calibrateAccelerometerOffset()`** : Accelerometer offset calibration requires you to explicitly state the device's orientation (which axis is vertical). Because the chip has no way to determine accelerometer bias without knowing gravity's direction.
  
- **FIFO (optional)**: simple, overflow-checked FIFO read support for
  smoother acquisition, if you need it. Not required for normal use.

>[!NOTE]
> *Filters Important Hardware Limitation*
>
> The QMI8658A chip only implements a **Low-Pass Filter**.
> It does **not** have a High-Pass Filter.

## Error handling

Nearly every function returns a `QMI8658A_Error`:

```cpp
enum class QMI8658A_Error : uint8_t {
    SUCCESS, I2C_ERROR, WHO_AM_I_MISMATCH, NOT_INITIALIZED,
    INVALID_PARAMETER, SENSOR_NOT_STATIONARY,
    TIMEOUT, FIFO_OVERFLOW, NOT_ENABLED
};
```

Please Always check the return value, especially for `begin()`.

## Examples

- `BasicReadings`  : Accelerometer, gyroscope and temperature.
- `MotionDetection`: Any-Motion / No-Motion setup and detection, with a comment showing how to use WoM instead.
- `Pedometer`      : Step counting.
- `Calibration`    : Gyroscope offset/gain calibration, and comment for accelerometer calibration.


## Details And Limitations

- **Software Reset command value**:

  the datasheet contains 2 different values for the same operation - **Table 27 (register description)** states
  writing *0xB0* to the RESET register triggers the reset, while **section
  7.4** states *0x0B*. This library follows *Table 27 (0xB0)*, which is
  the register's own formal description and matches widely-used QMI8658
  reference implementations. If your specific chip revision requires
  0x0B instead, you need to edit this : (`QMI8658A::softReset()` in `QMI8658A.cpp`).
  
- **Gyroscope offset sign convention**:

  the datasheet documents the format of the "host delta offset" registers (signed, 1/32 dps per LSB)
  but does not explicitly state whether the delta is added or subtracted internally.
  This library assumes it is **added** to the raw reading, and therefore writes the **negative** of the measured bias.
  If your specific chip behaves the other way around, flip the sign in `QMI8658A::calibrate()`.
  
- **Any-Motion and No-Motion share hardware registers**:

  the QMI8658A's `CTRL9` "Configure Motion" command transmits both Any-Motion and
  No-Motion parameters together in the same two command sets (see
  datasheet Table 34). Calling `configureAnyMotion()` therefore always
  resets the No-Motion thresholds to 0 unless No-Motion is configured
  again afterwards (and vice versa). Always call **both**
  `configureAnyMotion()` and `configureNoMotion()` if you intend to use
  both features simultaneously, in whichever order, finishing with the
  one you configured last.
  
- **Significant-Motion and Tap detection** are not implemented in this library.
  
- **I3C and SPI interfaces** are not implemented; only I2C is supported.

- **Pedometer default parameters** in `PedometerConfig` are the
  datasheet's own worked example, tuned for an accelerometer ODR around 50Hz.
  If you use a different ODR, re-tune `sampleWindow`, `timeoutSamples`, `minStepSamples` etc. accordingly.

- The exact I2C address of the onboard QMI8658A (0x6A vs 0x6B) is not
  explicitly stated in Waveshare's public documentation for this board;
  `begin()` auto-probes both, but if you know your board's actual strap,
  set `QMI8658A_Config::i2cAddress` explicitly to skip the probe.
