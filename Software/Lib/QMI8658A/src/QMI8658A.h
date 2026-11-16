/*
 * QMI8658A.h
 *
 * Arduino library for the QMI8658A 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
 * Made for ESP32-C6 Touch LCD 1.47 board and my custom pcb board
 *
 * Reference: https://files.waveshare.com/upload/5/5f/QMI8658A_Datasheet_Rev_A.pdf
 *            https://docs.waveshare.com/ESP32-C6-Touch-LCD-1.47
 *
 * All register addresses, bit fields and timing values used in this driver
 * come directly from that datasheet.
 *
 * This library NEVER drives the INT2 pin. 
 * INT2 is kept permanently in High-Z mode explicitely in begin()
 * (CTRL1.INT2_EN is always written as 0, no other register that routes a signal to INT2 is touched in a way that would activate it).
 * This is enforced in every function that touches CTRL1, CTRL7, CTRL8 and after a software reset.
 * All interrupts used by this library (FIFO, motion detection, Wake on Motion, CTRL9 handshake) are routed exclusively to INT1.
 *
 * Created 15 oct. 2026 by dgm91.dev
 *
 * License: MIT
*/
#ifndef QMI8658A_H
#define QMI8658A_H

#include <Arduino.h>
#include <Wire.h>

/* HARDWARE CONFIG
 * Values below match the Waveshare Documentation.
 * The onboard QMI8658A I2C address depends on how its SA0 pin is strapped
 *   SA0 = 0 -> I2C address 0x6B
 *   SA0 = 1 -> I2C address 0x6A
 * Waveshare documentation FAQ tell us that it should be 0x6B
 * but for compatibility purposes lets put them both.
*/
#define QMI8658A_DEFAULT_SDA_PIN         18
#define QMI8658A_DEFAULT_SCL_PIN         19
#define QMI8658A_DEFAULT_INT1_PIN        5

#define QMI8658A_I2C_ADDRESS_LOW         0x6B   // SA0 = 0
#define QMI8658A_I2C_ADDRESS_HIGH        0x6A   // SA0 = 1
#define QMI8658A_DEFAULT_I2C_ADDRESS     QMI8658A_I2C_ADDRESS_LOW

#define QMI8658A_DEFAULT_I2C_FREQUENCY   400000UL // Hz, fSCL max = 400kHz

/* SENSOR CONFIG
 * Suggested default sensor operating parameters.
 * These settings must be applied using configureAccelerometer()
 * and configureGyroscope().
*/
#define QMI8658A_DEFAULT_ACCEL_RANGE_G   8    // ± 8g  
#define QMI8658A_DEFAULT_ACCEL_ODR_HZ    500  // Accel-only ODR
#define QMI8658A_DEFAULT_GYRO_RANGE_DPS  512  // ± 512 dps
#define QMI8658A_DEFAULT_GYRO_ODR_HZ     896  // ~896.8Hz 6DOF ODR

/* ---------------------------------------------------------------------
 * Public enumerations
 * --------------------------------------------------------------------- 
*/

// Error and status codes returned by most library functions.
enum class QMI8658A_Error : uint8_t {
    SUCCESS = 0,
    I2C_ERROR,              // I2C transaction failed (NACK, bus error...)
    WHO_AM_I_MISMATCH,      // WHO_AM_I did not read 0x05
    NOT_INITIALIZED,        // begin() was not called or failed
    INVALID_PARAMETER,      // out-of-range argument
    SENSOR_NOT_STATIONARY,  // calibration aborted: device is moving
    TIMEOUT,                // a hardware handshake did not complete
    FIFO_OVERFLOW,          // FIFO data was lost before it could be read
    NOT_ENABLED             // requested sensor is currently disabled
};

// Accelerometer full-scale range (CTRL2.aFS)
enum class AccelRange : uint8_t {
    RANGE_2G  = 0x00,
    RANGE_4G  = 0x01,
    RANGE_8G  = 0x02,
    RANGE_16G = 0x03
};

// Gyroscope full-scale range (CTRL3.gFS)
enum class GyroRange : uint8_t {
    RANGE_16DPS   = 0x00,
    RANGE_32DPS   = 0x01,
    RANGE_64DPS   = 0x02,
    RANGE_128DPS  = 0x03,
    RANGE_256DPS  = 0x04,
    RANGE_512DPS  = 0x05,
    RANGE_1024DPS = 0x06,
    RANGE_2048DPS = 0x07
};

/* Accelerometer Output Data Rate (CTRL2.aODR)
 * Note: the three highest rates (7174.4 / 3587.2 / 1793.6 Hz) 
 * are only valid when the gyroscope is ALSO enabled (6DOF mode)
 * the datasheet says "N/A" for accelerometer-only operation (Table 22)
*/
enum class AccelODR : uint8_t {
    ODR_7174_4HZ_6DOF_ONLY  = 0x00,
    ODR_3587_2HZ_6DOF_ONLY  = 0x01,
    ODR_1793_6HZ_6DOF_ONLY  = 0x02,
    ODR_1000HZ              = 0x03, // 896.8Hz  if gyro also enabled
    ODR_500HZ               = 0x04, // 448.4Hz  if gyro also enabled
    ODR_250HZ               = 0x05, // 224.2Hz  if gyro also enabled
    ODR_125HZ               = 0x06, // 112.1Hz  if gyro also enabled
    ODR_62_5HZ              = 0x07, // 56.05Hz  if gyro also enabled
    ODR_31_25HZ             = 0x08, // 28.025Hz if gyro also enabled
    ODR_LOWPOWER_128HZ      = 0x0C,
    ODR_LOWPOWER_21HZ       = 0x0D,
    ODR_LOWPOWER_11HZ       = 0x0E,
    ODR_LOWPOWER_3HZ        = 0x0F
};

// Gyroscope Output Data Rate (CTRL3.gODR)
enum class GyroODR : uint8_t {
    ODR_7174_4HZ = 0x00,
    ODR_3587_2HZ = 0x01,
    ODR_1793_6HZ = 0x02,
    ODR_896_8HZ  = 0x03,
    ODR_448_4HZ  = 0x04,
    ODR_224_2HZ  = 0x05,
    ODR_112_1HZ  = 0x06,
    ODR_56_05HZ  = 0x07,
    ODR_28_025HZ = 0x08
};

/* Digital filter selection for accelerometer / gyroscope (CTRL5)
 *  The QMI8658A silicon only implements a programmable Low-Pass Filter;
 *  there is no High-Pass Filter in this chip.
*/
enum class FilterMode : uint8_t {
    FILTER_NONE = 0,
    FILTER_LOW_PASS
};

// Low-Pass Filter bandwidth, as a percentage of the configured ODR
enum class LpfBandwidth : uint8_t {
    BW_2_66_PCT_ODR  = 0x00,
    BW_3_63_PCT_ODR  = 0x01,
    BW_5_39_PCT_ODR  = 0x02,
    BW_13_37_PCT_ODR = 0x03
};

// Physical axis, used by calibrateAccelerometerOffset().
enum class Axis : uint8_t { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };

// FIFO operating mode (FIFO_CTRL.FIFO_MODE)
enum class FifoMode : uint8_t {
    FIFO_DISABLED = 0x00,            // Bypass
    FIFO_STOP_WHEN_FULL = 0x01,      // FIFO mode
    FIFO_OVERWRITE_WHEN_FULL = 0x02  // Stream mode
};

// FIFO depth, in samples (FIFO_CTRL.FIFO_SIZE)
enum class FifoSize : uint8_t {
    SIZE_16_SAMPLES  = 0x00,
    SIZE_32_SAMPLES  = 0x01,
    SIZE_64_SAMPLES  = 0x02,
    SIZE_128_SAMPLES = 0x03
};

/* ---------------------------------------------------------------------
 * Data structures
 * --------------------------------------------------------------------- 
*/

// Accelerometer data converted to g.
struct AccelData {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Gyroscope data converted to °/s.
struct GyroData {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Combined accelerometer, gyroscope and chip temperature data
struct SensorData {
    AccelData accel;
    GyroData gyro;
    float temperatureC = 0.0f; // C for Celcius
};

// Board / bus configuration passed to begin() - See Settings Above
struct QMI8658A_Config {
    uint8_t  sdaPin       = QMI8658A_DEFAULT_SDA_PIN;
    uint8_t  sclPin       = QMI8658A_DEFAULT_SCL_PIN;
    uint8_t  int1Pin      = QMI8658A_DEFAULT_INT1_PIN;
    uint8_t  i2cAddress   = QMI8658A_DEFAULT_I2C_ADDRESS;
    uint32_t i2cFrequency = QMI8658A_DEFAULT_I2C_FREQUENCY;

    /* If set to true, this configures INT1 as a push-pull output on the sensor and as an Arduino INPUT pin,
     * so hardware interrupts can be used. If false (default), 
     * INT1 stays in High-Z and the library is used purely by polling status registers. 
     * INT2 is NEVER enabled, regardless of this setting. 
    */
    bool useInt1Pin = false;
};

/* Any-Motion / No-Motion detection configuration. Thresholds are in g,
 * durations in accelerometer samples. (see QMI8658A datasheet section 9)
*/
struct MotionDetectConfig {
    bool  enableX = true;
    bool  enableY = true;
    bool  enableZ = true;
    bool  useAndLogic = false; // false = use OR between enabled axes (default) , true use AND logic.
    float thresholdXg = 0.3f;
    float thresholdYg = 0.3f;
    float thresholdZg = 0.3f;
    uint8_t durationSamples = 3;
};

// Wake on Motion configuration (datasheet section 12)
struct WakeOnMotionConfig {
    float   thresholdG = 0.255f;             // 1 LSB = 1 mg, 0.0f disables WoM , 0.255g is the Maximum Value
    uint8_t blankingSamples = 5;           // samples ignored right after enable : set to 5 to avoid instability but can be reduce to 0
    bool    interruptInitialHigh = false;  // initial level of INT1 for WoM
};

/*Pedometer configuration (datasheet section 11)
 *Defaults below assume an accelerometer ODR close to 50Hz is used; re-tune if you use a different ODR.
*/
struct PedometerConfig {
    uint16_t sampleWindow       = 50;    // ped_sample_cnt
    float    peakToPeakThreshG  = 0.2f;  // ped_fix_peak2peak
    float    peakThreshG        = 0.1f;  // ped_fix_peak
    uint16_t timeoutSamples     = 200;   // ped_time_up
    uint8_t  minStepSamples     = 20;    // ped_time_low
    uint8_t  entryStepCount     = 10;    // ped_time_cnt_entry
    uint8_t  reportEverySteps   = 4;     // ped_sig_count
};

// Result of calibrate() (gyroscope offset + optional gain refinement)
struct CalibrationResult {
    bool success = false;            // stationary check passed & offsets applied
    bool gyroGainCalibrated = false; // true if on-chip COD (gyro X/Y gain) succeeded
    uint8_t codStatusRaw = 0xFF;     // raw COD_STATUS register, for debugging
    QMI8658A_Error error = QMI8658A_Error::SUCCESS;
};

// One decoded FIFO sample. Fields are only valid if the corresponding sensor was enabled while the FIFO was filled.
struct FifoSample {
    AccelData accel;
    GyroData  gyro;
};

/* ---------------------------------------------------------------------
 * QMI8658A driver class
 * --------------------------------------------------------------------- 
*/

class QMI8658A {
public:
    explicit QMI8658A(TwoWire &wireBus = Wire);

    // Initialization
    QMI8658A_Error begin(const QMI8658A_Config &config = QMI8658A_Config());
    bool isConnected();
    QMI8658A_Error softReset();

    // Accelerometer
    QMI8658A_Error enableAccelerometer(bool enable);
    bool isAccelerometerEnabled() const { return _accelEnabled; }
    QMI8658A_Error configureAccelerometer(AccelRange range, AccelODR odr);
    QMI8658A_Error setAccelerometerFilter(FilterMode mode, LpfBandwidth bandwidth = LpfBandwidth::BW_2_66_PCT_ODR);
    QMI8658A_Error readAccelerometer(AccelData &data);

    // Gyroscope
    QMI8658A_Error enableGyroscope(bool enable);
    bool isGyroscopeEnabled() const { return _gyroEnabled; }
    QMI8658A_Error configureGyroscope(GyroRange range, GyroODR odr);
    QMI8658A_Error setGyroscopeFilter(FilterMode mode, LpfBandwidth bandwidth = LpfBandwidth::BW_2_66_PCT_ODR);
    QMI8658A_Error readGyroscope(GyroData &data);

    // Temperature
    QMI8658A_Error readTemperature(float &celsius);

    // Combined read
    QMI8658A_Error readSensorData(SensorData &data);

    // Any-Motion / No-Motion detection
    QMI8658A_Error configureAnyMotion(const MotionDetectConfig &config);
    QMI8658A_Error enableAnyMotion(bool enable);
    bool isAnyMotionDetected();    // reads & clears STATUS1

    QMI8658A_Error configureNoMotion(const MotionDetectConfig &config);
    QMI8658A_Error enableNoMotion(bool enable);
    bool isNoMotionDetected();     // reads & clears STATUS1

    /* Wake on Motion
     * WoM is a distinct low-power operating mode of the chip (datasheet section 12) : entering it disables normal accelerometer/gyroscope data output.
     * enterWakeOnMotion() and exitWakeOnMotion() switch the whole sensor mode, instead of being a simple enable bit.
    */
    QMI8658A_Error enterWakeOnMotion(const WakeOnMotionConfig &config);
    QMI8658A_Error exitWakeOnMotion();
    bool isWakeOnMotionDetected(); // reads & clears STATUS1

    // Pedometer
    QMI8658A_Error configurePedometer(const PedometerConfig &config);
    QMI8658A_Error enablePedometer(bool enable);
    bool isPedometerEnabled() const { return _pedometerEnabled; }
    QMI8658A_Error getStepCount(uint32_t &steps);
    QMI8658A_Error resetStepCount();
    bool isStepDetected();         // reads & clears STATUS1

    /* Device Calibrations
     * calibrate() computes gyroscope zero-rate offsets by averaging stationary samples 
     * (no external reference needed, since a still gyroscope must read 0 dps on every axis), writes them to the chip,
     * and additionally runs the on-chip Calibration-On-Demand (COD) routine to refine gyroscope X/Y sensitivity gain.
     *
     * Accelerometer offsets are intentionally NOT touched by calibrate():
     * computing them correctly requires knowing which axis is vertical
     * (an external reference about the device's orientation), which this function has no way to know on its own.
     * Use calibrateAccelerometerOffset() if you can supply that reference.
    */
    CalibrationResult calibrate(uint16_t samples = 64,
                                 float gyroStationaryThresholdDps = 1.0f,
                                 float accelStationaryThresholdG = 0.05f,
                                 unsigned long timeoutMs = 3000);

    /* Accelerometer offset calibration requires the caller to state
     * the device's orientation (which axis points up, and in which direction) 
     * as an explicit external reference. 
    */
    QMI8658A_Error calibrateAccelerometerOffset(Axis verticalAxis,
                                                 bool positiveDirection,
                                                 uint16_t samples = 64);

    /* FIFO (optional)
     * Simple, reliable FIFO support. Disabled by default; 
     * classic register-based reads (readAccelerometer/readGyroscope/readSensorData) work regardless of FIFO state. 
    */
    QMI8658A_Error enableFifo(FifoMode mode, FifoSize size, uint8_t watermarkSamples = 0);
    QMI8658A_Error disableFifo();
    QMI8658A_Error resetFifo();
    QMI8658A_Error getFifoSampleCount(uint16_t &sampleCount);

    /* Reads up to maxSamples from the FIFO into outSamples. samplesRead is
     * set to the number of samples actually decoded.
     * Returns QMI8658A_Error::FIFO_OVERFLOW if an overflow occurred (data already lost before the read);
     * samples successfully read are still valid. 
    */
    QMI8658A_Error readFifo(FifoSample *outSamples, uint16_t maxSamples, uint16_t &samplesRead);

private:
    TwoWire *_wire;
    uint8_t  _i2cAddress;
    uint8_t  _int1Pin;
    bool     _useInt1Pin;
    bool     _initialized;

    bool _accelEnabled;
    bool _gyroEnabled;
    bool _pedometerEnabled;
    bool _anyMotionEnabled;
    bool _noMotionEnabled;
    bool _fifoEnabled;
    bool _wakeOnMotionActive;

    AccelRange _accelRange;
    GyroRange  _gyroRange;

    uint8_t _ctrl1Shadow;
    uint8_t _ctrl7Shadow;
    uint8_t _ctrl8Shadow;

    // low level I2C helpers
    QMI8658A_Error writeRegister(uint8_t reg, uint8_t value);
    QMI8658A_Error readRegister(uint8_t reg, uint8_t &value);
    QMI8658A_Error readRegisters(uint8_t startReg, uint8_t *buffer, size_t length);
    QMI8658A_Error writeRegisterBits(uint8_t reg, uint8_t mask, uint8_t value, uint8_t &shadow);

    // Internal helpers
    QMI8658A_Error applyBaseConfiguration();
    QMI8658A_Error executeCtrl9Command(uint8_t command, unsigned long timeoutMs = 100);
    float accelSensitivityLsbPerG() const;
    float gyroSensitivityLsbPerDps() const;
    QMI8658A_Error readRawAccel(int16_t &x, int16_t &y, int16_t &z);
    QMI8658A_Error readRawGyro(int16_t &x, int16_t &y, int16_t &z);
    QMI8658A_Error setSensorFilter(bool isAccel, FilterMode mode, LpfBandwidth bandwidth);
};

#endif //QMI8658A_H
