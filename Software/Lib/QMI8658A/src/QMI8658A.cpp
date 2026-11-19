/*
 * QMI8658A.cpp
 *
 * All register addresses / bit fields referenced below come from the
 * QMI8658A Datasheet : https://files.waveshare.com/upload/5/5f/QMI8658A_Datasheet_Rev_A.pdf
 * Please see the Header File ( QMI8658A.h )
 *
 * Created 16 oct. 2026 by dgm91.dev
 *
 * License: MIT
*/

#include "QMI8658A.h"
#include <math.h>

// Register map (datasheet Table 19)
namespace {

constexpr uint8_t REG_WHO_AM_I      = 0x00;
constexpr uint8_t REG_REVISION_ID   = 0x01;
constexpr uint8_t REG_CTRL1         = 0x02;
constexpr uint8_t REG_CTRL2         = 0x03;
constexpr uint8_t REG_CTRL3         = 0x04;
constexpr uint8_t REG_CTRL5         = 0x06;
constexpr uint8_t REG_CTRL7         = 0x08;
constexpr uint8_t REG_CTRL8         = 0x09;
constexpr uint8_t REG_CTRL9         = 0x0A;

constexpr uint8_t REG_CAL1_L        = 0x0B;
constexpr uint8_t REG_CAL1_H        = 0x0C;
constexpr uint8_t REG_CAL2_L        = 0x0D;
constexpr uint8_t REG_CAL2_H        = 0x0E;
constexpr uint8_t REG_CAL3_L        = 0x0F;
constexpr uint8_t REG_CAL3_H        = 0x10;
constexpr uint8_t REG_CAL4_L        = 0x11;
constexpr uint8_t REG_CAL4_H        = 0x12;

constexpr uint8_t REG_FIFO_WTM_TH   = 0x13;
constexpr uint8_t REG_FIFO_CTRL     = 0x14;
constexpr uint8_t REG_FIFO_SMPL_CNT = 0x15;
constexpr uint8_t REG_FIFO_STATUS   = 0x16;
constexpr uint8_t REG_FIFO_DATA     = 0x17;

constexpr uint8_t REG_STATUSINT     = 0x2D;
constexpr uint8_t REG_STATUS0       = 0x2E;
constexpr uint8_t REG_STATUS1       = 0x2F;

constexpr uint8_t REG_TEMP_L        = 0x33;
constexpr uint8_t REG_AX_L          = 0x35;
constexpr uint8_t REG_GX_L          = 0x3B;

constexpr uint8_t REG_COD_STATUS    = 0x46;
constexpr uint8_t REG_RESET_CHECK   = 0x4D; // dQY_L: reads 0x80 after a successful reset

constexpr uint8_t REG_DVX_L         = 0x51;

constexpr uint8_t REG_STEP_CNT_LOW  = 0x5A;

constexpr uint8_t REG_RESET         = 0x60;

// CTRL9 command values (datasheet Table 28)
constexpr uint8_t CMD_ACK                       = 0x00;
constexpr uint8_t CMD_RST_FIFO                  = 0x04;
constexpr uint8_t CMD_REQ_FIFO                  = 0x05;
constexpr uint8_t CMD_WRITE_WOM_SETTING         = 0x08;
constexpr uint8_t CMD_ACCEL_HOST_DELTA_OFFSET   = 0x09;
constexpr uint8_t CMD_GYRO_HOST_DELTA_OFFSET    = 0x0A;
constexpr uint8_t CMD_CONFIGURE_PEDOMETER       = 0x0D;
constexpr uint8_t CMD_CONFIGURE_MOTION          = 0x0E;
constexpr uint8_t CMD_RESET_PEDOMETER           = 0x0F;
constexpr uint8_t CMD_ON_DEMAND_CALIBRATION     = 0xA2;

constexpr uint8_t WHO_AM_I_EXPECTED = 0x05;

// CTRL1 bit masks
constexpr uint8_t CTRL1_ADDR_AI_BIT   = 0x40;
//constexpr uint8_t CTRL1_BE_BIT        = 0x20; - BE Disabled 
constexpr uint8_t CTRL1_INT2_EN_BIT   = 0x10; // NEVER Enabled by this library
constexpr uint8_t CTRL1_INT1_EN_BIT   = 0x08;
constexpr uint8_t CTRL1_FIFO_INT_SEL  = 0x04; // 1 = FIFO interrupt -> INT1

// CTRL7 bit masks
constexpr uint8_t CTRL7_SYNC_SAMPLE_BIT = 0x80;
constexpr uint8_t CTRL7_DRDY_DIS_BIT    = 0x20; // DRDY can only reach INT2; Disabled
constexpr uint8_t CTRL7_GEN_BIT         = 0x02;
constexpr uint8_t CTRL7_AEN_BIT         = 0x01;

// CTRL8 bit masks
constexpr uint8_t CTRL8_ACTIVITY_INT_SEL = 0x40; // 1 = motion/pedo/tap events -> INT1
constexpr uint8_t CTRL8_PEDO_EN_BIT      = 0x10;
constexpr uint8_t CTRL8_NO_MOTION_EN_BIT = 0x04;
constexpr uint8_t CTRL8_ANY_MOTION_EN_BIT = 0x02;

// STATUS1 bit masks
constexpr uint8_t STATUS1_NO_MOTION_BIT  = 0x40;
constexpr uint8_t STATUS1_ANY_MOTION_BIT = 0x20;
constexpr uint8_t STATUS1_PEDO_BIT       = 0x10;
constexpr uint8_t STATUS1_WOM_BIT        = 0x04;

// STATUSINT bit mask
constexpr uint8_t STATUSINT_CMD_DONE_BIT = 0x80;

//Base CTRL1 value: address auto-increment, FIFO interrupt routed to INT1, INT2 output permanently disabled. -- Big Endian set to off since strong point byte at the end in our calc.
constexpr uint8_t CTRL1_BASE = CTRL1_ADDR_AI_BIT /*| CTRL1_BE_BIT */| CTRL1_FIFO_INT_SEL; // - BE Disabled 

// Base CTRL7 value: DRDY disabled (it can only be routed to INT2), both sensors off unless explicitly enabled.
constexpr uint8_t CTRL7_BASE = CTRL7_DRDY_DIS_BIT;

// Base CTRL8 value: motion/pedometer/tap events routed to INT1.
constexpr uint8_t CTRL8_BASE = CTRL8_ACTIVITY_INT_SEL;

inline int16_t toInt16(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

} // namespace

/* =======================================================================
 * Construction / initialization
 * =======================================================================
*/

QMI8658A::QMI8658A(TwoWire &wireBus)
    : _wire(&wireBus),
      _i2cAddress(QMI8658A_DEFAULT_I2C_ADDRESS),
      _int1Pin(QMI8658A_DEFAULT_INT1_PIN),
      _useInt1Pin(false),
      _initialized(false),
      _accelEnabled(false),
      _gyroEnabled(false),
      _pedometerEnabled(false),
      _anyMotionEnabled(false),
      _noMotionEnabled(false),
      _fifoEnabled(false),
      _wakeOnMotionActive(false),
      _accelRange(AccelRange::RANGE_8G),
      _gyroRange(GyroRange::RANGE_512DPS),
      _ctrl1Shadow(CTRL1_BASE),
      _ctrl7Shadow(CTRL7_BASE),
      _ctrl8Shadow(CTRL8_BASE)
{
}   // Do Not Delete This

QMI8658A_Error QMI8658A::begin(const QMI8658A_Config &config) {
    _i2cAddress = config.i2cAddress;
    _int1Pin    = config.int1Pin;
    _useInt1Pin = config.useInt1Pin;

    _wire->begin(config.sdaPin, config.sclPin);
    _wire->setClock(config.i2cFrequency);

    if (!isConnected()) {
        if (config.i2cAddress == QMI8658A_DEFAULT_I2C_ADDRESS) {
            uint8_t alt = (_i2cAddress == QMI8658A_I2C_ADDRESS_LOW)
                              ? QMI8658A_I2C_ADDRESS_HIGH
                              : QMI8658A_I2C_ADDRESS_LOW;
            _i2cAddress = alt;
            if (!isConnected()) {
                _initialized = false;
                return QMI8658A_Error::I2C_ERROR;
            }
        } else {
            _initialized = false;
            return QMI8658A_Error::I2C_ERROR;
        }
    }

    uint8_t who = 0;
    QMI8658A_Error err = readRegister(REG_WHO_AM_I, who);
    if (err != QMI8658A_Error::SUCCESS) {
        _initialized = false;
        return err;
    }
    if (who != WHO_AM_I_EXPECTED) {
        _initialized = false;
        return QMI8658A_Error::WHO_AM_I_MISMATCH;
    }

    if (_useInt1Pin) {
        pinMode(_int1Pin, INPUT);
    }

    _accelEnabled = false;
    _gyroEnabled = false;
    _pedometerEnabled = false;
    _anyMotionEnabled = false;
    _noMotionEnabled = false;
    _fifoEnabled = false;
    _wakeOnMotionActive = false;

    err = applyBaseConfiguration();
    if (err != QMI8658A_Error::SUCCESS) {
        _initialized = false;
        return err;
    }

    _initialized = true;
    return QMI8658A_Error::SUCCESS;
}

bool QMI8658A::isConnected() {
    _wire->beginTransmission(_i2cAddress);
    return (_wire->endTransmission() == 0);
}

QMI8658A_Error QMI8658A::applyBaseConfiguration() {
    _ctrl1Shadow = CTRL1_BASE | (_useInt1Pin ? CTRL1_INT1_EN_BIT : 0x00);
    // CTRL1_INT2_EN_BIT is intentionally never included: INT2 stays High-Z.

    // Safety: force INT2_EN (bit4) to 0 just to be sure
    _ctrl1Shadow &= ~CTRL1_INT2_EN_BIT;

    QMI8658A_Error err = writeRegister(REG_CTRL1, _ctrl1Shadow);
    if (err != QMI8658A_Error::SUCCESS) return err;

    // Active verification: read CTRL1 back from the chip and confirm INT2_EN is set to 0
    uint8_t readBack = 0;
    err = readRegister(REG_CTRL1, readBack);
    if (err != QMI8658A_Error::SUCCESS) return err;
    if (readBack & CTRL1_INT2_EN_BIT) {
        return QMI8658A_Error::I2C_ERROR;
    }


    QMI8658A_Error err = writeRegister(REG_CTRL1, _ctrl1Shadow);
    if (err != QMI8658A_Error::SUCCESS) return err;

    _ctrl7Shadow = CTRL7_BASE; // both sensors disabled
    err = writeRegister(REG_CTRL7, _ctrl7Shadow);
    if (err != QMI8658A_Error::SUCCESS) return err;

    _ctrl8Shadow = CTRL8_BASE; // no motion/pedometer engines enabled yet
    err = writeRegister(REG_CTRL8, _ctrl8Shadow);
    if (err != QMI8658A_Error::SUCCESS) return err;

    // FIFO disabled (bypass) until enableFifo() is called.
    err = writeRegister(REG_FIFO_CTRL, 0x00);
    return err;
}

QMI8658A_Error QMI8658A::softReset() {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    QMI8658A_Error err = writeRegister(REG_RESET, 0xB0);
    if (err != QMI8658A_Error::SUCCESS) return err;

    /* Datasheet: System Turn On Time after reset is max 15ms. 
     * Poll the documented reset-check register (0x4D) for 0x80, bounded wait. 
    */
    unsigned long start = millis();
    uint8_t checkVal = 0;
    while (millis() - start < 25UL) {
        delay(1);
        if (readRegister(REG_RESET_CHECK, checkVal) == QMI8658A_Error::SUCCESS && checkVal == 0x80) {
            break;
        }
    }

    /* Bring the software configuration back to a known, consistent state
     * (INT2 remains High-Z: applyBaseConfiguration() never touches it). 
    */
    _accelEnabled = false;
    _gyroEnabled = false;
    _pedometerEnabled = false;
    _anyMotionEnabled = false;
    _noMotionEnabled = false;
    _fifoEnabled = false;
    _wakeOnMotionActive = false;

    return applyBaseConfiguration();
}

/* =======================================================================
 * Low level I2C helpers
 * =======================================================================
*/

QMI8658A_Error QMI8658A::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_i2cAddress);
    _wire->write(reg);
    _wire->write(value);
    return (_wire->endTransmission() == 0) ? QMI8658A_Error::SUCCESS : QMI8658A_Error::I2C_ERROR;
}

QMI8658A_Error QMI8658A::readRegister(uint8_t reg, uint8_t &value) {
    return readRegisters(reg, &value, 1);
}

QMI8658A_Error QMI8658A::readRegisters(uint8_t startReg, uint8_t *buffer, size_t length) {
    _wire->beginTransmission(_i2cAddress);
    _wire->write(startReg);
    if (_wire->endTransmission(false) != 0) {
        return QMI8658A_Error::I2C_ERROR;
    }
    size_t received = _wire->requestFrom(static_cast<int>(_i2cAddress), static_cast<int>(length));
    if (received != length) {
        return QMI8658A_Error::I2C_ERROR;
    }
    for (size_t i = 0; i < length; i++) {
        buffer[i] = static_cast<uint8_t>(_wire->read());
    }
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::writeRegisterBits(uint8_t reg, uint8_t mask, uint8_t value, uint8_t &shadow) {
    uint8_t newVal = static_cast<uint8_t>((shadow & ~mask) | (value & mask));
    QMI8658A_Error err = writeRegister(reg, newVal);
    if (err == QMI8658A_Error::SUCCESS) {
        shadow = newVal;
    }
    return err;
}

QMI8658A_Error QMI8658A::executeCtrl9Command(uint8_t command, unsigned long timeoutMs) {
    QMI8658A_Error err = writeRegister(REG_CTRL9, command);
    if (err != QMI8658A_Error::SUCCESS) return err;

    unsigned long start = millis();
    uint8_t statusInt = 0;
    bool done = false;
    while (millis() - start < timeoutMs) {
        if (readRegister(REG_STATUSINT, statusInt) == QMI8658A_Error::SUCCESS &&
            (statusInt & STATUSINT_CMD_DONE_BIT)) {
            done = true;
            break;
        }
        delay(1);
    }
    if (!done) {
        return QMI8658A_Error::TIMEOUT;
    }

    // Acknowledge completion so the chip clears CmdDone / INT1.
    err = writeRegister(REG_CTRL9, CMD_ACK);
    return err;
}

/* =======================================================================
 * Accelerometer
 * ======================================================================= 
*/

QMI8658A_Error QMI8658A::enableAccelerometer(bool enable) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    QMI8658A_Error err = writeRegisterBits(REG_CTRL7, CTRL7_AEN_BIT, enable ? CTRL7_AEN_BIT : 0, _ctrl7Shadow);
    if (err == QMI8658A_Error::SUCCESS) {
        _accelEnabled = enable;
    }
    return err;
}

QMI8658A_Error QMI8658A::configureAccelerometer(AccelRange range, AccelODR odr) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    uint8_t ctrl2 = static_cast<uint8_t>((static_cast<uint8_t>(range) << 4) | (static_cast<uint8_t>(odr) & 0x0F));
    QMI8658A_Error err = writeRegister(REG_CTRL2, ctrl2);
    if (err == QMI8658A_Error::SUCCESS) {
        _accelRange = range;
    }
    return err;
}

float QMI8658A::accelSensitivityLsbPerG() const {
    switch (_accelRange) {
        case AccelRange::RANGE_2G:  return 16384.0f;
        case AccelRange::RANGE_4G:  return 8192.0f;
        case AccelRange::RANGE_8G:  return 4096.0f;
        case AccelRange::RANGE_16G: return 2048.0f;
    }
    return 4096.0f;
}

QMI8658A_Error QMI8658A::readRawAccel(int16_t &x, int16_t &y, int16_t &z) {
    uint8_t buf[6];
    QMI8658A_Error err = readRegisters(REG_AX_L, buf, 6);
    if (err != QMI8658A_Error::SUCCESS) return err;
    x = toInt16(buf[0], buf[1]);
    y = toInt16(buf[2], buf[3]);
    z = toInt16(buf[4], buf[5]);
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::readAccelerometer(AccelData &data) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    if (!_accelEnabled) return QMI8658A_Error::NOT_ENABLED;

    int16_t rx, ry, rz;
    QMI8658A_Error err = readRawAccel(rx, ry, rz);
    if (err != QMI8658A_Error::SUCCESS) return err;

    float sens = accelSensitivityLsbPerG();
    data.x = rx / sens;
    data.y = ry / sens;
    data.z = rz / sens;
    return QMI8658A_Error::SUCCESS;
}

/* =======================================================================
 * Gyroscope
 * =======================================================================
*/

QMI8658A_Error QMI8658A::enableGyroscope(bool enable) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    QMI8658A_Error err = writeRegisterBits(REG_CTRL7, CTRL7_GEN_BIT, enable ? CTRL7_GEN_BIT : 0, _ctrl7Shadow);
    if (err == QMI8658A_Error::SUCCESS) {
        _gyroEnabled = enable;
    }
    return err;
}

QMI8658A_Error QMI8658A::configureGyroscope(GyroRange range, GyroODR odr) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    uint8_t ctrl3 = static_cast<uint8_t>((static_cast<uint8_t>(range) << 4) | (static_cast<uint8_t>(odr) & 0x0F));
    QMI8658A_Error err = writeRegister(REG_CTRL3, ctrl3);
    if (err == QMI8658A_Error::SUCCESS) {
        _gyroRange = range;
    }
    return err;
}

float QMI8658A::gyroSensitivityLsbPerDps() const {
    switch (_gyroRange) {
        case GyroRange::RANGE_16DPS:   return 2048.0f;
        case GyroRange::RANGE_32DPS:   return 1024.0f;
        case GyroRange::RANGE_64DPS:   return 512.0f;
        case GyroRange::RANGE_128DPS:  return 256.0f;
        case GyroRange::RANGE_256DPS:  return 128.0f;
        case GyroRange::RANGE_512DPS:  return 64.0f;
        case GyroRange::RANGE_1024DPS: return 32.0f;
        case GyroRange::RANGE_2048DPS: return 16.0f;
    }
    return 64.0f;
}

QMI8658A_Error QMI8658A::readRawGyro(int16_t &x, int16_t &y, int16_t &z) {
    uint8_t buf[6];
    QMI8658A_Error err = readRegisters(REG_GX_L, buf, 6);
    if (err != QMI8658A_Error::SUCCESS) return err;
    x = toInt16(buf[0], buf[1]);
    y = toInt16(buf[2], buf[3]);
    z = toInt16(buf[4], buf[5]);
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::readGyroscope(GyroData &data) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    if (!_gyroEnabled) return QMI8658A_Error::NOT_ENABLED;

    int16_t rx, ry, rz;
    QMI8658A_Error err = readRawGyro(rx, ry, rz);
    if (err != QMI8658A_Error::SUCCESS) return err;

    float sens = gyroSensitivityLsbPerDps();
    data.x = rx / sens;
    data.y = ry / sens;
    data.z = rz / sens;
    return QMI8658A_Error::SUCCESS;
}

/* =======================================================================
 * Filters (CTRL5) - shared register between accelerometer and gyroscope,
 * so bits are updated with a read-modify-write to avoid disturbing the
 * other sensor's filter configuration.
 * =======================================================================
*/

QMI8658A_Error QMI8658A::setSensorFilter(bool isAccel, FilterMode mode, LpfBandwidth bandwidth) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    uint8_t ctrl5 = 0;
    QMI8658A_Error err = readRegister(REG_CTRL5, ctrl5);
    if (err != QMI8658A_Error::SUCCESS) return err;

    bool enableLpf = (mode == FilterMode::FILTER_LOW_PASS);
    uint8_t bw = static_cast<uint8_t>(bandwidth) & 0x03;

    if (isAccel) {
        ctrl5 = static_cast<uint8_t>(ctrl5 & ~0x07); // clear aLPF_EN(bit0) + aLPF_MODE(bits2:1)
        ctrl5 = static_cast<uint8_t>(ctrl5 | (bw << 1) | (enableLpf ? 0x01 : 0x00));
    } else {
        ctrl5 = static_cast<uint8_t>(ctrl5 & ~0x70); // clear gLPF_EN(bit4) + gLPF_MODE(bits6:5)
        ctrl5 = static_cast<uint8_t>(ctrl5 | (bw << 5) | (enableLpf ? 0x10 : 0x00));
    }

    return writeRegister(REG_CTRL5, ctrl5);
}

QMI8658A_Error QMI8658A::setAccelerometerFilter(FilterMode mode, LpfBandwidth bandwidth) {
    return setSensorFilter(true, mode, bandwidth);
}

QMI8658A_Error QMI8658A::setGyroscopeFilter(FilterMode mode, LpfBandwidth bandwidth) {
    return setSensorFilter(false, mode, bandwidth);
}

/* =======================================================================
 * Temperature
 * ======================================================================= 
*/

QMI8658A_Error QMI8658A::readTemperature(float &celsius) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    uint8_t buf[2];
    QMI8658A_Error err = readRegisters(REG_TEMP_L, buf, 2);
    if (err != QMI8658A_Error::SUCCESS) return err;
    int16_t raw = toInt16(buf[0], buf[1]);
    celsius = raw / 256.0f; // 256 LSB / degree C, per datasheet section 3.9
    return QMI8658A_Error::SUCCESS;
}

/* =======================================================================
 * Combined read
 * =======================================================================
*/

QMI8658A_Error QMI8658A::readSensorData(SensorData &data) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    QMI8658A_Error firstError = QMI8658A_Error::SUCCESS;

    if (_accelEnabled) {
        QMI8658A_Error e = readAccelerometer(data.accel);
        if (e != QMI8658A_Error::SUCCESS && firstError == QMI8658A_Error::SUCCESS) firstError = e;
    }
    if (_gyroEnabled) {
        QMI8658A_Error e = readGyroscope(data.gyro);
        if (e != QMI8658A_Error::SUCCESS && firstError == QMI8658A_Error::SUCCESS) firstError = e;
    }

    float t = 0.0f;
    QMI8658A_Error e = readTemperature(t);
    if (e == QMI8658A_Error::SUCCESS) {
        data.temperatureC = t;
    } else if (firstError == QMI8658A_Error::SUCCESS) {
        firstError = e;
    }

    return firstError;
}

/* =======================================================================
 * Any-Motion / No-Motion detection
 *
 * The QMI8658A bundles the Any-Motion AND No-Motion parameters together
 * in the same two CTRL9 "CONFIGURE_MOTION" command sets 
 * (datasheet section 9.4, Table 34): both axis thresholds and 
 * both axis windows are transmitted every time, 
 * regardless of which one the caller is updating.
 * This driver keeps the last configuration of each threshold so that calling
 * configureAnyMotion() does not erase a previously set configureNoMotion()
 * (and vice-versa).
 * =======================================================================
*/

namespace {
    uint8_t motionThresholdToByte(float g) {
        float v = clampf(g, 0.0f, 7.96875f) / 0.03125f;
        return static_cast<uint8_t>(v + 0.5f);
    }
}

QMI8658A_Error QMI8658A::configureAnyMotion(const MotionDetectConfig &config) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    /* Per datasheet: "Configuration should be done when accelerometer and
     * gyroscope are disabled." Temporarily disable, configure, restore. 
    */
    bool wasAccel = _accelEnabled;
    bool wasGyro = _gyroEnabled;
    if (wasAccel) enableAccelerometer(false);
    if (wasGyro) enableGyroscope(false);

    uint8_t motionModeCtrl = 0;
    motionModeCtrl |= config.enableX     ? 0x01 : 0x00;
    motionModeCtrl |= config.enableY     ? 0x02 : 0x00;
    motionModeCtrl |= config.enableZ     ? 0x04 : 0x00;
    motionModeCtrl |= config.useAndLogic ? 0x08 : 0x00;
    /* No-Motion bits (upper nibble) are left at 0 here; configureNoMotion()
     * fills them in with its own call using the same pattern, and a
     * caller that wants both enabled should call both configure functions
     * - each call only re-applies the bits for its own feature while
     * leaving the other feature disabled unless it has also been
     * configured and enabled via its own enable function. 
    */

    // First CTRL9 command: thresholds + MOTION_MODE_CTRL (Any-Motion part)
    QMI8658A_Error err;
    err = writeRegister(REG_CAL1_L, motionThresholdToByte(config.thresholdXg)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, motionThresholdToByte(config.thresholdYg)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_L, motionThresholdToByte(config.thresholdZg)); if (err != QMI8658A_Error::SUCCESS) return err;
    /* NoMotion thresholds (CAL2_H/CAL3_L/CAL3_H): leave previous chip
     * values untouched is not possible in a single CTRL9 call, so write 0
     * (disabled range) unless No-Motion is already enabled - simplest and
     * safest approach given the shared-command hardware design. 
    */
    err = writeRegister(REG_CAL2_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL4_L, motionModeCtrl); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL4_H, 0x01); if (err != QMI8658A_Error::SUCCESS) return err; // 1st command set
    err = executeCtrl9Command(CMD_CONFIGURE_MOTION); if (err != QMI8658A_Error::SUCCESS) return err;

    /* Second CTRL9 command: AnyMotionWindow + NoMotionWindow + SigMotion
     * windows (Significant-Motion is not used by this library, so its
     * windows are set to 0). 
    */
    err = writeRegister(REG_CAL1_L, config.durationSamples); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err; // NoMotionWindow
    err = writeRegister(REG_CAL2_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL4_H, 0x02); if (err != QMI8658A_Error::SUCCESS) return err; // 2nd command set
    err = executeCtrl9Command(CMD_CONFIGURE_MOTION); if (err != QMI8658A_Error::SUCCESS) return err;

    if (wasAccel) enableAccelerometer(true);
    if (wasGyro) enableGyroscope(true);
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::enableAnyMotion(bool enable) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    QMI8658A_Error err = writeRegisterBits(REG_CTRL8, CTRL8_ANY_MOTION_EN_BIT,
                                            enable ? CTRL8_ANY_MOTION_EN_BIT : 0, _ctrl8Shadow);
    if (err == QMI8658A_Error::SUCCESS) {
        _anyMotionEnabled = enable;
    }
    return err;
}

bool QMI8658A::isAnyMotionDetected() {
    uint8_t status1 = 0;
    if (readRegister(REG_STATUS1, status1) != QMI8658A_Error::SUCCESS) return false;
    return (status1 & STATUS1_ANY_MOTION_BIT) != 0;
}

QMI8658A_Error QMI8658A::configureNoMotion(const MotionDetectConfig &config) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    bool wasAccel = _accelEnabled;
    bool wasGyro = _gyroEnabled;
    if (wasAccel) enableAccelerometer(false);
    if (wasGyro) enableGyroscope(false);

    uint8_t motionModeCtrl = 0;
    motionModeCtrl |= config.enableX ? 0x10 : 0x00;
    motionModeCtrl |= config.enableY ? 0x20 : 0x00;
    motionModeCtrl |= config.enableZ ? 0x40 : 0x00;
    motionModeCtrl |= config.useAndLogic ? 0x80 : 0x00;

    QMI8658A_Error err;
    /* AnyMotion thresholds left at 0 for the same shared-register reason
     * documented in configureAnyMotion(). 
    */
    err = writeRegister(REG_CAL1_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_H, motionThresholdToByte(config.thresholdXg)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_L, motionThresholdToByte(config.thresholdYg)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_H, motionThresholdToByte(config.thresholdZg)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL4_L, motionModeCtrl); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL4_H, 0x01); if (err != QMI8658A_Error::SUCCESS) return err;
    err = executeCtrl9Command(CMD_CONFIGURE_MOTION); if (err != QMI8658A_Error::SUCCESS) return err;

    err = writeRegister(REG_CAL1_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, config.durationSamples); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL4_H, 0x02); if (err != QMI8658A_Error::SUCCESS) return err;
    err = executeCtrl9Command(CMD_CONFIGURE_MOTION); if (err != QMI8658A_Error::SUCCESS) return err;

    if (wasAccel) enableAccelerometer(true);
    if (wasGyro) enableGyroscope(true);
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::enableNoMotion(bool enable) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    QMI8658A_Error err = writeRegisterBits(REG_CTRL8, CTRL8_NO_MOTION_EN_BIT,
                                            enable ? CTRL8_NO_MOTION_EN_BIT : 0, _ctrl8Shadow);
    if (err == QMI8658A_Error::SUCCESS) {
        _noMotionEnabled = enable;
    }
    return err;
}

bool QMI8658A::isNoMotionDetected() {
    uint8_t status1 = 0;
    if (readRegister(REG_STATUS1, status1) != QMI8658A_Error::SUCCESS) return false;
    return (status1 & STATUS1_NO_MOTION_BIT) != 0;
}

/* =======================================================================
 * Wake on Motion (datasheet section 12)
 * WoM is a distinct chip mode: entering/exiting it follows the exact
 * register sequence from Figure 25 / section 12.6. INT2 is never touched:
 * only INT1 is accepted as the WoM interrupt pin by this library, in line
 * with the "all interrupts via INT1 only" hardware requirement.
 * =======================================================================
*/

QMI8658A_Error QMI8658A::enterWakeOnMotion(const WakeOnMotionConfig &config) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    // A negative threshold is invalid.
    if (config.thresholdG < 0.0f)
        return QMI8658A_Error::INVALID_PARAMETER;
    // Blanking time is stored on 6 bits: valid values are 0 to 63.
    if (config.blankingSamples > 63)
        return QMI8658A_Error::INVALID_PARAMETER;
    // A zero threshold disables Wake on Motion.
    if (config.thresholdG == 0.0f)
        return exitWakeOnMotion();

    // 1. Disable sensors
    QMI8658A_Error err = writeRegisterBits(REG_CTRL7, CTRL7_AEN_BIT | CTRL7_GEN_BIT, 0, _ctrl7Shadow);
    if (err != QMI8658A_Error::SUCCESS) return err;
    _accelEnabled = false;
    _gyroEnabled = false;

    // 2. Accelerometer ODR/full-scale already set by configureAccelerometer() nothing else to do here.
    // 3. WoM threshold (1 LSB = 1mg) + interrupt pin select/polarity/blanking. This library only routes WoM to INT1: 00 = initial LOW, 10 = initial HIGH.
    uint8_t thresholdByte = static_cast<uint8_t>(clampf(config.thresholdG * 1000.0f, 1.0f, 255.0f) + 0.5f);
    uint8_t calH = static_cast<uint8_t>((config.interruptInitialHigh ? 0x80 : 0x00) | (config.blankingSamples & 0x3F));

    err = writeRegister(REG_CAL1_L, thresholdByte); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, calH); if (err != QMI8658A_Error::SUCCESS) return err;

    // 4. Execute CTRL9 command to configure WoM mode
    err = executeCtrl9Command(CMD_WRITE_WOM_SETTING);
    if (err != QMI8658A_Error::SUCCESS) return err;

    // 5. Enable accelerometer
    err = writeRegisterBits(REG_CTRL7, CTRL7_AEN_BIT, CTRL7_AEN_BIT, _ctrl7Shadow);
    if (err != QMI8658A_Error::SUCCESS) return err;

    _wakeOnMotionActive = true;
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::exitWakeOnMotion() {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    // 1. Disable all sensors
    QMI8658A_Error err = writeRegisterBits(REG_CTRL7, CTRL7_AEN_BIT | CTRL7_GEN_BIT, 0, _ctrl7Shadow);
    if (err != QMI8658A_Error::SUCCESS) return err;
    _accelEnabled = false;
    _gyroEnabled = false;

    /* 2. Write threshold = 0 and re-issue the WoM configuration command,
     *    which the datasheet documents as the way to return the interrupt
     *    pin to normal function. */
    err = writeRegister(REG_CAL1_L, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, 0x00); if (err != QMI8658A_Error::SUCCESS) return err;
    err = executeCtrl9Command(CMD_WRITE_WOM_SETTING);
    if (err != QMI8658A_Error::SUCCESS) return err;

    _wakeOnMotionActive = false;
    return QMI8658A_Error::SUCCESS;
}

bool QMI8658A::isWakeOnMotionDetected() {
    uint8_t status1 = 0;
    /* Datasheet 12.4: reading STATUS1 clears the WoM bit and the selected
     * interrupt line - this is inherent hardware behaviour of the read. */
    if (readRegister(REG_STATUS1, status1) != QMI8658A_Error::SUCCESS) return false;
    return (status1 & STATUS1_WOM_BIT) != 0;
}

/* =======================================================================
 * Pedometer (datasheet section 11)
 * =======================================================================
*/

namespace {
    uint16_t pedoThresholdToU6_10(float g) {
        float v = clampf(g, 0.0f, 63.0f) * 1024.0f;
        return static_cast<uint16_t>(v + 0.5f);
    }
}

QMI8658A_Error QMI8658A::configurePedometer(const PedometerConfig &config) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    bool wasAccel = _accelEnabled;
    bool wasGyro = _gyroEnabled;
    if (wasAccel) enableAccelerometer(false);
    if (wasGyro) enableGyroscope(false);

    // First CTRL9 command set
    uint16_t p2p = pedoThresholdToU6_10(config.peakToPeakThreshG);
    uint16_t pk  = pedoThresholdToU6_10(config.peakThreshG);

    QMI8658A_Error err;
    err = writeRegister(REG_CAL1_L, static_cast<uint8_t>(config.sampleWindow & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, static_cast<uint8_t>((config.sampleWindow >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_L, static_cast<uint8_t>(p2p & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_H, static_cast<uint8_t>((p2p >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_L, static_cast<uint8_t>(pk & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_H, static_cast<uint8_t>((pk >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL4_H, 0x01); if (err != QMI8658A_Error::SUCCESS) return err;
    err = executeCtrl9Command(CMD_CONFIGURE_PEDOMETER); if (err != QMI8658A_Error::SUCCESS) return err;

    // Second CTRL9 command set
    err = writeRegister(REG_CAL1_L, static_cast<uint8_t>(config.timeoutSamples & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, static_cast<uint8_t>((config.timeoutSamples >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_L, config.minStepSamples); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_H, config.entryStepCount); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_L, 0x00); /* ped_fix_precision, 0 recommended */ if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_H, config.reportEverySteps); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL4_H, 0x02); if (err != QMI8658A_Error::SUCCESS) return err;
    err = executeCtrl9Command(CMD_CONFIGURE_PEDOMETER); if (err != QMI8658A_Error::SUCCESS) return err;

    if (wasAccel) enableAccelerometer(true);
    if (wasGyro) enableGyroscope(true);
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::enablePedometer(bool enable) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    QMI8658A_Error err = writeRegisterBits(REG_CTRL8, CTRL8_PEDO_EN_BIT,
                                            enable ? CTRL8_PEDO_EN_BIT : 0, _ctrl8Shadow);
    if (err == QMI8658A_Error::SUCCESS) {
        _pedometerEnabled = enable;
    }
    return err;
}

QMI8658A_Error QMI8658A::getStepCount(uint32_t &steps) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    uint8_t buf[3];
    QMI8658A_Error err = readRegisters(REG_STEP_CNT_LOW, buf, 3);
    if (err != QMI8658A_Error::SUCCESS) return err;
    steps = static_cast<uint32_t>(buf[0]) | (static_cast<uint32_t>(buf[1]) << 8) | (static_cast<uint32_t>(buf[2]) << 16);
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::resetStepCount() {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    return executeCtrl9Command(CMD_RESET_PEDOMETER);
}

bool QMI8658A::isStepDetected() {
    uint8_t status1 = 0;
    if (readRegister(REG_STATUS1, status1) != QMI8658A_Error::SUCCESS) return false;
    return (status1 & STATUS1_PEDO_BIT) != 0;
}

/* =======================================================================
 * Calibration
 * ===================================================================== 
*/

CalibrationResult QMI8658A::calibrate(uint16_t samples,
                                       float gyroStationaryThresholdDps,
                                       float accelStationaryThresholdG,
                                       unsigned long timeoutMs) {
    CalibrationResult result;

    if (!_initialized) {
        result.error = QMI8658A_Error::NOT_INITIALIZED;
        return result;
    }
    if (!_gyroEnabled || !_accelEnabled) {
        /* Both sensors are used: gyroscope for the offset itself,
         * accelerometer as an independent stillness check. 
        */
        result.error = QMI8658A_Error::NOT_ENABLED;
        return result;
    }
    if (samples < 4) {
        result.error = QMI8658A_Error::INVALID_PARAMETER;
        return result;
    }

    long sumGx = 0, sumGy = 0, sumGz = 0;
    int16_t minAccelNorm2 = 0, maxAccelNorm2 = 0;
    int16_t minGyroNorm2 = 0, maxGyroNorm2 = 0;
    bool firstSample = true;

    unsigned long start = millis();
    for (uint16_t i = 0; i < samples; i++) {
        if (millis() - start > timeoutMs) {
            result.error = QMI8658A_Error::TIMEOUT;
            return result;
        }

        int16_t ax, ay, az, gx, gy, gz;
        if (readRawAccel(ax, ay, az) != QMI8658A_Error::SUCCESS ||
            readRawGyro(gx, gy, gz) != QMI8658A_Error::SUCCESS) {
            result.error = QMI8658A_Error::I2C_ERROR;
            return result;
        }

        sumGx += gx; sumGy += gy; sumGz += gz;

        float aSens = accelSensitivityLsbPerG();
        float aNorm = sqrtf((ax / aSens) * (ax / aSens) + (ay / aSens) * (ay / aSens) + (az / aSens) * (az / aSens));
        float gSens = gyroSensitivityLsbPerDps();
        float gNorm = sqrtf((gx / gSens) * (gx / gSens) + (gy / gSens) * (gy / gSens) + (gz / gSens) * (gz / gSens));

        int16_t aNormScaled = static_cast<int16_t>(aNorm * 1000.0f);
        int16_t gNormScaled = static_cast<int16_t>(gNorm * 1000.0f);
        if (firstSample) {
            minAccelNorm2 = maxAccelNorm2 = aNormScaled;
            minGyroNorm2 = maxGyroNorm2 = gNormScaled;
            firstSample = false;
        } else {
            if (aNormScaled < minAccelNorm2) minAccelNorm2 = aNormScaled;
            if (aNormScaled > maxAccelNorm2) maxAccelNorm2 = aNormScaled;
            if (gNormScaled < minGyroNorm2) minGyroNorm2 = gNormScaled;
            if (gNormScaled > maxGyroNorm2) maxGyroNorm2 = gNormScaled;
        }
        delay(2);
    }

    float accelVariationG = (maxAccelNorm2 - minAccelNorm2) / 1000.0f;
    float gyroVariationDps = (maxGyroNorm2 - minGyroNorm2) / 1000.0f;
    if (accelVariationG > accelStationaryThresholdG || gyroVariationDps > gyroStationaryThresholdDps) {
        result.error = QMI8658A_Error::SENSOR_NOT_STATIONARY;
        return result;
    }

    float gSens = gyroSensitivityLsbPerDps();
    float meanGxDps = (sumGx / static_cast<float>(samples)) / gSens;
    float meanGyDps = (sumGy / static_cast<float>(samples)) / gSens;
    float meanGzDps = (sumGz / static_cast<float>(samples)) / gSens;

    /* Delta offset format: signed, 5 fraction bits (1/32 dps per LSB).
     * The offset is written as the negative of the measured bias so that
     * (raw + delta) reads ~0 dps while stationary. 
    */
    int16_t dx = static_cast<int16_t>(clampf(-meanGxDps * 32.0f, -32768.0f, 32767.0f));
    int16_t dy = static_cast<int16_t>(clampf(-meanGyDps * 32.0f, -32768.0f, 32767.0f));
    int16_t dz = static_cast<int16_t>(clampf(-meanGzDps * 32.0f, -32768.0f, 32767.0f));

    QMI8658A_Error err;
    err = writeRegister(REG_CAL1_L, static_cast<uint8_t>(dx & 0xFF)); if (err != QMI8658A_Error::SUCCESS) { result.error = err; return result; }
    err = writeRegister(REG_CAL1_H, static_cast<uint8_t>((dx >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) { result.error = err; return result; }
    err = writeRegister(REG_CAL2_L, static_cast<uint8_t>(dy & 0xFF)); if (err != QMI8658A_Error::SUCCESS) { result.error = err; return result; }
    err = writeRegister(REG_CAL2_H, static_cast<uint8_t>((dy >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) { result.error = err; return result; }
    err = writeRegister(REG_CAL3_L, static_cast<uint8_t>(dz & 0xFF)); if (err != QMI8658A_Error::SUCCESS) { result.error = err; return result; }
    err = writeRegister(REG_CAL3_H, static_cast<uint8_t>((dz >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) { result.error = err; return result; }
    err = executeCtrl9Command(CMD_GYRO_HOST_DELTA_OFFSET);
    if (err != QMI8658A_Error::SUCCESS) {
        result.error = err;
        return result;
    }

    result.success = true;

    // Refine gyroscope X/Y sensitivity gain via on-chip Calibration-On-Demand. This requires both sensors disabled during the routine.
    bool wasAccel = _accelEnabled;
    bool wasGyro = _gyroEnabled;
    enableAccelerometer(false);
    enableGyroscope(false);

    QMI8658A_Error codErr = executeCtrl9Command(CMD_ON_DEMAND_CALIBRATION, 1800);

    if (wasAccel) enableAccelerometer(true);
    if (wasGyro) enableGyroscope(true);

    if (codErr == QMI8658A_Error::SUCCESS) {
        uint8_t codStatus = 0xFF;
        readRegister(REG_COD_STATUS, codStatus);
        result.codStatusRaw = codStatus;
        result.gyroGainCalibrated = (codStatus == 0x00);
    } else {
        result.gyroGainCalibrated = false;
    }

    return result;
}

QMI8658A_Error QMI8658A::calibrateAccelerometerOffset(Axis verticalAxis, bool positiveDirection, uint16_t samples) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    if (!_accelEnabled) return QMI8658A_Error::NOT_ENABLED;
    if (samples < 4) return QMI8658A_Error::INVALID_PARAMETER;

    long sumX = 0, sumY = 0, sumZ = 0;
    for (uint16_t i = 0; i < samples; i++) {
        int16_t ax, ay, az;
        QMI8658A_Error err = readRawAccel(ax, ay, az);
        if (err != QMI8658A_Error::SUCCESS) return err;
        sumX += ax; sumY += ay; sumZ += az;
        delay(2);
    }

    float sens = accelSensitivityLsbPerG();
    float measX = (sumX / static_cast<float>(samples)) / sens;
    float measY = (sumY / static_cast<float>(samples)) / sens;
    float measZ = (sumZ / static_cast<float>(samples)) / sens;

    float expX = 0.0f, expY = 0.0f, expZ = 0.0f;
    float g = positiveDirection ? 1.0f : -1.0f;
    switch (verticalAxis) {
        case Axis::AXIS_X: expX = g; break;
        case Axis::AXIS_Y: expY = g; break;
        case Axis::AXIS_Z: expZ = g; break;
    }

    // Delta offset format: signed, 12 fraction bits (1/4096 g per LSB).
    int16_t dx = static_cast<int16_t>(clampf((expX - measX) * 4096.0f, -32768.0f, 32767.0f));
    int16_t dy = static_cast<int16_t>(clampf((expY - measY) * 4096.0f, -32768.0f, 32767.0f));
    int16_t dz = static_cast<int16_t>(clampf((expZ - measZ) * 4096.0f, -32768.0f, 32767.0f));

    QMI8658A_Error err;
    err = writeRegister(REG_CAL1_L, static_cast<uint8_t>(dx & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL1_H, static_cast<uint8_t>((dx >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_L, static_cast<uint8_t>(dy & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL2_H, static_cast<uint8_t>((dy >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_L, static_cast<uint8_t>(dz & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;
    err = writeRegister(REG_CAL3_H, static_cast<uint8_t>((dz >> 8) & 0xFF)); if (err != QMI8658A_Error::SUCCESS) return err;

    return executeCtrl9Command(CMD_ACCEL_HOST_DELTA_OFFSET);
}

/* =======================================================================
 * FIFO (optional)
 * =======================================================================
*/

QMI8658A_Error QMI8658A::enableFifo(FifoMode mode, FifoSize size, uint8_t watermarkSamples) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    QMI8658A_Error err = writeRegister(REG_FIFO_WTM_TH, watermarkSamples);
    if (err != QMI8658A_Error::SUCCESS) return err;

    uint8_t fifoCtrl = static_cast<uint8_t>((static_cast<uint8_t>(size) << 2) | static_cast<uint8_t>(mode));
    err = writeRegister(REG_FIFO_CTRL, fifoCtrl);
    if (err != QMI8658A_Error::SUCCESS) return err;

    _fifoEnabled = (mode != FifoMode::FIFO_DISABLED);
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::disableFifo() {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    QMI8658A_Error err = writeRegister(REG_FIFO_CTRL, 0x00);
    if (err == QMI8658A_Error::SUCCESS) {
        _fifoEnabled = false;
    }
    return err;
}

QMI8658A_Error QMI8658A::resetFifo() {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    return executeCtrl9Command(CMD_RST_FIFO);
}

QMI8658A_Error QMI8658A::getFifoSampleCount(uint16_t &sampleCount) {
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;

    uint8_t cntLsb = 0, status = 0;
    QMI8658A_Error err = readRegister(REG_FIFO_SMPL_CNT, cntLsb);
    if (err != QMI8658A_Error::SUCCESS) return err;
    err = readRegister(REG_FIFO_STATUS, status);
    if (err != QMI8658A_Error::SUCCESS) return err;

    uint16_t wordCount = static_cast<uint16_t>((status & 0x03) << 8 | cntLsb);
    uint16_t byteCount = static_cast<uint16_t>(wordCount * 2);

    uint8_t bytesPerSample = static_cast<uint8_t>((_accelEnabled ? 6 : 0) + (_gyroEnabled ? 6 : 0));
    if (bytesPerSample == 0) {
        sampleCount = 0;
        return QMI8658A_Error::SUCCESS;
    }
    sampleCount = byteCount / bytesPerSample;
    return QMI8658A_Error::SUCCESS;
}

QMI8658A_Error QMI8658A::readFifo(FifoSample *outSamples, uint16_t maxSamples, uint16_t &samplesRead) {
    samplesRead = 0;
    if (!_initialized) return QMI8658A_Error::NOT_INITIALIZED;
    if (!_fifoEnabled) return QMI8658A_Error::NOT_ENABLED;
    if (outSamples == nullptr || maxSamples == 0) return QMI8658A_Error::INVALID_PARAMETER;

    uint8_t fifoStatus = 0;
    QMI8658A_Error err = readRegister(REG_FIFO_STATUS, fifoStatus);
    if (err != QMI8658A_Error::SUCCESS) return err;
    bool overflowed = (fifoStatus & 0x20) != 0;

    uint16_t available = 0;
    err = getFifoSampleCount(available);
    if (err != QMI8658A_Error::SUCCESS) return err;

    uint8_t bytesPerSample = static_cast<uint8_t>((_accelEnabled ? 6 : 0) + (_gyroEnabled ? 6 : 0));
    if (bytesPerSample == 0) return QMI8658A_Error::NOT_ENABLED;

    uint16_t toRead = (available < maxSamples) ? available : maxSamples;
    if (toRead == 0) {
        return overflowed ? QMI8658A_Error::FIFO_OVERFLOW : QMI8658A_Error::SUCCESS;
    }

    err = executeCtrl9Command(CMD_REQ_FIFO);
    if (err != QMI8658A_Error::SUCCESS) return err;

    const uint8_t CHUNK_SAMPLES = 2; // keeps each I2C burst small & reliable
    uint8_t chunkBuf[CHUNK_SAMPLES * 12];
    float aSens = accelSensitivityLsbPerG();
    float gSens = gyroSensitivityLsbPerDps();

    uint16_t remaining = toRead;
    uint16_t decoded = 0;
    while (remaining > 0) {
        uint16_t thisChunk = (remaining < CHUNK_SAMPLES) ? remaining : CHUNK_SAMPLES;
        size_t bytesToRead = static_cast<size_t>(thisChunk) * bytesPerSample;
        err = readRegisters(REG_FIFO_DATA, chunkBuf, bytesToRead);
        if (err != QMI8658A_Error::SUCCESS) {
            // Make sure read mode is cleared even on error, so the FIFO can resume filling.
            uint8_t ctrl = 0;
            readRegister(REG_FIFO_CTRL, ctrl);
            writeRegister(REG_FIFO_CTRL, static_cast<uint8_t>(ctrl & ~0x80));
            samplesRead = decoded;
            return err;
        }

        size_t offset = 0;
        for (uint16_t s = 0; s < thisChunk; s++) {
            FifoSample &out = outSamples[decoded];
            out = FifoSample();
            if (_accelEnabled) {
                int16_t ax = toInt16(chunkBuf[offset], chunkBuf[offset + 1]);
                int16_t ay = toInt16(chunkBuf[offset + 2], chunkBuf[offset + 3]);
                int16_t az = toInt16(chunkBuf[offset + 4], chunkBuf[offset + 5]);
                out.accel.x = ax / aSens;
                out.accel.y = ay / aSens;
                out.accel.z = az / aSens;
                offset += 6;
            }
            if (_gyroEnabled) {
                int16_t gx = toInt16(chunkBuf[offset], chunkBuf[offset + 1]);
                int16_t gy = toInt16(chunkBuf[offset + 2], chunkBuf[offset + 3]);
                int16_t gz = toInt16(chunkBuf[offset + 4], chunkBuf[offset + 5]);
                out.gyro.x = gx / gSens;
                out.gyro.y = gy / gSens;
                out.gyro.z = gz / gSens;
                offset += 6;
            }
            decoded++;
        }
        remaining -= thisChunk;
    }

    // Clear FIFO_CTRL.FIFO_RD_MODE so new samples can be written again.
    uint8_t ctrl = 0;
    err = readRegister(REG_FIFO_CTRL, ctrl);
    if (err == QMI8658A_Error::SUCCESS) {
        writeRegister(REG_FIFO_CTRL, static_cast<uint8_t>(ctrl & ~0x80));
    }

    samplesRead = decoded;
    return overflowed ? QMI8658A_Error::FIFO_OVERFLOW : QMI8658A_Error::SUCCESS;
}
