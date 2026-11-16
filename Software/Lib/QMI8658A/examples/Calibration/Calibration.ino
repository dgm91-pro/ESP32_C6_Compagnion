/*
 * Calibration.ino
 *
 * Demonstrates gyroscope offset + gain calibration 'calibrate()', and
 * the optional accelerometer offset calibration that requires the user
 * to confirm the device's resting orientation.
 * Place the board perfectly still and flat before running this sketch.
 *
 * Created 16 oct. 2026 by dgm91.dev
 *
 * License: MIT
*/

#include <QMI8658A.h>

QMI8658A imu;

void setup() {

    Serial.begin(115200);
    while (!Serial) { 
        delay(10); 
    }

    QMI8658A_Config config;
    if (imu.begin(config) != QMI8658A_Error::SUCCESS) {
        Serial.println("[-] QMI8658A begin() failed, Error: ");
        while (true) { delay(1000); }
    }

    imu.configureAccelerometer(AccelRange::RANGE_4G, AccelODR::ODR_250HZ);
    imu.configureGyroscope(GyroRange::RANGE_256DPS, GyroODR::ODR_224_2HZ);
    imu.enableAccelerometer(true);
    imu.enableGyroscope(true);

    Serial.println("[!] Keep the board perfectly still. Calibrating gyroscope...");
    delay(500);

    CalibrationResult result = imu.calibrate();
    if (!result.success) {
        Serial.print("[-] Gyroscope calibration failed, Error: ");
        Serial.println(static_cast<int>(result.error));
    } else {
        Serial.println("[+] Gyroscope offset calibration succeeded.");
        Serial.print("[+] Gyroscope gain (COD) calibrated ? : ");
        Serial.println(result.gyroGainCalibrated ? "true" : "false");
        Serial.print("Raw COD_STATUS: 0x");
        Serial.println(result.codStatusRaw, HEX);
    }

    /*
     * Accelerometer offset calibration requires you to state which axis
     * is vertical and pointing up while the board is lying still - this
     * is an external reference the library cannot guess on its own.
     * Uncomment the line below if the board is lying flat with its Z
     * axis pointing up (typical flat-on-a-table orientation):
     *
     *   imu.calibrateAccelerometerOffset(Axis::AXIS_Z, true);
    */
}

void loop() {
    // Nothing to do; calibration runs once in setup().
    delay(1000);
}
