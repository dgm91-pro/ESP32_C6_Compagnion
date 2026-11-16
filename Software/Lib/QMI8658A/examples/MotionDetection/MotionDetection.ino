/*
 * MotionDetection.ino
 *
 * Demonstrates Any-Motion, No-Motion and Wake on Motion detection with the QMI8658A. 
 * All interrupts are read by polling status registers (INT1 hardware interrupt usage is optional - see QMI8658A_Config).
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
    imu.enableAccelerometer(true);

    MotionDetectConfig anyMotion;
    anyMotion.thresholdXg = 0.25f;
    anyMotion.thresholdYg = 0.25f;
    anyMotion.thresholdZg = 0.25f;
    anyMotion.durationSamples = 3;
    imu.configureAnyMotion(anyMotion);
    imu.enableAnyMotion(true);

    MotionDetectConfig noMotion;
    noMotion.thresholdXg = 0.05f;
    noMotion.thresholdYg = 0.05f;
    noMotion.thresholdZg = 0.05f;
    noMotion.durationSamples = 20;
    imu.configureNoMotion(noMotion);
    imu.enableNoMotion(true);

    Serial.println("[+] Any-Motion / No-Motion detection active.");
}

void loop() {
    if (imu.isAnyMotionDetected()) {
        Serial.println("[+] Any-Motion detected a movement !");
    }
    if (imu.isNoMotionDetected()) {
        Serial.println("[+] No-Motion detected device is still.");
    }
    delay(100);
}

/*
 * To use Wake on Motion instead (very low power), comment out the
 * Any-Motion/No-Motion setup above and use:
 *
 *   WakeOnMotionConfig wom;
 *   wom.thresholdG = 0.255f; // Max Value. If you set it to 0 it will disable WoM
 *   imu.enterWakeOnMotion(wom);
 *   ...
 *   if (imu.isWakeOnMotionDetected()) { ... }
 *   ...
 *   imu.exitWakeOnMotion(); // Required before using normal accel/gyro readings again
*/
