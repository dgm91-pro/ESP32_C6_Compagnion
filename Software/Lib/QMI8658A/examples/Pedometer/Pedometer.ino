/*
 * Pedometer.ino
 *
 * Enables the QMI8658A's internal pedometer and prints the step count.
 * Default PedometerConfig values are tuned for an accelerometer ODR
 * around 50Hz (see QMI8658A.h )
 * here we use the closest available low-power ODR (31.25Hz) for a reasonable battery usage demo; 
 * re-tune PedometerConfig fields if you change the ODR.
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

    imu.configureAccelerometer(AccelRange::RANGE_4G, AccelODR::ODR_31_25HZ);
    imu.enableAccelerometer(true);

    PedometerConfig pedoConfig; // datasheet example defaults
    imu.configurePedometer(pedoConfig);
    imu.enablePedometer(true);

    Serial.println("[+] Pedometer running. Walk with the board to see step counts."); // Be carefull if you power the board with the USB-C cable , to not rip everything off
}

void loop() {
    uint32_t steps = 0;
    if (imu.getStepCount(steps) == QMI8658A_Error::SUCCESS) {
        Serial.print("[+] Step count: ");
        Serial.println(steps);
    }
    delay(1000);
}
