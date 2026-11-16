/*
 * BasicReadings.ino
 *
 * Reads accelerometer, gyroscope and temperature from the QMI8658A of the Waveshare ESP32-C6 Touch LCD 1.47 board using the QMI8658A library.
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

    QMI8658A_Config config; // defaults match the ESP32-C6 Touch LCD 1.47 board
    QMI8658A_Error err = imu.begin(config);
    if (err != QMI8658A_Error::SUCCESS) {
        Serial.print("[-] QMI8658A begin() failed, Error: ");
        Serial.println(static_cast<int>(err));
        while (true) { delay(1000); }
    }

    Serial.println("[+] QMI8658A initialized");

    imu.configureAccelerometer(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);
    imu.configureGyroscope(GyroRange::RANGE_512DPS, GyroODR::ODR_448_4HZ);
    imu.setAccelerometerFilter(FilterMode::FILTER_LOW_PASS, LpfBandwidth::BW_2_66_PCT_ODR);
    imu.setGyroscopeFilter(FilterMode::FILTER_LOW_PASS, LpfBandwidth::BW_2_66_PCT_ODR);

    imu.enableAccelerometer(true);
    imu.enableGyroscope(true);
}

void loop() {
    SensorData data;
    QMI8658A_Error err = imu.readSensorData(data);
    if (err == QMI8658A_Error::SUCCESS) {

        Serial.println();

        Serial.print("Accel (g): X=");
        Serial.print(data.accel.x, 3);
        Serial.print(" Y=");
        Serial.print(data.accel.y, 3);
        Serial.print(" Z=");
        Serial.print(data.accel.z, 3);

        Serial.println();

        Serial.print("  Gyro (dps): X=");
        Serial.print(data.gyro.x, 2);
        Serial.print(" Y=");
        Serial.print(data.gyro.y, 2);
        Serial.print(" Z=");
        Serial.print(data.gyro.z, 2);

        Serial.println();

        Serial.print("  Temp (C): ");
        Serial.print(data.temperatureC, 2);

    } else {
        Serial.println();
        Serial.print("[-] Failed To Read Sensors Data, Error: ");
        Serial.println(static_cast<int>(err));
    }
    delay(200);
}
