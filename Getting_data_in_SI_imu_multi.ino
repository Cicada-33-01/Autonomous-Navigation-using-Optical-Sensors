#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
int printer()
{Serial.print("LOL");
return 7;}
SparkFun_ISM330DHCX myISM1;
SparkFun_ISM330DHCX myISM2;


//Serial.print("1");
// Structs for X, Y, Z data
sfe_ism_data_t accelData1, gyroData1;
sfe_ism_data_t accelData2, gyroData2;

double factor = 9.4 * 0.001;

double factor_x1 = 9.80665 * 0.001;
double factor_y1 = 9.80665 * 0.001;
double factor_z1 = 9.80665 * 0.001;
double factor_x2 = 9.80665 * 0.001;
double factor_y2 = 9.80665 * 0.001;
double factor_z2 = 9.80665 * 0.001;
double Gyrofactor = 0.000001;

double correction_acc_x1 = -0.0137 + 0.074 - 0.08 ;
double correction_acc_y1 = 0.1174 + 0.054 - 0.2050;
double correction_acc_z1 = -0.1766 - 0.2;

double correction_acc_x2 = -0.0137 + 0.074 - 0.04 - 0.03975; // Replace with appropriate correction values for the second IMU
double correction_acc_y2 = 0.1174 + 0.054 - 0.1080; // Replace with appropriate correction values for the second IMU
double correction_acc_z2 = -0.1766; // Replace with appropriate correction values for the second IMU

void setup() {
    Wire.begin();
    Serial.begin(115200);
//    Serial.print("val");
//    bool a = myISM2.begin();
//    Serial.print("a:"+a);
//    bool b = myISM1.begin();
//    Serial.print("b:"+b);
    if (!myISM1.begin(ISM330DHCX_ADDRESS_HIGH)) {
        Serial.println("Did not begin IMU 1.");
//        while (1);
    }
//    Serial.print("3");
    if (!myISM2.begin(ISM330DHCX_ADDRESS_LOW)) {
        Serial.println("Did not begin IMU 2.");
//        while (1);
    }
//    Serial.print("3");
    // Reset and configure both IMUs
    myISM1.deviceReset();
    myISM2.deviceReset();

    while (!myISM1.getDeviceReset() || !myISM2.getDeviceReset()) {
        delay(1);
    }

    myISM1.setDeviceConfig();
    myISM2.setDeviceConfig();

    myISM1.setBlockDataUpdate();
    myISM2.setBlockDataUpdate();
//    Serial.print("end");


    myISM1.setDeviceConfig();
    myISM1.setBlockDataUpdate();
    // Set the output data rate and precision of the accelerometer
    myISM1.setAccelDataRate(ISM_XL_ODR_52Hz);
    myISM1.setAccelFullScale(ISM_2g);
    // Set the output data rate and precision of the gyroscope
    myISM1.setGyroDataRate(ISM_GY_ODR_52Hz);
    myISM1.setGyroFullScale(ISM_125dps);
    // Turn on the accelerometer's filter and apply settings.
    myISM1.setAccelFilterLP2();
    myISM1.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);
    // Turn on the gyroscope's filter and apply settings.
    myISM1.setGyroFilterLP1();
    myISM1.setGyroLP1Bandwidth(ISM_MEDIUM);

    myISM2.setDeviceConfig();
    myISM2.setBlockDataUpdate();
    // Set the output data rate and precision of the accelerometer
    myISM2.setAccelDataRate(ISM_XL_ODR_52Hz);
    myISM2.setAccelFullScale(ISM_2g);
    // Set the output data rate and precision of the gyroscope
    myISM2.setGyroDataRate(ISM_GY_ODR_52Hz);
    myISM2.setGyroFullScale(ISM_125dps);
    // Turn on the accelerometer's filter and apply settings.
    myISM2.setAccelFilterLP2();
    myISM2.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);
    // Turn on the gyroscope's filter and apply settings.
    myISM2.setGyroFilterLP1();
    myISM2.setGyroLP1Bandwidth(ISM_MEDIUM);
    // Configure the accelerometer and gyroscope for both IMUs
    // ... (same configuration as before)
}

void loop() {
    // Check if data is available for both IMUs
    if (myISM1.checkStatus() && myISM2.checkStatus()) {
        myISM1.getAccel(&accelData1);
        myISM1.getGyro(&gyroData1);

        myISM2.getAccel(&accelData2);
        myISM2.getGyro(&gyroData2);

        // Send data from IMU 1
//        Serial.println("IMU1");
        Serial.print(accelData1.xData * factor + correction_acc_x1, 4);
        Serial.print(",");
        Serial.print(accelData1.yData * factor + correction_acc_y1, 4);
        Serial.print(",");
        Serial.print(accelData1.zData * factor *0.9965 + correction_acc_z1, 4);
        Serial.print(",");
        Serial.print(gyroData1.xData * Gyrofactor, 4);
        Serial.print(",");
        Serial.print(gyroData1.yData * Gyrofactor, 4);
        Serial.print(",");
        Serial.print(gyroData1.zData * Gyrofactor, 4);
//        Serial.println("-----------------");
//
//        // Send data from IMU 2
//        Serial.println("IMU 2");
        Serial.print(",");
        Serial.print(accelData2.xData * factor + correction_acc_x2, 4);
        Serial.print(",");
        Serial.print(accelData2.yData * factor + correction_acc_y2, 4);
        Serial.print(",");  
        Serial.print(accelData2.zData * factor + correction_acc_z2, 4);
        Serial.print(",");
        Serial.print(gyroData2.xData * Gyrofactor, 4);
        Serial.print(",");
        Serial.print(gyroData2.yData * Gyrofactor, 4);
        Serial.print(",");
        Serial.print(gyroData2.zData * Gyrofactor, 4);
        

        Serial.println();

        delay(100);
    }
}
