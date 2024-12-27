# AM982-LORAV1 Firmware

This project is the firmware for the HederaTech AM982-LORAV1 module. It is designed for both end users and developers. For developers, this project provides a good starting point. For end users, it offers basic functionality to interact with the UM982, MPU6050 and E32-433T20S via USB.

## Connection

Use a USB Type-C cable to connect the host to the AM982-STDV1 module. A separate DC power supply is required for the Lora module, not just USB power supply.

## Protocol

The structure of a complete frame transmitted between the host and the module is as follows, in HEX.

|Header|Command|Data|Checksum|Trailer|
|:---:|:---:|:---:|:---:|:---:|
|AC 53|XX XX|XX XX ... XX|XX|35 CA|

* The size of **Data** is variable and can be 0, but no more than 65533.
* **Checksum** is derived from both **Command** and **Data**.

### Host to Module

|Command|Data|Description|
|:---:|:---:|:---|
|00 00|XX XX ... XX|Forward **Data** to the COM1 of the UM982.|
|00 01|Latitude Longitude Altitude *or* Seconds|Set the module to RTK base with the given position or initial time. The types of position **Latitude**, **Longitude**, and **Altitude** are doubles, each with 8 bytes. The type of initial time **Seconds** is unsigned integer with 1 byte. If initial time was set, the RTK base will automatically initialize its position within the initial time.|
|00 02|XX|Set the module to RTK rover with frequency unsigned integer **XX**. Options are 1Hz, 5Hz, 10Hz.|
|01 00|XX|Read data from the register **XX** of the MPU6050.|
|01 01|XX YY|Write data **YY** to the register **XX** of the MPU6050.|
|01 02|XX|Set the frequency unsigned integer **XX** of the IMU data. Options are 1Hz, 5Hz, 10Hz, 50Hz, 100Hz.|
|02 00|XX XX YY|Set the address XX XX \(from 00 00 to FF FF\) and channel YY \(from 00 to 1F\) of the Lora module.|
|02 01||Get the address and channel of the Lora module.|

### Module to Host

|Command|Data|Description|
|:---:|:---:|:---|
|80 00|XX XX ... XX|Forward **Data** from the COM1 of the UM982.|
|80 01|XX|**XX** is 00 for success, and 01 for failure.|
|80 02|XX|**XX** is 00 for success, and 01 for failure.|
|80 03|XX XX ... XX|Forward **Data** from the COM3 of the UM982, if the module is set to RTK rover.|
|81 00|XX YY|Return data **YY** in the register **XX** of the MPU6050.|
|81 01|XX|**XX** is 00 for success, and 01 for failure.|
|81 02|XX|**XX** is 00 for success, and 01 for failure.|
|81 03|ACX ACY ACZ GRX GRY GRZ|Return the accelerations and angular velocities measured by the MPU6050. **ACX**, **ACY**, and **ACZ** represent the accelerations along the X-axis, Y-axis, and Z-axis, respectively. **GRX**, **GRY**, and **GRZ** represent the angular velocities along the X-axis, Y-axis, and Z-axis, respectively. Each value of acceleration and angular velocity is a float, with 4 bytes.|
|82 00|XX|**XX** is 00 for success, and 01 for failure.|
|82 01|XX XX YY|Return address XX XX and channel YY of the Lora module.|

### Protocol Summary

|Request|Response|Description|
|---|---|---|
|00 00|80 00|Send to and receive from the COM1 of the UM982.|
|00 01|80 01|Set the module as an RTK base.|
|00 02|80 02|Set the module as an RTK rover with frequency.|
||80 03|Receive from the COM3 of the UM982, if the module is set to RTK rover.|
|01 00|81 00|Read data from the register of the MPU6050.|
|01 01|81 01|Write data to the register of the MPU6050.|
|01 02|81 02|Set the frequency of the IMU data.|
||81 03|Receive measurements from the IMU.|
|02 00|82 00|Set the address and channel of the Lora module.|
|02 01|82 01|Get the address and channel of the Lora module.|

## RTK Initialization

### Base

Send the following strings to UM982 via COM1:

```c
/* Reset UM982. */
uint8_t cmd_0[] = "freset\r\n";
/* Automatically initialize position within 60 seconds. */
uint8_t cmd_1[] = "mode base time 60\r\n";
/* Send RTK base data via COM3. */
uint8_t cmd_2[] = "rtcm1006 com3 10\r\n";
uint8_t cmd_3[] = "rtcm1033 com3 10\r\n";
uint8_t cmd_4[] = "rtcm1074 com3 1\r\n";
uint8_t cmd_5[] = "rtcm1124 com3 1\r\n";
uint8_t cmd_6[] = "rtcm1084 com3 1\r\n";
uint8_t cmd_7[] = "rtcm1094 com3 1\r\n";
/* Save */
uint8_t cmd_8[] = "saveconfig\r\n";
```

or

```c
/* Reset UM982. */
uint8_t cmd_0[] = "freset\r\n";
/* Set the latitude, longitude, and altitude if known. */
uint8_t cmd_1[] = "mode base 40.078983248 116.236601977 60.42\r\n";
/* Send RTK base data via COM3. */
uint8_t cmd_2[] = "rtcm1006 com3 10\r\n";
uint8_t cmd_3[] = "rtcm1033 com3 10\r\n";
uint8_t cmd_4[] = "rtcm1074 com3 1\r\n";
uint8_t cmd_5[] = "rtcm1124 com3 1\r\n";
uint8_t cmd_6[] = "rtcm1084 com3 1\r\n";
uint8_t cmd_7[] = "rtcm1094 com3 1\r\n";
/* Save */
uint8_t cmd_8[] = "saveconfig\r\n";
```

### Rover

Send the following strings to UM982 via COM1:

```c
/* Reset UM982. */
uint8_t cmd_0[] = "freset\r\n";
/* Set to rover. */
uint8_t cmd_1[] = "mode rover\r\n";
/* Receive latitude, longitude, and altitude message via COM3. */
uint8_t cmd_2[] = "gpgga com3 1\r\n";
/* Receive heading message via COM3. */
uint8_t cmd_3[] = "gpths com3 1\r\n";
/* Save */
uint8_t cmd_4[] = "saveconfig\r\n";
```

## IMU Initialization

```c
/* Set accel range to 2g */
WriteIMUReg(MPU6050_RA_ACCEL_CONFIG, 0x00);
/* Set gyro range to 250 degree/s */
WriteIMUReg(MPU6050_RA_GYRO_CONFIG, 0x00);
/* Set digital low pass filter to 5Hz */
WriteIMUReg(MPU6050_RA_CONFIG, 0x06);
/* Set sample rate to 100Hz */
WriteIMUReg(MPU6050_RA_SMPLRT_DIV, 0x09);
/* Init IMU */
WriteIMUReg(MPU6050_RA_PWR_MGMT_1, 0x01);
```

## Lora Initialization

Send the following strings to E32-433T20S with M0=high and M1=high:

```c
/* Address: 00 03 */
/* Channel: 14 */
/* Settings saved on power down */
uint8_t cmd[6] = {0xc0, 0x00, 0x03, 0x1d, 0x14, 0x40};
```

or

```c
/* Address: 00 03 */
/* Channel: 14 */
/* Settings lost on power down */
uint8_t cmd[6] = {0xc2, 0x00, 0x03, 0x1d, 0x14, 0x40};
```
