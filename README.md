# AM982-LORAV1 Firmware

This project is the firmware for the HederaTech AM982-LORAV1 module. It is designed for both end users and developers. For developers, this project provides a good starting point. For end users, it offers basic functionality to interact with the UM982, MPU6050 and E32-433T20S via USB.

## Connection

Use a USB Type-C cable to connect the host to the AM982-STDV1 module. A separate DC power supply is required for the Lora module.

## Protocol

The structure of a complete frame transmitted between the host and the module is as follows, in HEX.

|Header|Length|Command|Data|Checksum|Trailer|
|:---:|:---:|:---:|:---:|:---:|:---:|
|AC 53|XX XX|XX XX|XX XX ... XX|XX|35 CA|

* **Length** is the totle size of **Command** and **Data** in bytes.
* The size of **Data** is variable and can be 0, but no more than 65533.
* **Checksum** is derived from both **Command** and **Data**.

### Host to Module

|Command|Data|Description|
|:---:|:---:|:---|
|00 00|XX XX ... XX|Forward **Data** to the COM1 of the UM982.|
|00 01|Latitude Longitude Altitude|Set the module to RTK base with the given latitude, longitude, and altitude. The types of latitude, longitude, and altitude are doubles, each with 8 bytes. **Data** is optional and, if not provided, the RTK base will automatically initialize its position within the first few minutes.|
|00 02||Set the module to RTK rover.|
|00 03|XX|Set the frequency of the RTK rover data. Options are 1Hz, 5Hz, 10Hz.|
|00 04||Get the RTK role.|
|01 00|XX|Read data from the register **XX** of the MPU6050.|
|01 01|XX YY|Write data **YY** to the register **XX** of the MPU6050.|
|01 02|XX XX|Set the frequency of the IMU data. Options are 1Hz, 5Hz, 10Hz, 50Hz, 100Hz, 500Hz, 1000Hz.|
|02 00|XX XX YY|Set the address XX XX \(from 00 00 to FF FF\) and channel YY \(from 00 to 1F\) of the Lora module.|
|02 01||Get the address and channel of the Lora module.|

### Module to Host

|Command|Data|Description|
|:---:|:---:|:---|
|80 00|XX XX ... XX|Forward **Data** from the COM1 of the UM982.|
|80 04||Return the RTK role. 0 for base and 1 for rover.|
|80 05|XX XX ... XX|Forward **Data** from the COM3 of the UM982, if the module is set to RTK rover.|
|81 00|XX YY|Return data **YY** in the register **XX** of the MPU6050.|
|81 03|ACX ACY ACZ AVX AVY AVZ|Return the accelerations and angular velocities measured by the MPU6050. ACX, ACY, and ACZ represent the accelerations along the X-axis, Y-axis, and Z-axis, respectively. AVX, AVY, and AVZ represent the angular velocities along the X-axis, Y-axis, and Z-axis, respectively. Each value of acceleration and angular velocity is a double, with 8 bytes.|
|82 01|XX XX YY|Return address XX XX and channel YY of the Lora module.|
|8F 00||Command parsing succeed.|
|8F 01||Command parsing Failed.|

### Protocol Summary

|Request|Response|Description|
|---|---|---|
|00 00|80 00|Send to and receive from the COM1 of the UM982.|
|00 01|8F 00 *or* 8F 01|Set the module as an RTK base.|
|00 02|8F 00 *or* 8F 01|Set the module as an RTK rover.|
|00 03|8F 00 *or* 8F 01|Set the frequency of the RTK rover data.|
|00 04|80 04|Get the RTK role.|
||80 05|Receive from the COM3 of the UM982, if the module is set to RTK rover.|
|01 00|81 00|Read data from the register of the MPU6050.|
|01 01|8F 00 *or* 8F 01|Write data to the register of the MPU6050.|
|01 02|8F 00 *or* 8F 01|Set the frequency of the IMU data.|
||81 03|Receive measurements from the IMU.|
|02 00|8F 00 *or* 8F 01|Set the address and channel of the Lora module.|
|02 01|82 01|Get the address and channel of the Lora module.|

## RTK Initialization

### Base

Send the following strings to UM982 via COM1:

```c
/* Reset UM982. */
uint8_t cmd_1[] = "freset\r\n";
/* Automatically initialize position within 60 seconds. */
uint8_t cmd_2[] = "mode base time 60\r\n";
/* Send RTK base data via COM2. */
uint8_t cmd_3[] = "rtcm1006 com2 10\r\n";
uint8_t cmd_4[] = "rtcm1033 com2 10\r\n";
uint8_t cmd_5[] = "rtcm1074 com2 1\r\n";
uint8_t cmd_6[] = "rtcm1124 com2 1\r\n";
uint8_t cmd_7[] = "rtcm1084 com2 1\r\n";
uint8_t cmd_8[] = "rtcm1094 com2 1\r\n";
```

or

```c
/* Reset UM982. */
uint8_t cmd_1[] = "freset\r\n";
/* Set the latitude, longitude, and altitude if known. */
uint8_t cmd_2[] = "mode base 40.078983248 116.236601977 60.42\r\n";
/* Send RTK base data via COM2. */
uint8_t cmd_3[] = "rtcm1006 com3 10\r\n";
uint8_t cmd_4[] = "rtcm1033 com3 10\r\n";
uint8_t cmd_5[] = "rtcm1074 com3 1\r\n";
uint8_t cmd_6[] = "rtcm1124 com3 1\r\n";
uint8_t cmd_7[] = "rtcm1084 com3 1\r\n";
uint8_t cmd_8[] = "rtcm1094 com3 1\r\n";
```

### Rover

Send the following strings to UM982 via COM1:

```c
/* Reset UM982. */
uint8_t cmd_1[] = "freset\r\n";
/* Set to rover. */
uint8_t cmd_2[] = "mode rover\r\n";
/* Receive latitude, longitude, and altitude message via COM3. */
uint8_t cmd_3[] = "gpgga com3 1\r\n";
/* Receive heading message via COM3. */
uint8_t cmd_4[] = "gpths com3 1\r\n";
```

## Lora Initialization

Send the following strings to E32-433T20S with M0=high and M1=high:

```c
/* Address: 00 03 */
/* Channel: 14 */
/* Settings saved on power down */
uint8_t cmd[6] = {0xc0, 0x00, 0x03, 0x3d, 0x14, 0x40};
```

or

```c
/* Address: 00 03 */
/* Channel: 14 */
/* Settings lost on power down */
uint8_t cmd[6] = {0xc2, 0x00, 0x03, 0x3d, 0x14, 0x40};
```

Notice: When setting, baud rate is 9600 bps. But when transmitting, baud rate is 115200.