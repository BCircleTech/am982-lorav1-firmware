#include "am982lorav1.h"

UART_HandleTypeDef *rtkCOM1Ptr = &huart9;
UART_HandleTypeDef *rtkCOM3Ptr = &huart2;
UART_HandleTypeDef *boardUARTPtr = &huart4;
UART_HandleTypeDef *loraUARTPtr = &huart10;
I2C_HandleTypeDef *imuI2CPtr = &hi2c5;

float imuAccelSensitivity = 0;
float imuGyroSensitivity = 0;
float gravity = 9.80665;
unsigned int imuAccelFs = 1000;
unsigned int imuGyroFs = 1000;
uint16_t loraAddr;
uint8_t loraChannel;
uint8_t loraConfFlag = 0;
uint8_t rtkMode;
uint8_t rtkModeFlag = 0;

void LedRunOn()
{
    HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET);
}

void LedRunOff()
{
    HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
}

void LedErrOn()
{
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
}

void LedErrOff()
{
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
}

void ResetRTK()
{
    HAL_GPIO_WritePin(RTK_NRST_GPIO_Port, RTK_NRST_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(RTK_NRST_GPIO_Port, RTK_NRST_Pin, GPIO_PIN_SET);
}

void SetRTKConf(uint8_t *cmd, uint32_t size)
{
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd, size);
}

void SetRTKBaseData(uint8_t *data, uint32_t size)
{
    HAL_UART_Transmit_DMA(rtkCOM3Ptr, data, size);
}

void SetRTKBaseWithPosition(double latitude, double longitude, double altitude)
{
    uint8_t cmd_0[] = "freset\r\n";
    uint8_t cmd_1[128] = {0};
    uint8_t cmd_2[] = "rtcm1006 com3 10\r\n";
    uint8_t cmd_3[] = "rtcm1033 com3 10\r\n";
    uint8_t cmd_4[] = "rtcm1074 com3 1\r\n";
    uint8_t cmd_5[] = "rtcm1124 com3 1\r\n";
    uint8_t cmd_6[] = "rtcm1084 com3 1\r\n";
    uint8_t cmd_7[] = "rtcm1094 com3 1\r\n";
    uint8_t cmd_8[] = "saveconfig\r\n";
    int cmd_1_len = snprintf((char *)cmd_1, sizeof(cmd_1), "mode base %.9f %.9f %.2f\r\n", latitude, longitude, altitude);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_0, sizeof(cmd_0) - 1);
    osDelay(10000);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_1, cmd_1_len);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2, sizeof(cmd_2) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3, sizeof(cmd_3) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_4, sizeof(cmd_4) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_5, sizeof(cmd_5) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_6, sizeof(cmd_6) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_7, sizeof(cmd_7) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_8, sizeof(cmd_8) - 1);
    osDelay(100);
}

void SetRTKBaseWithTime(unsigned int seconds)
{
    uint8_t cmd_0[] = "freset\r\n";
    uint8_t cmd_1[128] = {0};
    uint8_t cmd_2[] = "rtcm1006 com3 10\r\n";
    uint8_t cmd_3[] = "rtcm1033 com3 10\r\n";
    uint8_t cmd_4[] = "rtcm1074 com3 1\r\n";
    uint8_t cmd_5[] = "rtcm1124 com3 1\r\n";
    uint8_t cmd_6[] = "rtcm1084 com3 1\r\n";
    uint8_t cmd_7[] = "rtcm1094 com3 1\r\n";
    uint8_t cmd_8[] = "saveconfig\r\n";
    int cmd_1_len = snprintf((char *)cmd_1, sizeof(cmd_1), "mode base time %d\r\n", seconds);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_0, sizeof(cmd_0) - 1);
    osDelay(10000);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_1, cmd_1_len);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2, sizeof(cmd_2) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3, sizeof(cmd_3) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_4, sizeof(cmd_4) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_5, sizeof(cmd_5) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_6, sizeof(cmd_6) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_7, sizeof(cmd_7) - 1);
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_8, sizeof(cmd_8) - 1);
    osDelay(100);
}

/*
freq:
    RTK_ROVER_FREQ_1HZ
    RTK_ROVER_FREQ_2HZ
    RTK_ROVER_FREQ_5HZ
    RTK_ROVER_FREQ_10HZ
    RTK_ROVER_FREQ_20HZ
    RTK_ROVER_FREQ_50HZ
*/
void SetRTKRover(unsigned int freq)
{
    uint8_t cmd_0[] = "freset\r\n";
    uint8_t cmd_1[] = "mode rover\r\n";
    uint8_t cmd_2_1hz[] = "gpgga com3 1\r\n";
    uint8_t cmd_2_2hz[] = "gpgga com3 0.5\r\n";
    uint8_t cmd_2_5hz[] = "gpgga com3 0.2\r\n";
    uint8_t cmd_2_10hz[] = "gpgga com3 0.1\r\n";
    uint8_t cmd_2_20hz[] = "gpgga com3 0.05\r\n";
    uint8_t cmd_2_50hz[] = "gpgga com3 0.02\r\n";
    uint8_t cmd_3_1hz[] = "gpths com3 1\r\n";
    uint8_t cmd_3_2hz[] = "gpths com3 0.5\r\n";
    uint8_t cmd_3_5hz[] = "gpths com3 0.2\r\n";
    uint8_t cmd_3_10hz[] = "gpths com3 0.1\r\n";
    uint8_t cmd_3_20hz[] = "gpths com3 0.05\r\n";
    uint8_t cmd_3_50hz[] = "gpths com3 0.02\r\n";
    uint8_t cmd_4[] = "saveconfig\r\n";
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_0, sizeof(cmd_0) - 1);
    osDelay(10000);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_1, sizeof(cmd_1) - 1);
    osDelay(100);
    switch (freq)
    {
    case RTK_ROVER_FREQ_1HZ:
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2_1hz, sizeof(cmd_2_1hz) - 1);
        osDelay(100);
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3_1hz, sizeof(cmd_3_1hz) - 1);
        break;
    case RTK_ROVER_FREQ_2HZ:
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2_2hz, sizeof(cmd_2_2hz) - 1);
        osDelay(100);
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3_2hz, sizeof(cmd_3_2hz) - 1);
        break;
    case RTK_ROVER_FREQ_5HZ:
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2_5hz, sizeof(cmd_2_5hz) - 1);
        osDelay(100);
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3_5hz, sizeof(cmd_3_5hz) - 1);
        break;
    case RTK_ROVER_FREQ_10HZ:
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2_10hz, sizeof(cmd_2_10hz) - 1);
        osDelay(100);
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3_10hz, sizeof(cmd_3_10hz) - 1);
        break;
    case RTK_ROVER_FREQ_20HZ:
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2_20hz, sizeof(cmd_2_20hz) - 1);
        osDelay(100);
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3_20hz, sizeof(cmd_3_20hz) - 1);
        break;
    case RTK_ROVER_FREQ_50HZ:
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2_50hz, sizeof(cmd_2_50hz) - 1);
        osDelay(100);
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3_50hz, sizeof(cmd_3_50hz) - 1);
        break;
    default:
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_2_1hz, sizeof(cmd_2_1hz) - 1);
        osDelay(100);
        HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_3_1hz, sizeof(cmd_3_1hz) - 1);
        break;
    }
    osDelay(100);
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd_4, sizeof(cmd_4) - 1);
    osDelay(100);
}

void GetRTKMode(uint8_t *mode)
{
    uint8_t cmd[] = "mode\r\n";
    rtkModeFlag = 1;
    HAL_UART_Transmit_DMA(rtkCOM1Ptr, cmd, sizeof(cmd) - 1);
    osDelay(100);
    while (rtkModeFlag)
    {
        osDelay(1);
    }
    *mode = rtkMode;
}

void GetRTKPPS(uint8_t *pps)
{
    *pps = HAL_GPIO_ReadPin(RTK_PPS_GPIO_Port, RTK_PPS_Pin);
}

void RTKModeCallback(uint8_t *data, uint32_t size)
{
    if (rtkModeFlag == 1)
    {
        uint32_t loop = size - 9;
        for (uint32_t i = 0; i < loop; i++)
        {
            if (data[i] == 'M' &&
                data[i + 1] == 'O' &&
                data[i + 2] == 'D' &&
                data[i + 3] == 'E' &&
                data[i + 4] == ' ')
            {
                if (data[i + 5] == 'B' &&
                    data[i + 6] == 'A' &&
                    data[i + 7] == 'S' &&
                    data[i + 8] == 'E')
                {
                    rtkMode = RTK_MODE_BASE;
                    break;
                }
                if (data[i + 5] == 'R' &&
                    data[i + 6] == 'O' &&
                    data[i + 7] == 'V' &&
                    data[i + 8] == 'E' &&
                    data[i + 9] == 'R')
                {
                    rtkMode = RTK_MODE_ROVER;
                    break;
                }
            }
        }
        rtkModeFlag = 0;
    }
}

void ResetIMU()
{
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    osDelay(2000);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
}

void ReadIMUReg(uint8_t addr, uint8_t *data)
{
    HAL_I2C_Master_Transmit(imuI2CPtr, IMU_I2C_ADDR, &addr, 1, IMU_I2C_TIMEOUT);
    HAL_I2C_Master_Receive(imuI2CPtr, IMU_I2C_ADDR, data, 1, IMU_I2C_TIMEOUT);
}

void WriteIMUReg(uint8_t addr, uint8_t data)
{
    uint8_t *tmp = malloc(2);
    tmp[0] = addr;
    tmp[1] = data;
    HAL_I2C_Master_Transmit(imuI2CPtr, IMU_I2C_ADDR, tmp, 2, IMU_I2C_TIMEOUT);
    free(tmp);
}

void WriteIMURegBits(uint8_t addr, uint32_t start, uint32_t len, uint8_t data)
{
    uint8_t tmp, mask;
    ReadIMUReg(addr, &tmp);
    mask = ((1 << len) - 1) << (start - len + 1);
    data <<= (start - len + 1);
    data &= mask;
    tmp &= ~(mask);
    tmp |= data;
    WriteIMUReg(addr, tmp);
}
/*
clkSource:
    MPU6050_CLOCK_INTERNAL
    MPU6050_CLOCK_PLL_XGYRO
    MPU6050_CLOCK_PLL_YGYRO
    MPU6050_CLOCK_PLL_ZGYRO
    MPU6050_CLOCK_PLL_EXT32K
    MPU6050_CLOCK_PLL_EXT19M
    MPU6050_CLOCK_KEEP_RESET
accelRange:
    MPU6050_ACCEL_FS_2
    MPU6050_ACCEL_FS_4
    MPU6050_ACCEL_FS_8
    MPU6050_ACCEL_FS_16
gyroRange:
    MPU6050_GYRO_FS_250
    MPU6050_GYRO_FS_500
    MPU6050_GYRO_FS_1000
    MPU6050_GYRO_FS_2000
*/
void InitIMU(uint8_t clkSource, uint8_t accelRange, uint8_t gyroRange)
{
    // set clock source
    WriteIMURegBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, clkSource);
    // set full scale accel range
    switch (accelRange)
    {
    case MPU6050_ACCEL_FS_2:
        imuAccelSensitivity = 16384.0;
        break;
    case MPU6050_ACCEL_FS_4:
        imuAccelSensitivity = 8192.0;
        break;
    case MPU6050_ACCEL_FS_8:
        imuAccelSensitivity = 4096.0;
        break;
    case MPU6050_ACCEL_FS_16:
        imuAccelSensitivity = 2048.0;
        break;
    default:
        accelRange = MPU6050_ACCEL_FS_16;
        imuAccelSensitivity = 2048.0;
    }
    WriteIMURegBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, accelRange);
    // set full scale gyro range
    switch (gyroRange)
    {
    case MPU6050_GYRO_FS_250:
        imuGyroSensitivity = 131.0;
        break;
    case MPU6050_GYRO_FS_500:
        imuGyroSensitivity = 65.5;
        break;
    case MPU6050_GYRO_FS_1000:
        imuGyroSensitivity = 32.8;
        break;
    case MPU6050_GYRO_FS_2000:
        imuGyroSensitivity = 16.4;
        break;
    default:
        gyroRange = MPU6050_GYRO_FS_2000;
        imuGyroSensitivity = 16.4;
    }
    WriteIMURegBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, gyroRange);
    // set sleep enabled
    WriteIMURegBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1, 0);
}

/*
bandwidth:
    MPU6050_DLPF_BW_256
    MPU6050_DLPF_BW_188
    MPU6050_DLPF_BW_98
    MPU6050_DLPF_BW_42
    MPU6050_DLPF_BW_20
    MPU6050_DLPF_BW_10
    MPU6050_DLPF_BW_5
*/
void SetIMUDigitalLowPassFilter(uint8_t bandwidth)
{
    switch (bandwidth)
    {
    case MPU6050_DLPF_BW_5:
    case MPU6050_DLPF_BW_10:
    case MPU6050_DLPF_BW_20:
    case MPU6050_DLPF_BW_42:
    case MPU6050_DLPF_BW_98:
    case MPU6050_DLPF_BW_188:
        break;
    case MPU6050_DLPF_BW_256:
        imuGyroFs = 8000;
        break;
    default:
        imuGyroFs = 8000;
    }
    WriteIMURegBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, bandwidth);
}

void SetIMUSampleRate(unsigned int rate)
{
    uint8_t div = (imuGyroFs / rate) - 1;
    WriteIMUReg(MPU6050_RA_SMPLRT_DIV, div);
}

void GetIMUAccel(float accel[3])
{
    uint8_t axl, axh, ayl, ayh, azl, azh;
    int16_t ax, ay, az;
    ReadIMUReg(MPU6050_RA_ACCEL_XOUT_L, &axl);
    ReadIMUReg(MPU6050_RA_ACCEL_XOUT_H, &axh);
    ReadIMUReg(MPU6050_RA_ACCEL_YOUT_L, &ayl);
    ReadIMUReg(MPU6050_RA_ACCEL_YOUT_H, &ayh);
    ReadIMUReg(MPU6050_RA_ACCEL_ZOUT_L, &azl);
    ReadIMUReg(MPU6050_RA_ACCEL_ZOUT_H, &azh);
    ax = axh << 8 | axl;
    ay = ayh << 8 | ayl;
    az = azh << 8 | azl;
    accel[0] = ax / imuAccelSensitivity * gravity;
    accel[1] = ay / imuAccelSensitivity * gravity;
    accel[2] = az / imuAccelSensitivity * gravity;
}

void GetIMUGyro(float gyro[3])
{
    uint8_t gxl, gxh, gyl, gyh, gzl, gzh;
    int16_t gx, gy, gz;
    ReadIMUReg(MPU6050_RA_GYRO_XOUT_L, &gxl);
    ReadIMUReg(MPU6050_RA_GYRO_XOUT_H, &gxh);
    ReadIMUReg(MPU6050_RA_GYRO_YOUT_L, &gyl);
    ReadIMUReg(MPU6050_RA_GYRO_YOUT_H, &gyh);
    ReadIMUReg(MPU6050_RA_GYRO_ZOUT_L, &gzl);
    ReadIMUReg(MPU6050_RA_GYRO_ZOUT_H, &gzh);
    gx = gxh << 8 | gxl;
    gy = gyh << 8 | gyl;
    gz = gzh << 8 | gzl;
    gyro[0] = gx / imuGyroSensitivity;
    gyro[1] = gy / imuGyroSensitivity;
    gyro[2] = gz / imuGyroSensitivity;
}

void GetIMUTemp(float *temp)
{
    uint8_t tl, th;
    int16_t t;
    ReadIMUReg(MPU6050_RA_TEMP_OUT_L, &tl);
    ReadIMUReg(MPU6050_RA_TEMP_OUT_H, &th);
    t = th << 8 | tl;
    *temp = (t - 521) / 340.0 + 36.53;
}

/*
mode:
    LORA_MODE_NORMAL
    LORA_MODE_WAKE
    LORA_MODE_SLEEP
    LORA_MODE_CONF
*/
void SetLoraMode(uint8_t mode)
{
    switch (mode)
    {
    case LORA_MODE_NORMAL:
        HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
        break;
    case LORA_MODE_WAKE:
        HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
        break;
    case LORA_MODE_SLEEP:
        HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);
        break;
    case LORA_MODE_CONF:
        HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);
        break;
    default:
        HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
        break;
    }
}

void GetAUX(uint8_t *aux)
{
    *aux = HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin);
}

void SetLoraData(uint8_t *data, uint32_t size)
{
    uint32_t count = 0;
    uint32_t len;
    uint8_t current = 1;
    uint8_t last;
    while (count != size)
    {
        len = (size - count) > 512 ? 512 : (size - count);
        HAL_UART_Transmit_DMA(loraUARTPtr, data + count, len);
        do
        {
            osDelay(1);
            last = current;
            current = HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin);
        } while (current == 0 || last == 1);
        osDelay(100);
        count += len;
    }
}

void SetLoraConf(uint16_t addr, uint8_t channel)
{
    uint8_t cmd[6] = {0xc0, 0x00, 0x00, 0x1d, 0x00, 0x40};
    cmd[1] = addr >> 8 & 0xff;
    cmd[2] = addr & 0xff;
    cmd[4] = channel;
    SetLoraMode(LORA_MODE_CONF);
    osDelay(100);
    loraConfFlag = 1;
    HAL_UART_Transmit_DMA(loraUARTPtr, cmd, sizeof(cmd));
    osDelay(100);
    while (loraConfFlag)
    {
        osDelay(1);
    }
    SetLoraMode(LORA_MODE_NORMAL);
    osDelay(100);
}

void GetLoraConf(uint16_t *addr, uint8_t *channel)
{
    uint8_t cmd[3] = {0xc1, 0xc1, 0xc1};
    SetLoraMode(LORA_MODE_CONF);
    osDelay(100);
    loraConfFlag = 1;
    HAL_UART_Transmit_DMA(loraUARTPtr, cmd, sizeof(cmd));
    osDelay(100);
    while (loraConfFlag)
    {
        osDelay(1);
    }
    *addr = loraAddr;
    *channel = loraChannel;
    SetLoraMode(LORA_MODE_NORMAL);
    osDelay(100);
}

void LoraConfCallback(uint8_t *data, uint32_t size, uint8_t *res)
{
    if (loraConfFlag == 1 && size == 6)
    {
        loraAddr = data[1];
        loraAddr = loraAddr << 8 | data[2];
        loraChannel = data[4];
        loraConfFlag = 0;
        *res = 0x01;
    }
    else
    {
        *res = 0x00;
    }
}