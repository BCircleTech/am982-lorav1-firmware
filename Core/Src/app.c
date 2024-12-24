#include "am982lorav1.h"

uint8_t rtkCOM1RxBuff[512];
uint8_t rtkCOM3RxBuff[512];
uint8_t boardUARTRxBuff[512];
uint8_t loraUARTRxBuff[512];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == rtkCOM1Ptr->Instance)
    {
        HAL_UART_Transmit_DMA(boardUARTPtr, rtkCOM1RxBuff, size);
        HAL_UARTEx_ReceiveToIdle_DMA(rtkCOM1Ptr, rtkCOM1RxBuff, sizeof(rtkCOM1RxBuff));
    }
    else if (huart->Instance == rtkCOM3Ptr->Instance)
    {
        HAL_UART_Transmit_DMA(boardUARTPtr, rtkCOM3RxBuff, size);
        HAL_UARTEx_ReceiveToIdle_DMA(rtkCOM3Ptr, rtkCOM3RxBuff, sizeof(rtkCOM3RxBuff));
    }
    else if (huart->Instance == boardUARTPtr->Instance)
    {
        HAL_UART_Transmit_DMA(boardUARTPtr, boardUARTRxBuff, size);
        HAL_UARTEx_ReceiveToIdle_DMA(boardUARTPtr, boardUARTRxBuff, sizeof(boardUARTRxBuff));
    }
    else if (huart->Instance == loraUARTPtr->Instance)
    {
        LoraConfCallback(loraUARTRxBuff, size);
        HAL_UART_Transmit_DMA(boardUARTPtr, loraUARTRxBuff, size);
        HAL_UARTEx_ReceiveToIdle_DMA(loraUARTPtr, loraUARTRxBuff, sizeof(loraUARTRxBuff));
    }
}

void StartMain(void *argument)
{
    MX_USB_DEVICE_Init();

    HAL_UARTEx_ReceiveToIdle_DMA(rtkCOM1Ptr, rtkCOM1RxBuff, sizeof(rtkCOM1RxBuff));
    HAL_UARTEx_ReceiveToIdle_DMA(rtkCOM3Ptr, rtkCOM3RxBuff, sizeof(rtkCOM3RxBuff));
    HAL_UARTEx_ReceiveToIdle_DMA(boardUARTPtr, boardUARTRxBuff, sizeof(boardUARTRxBuff));
    HAL_UARTEx_ReceiveToIdle_DMA(loraUARTPtr, loraUARTRxBuff, sizeof(loraUARTRxBuff));

    ResetRTK();
    ResetIMU();
    uint16_t loraAddr = 0x0000;
    uint8_t loraChannel = 0x00;
    SetLoraConf(loraAddr, loraChannel);
    osDelay(100);
    // SetRTKBaseWithPosition(1, 2, 3);
    // SetRTKBaseWithTime(10);
    SetRTKRover(RTK_ROVER_FREQ_1HZ);
    InitIMU(MPU6050_CLOCK_PLL_XGYRO, MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_250);
    SetIMUDigitalLowPassFilter(MPU6050_DLPF_BW_5);
    SetIMUSampleRate(100);

    uint8_t dbug[128] = {0};
    int dbug_len;

    float accel[3] = {0};
    float gyro[3] = {0};
    float temp = 0;
    while (1)
    {
        GetIMUAccel(accel);
        GetIMUGyro(gyro);
        GetIMUTemp(&temp);
        dbug_len = snprintf((char *)dbug, sizeof(dbug), "(%+6.2f %+6.2f %+6.2f) (%+6.2f %+6.2f %+6.2f) (%+6.2f)\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temp);
        HAL_UART_Transmit_DMA(boardUARTPtr, dbug, dbug_len);
        osDelay(100);
    }
}

void StartRTKCOM1(void *argument)
{
    while (1)
    {
        LedRunOn();
        osDelay(500);
        LedRunOff();
        osDelay(500);
    }
}
