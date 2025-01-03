#include <string>
#include "protocol.h"

extern "C"
{
#include "am982lorav1.h"
#include "enu.h"
#include "message_buffer.h"
#include "stdio.h"
#include "string.h"
#include "usbd_cdc_if.h"

#define CMD_00_00 (uint8_t *)"\x00\x00"
#define CMD_00_01 (uint8_t *)"\x00\x01"
#define CMD_00_02 (uint8_t *)"\x00\x02"
#define CMD_01_00 (uint8_t *)"\x01\x00"
#define CMD_01_01 (uint8_t *)"\x01\x01"
#define CMD_01_02 (uint8_t *)"\x01\x02"
#define CMD_02_00 (uint8_t *)"\x02\x00"
#define CMD_03_00 (uint8_t *)"\x03\x00"
#define CMD_03_01 (uint8_t *)"\x03\x01"
#define CMD_80_00 (uint8_t *)"\x80\x00"
#define CMD_80_01 (uint8_t *)"\x80\x01"
#define CMD_80_02 (uint8_t *)"\x80\x02"
#define CMD_80_03 (uint8_t *)"\x80\x03"
#define CMD_81_00 (uint8_t *)"\x81\x00"
#define CMD_81_01 (uint8_t *)"\x81\x01"
#define CMD_81_02 (uint8_t *)"\x81\x02"
#define CMD_81_03 (uint8_t *)"\x81\x03"
#define CMD_82_00 (uint8_t *)"\x82\x00"
#define CMD_83_00 (uint8_t *)"\x83\x00"
#define CMD_83_01 (uint8_t *)"\x83\x01"
#define CMD_83_02 (uint8_t *)"\x83\x02"

uint8_t initFlag = 0;
uint8_t rtkModeValue = 0;
uint32_t rtkCOM3RxBuffAbortSize = 0;
TickType_t lastRTKBaseTimestamp = 0;
ENU *enu = nullptr;

uint8_t rtkCOM1RxBuff[512];
uint8_t rtkCOM3RxBuff[2048];
uint8_t rtkCOM3RxBuffAbort[2048];
uint8_t loraRxBuff[512];

// uint8_t dbug[128] = {0};
// int dbugLen;

MessageBufferHandle_t usbToMain;
MessageBufferHandle_t mainToRTKCOM1;
MessageBufferHandle_t rtkCOM1ToMain;
MessageBufferHandle_t mainToRTKCOM3;
MessageBufferHandle_t rtkCOM3ToMain;
MessageBufferHandle_t mainToIMU;
MessageBufferHandle_t mainToLora;
MessageBufferHandle_t loraToMain;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == rtkCOM1Ptr->Instance)
    {
        RTKModeCallback(rtkCOM1RxBuff, size);
        if (initFlag)
        {
            xMessageBufferSendFromISR(rtkCOM1ToMain, rtkCOM1RxBuff, size, &xHigherPriorityTaskWoken);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(rtkCOM1Ptr, rtkCOM1RxBuff, sizeof(rtkCOM1RxBuff));
    }
    else if (huart->Instance == rtkCOM3Ptr->Instance)
    {
        TickType_t currentRTKBaseTimestamp = xTaskGetTickCountFromISR();
        TickType_t deltaTime = currentRTKBaseTimestamp - lastRTKBaseTimestamp;
        if (initFlag)
        {
            if (xMessageBufferIsEmpty(rtkCOM3ToMain) == pdTRUE || deltaTime < 750)
            {
                if (rtkCOM3RxBuffAbortSize && deltaTime < 750)
                {
                    memcpy(rtkCOM3RxBuffAbort + rtkCOM3RxBuffAbortSize, rtkCOM3RxBuff, size);
                    xMessageBufferSendFromISR(rtkCOM3ToMain, rtkCOM3RxBuffAbort, rtkCOM3RxBuffAbortSize + size, &xHigherPriorityTaskWoken);
                    // dbugLen = snprintf((char *)dbug, sizeof(dbug), "Send: %ld*\n", rtkCOM3RxBuffAbortSize + size);
                    // HAL_UART_Transmit_DMA(boardUARTPtr, dbug, dbugLen);
                }
                else
                {
                    xMessageBufferSendFromISR(rtkCOM3ToMain, rtkCOM3RxBuff, size, &xHigherPriorityTaskWoken);
                    // dbugLen = snprintf((char *)dbug, sizeof(dbug), "Send: %d\n", size);
                    // HAL_UART_Transmit_DMA(boardUARTPtr, dbug, dbugLen);
                }
                rtkCOM3RxBuffAbortSize = 0;
                LedErrOff();
            }
            else
            {
                memcpy(rtkCOM3RxBuffAbort, rtkCOM3RxBuff, size);
                rtkCOM3RxBuffAbortSize = size;
                // dbugLen = snprintf((char *)dbug, sizeof(dbug), "Abort: %d\n", size);
                // HAL_UART_Transmit_DMA(boardUARTPtr, dbug, dbugLen);
                LedErrOn();
            }
        }
        lastRTKBaseTimestamp = currentRTKBaseTimestamp;
        HAL_UARTEx_ReceiveToIdle_DMA(rtkCOM3Ptr, rtkCOM3RxBuff, sizeof(rtkCOM3RxBuff));
    }
    else if (huart->Instance == loraUARTPtr->Instance)
    {
        uint8_t res;
        LoraConfCallback(loraRxBuff, size, &res);
        if (initFlag && !res)
        {
            xMessageBufferSendFromISR(loraToMain, loraRxBuff, size, &xHigherPriorityTaskWoken);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(loraUARTPtr, loraRxBuff, sizeof(loraRxBuff));
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USB_CDC_RxHandler(uint8_t *data, uint32_t size)
{
    if (initFlag)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xMessageBufferSendFromISR(usbToMain, data, size, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void USB_Transmit(uint8_t cmd[2], uint8_t *data, uint32_t len)
{
    std::string res;
    Protocol::Pack(res, cmd, std::string((char *)data, len));
    for (unsigned int i = 0; i < 5; i++)
    {
        if (CDC_Transmit_HS((uint8_t *)res.c_str(), res.length()) == USBD_OK)
        {
            break;
        }
        osDelay(1);
    }
}

void StartMain(void *argument)
{
    MX_USB_DEVICE_Init();

    HAL_UARTEx_ReceiveToIdle_DMA(rtkCOM1Ptr, rtkCOM1RxBuff, sizeof(rtkCOM1RxBuff));
    HAL_UARTEx_ReceiveToIdle_DMA(rtkCOM3Ptr, rtkCOM3RxBuff, sizeof(rtkCOM3RxBuff));
    HAL_UARTEx_ReceiveToIdle_DMA(loraUARTPtr, loraRxBuff, sizeof(loraRxBuff));

    usbToMain = xMessageBufferCreate(1024);
    mainToRTKCOM1 = xMessageBufferCreate(1024);
    rtkCOM1ToMain = xMessageBufferCreate(1024);
    mainToRTKCOM3 = xMessageBufferCreate(1024);
    rtkCOM3ToMain = xMessageBufferCreate(4096);
    mainToIMU = xMessageBufferCreate(1024);
    mainToLora = xMessageBufferCreate(1024);
    loraToMain = xMessageBufferCreate(1024);

    ResetRTK();
    ResetIMU();
    osDelay(2000);
    InitIMU(MPU6050_CLOCK_PLL_XGYRO, MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_250);
    SetIMUDigitalLowPassFilter(MPU6050_DLPF_BW_5);
    SetIMUSampleRate(100);
    GetRTKMode(&rtkModeValue);
    osDelay(2000);

    initFlag = 1;

    uint8_t usbRxBuffer[1024];
    uint32_t usbRxBufferLen;
    uint8_t rtkCOM1RxBuffer[1024];
    uint32_t rtkCOM1RxBufferLen;

    Protocol protocol;
    std::string line;

    while (1)
    {
        usbRxBufferLen = xMessageBufferReceive(usbToMain, usbRxBuffer, sizeof(usbRxBuffer), 0);
        if (usbRxBufferLen > 0)
        {
            // parse usbRxBuffer
            protocol.Receive(usbRxBuffer, usbRxBufferLen);
        }
        rtkCOM1RxBufferLen = xMessageBufferReceive(rtkCOM1ToMain, rtkCOM1RxBuffer, sizeof(rtkCOM1RxBuffer), 0);
        if (rtkCOM1RxBufferLen > 0)
        {
            // parse rtkCOM1RxBuffer
            USB_Transmit(CMD_80_00, rtkCOM1RxBuffer, rtkCOM1RxBufferLen);
        }

        if (protocol.GetOne(line))
        {
            if (line[0] == 0x00)
            {
                xMessageBufferSend(mainToRTKCOM1, (const void *)line.c_str(), line.length(), 10);
            }
            else if (line[0] == 0x01)
            {
                xMessageBufferSend(mainToIMU, (const void *)line.c_str(), line.length(), 10);
            }
            else if (line[0] == 0x02)
            {
                xMessageBufferSend(mainToLora, (const void *)line.c_str(), line.length(), 10);
            }
            else if (line[0] == 0x03)
            {
                xMessageBufferSend(mainToRTKCOM3, (const void *)line.c_str(), line.length(), 10);
            }
        }

        osDelay(100);
    }
}

void StartRTKCOM1(void *argument)
{
    uint8_t mainRxBuffer[1024];
    uint32_t mainRxBufferLen;

    while (1)
    {
        if (initFlag)
        {
            mainRxBufferLen = xMessageBufferReceive(mainToRTKCOM1, mainRxBuffer, sizeof(mainRxBuffer), 0);
            if (mainRxBufferLen > 0)
            {
                // parse mainRxBuffer
                if (mainRxBuffer[1] == 0x00)
                {
                    SetRTKConf(mainRxBuffer + 2, mainRxBufferLen - 2);
                }
                else if (mainRxBuffer[1] == 0x01)
                {
                    if (mainRxBufferLen == 26)
                    {
                        double latitude, longitude, altitude;
                        latitude = *(double *)(mainRxBuffer + 2);
                        longitude = *(double *)(mainRxBuffer + 10);
                        altitude = *(double *)(mainRxBuffer + 18);
                        uint8_t data = 0x00;
                        USB_Transmit(CMD_80_01, &data, 1);
                        SetRTKBaseWithPosition(latitude, longitude, altitude);
                        GetRTKMode(&rtkModeValue);
                    }
                    else if (mainRxBufferLen == 3)
                    {
                        uint8_t seconds;
                        seconds = mainRxBuffer[2];
                        uint8_t data = 0x00;
                        USB_Transmit(CMD_80_01, &data, 1);
                        SetRTKBaseWithTime(seconds);
                        GetRTKMode(&rtkModeValue);
                    }
                    else
                    {
                        uint8_t data = 0x01;
                        USB_Transmit(CMD_80_01, &data, 1);
                    }
                }
                else if (mainRxBuffer[1] == 0x02)
                {
                    if (mainRxBufferLen == 3)
                    {
                        uint8_t freq;
                        freq = mainRxBuffer[2];
                        uint8_t data = 0x00;
                        USB_Transmit(CMD_80_02, &data, 1);
                        SetRTKRover(freq);
                        GetRTKMode(&rtkModeValue);
                    }
                    else
                    {
                        uint8_t data = 0x01;
                        USB_Transmit(CMD_80_02, &data, 1);
                    }
                }
            }
        }

        osDelay(100);
    }
}

void StartRTKCOM3(void *argument)
{
    uint8_t mainRxBuffer[1024];
    uint32_t mainRxBufferLen;
    uint8_t rtkCOM3RxBuffer[2048];
    uint32_t rtkCOM3RxBufferLen;

    double lla[3];
    double xyz[3];

    char nmea[6][16];
    int nmeaRes;

    while (1)
    {
        if (initFlag)
        {
            mainRxBufferLen = xMessageBufferReceive(mainToRTKCOM3, mainRxBuffer, sizeof(mainRxBuffer), 0);
            if (mainRxBufferLen > 0)
            {
                // parse mainRxBuffer
                if (mainRxBuffer[1] == 0x00)
                {
                    if (mainRxBufferLen == 26)
                    {
                        double latitude, longitude, altitude;
                        latitude = *(double *)(mainRxBuffer + 2);
                        longitude = *(double *)(mainRxBuffer + 10);
                        altitude = *(double *)(mainRxBuffer + 18);
                        if (enu)
                        {
                            delete enu;
                        }
                        enu = new ENU(latitude, longitude, altitude);
                        uint8_t data = 0x00;
                        USB_Transmit(CMD_83_00, &data, 1);
                    }
                    else
                    {
                        uint8_t data = 0x01;
                        USB_Transmit(CMD_83_00, &data, 1);
                    }
                }
                else if (mainRxBuffer[1] == 0x01)
                {
                    if (mainRxBufferLen == 2)
                    {
                        if (enu)
                        {
                            delete enu;
                            enu = nullptr;
                        }
                        uint8_t data = 0x00;
                        USB_Transmit(CMD_83_01, &data, 1);
                    }
                    else
                    {
                        uint8_t data = 0x01;
                        USB_Transmit(CMD_83_01, &data, 1);
                    }
                }
            }
            rtkCOM3RxBufferLen = xMessageBufferReceive(rtkCOM3ToMain, rtkCOM3RxBuffer, sizeof(rtkCOM3RxBuffer), 0);
            if (rtkCOM3RxBufferLen > 0)
            {
                // parse rtkCOM3RxBuffer
                if (rtkModeValue)
                {
                    USB_Transmit(CMD_80_03, rtkCOM3RxBuffer, rtkCOM3RxBufferLen);
                    if (enu)
                    {
                        nmeaRes = sscanf((char *)rtkCOM3RxBuffer, "$GNGGA,%*[^,],%[^,],%[^,],%[^,],%[^,],%*[^,],%*[^,],%*[^,],%[^,],%*[^,],%[^,]",
                                         nmea[0], nmea[1], nmea[2], nmea[3], nmea[4], nmea[5]);
                        if (nmeaRes == 6)
                        {
                            std::string ad(nmea[0], 2);
                            std::string am(nmea[0] + 2);
                            std::string bd(nmea[2], 3);
                            std::string bm(nmea[2] + 3);
                            lla[0] = std::stoi(ad) + std::stod(am) / 60.0;
                            lla[1] = std::stoi(bd) + std::stod(bm) / 60.0;
                            lla[2] = std::stod(nmea[4]) + std::stod(nmea[5]);
                            if (nmea[1][0] == 'S')
                            {
                                lla[0] = -lla[0];
                            }
                            if (nmea[3][0] == 'W')
                            {
                                lla[1] = -lla[1];
                            }
                            enu->LLA2ENU(lla[0], lla[1], lla[2], xyz[0], xyz[1], xyz[2]);
                            USB_Transmit(CMD_83_02, (uint8_t *)xyz, 24);
                        }
                    }
                }
                else
                {
                    SetLoraData(rtkCOM3RxBuffer, rtkCOM3RxBufferLen);
                }
            }
        }

        osDelay(10);
    }
}

void StartIMU(void *argument)
{
    uint8_t mainRxBuffer[1024];
    uint32_t mainRxBufferLen;

    float measurements[6];
    uint8_t pps;

    uint32_t delay = 0;
    uint32_t delayCount = 0;

    while (1)
    {
        if (initFlag)
        {
            mainRxBufferLen = xMessageBufferReceive(mainToIMU, mainRxBuffer, sizeof(mainRxBuffer), 0);
            if (mainRxBufferLen > 0)
            {
                // parse mainRxBuffer
                if (mainRxBuffer[1] == 0x00)
                {
                    if (mainRxBufferLen == 3)
                    {
                        uint8_t value;
                        ReadIMUReg(mainRxBuffer[2], &value);
                        uint8_t data[2];
                        data[0] = mainRxBuffer[2];
                        data[1] = value;
                        USB_Transmit(CMD_81_00, data, 2);
                    }
                }
                else if (mainRxBuffer[1] == 0x01)
                {
                    if (mainRxBufferLen == 4)
                    {
                        WriteIMUReg(mainRxBuffer[2], mainRxBuffer[3]);
                        uint8_t data = 0x00;
                        USB_Transmit(CMD_81_01, &data, 1);
                    }
                    else
                    {
                        uint8_t data = 0x01;
                        USB_Transmit(CMD_81_01, &data, 1);
                    }
                }
                else if (mainRxBuffer[1] == 0x02)
                {
                    if (mainRxBufferLen == 3)
                    {
                        bool valid = true;
                        switch (mainRxBuffer[2])
                        {
                        case 1:
                            delay = 100;
                            break;
                        case 5:
                            delay = 20;
                            break;
                        case 10:
                            delay = 10;
                            break;
                        case 50:
                            delay = 2;
                            break;
                        case 100:
                            delay = 1;
                            break;
                        default:
                            valid = false;
                            break;
                        }
                        uint8_t data = 0x00;
                        if (!valid)
                        {
                            data = 0x01;
                        }
                        USB_Transmit(CMD_81_02, &data, 1);
                    }
                    else
                    {
                        uint8_t data = 0x01;
                        USB_Transmit(CMD_81_02, &data, 1);
                    }
                }
            }
        }

        if (delayCount >= delay)
        {
            delayCount = 0;
            GetIMUAccel(measurements);
            GetIMUGyro(measurements + 3);
            if (rtkModeValue)
            {
                USB_Transmit(CMD_81_03, (uint8_t *)measurements, 24);
            }
        }
        delayCount++;

        GetRTKPPS(&pps);
        if (pps)
        {
            LedRunOff();
        }
        else
        {
            LedRunOn();
        }

        osDelay(10);
    }
}

void StartLORA(void *argument)
{
    uint8_t mainRxBuffer[1024];
    uint32_t mainRxBufferLen;
    uint8_t loraRxBuffer[1024];
    uint32_t loraRxBufferLen;

    while (1)
    {
        if (initFlag)
        {
            mainRxBufferLen = xMessageBufferReceive(mainToLora, mainRxBuffer, sizeof(mainRxBuffer), 0);
            if (mainRxBufferLen > 0)
            {
                // parse mainRxBuffer
                if (mainRxBuffer[1] == 0x00)
                {
                    if (mainRxBufferLen == 5)
                    {
                        uint16_t addr;
                        uint8_t channel;
                        addr = mainRxBuffer[2];
                        addr = addr << 8 | mainRxBuffer[3];
                        channel = mainRxBuffer[4];
                        SetLoraConf(addr, channel);
                        uint8_t data = 0x00;
                        USB_Transmit(CMD_82_00, &data, 1);
                    }
                    else
                    {
                        uint8_t data = 0x01;
                        USB_Transmit(CMD_82_00, &data, 1);
                    }
                }
            }
            loraRxBufferLen = xMessageBufferReceive(loraToMain, loraRxBuffer, sizeof(loraRxBuffer), 0);
            if (loraRxBufferLen > 0)
            {
                // parse loraRxBuffer
                // dbugLen = snprintf((char *)dbug, sizeof(dbug), "Receive: %ld\n", loraRxBufferLen);
                // HAL_UART_Transmit_DMA(boardUARTPtr, dbug, dbugLen);
                SetRTKBaseData(loraRxBuffer, loraRxBufferLen);
            }
        }

        osDelay(10);
    }
}
}