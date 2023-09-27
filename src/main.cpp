#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <string.h>
#include "pin_definitions.h"

#define DEBUG_PRINT         1
#define START_STOP_COUNTER  150 /* 150*20 ms => 3 second */
#define BATERRY_FULL_VALUE  0xCE /* Approximately 8V on a 6V battery */
#define BATERRY_LOW_VALUE   0x8C /* Approximately 5.4V on a 6V battery */
#define ACK_MESSAGE         "ACK"

void vTCPConnectionTask(void *pvParameters);
void xIOReaderTask(void *pvParameters);
void CalculateChecksum(uint8_t *dataPointer, uint8_t size);
void SerialDebugPrint(int number);
void SerialDebugPrint(const char *text);
void SetTCPMessageDefault(void);

const int test_pin = BATTERY_LED_RED;
const int pins[PIN_COUNT] = {JOYSTICK_X, JOYSTICK_Y, DRIVE_SPEED, BRUSH_SPEED, BATTERY_VOLTAGE, WATER_BUTTON, LAMP_BUTTON, START_BUTTON, PASS_BUTTON, BRUSH_FRONT_CW, BRUSH_FRONT_CCW, BRUSH_REAR_CW, BRUSH_REAR_CCW, EMERGENCY_STOP_BUTTON, WARNING_LED_RED, WARNING_LED_GREEN, BATTERY_LED_RED, BATTERY_LED_GREEN};
int pin_values[PIN_COUNT];
int old_pin_values[PIN_COUNT];

TaskHandle_t xIOReaderTaskHandle = NULL;
TaskHandle_t xTCPConnectionTaskHandle = NULL;

SemaphoreHandle_t xSerialPrintSemaphore;
SemaphoreHandle_t xTCPMessageUpdateSemaphore;
QueueHandle_t xTCPMessageQueue;

typedef enum
{
	CONNECTING_TO_WIFI,
	CONNECTING_TO_TCP_SERVER,
	CONNECTION_ESTABLISHED
} TCPConnectionState_t;
TCPConnectionState_t TCPConnectionState;

WiFiClient client;

const char *ssid = "ESP32-AP";
const char *password = "88888888";
const char *ip = "192.168.31.2";
const uint16_t port = 3131;
const int TCPConnectTimeout = 1000;

TimerHandle_t TCPMessageTimer;
bool TCPMessageTimeout = false;
bool remoteStarted = false;

typedef struct
{
	uint8_t headerT;
	uint8_t headerC;
	uint8_t headerP;
	uint8_t joystickX;
	uint8_t joystickY;
    uint8_t driveSpeed;
    uint8_t brushSpeed;
	union
	{
		uint8_t buttons;
		struct
		{
			uint8_t waterButton     :1;
            uint8_t lampButton      :1;
			uint8_t startButton     :1;
			uint8_t passButton      :1;
			uint8_t brushFrontCW    :1;
			uint8_t brushFrontCCW   :1;
			uint8_t brushRearCW     :1;
            uint8_t brushRearCCW    :1;
		};
	};
    union
	{
		uint8_t data;
		struct
		{
			uint8_t emergencyButton :1;
            uint8_t                 :7;
		};
	};
	uint8_t checksum;
}TCPMessage_t;
TCPMessage_t TCPMessage;

void TCPMessageTimerCallback(TimerHandle_t xTimer)
{
    TCPMessageTimeout = true;
}

void setup()
{
    Serial.begin(115200);
    xSerialPrintSemaphore = xSemaphoreCreateMutex();
    xTCPMessageUpdateSemaphore = xSemaphoreCreateMutex();
    SerialDebugPrint("Serial Begin");
    for (int i = 0; i < PIN_COUNT; i++)
    {
        if (i < 5)
        {
            pinMode(pins[i], INPUT);
        }
        else if (i >= 5 && i < 14)
        {
            pinMode(pins[i], INPUT_PULLDOWN);
        }
        else
        {
            pinMode(pins[i], OUTPUT);
        }
    }
    SerialDebugPrint("Pin mode set");
    TCPConnectionState = CONNECTING_TO_WIFI;
    digitalWrite(WARNING_LED_RED, HIGH);
    digitalWrite(WARNING_LED_GREEN, LOW);
    SerialDebugPrint("xSerialPrintSemaphore Created");
    SerialDebugPrint("xTCPMessageUpdateSemaphore Created");
    SetTCPMessageDefault();
    SerialDebugPrint("TCP Message Default");
    TCPMessageTimer = xTimerCreate("TCPMessageTimer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, TCPMessageTimerCallback);
    if (TCPMessageTimer == NULL)
    {
        SerialDebugPrint("Timer Creation Error!");
        while(1)
        {
            /* Error! */
        }
    }
    SerialDebugPrint("TCPMessageTimer Created");
    BaseType_t xReturned;
    xReturned = xTaskCreate(xIOReaderTask,
                           "xIOReaderTaskHandle",
                           2048,
                           (void *)1,
                           tskIDLE_PRIORITY,
                           &xIOReaderTaskHandle);

    if (xReturned != pdPASS)
    {
        SerialDebugPrint("xIOReaderTask Creation Failed!");
        while (1)
        {
            /* Error */
        }
    }
    else
    {
        SerialDebugPrint("xIOReaderTask Created!");
    }
    xReturned = xTaskCreate(vTCPConnectionTask,
                           "vTCPConnectionTask",
                           4096,
                           (void *)1,
                           tskIDLE_PRIORITY,
                           &xTCPConnectionTaskHandle);

    if (xReturned != pdPASS)
    {
        SerialDebugPrint("vTCPConnectionTask Creation Failed!");
        while (1)
        {
            /* Error */
        }
    }
    else
    {
        SerialDebugPrint("vTCPConnectionTask Created!");
    }
}

void loop()
{
    /* Leave Empty! */
}

void xIOReaderTask(void *pvParameters)
{
    configASSERT(((uint32_t)pvParameters) == 1);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t startButtonTimerCounter = 0;
    uint8_t batteryVoltage = 0;
    TCPMessage_t localTCPMessage = {.headerT = 'T', .headerC = 'C', .headerP = 'P'};
    for (;;)
    {
        /* Read Analog Inputs */
        localTCPMessage.joystickX       = analogRead(JOYSTICK_X) / 16;
        localTCPMessage.joystickY       = analogRead(JOYSTICK_Y) / 16;
        localTCPMessage.driveSpeed      = (4095 - analogRead(DRIVE_SPEED)) / 16; /* Because pins are connected reverse */
        localTCPMessage.brushSpeed      = (4095 - analogRead(BRUSH_SPEED)) / 16; /* Because pins are connected reverse */
        batteryVoltage                  = analogRead(BATTERY_VOLTAGE) / 16;
        
        /* Check the battery value */
        if (batteryVoltage <= BATERRY_LOW_VALUE)
        {
            digitalWrite(BATTERY_LED_RED, HIGH);
            digitalWrite(BATTERY_LED_GREEN, LOW);
        }
        else
        {
            digitalWrite(BATTERY_LED_RED, LOW);
            digitalWrite(BATTERY_LED_GREEN, HIGH);
        }

        /* Read Digital Inputs */
        localTCPMessage.waterButton     = digitalRead(WATER_BUTTON);
        localTCPMessage.lampButton      = digitalRead(LAMP_BUTTON);
        localTCPMessage.passButton      = digitalRead(PASS_BUTTON);
        localTCPMessage.brushFrontCW    = digitalRead(BRUSH_FRONT_CW);
        localTCPMessage.brushFrontCCW   = digitalRead(BRUSH_FRONT_CCW);
        localTCPMessage.brushRearCW     = digitalRead(BRUSH_REAR_CW);
        localTCPMessage.brushRearCCW    = digitalRead(BRUSH_REAR_CCW);
        localTCPMessage.emergencyButton = digitalRead(EMERGENCY_STOP_BUTTON);
        localTCPMessage.startButton     = digitalRead(START_BUTTON);

        /* Check the start button for remote start/stop */
        if (localTCPMessage.startButton == HIGH)
        {
            startButtonTimerCounter++;
            if (startButtonTimerCounter == START_STOP_COUNTER)
            {
                startButtonTimerCounter = 0;
                remoteStarted = !remoteStarted;
            }
        }
        else
        {
            startButtonTimerCounter = 0;
        }

        /* Update TCPMessage */
        xSemaphoreTake(xTCPMessageUpdateSemaphore, portMAX_DELAY);   
        memcpy((uint8_t*)&TCPMessage, (uint8_t*)&localTCPMessage, sizeof(TCPMessage_t));
        xSemaphoreGive(xTCPMessageUpdateSemaphore);

        /* Repeat the task after 20 ms */
        vTaskDelayUntil(&xLastWakeTime, (20/portTICK_RATE_MS));
    }
}

void vTCPConnectionTask(void *pvParameters)
{
    configASSERT(((uint32_t)pvParameters) == 1);
    for (;;)
    {
        if (remoteStarted == true)
        {
            switch (TCPConnectionState)
            {
                case CONNECTING_TO_WIFI:
                {
                    wl_status_t wifiBeginStatus = WiFi.begin(ssid, password);
                    while (WiFi.status() != WL_CONNECTED)
                    {
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        SerialDebugPrint("Connecting to WiFi..");
                    }
                    SerialDebugPrint("Wifi Connected! SSID: ");
                    SerialDebugPrint(ssid);
                    SerialDebugPrint(" - Password: ");
                    SerialDebugPrint(password);
                    TCPConnectionState = CONNECTING_TO_TCP_SERVER;
                    break;
                }
                case CONNECTING_TO_TCP_SERVER:
                {
                    if (client.connect(ip, port, TCPConnectTimeout) == 0)
                    {
                        SerialDebugPrint("Client Connect Fail!");
                        SerialDebugPrint("Error No: ");
                        SerialDebugPrint(errno);
                        SerialDebugPrint(" - Error Text: ");
                        SerialDebugPrint(strerror(errno));
                        if (WiFi.status() != WL_CONNECTED)
                        {
                            WiFi.disconnect();
                            close(client.fd());
                            client.stop();
                            TCPConnectionState = CONNECTING_TO_WIFI;
                        }
                    }
                    else
                    {
                        SerialDebugPrint("Client Connected! IP: ");
                        SerialDebugPrint(ip);
                        SerialDebugPrint(" - Port: ");
                        SerialDebugPrint(port);
                        TCPConnectionState = CONNECTION_ESTABLISHED;
                        digitalWrite(WARNING_LED_GREEN, HIGH);
                        digitalWrite(WARNING_LED_RED, LOW);
                    }
                    break;
                }
                case CONNECTION_ESTABLISHED:
                {
                    if (xTimerReset(TCPMessageTimer, 10) != pdPASS) 
                    {
                        SerialDebugPrint("Failed to start timer!");
                    }
                    while (client.connected())
                    {
                        if (TCPMessageTimeout)
                        {
                            TCPMessageTimeout = false;
                            break;
                        }

                        if (remoteStarted == false)
                        {
                            SetTCPMessageDefault();
                            xSemaphoreTake(xTCPMessageUpdateSemaphore, portMAX_DELAY);   
                            client.write((char *)&TCPMessage, sizeof(TCPMessage_t));
                            xSemaphoreGive(xTCPMessageUpdateSemaphore);
                            break;
                        }

                        int tcpBufferCount = client.available();
                        if (tcpBufferCount > 2)
                        {
                            uint8_t message[3] = {0};
                            client.readBytes(message, 3);
                            if (strncmp((const char*)message, ACK_MESSAGE, 3) == 0)
                            {
                                if (xTimerReset(TCPMessageTimer, 10) != pdPASS) 
                                {
                                    SerialDebugPrint("Failed to reset timer!");
                                }
                            }
                            else
                            {
                                SerialDebugPrint("Not ACK!!");
                            }
                        }
                        CalculateChecksum((uint8_t*)&TCPMessage, sizeof(TCPMessage_t));
                        xSemaphoreTake(xTCPMessageUpdateSemaphore, portMAX_DELAY);   
                        client.write((char *)&TCPMessage, sizeof(TCPMessage_t));
                        xSemaphoreGive(xTCPMessageUpdateSemaphore);
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }

                    if ((WiFi.status() != WL_CONNECTED) || (remoteStarted == false))
                    {
                        close(client.fd());
                        client.stop();
                        WiFi.disconnect();
                        TCPConnectionState = CONNECTING_TO_WIFI;
                    }
                    else
                    {
                        close(client.fd());
                        client.stop();
                        TCPConnectionState = CONNECTING_TO_TCP_SERVER;
                    }
                    
                    if (xTimerStop(TCPMessageTimer, 10) != pdPASS) 
                    {
                        SerialDebugPrint("Failed to stop timer!");
                    }
                    SerialDebugPrint("Reconnection...");
                    digitalWrite(WARNING_LED_GREEN, LOW);
                    digitalWrite(WARNING_LED_RED, HIGH);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        taskYIELD();
    }
}

void CalculateChecksum(uint8_t *dataPointer, uint8_t size)
{
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < size - 1; i++)
	{
		checksum ^= dataPointer[i];
	}
	checksum ^= 255;
    dataPointer[size - 1] = checksum;
}

void SetTCPMessageDefault()
{
    TCPMessage_t localTCPMessage = 
    {
        .headerT    = 'T', 
        .headerC    = 'C', 
        .headerP    = 'P',
        .joystickX  = 128,
        .joystickY  = 128,
        .driveSpeed = 0,
        .brushSpeed = 0,
        .buttons    = 0,
        .data       = 0,
        .checksum   = 184
    };
    xSemaphoreTake(xTCPMessageUpdateSemaphore, portMAX_DELAY);   
    memcpy((uint8_t*)&TCPMessage, (uint8_t*)&localTCPMessage, sizeof(TCPMessage_t));
    xSemaphoreGive(xTCPMessageUpdateSemaphore);
}

void SerialDebugPrint(const char *text)
{
#if DEBUG_PRINT
	xSemaphoreTake(xSerialPrintSemaphore, portMAX_DELAY);   
	Serial.println(text);
	xSemaphoreGive(xSerialPrintSemaphore);
#endif
}

void SerialDebugPrint(int number)
{
#if DEBUG_PRINT
	xSemaphoreTake(xSerialPrintSemaphore, portMAX_DELAY);   
	Serial.println(number);
	xSemaphoreGive(xSerialPrintSemaphore);
#endif
}