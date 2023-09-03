#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <string.h>
#include "pin_definitions.h"

#define DEBUG_PRINT 1

void vTCPConnectionTask(void *pvParameters);
void xIOReaderTask(void *pvParameters);
void CalculateChecksum(uint8_t *dataPointer, uint8_t size);
void SerialDebugPrint(int number);
void SerialDebugPrint(const char *text);

const int test_pin = BATTERY_LED_RED;
const int pins[PIN_COUNT] = {JOYSTICK_X, JOYSTICK_Y, DRIVE_SPEED, BRUSH_SPEED, BATTERY_VOLTAGE, WATER_BUTTON, LAMP_BUTTON, START_BUTTON, PASS_BUTTON, BRUSH_FRONT_CW, BRUSH_FRONT_CCW, BRUSH_REAR_CW, BRUSH_REAR_CCW, EMERGENCY_STOP_BUTTON, WARNING_LED_RED, WARNING_LED_GREEN, BATTERY_LED_RED, BATTERY_LED_GREEN};
int pin_values[PIN_COUNT];
int old_pin_values[PIN_COUNT];

TaskHandle_t xIOReaderTaskHandle = NULL;
TaskHandle_t xTCPConnectionTaskHandle = NULL;

SemaphoreHandle_t xSerialPrintSemaphore;
SemaphoreHandle_t xIOControlSemaphore;
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

typedef struct
{
	uint8_t headerT;
	uint8_t headerC;
	uint8_t headerP;
	uint8_t joystickX;
	uint8_t joystickY;
    uint8_t driveSpeed;
    uint8_t brushSpeed;
    uint8_t batteryVoltage;
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
    union
	{
		uint8_t leds;
		struct
		{
			uint8_t warningLedRed   :1;
            uint8_t warningLedGreen :1;
            uint8_t batteryLedRed   :1;
            uint8_t batteryLedGreen :1;
            uint8_t                 :4;
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

    TCPConnectionState = CONNECTING_TO_WIFI;
    digitalWrite(WARNING_LED_RED, HIGH);
    digitalWrite(WARNING_LED_GREEN, LOW);

    Serial.begin(9600);

    xSerialPrintSemaphore = xSemaphoreCreateMutex();
    xIOControlSemaphore = xSemaphoreCreateMutex();
    xTCPMessageUpdateSemaphore = xSemaphoreCreateMutex();

    TCPMessageTimer = xTimerCreate("TCPMessageTimer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, TCPMessageTimerCallback);
    if (TCPMessageTimer == NULL)
    {
        SerialDebugPrint("Timer Creation Error!");
        while(1)
        {
            /* Error! */
        }
    }

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
    /*
    for (int i = 0; i < PIN_COUNT; i++)
    {
        if (i < 4)
        {
            pin_values[i] = analogRead(pins[i]);
        }
        else
        {
            pin_values[i] = digitalRead(pins[i]);
        }
    }

    for (int i = 4; i < PIN_COUNT; i++)
    {
        if (old_pin_values[i] != pin_values[i])
        {
            Serial.println("Pin number : " + String(pins[i]) + " " + "Pin val : " + String(pin_values[i]));
            old_pin_values[i] = pin_values[i];
        }
    }
    Serial.println("Joystick X :  " + String(pin_values[0]));
    Serial.println("Joystick Y :  " + String(pin_values[1]));
    Serial.println("BRUSH SPEED :  " + String(pin_values[2]));
    Serial.println("DRIVE SPEED :  " + String(pin_values[3]));
    delay(1000);
    */
}

void xIOReaderTask(void *pvParameters)
{
    configASSERT(((uint32_t)pvParameters) == 1);
    TCPMessage_t localTCPMessage = {.headerT = 'T', .headerC = 'C', .headerP = 'P'};
    for (;;)
    {
        /* Analog Inputs */
        localTCPMessage.joystickX        = analogRead(JOYSTICK_X) / 16;
        localTCPMessage.joystickY        = analogRead(JOYSTICK_Y) / 16;
        localTCPMessage.driveSpeed       = analogRead(DRIVE_SPEED) / 16;
        localTCPMessage.brushSpeed       = analogRead(BRUSH_SPEED) / 16;
        localTCPMessage.batteryVoltage   = analogRead(BATTERY_VOLTAGE) / 16;

        /* Digital Inputs */
        localTCPMessage.waterButton      = digitalRead(WATER_BUTTON);
        localTCPMessage.lampButton       = digitalRead(LAMP_BUTTON);
        localTCPMessage.startButton      = digitalRead(START_BUTTON);
        localTCPMessage.passButton       = digitalRead(PASS_BUTTON);
        localTCPMessage.brushFrontCW     = digitalRead(BRUSH_FRONT_CW);
        localTCPMessage.brushFrontCCW    = digitalRead(BRUSH_FRONT_CCW);
        localTCPMessage.brushRearCW      = digitalRead(BRUSH_REAR_CW);
        localTCPMessage.brushRearCCW     = digitalRead(BRUSH_REAR_CCW);
        localTCPMessage.emergencyButton  = digitalRead(EMERGENCY_STOP_BUTTON);

        /* Digital Outputs */
        xSemaphoreTake(xIOControlSemaphore, portMAX_DELAY);   
        localTCPMessage.warningLedRed    = digitalRead(WARNING_LED_RED);
        localTCPMessage.warningLedGreen  = digitalRead(WARNING_LED_GREEN);
        localTCPMessage.batteryLedRed    = digitalRead(BATTERY_LED_RED);
        localTCPMessage.batteryLedGreen  = digitalRead(BATTERY_LED_GREEN);
        xSemaphoreGive(xIOControlSemaphore);

        /* Updating TCPMessage */
        xSemaphoreTake(xTCPMessageUpdateSemaphore, portMAX_DELAY);   
        memcpy((uint8_t*)&TCPMessage, (uint8_t*)&localTCPMessage, sizeof(TCPMessage_t));
        xSemaphoreGive(xTCPMessageUpdateSemaphore);

        taskYIELD();
    }
}

void vTCPConnectionTask(void *pvParameters)
{
    configASSERT(((uint32_t)pvParameters) == 1);

    /* 
        Assigning static IP because TP-Link AP does not have DHCP
        IPAddress localIP(192, 168, 0, 201);
        IPAddress subnet(255, 255, 0, 0);

        /* Subnet and IP is enough
        IPAddress primaryDNS(0, 0, 0, 0);
        IPAddress secondaryDNS(0, 0, 0, 0);
        IPAddress gateway(0, 0, 0, 0);

        if (!WiFi.config(localIP, gateway, subnet, primaryDNS, secondaryDNS))
        {
            SerialDebugPrint("Failed to IP and subnet");
            while (1)
            {
                vTaskDelay(10000);
            }
        }
    */

    for (;;)
    {
        switch (TCPConnectionState)
        {
            case CONNECTING_TO_WIFI:
            {
                wl_status_t wifiBeginStatus = WiFi.begin(ssid, password);
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(1000);
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
                    xSemaphoreTake(xIOControlSemaphore, portMAX_DELAY);   
                    digitalWrite(WARNING_LED_GREEN, HIGH);
                    digitalWrite(WARNING_LED_RED, LOW);
                    xSemaphoreGive(xIOControlSemaphore);
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

                    int tcpBufferCount = client.available();
                    if (tcpBufferCount > 0)
                    {
                        if (xTimerReset(TCPMessageTimer, 10) != pdPASS) 
                        {
                            SerialDebugPrint("Failed to reset timer!");
                        }
                        for (int i = 0; i < tcpBufferCount; i++)
                        {
                            int discardData = client.read();
                        }
                    }

                    xSemaphoreTake(xTCPMessageUpdateSemaphore, portMAX_DELAY);   
                    client.write((char *)&TCPMessage, sizeof(TCPMessage_t));
                    xSemaphoreGive(xTCPMessageUpdateSemaphore);
                    delay(100);
                }

                if (WiFi.status() != WL_CONNECTED)
                {
                    WiFi.disconnect();
                    close(client.fd());
                    client.stop();
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
                xSemaphoreTake(xIOControlSemaphore, portMAX_DELAY);   
                digitalWrite(WARNING_LED_GREEN, LOW);
                digitalWrite(WARNING_LED_RED, HIGH);
                xSemaphoreGive(xIOControlSemaphore);
                break;
            }
            default:
            {
                break;
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
    dataPointer[size - 1] == checksum;
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