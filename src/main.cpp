#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "esp_camera.h"
#include "Freenove_4WD_Car_WiFi.h"
#include "Freenove_4WD_Car_Emotion.h"
#include "Freenove_4WD_Car_WS2812.h"
#include "Freenove_4WD_Car_For_ESP32.h"
#include <PubSubClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

const char *ssid_wifi = "ReseauPasMasquer";    // WiFi SSID
const char *password_wifi = "4xrkhkdsuuu3s4i"; // WiFi Password
const char *mqtt_server = "192.168.109.87";    // MQTT Broker IP
const int mqtt_interval_ms = 5000;             // Interval in ms for MQTT data transmission

const IPAddress localIP(192, 168, 109, 50);       // Car IP
const IPAddress localGateway(192, 168, 109, 136); // Gateway IP
const IPAddress localSubnet(255, 255, 255, 0);    // Subnet Mask
const IPAddress primaryDNS(8, 8, 8, 8);           // Primary DNS
const IPAddress secondaryDNS(8, 8, 4, 4);         // Secondary DNS

AsyncWebServer server(80);
AsyncWebSocket ws("/carwebsocket"); // WebSocket endpoint

WiFiClient espClient;
PubSubClient client(espClient);
WiFiServer server_Camera(7000);

bool videoFlag = false;
long last_message = 0;

char battery_buffer[6];     // Buffer to store battery voltage data
char ultrasonic_buffer[10]; // Buffer to store ultrasonic data

// Function declarations
void WiFi_Init();
void loopTask_Camera(void *pvParameters);
void notifyClients();
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSocket();
void reconnect();

void WiFi_Init()
{
    ssid_Router = ssid_wifi;
    password_Router = password_wifi;
    ssid_AP = "Sunshine";       // AP SSID
    password_AP = "Sunshine";   // AP Password
    frame_size = FRAMESIZE_CIF; // Frame size for the camera
}

void setup()
{
    delay(5000);

    Serial.begin(115200);
    Serial.setDebugOutput(true);

    if (!WiFi.config(localIP, localGateway, localSubnet, primaryDNS, secondaryDNS))
    {
        Serial.println("STA Failed to configure");
    }

    Buzzer_Setup();
    WiFi_Init();
    WiFi_Setup(0); // Start AP Mode

    server_Camera.begin(7000); // Start camera server

    cameraSetup();
    camera_vflip(true);
    camera_hmirror(true);
    Emotion_Setup();
    WS2812_Setup();
    PCA9685_Setup();
    Light_Setup();
    Track_Setup();
    Ultrasonic_Setup();

    disableCore0WDT(); // Disable watchdog on core 0
    xTaskCreateUniversal(loopTask_Camera, "loopTask_Camera", 8192, NULL, 0, NULL, 0);
    xTaskCreateUniversal(loopTask_WTD, "loopTask_WTD", 8192, NULL, 0, NULL, 0);

    client.setServer(mqtt_server, 1883);

    initWebSocket();
    server.begin();

    // Initialize car state
    Emotion_SetMode(1);
    WS2812_SetMode(1);
}

void loop()
{
    ws.cleanupClients();

    Emotion_Show(emotion_task_mode);
    WS2812_Show(ws2812_task_mode);

    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    long now = millis();
    if (now - last_message > mqtt_interval_ms)
    {
        last_message = now;

        // Battery level
        // dtostrf(Get_Battery_Voltage(), 5, 2, battery_buffer);
        // client.publish("esp32/battery", battery_buffer);

        // Track sensor data
        Track_Read();
        int sensor_v = static_cast<int>(sensorValue[3]);
        client.publish("esp32/track", String(sensor_v).c_str());

        // Ultrasonic data
        dtostrf(Get_Sonar(), 5, 2, ultrasonic_buffer);
        client.publish("esp32/sonar", ultrasonic_buffer);

        // Photosensitive data
        dtostrf(Get_Photosensitive(), 5, 2, ultrasonic_buffer);
        client.publish("esp32/light", ultrasonic_buffer);
    }
}

void notifyClients()
{
    ws.textAll("ok");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;

    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
        data[len] = 0;

        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, (char *)data);

        if (error)
        {
            Serial.printf("deserializeJson() failed: %s\n", error.c_str());
            return;
        }

        int cmd = doc["cmd"];

        switch (cmd)
        {
        case 1:
        {
            JsonArray dataArray = doc["data"];
            Motor_Move(dataArray[0], dataArray[1], dataArray[2], dataArray[3]);
            break;
        }
        case 2:
            Emotion_SetMode(doc["data"]);
            break;
        case 3:
        {
            JsonArray angles = doc["data"];
            Servo_1_Angle(angles[0]);
            Servo_2_Angle(angles[1]);
            break;
        }
        case 4:
            WS2812_SetMode(doc["data"]);
            break;
        case 5:
        {
            JsonArray led_color = doc["data"];
            WS2812_Set_Color_1(led_color[0], led_color[1], led_color[2], led_color[3]);
            break;
        }
        case 6:
        {
            JsonArray led_color_2 = doc["data"];
            WS2812_Set_Color_2(led_color_2[0], led_color_2[1], led_color_2[2], led_color_2[3]);
            break;
        }
        case 7:
            Buzzer_Alarm(doc["data"] == 1);
            break;
        case 8:
        {
            JsonArray buzzer_data = doc["data"];
            Buzzer_Variable(buzzer_data[0] == 1, buzzer_data[1]);
            break;
        }
        case 9:
            videoFlag = doc["data"] == 1;
            break;
        default:
            Serial.println("Unknown command received");
            break;
        }

        notifyClients();
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
    case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    }
}

void initWebSocket()
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP8266Client"))
        {
            Serial.println("connected");
            client.subscribe("esp32/output");
        }
        else
        {
            Serial.printf("failed, rc=%d. Try again in 5 seconds\n", client.state());
            delay(5000);
        }
    }
}

void loopTask_Camera(void *pvParameters)
{
    while (1)
    {
        WiFiClient wf_client = server_Camera.available();
        if (wf_client)
        {
            Serial.println("Camera_Server connected to a client.");
            if (wf_client.connected())
            {
                char size_buf[12];
                camera_fb_t *fb = NULL;
                wf_client.print("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n");

                while (wf_client.connected())
                {
                    if (videoFlag)
                    {
                        fb = esp_camera_fb_get();
                        if (fb)
                        {
                            wf_client.printf("\r\n--123456789000000000000987654321\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", fb->len);
                            wf_client.write(fb->buf, fb->len);
                            esp_camera_fb_return(fb);
                        }
                    }
                }

                wf_client.stop();
                Serial.println("Camera Client Disconnected.");
            }
        }
    }
}
