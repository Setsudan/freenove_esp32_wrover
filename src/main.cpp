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
#include <limits.h>
#include <Melody.h>

#define STREAM_CONTENT_BOUNDARY "123456789000000000000987654321"

char *ssid_wifi = "EthLny-TestWifi";
char *password_wifi = "TrulyAnAmazingPassword9562!";

const char *mqtt_server = "192.168.137.1";
const int mqtt_interval_ms = 5000;

const char *mqtt_server = "192.168.64.40"; // L'IP de votre broker MQTT
const int mqtt_interval_ms = 5000;         // L'interval en ms entre deux envois de données

IPAddress localIP(192, 168, 64, 50); // l'IP que vous voulez donner à votre voiture

IPAddress localGateway(192, 168, 64, 136); // L'IP de la gateway de votre réseau
IPAddress localSubnet(255, 255, 255, 0);   // Le masque de sous réseau

IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

AsyncWebServer server(80);
AsyncWebSocket ws("/carwebsocket");

WiFiClient espClient;
PubSubClient client(espClient);

WiFiServer server_Camera(7000);
bool videoFlag = 0;

long last_message = 0;

int distance[4];
int sensor_v;
char buff[6];
char ultrasonic_buff[10];

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
    ssid_AP = "Sunshine";
    password_AP = "Sunshine";
    frame_size = FRAMESIZE_CIF;
}

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    if (!WiFi.config(localIP, localGateway, localSubnet, primaryDNS, secondaryDNS))
    {
        Serial.println("STA Failed to configure");
    }

    Buzzer_Setup();
    WiFi_Init();
    WiFi_Setup(0);
    server_Camera.begin(7000);

    cameraSetup();
    camera_vflip(true);
    camera_hmirror(true);
    Emotion_Setup();
    WS2812_Setup();
    PCA9685_Setup();
    Light_Setup();
    Track_Setup();
    Ultrasonic_Setup();

    disableCore0WDT();
    xTaskCreateUniversal(loopTask_Camera, "loopTask_Camera", 8192, NULL, 0, NULL, 0);
    xTaskCreateUniversal(loopTask_WTD, "loopTask_WTD", 8192, NULL, 0, NULL, 0);

    client.setServer(mqtt_server, 1883);

    initWebSocket();

    server.begin();

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

        Track_Read();
        sensor_v = static_cast<int>(sensorValue[3]);
        char const *n_char = std::to_string(sensor_v).c_str();
        client.publish("esp32/track", n_char);

        dtostrf(Get_Sonar(), 5, 2, ultrasonic_buff);
        client.publish("esp32/sonar", ultrasonic_buff);

        dtostrf(Get_Photosensitive(), 5, 2, ultrasonic_buff);
        client.publish("esp32/light", ultrasonic_buff);
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
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            return;
        }

        int cmd = doc["cmd"];

        if (1 == cmd)
        {
            JsonArray data = doc["data"];
            int data_0 = data[0];
            int data_1 = data[1];
            int data_2 = data[2];
            int data_3 = data[3];

            Motor_Move(data_0, data_1, data_2, data_3);
        }
        else if (2 == cmd)
        {
            int data = doc["data"];
            Emotion_SetMode(data);
        }
        else if (3 == cmd)
        {
            JsonArray angles = doc["data"];
            int angle_0 = angles[0];
            int angle_1 = angles[1];
            Servo_1_Angle(angle_0);
            Servo_2_Angle(angle_1);
        }
        else if (4 == cmd)
        {
            int led_mode = doc["data"];
            WS2812_SetMode(led_mode);
        }
        else if (5 == cmd)
        {
            JsonArray led_color = doc["data"];
            int led_color_0 = led_color[0];
            int led_color_1 = led_color[1];
            int led_color_2 = led_color[2];
            int led_color_3 = led_color[3];

            WS2812_Set_Color_1(led_color_0, led_color_1, led_color_2, led_color_3);
        }
        else if (6 == cmd)
        {
            JsonArray led_color_2 = doc["data"];
            int led_color_2_0 = led_color_2[0];
            int led_color_2_1 = led_color_2[1];
            int led_color_2_2 = led_color_2[2];
            int led_color_2_3 = led_color_2[3];

            WS2812_Set_Color_2(led_color_2_0, led_color_2_1, led_color_2_2, led_color_2_3);
        }
        else if (7 == cmd)
        {
            bool alarm = doc["data"] == 1;
            playRickRoll();
        }
        else if (8 == cmd)
        {
            JsonArray buzzer_data = doc["data"];
            int alarm_on = buzzer_data[0] == 1;
            int frequency_hz = buzzer_data[1];
            Buzzer_Variable(alarm_on, frequency_hz);
        }
        else if (9 == cmd)
        {
            bool video_activation = doc["data"] == 1;
            videoFlag = video_activation;
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
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void loopTask_Camera(void *pvParameters)
{
    while (1)
    {
        char size_buf[12];
        WiFiClient wf_client = server_Camera.available();
        if (wf_client)
        {
            Serial.println("Camera_Server connected to a client.");
            if (wf_client.connected())
            {
                camera_fb_t *fb = NULL;
                wf_client.write("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: multipart/x-mixed-replace; boundary=" STREAM_CONTENT_BOUNDARY "\r\n");
                while (wf_client.connected())
                {
                    if (videoFlag == 1)
                    {
                        fb = esp_camera_fb_get();
                        if (fb != NULL)
                        {
                            wf_client.write("\r\n--" STREAM_CONTENT_BOUNDARY "\r\n");
                            wf_client.write("Content-Type: image/jpeg\r\nContent-Length: ");
                            sprintf(size_buf, "%d\r\n\r\n", fb->len);
                            wf_client.write(size_buf);
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
