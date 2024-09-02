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
#include <SPIFFS.h>
#include <functional>

#define STREAM_CONTENT_BOUNDARY "123456789000000000000987654321"

// Ressources
// https://randomnerdtutorials.com/esp32-websocket-server-arduino/#1
// https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
// https://randomnerdtutorials.com/esp32-static-fixed-ip-address-arduino-ide/
// https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32/tree/master


// Variables pour le WiFi et MQTT
char ssid_wifi[32];            // Le nom du réseau WiFi
char password_wifi[32];        // Le password du WiFi

char mqtt_server[32]; // L'IP de votre broker MQTT
int mqtt_port;
int mqtt_interval_ms = 5000;       // L'interval en ms entre deux envois de données

IPAddress localIP;
IPAddress localGateway;
IPAddress localSubnet;
IPAddress primaryDNS;
IPAddress secondaryDNS;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // Changez le nom de ce point d'accès pour "sécuriser" l'accès à votre voiture

WiFiClient espClient;
PubSubClient client(espClient);

// WiFiServer server_Cmd(4000);
WiFiServer server_Camera(7000);
bool videoFlag = 0;

long last_message = 0;

int distance[4];          // Storage of ultrasonic data
int sensor_v;             // Int cast of track sensor data
char buff[6];             // Buffer to store the battery voltage data
char ultrasonic_buff[10]; // Buffer to store the Ultrasonic data
char distance_buff[10];   // Buffer to store the distance
char speed_buff[10];      // Buffer to store the speed
char  race_id_buffer[10];  // buffer to store the rice_id

// Variables for the timer
unsigned long startTime = 0;
bool timerActive = false;

// variable du calcul de distance 
int data_total_0 = 0;
int data_total_1 = 0;
int data_total_2 = 0;
int data_total_3 = 0;
float total_Distance = 0.0;

// Variable vitesse 
float total_speed = 0.0;

// race variable
 int race_id = 0;
 bool race_change = false;

// put function declarations here:
void WiFi_Init();
void loopTask_Camera(void *pvParameters);
void notifyClients();
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSocket();
void reconnect();
void testDNS();
void testInternetConnectivity();

void WiFi_Init()
{
    ssid_Router = ssid_wifi;         // Modify according to your router name
    password_Router = password_wifi; // Modify according to your router password
    ssid_AP = "Sunshine";            // ESP32 turns on an AP and calls it Sunshine
    password_AP = "Sunshine";        // Set your AP password for ESP32 to Sunshine
    frame_size = FRAMESIZE_CIF;      // 400*296
}

void processCommand(const char* jsonData) {
    StaticJsonDocument<200> doc;

    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }

    int cmd = doc["cmd"];

    if (1 == cmd) {
        JsonArray data = doc["data"];
        int data_0 = data[0];
        int data_1 = data[1];
        int data_2 = data[2];
        int data_3 = data[3];

        Motor_Move(data_0, data_1, data_2, data_3);

        if (data_0 == 0 && data_1 == 0 && data_2 == 0 && data_3 == 0) {
            // Stop the timer
            startTime = 0;
            timerActive = false;
        } else {    
            // Start the timer
            startTime = millis();
            timerActive = true;
            // initialize wheelSpeed variable
            data_total_0 = data_0;
            data_total_1 = data_1;
            data_total_2 = data_2;
            data_total_3 = data_3;
        }

    } else if (2 == cmd) {
        int data = doc["data"];
        Emotion_SetMode(data);
    } else if (3 == cmd) {
        JsonArray angles = doc["data"];
        int angle_0 = angles[0];
        int angle_1 = angles[1];
        Servo_1_Angle(angle_0); // Set the Angle value of servo 1 to 0 to 180°
        Servo_2_Angle(angle_1);
    } else if (4 == cmd) {
        int led_mode = doc["data"];
        WS2812_SetMode(led_mode);
    } else if (5 == cmd) {
        JsonArray led_color = doc["data"];
        int led_color_0 = led_color[0];
        int led_color_1 = led_color[1];
        int led_color_2 = led_color[2];
        int led_color_3 = led_color[3];

        WS2812_Set_Color_1(led_color_0, led_color_1, led_color_2, led_color_3);
    } else if (6 == cmd) {
        JsonArray led_color_2 = doc["data"];
        int led_color_2_0 = led_color_2[0];
        int led_color_2_1 = led_color_2[1];
        int led_color_2_2 = led_color_2[2];
        int led_color_2_3 = led_color_2[3];

        WS2812_Set_Color_2(led_color_2_0, led_color_2_1, led_color_2_2, led_color_2_3);
    } else if (7 == cmd) {
        bool alarm = doc["data"] == 1;
        Buzzer_Alarm(alarm);
    } else if (8 == cmd) {
        JsonArray buzzer_data = doc["data"];
        int alarm_on = buzzer_data[0] == 1;
        int frequency_hz = buzzer_data[1];
        Buzzer_Variable(alarm_on, frequency_hz);
    } else if (9 == cmd) {
        bool video_activation = doc["data"] == 1;
        videoFlag = video_activation;
    } else if (10 == cmd) {
        race_id = doc["data"];
        if (race_id == 0)
        {
            race_change = false;
        }else{
            race_change = true;
        }   
        
    } else if (11 == cmd) {
        int mode = doc["data"];
         while (mode == 1) {  // Continuer à lire les capteurs tant que le mode suivi de ligne est activé
            Track_Car(1);

            // Ajoutez un petit délai pour éviter de saturer le processeur
            delay(10);

            // Vous pouvez ajouter une condition pour sortir de la boucle, comme une nouvelle commande ou un autre critère
            if (mode == 0) {
                break;
                Motor_Move(0,0,0,0);
            }
        }
    }

    notifyClients();
}

void handleMQTTMessage(char* topic, uint8_t* payload, unsigned int length) {
    payload[length] = '\0';  // Assure la terminaison de la chaîne
    processCommand((char*)payload);
}

void setup()
{
    // delay(5000);

    Serial.begin(115200);
    Serial.setDebugOutput(true);

    // le fichier spiffs
    if (!SPIFFS.begin(true)) {
        Serial.println("An error occurred while mounting SPIFFS");
        return;
    }

    // Ouvrir et lire le fichier .env
    File envFile = SPIFFS.open("/.env", "r");
    if (!envFile) {
        Serial.println("Failed to open .env file for reading");
        return;
    }

    while (envFile.available()) {
        String line = envFile.readStringUntil('\n');
        line.trim(); // Enlever les espaces en début et fin de ligne

        if (line.startsWith("WIFI_SSID=")) {
            strlcpy(ssid_wifi, line.substring(10).c_str(), sizeof(ssid_wifi));

        } else if (line.startsWith("WIFI_PASSWORD=")) {
            strlcpy(password_wifi, line.substring(14).c_str(), sizeof(password_wifi));

        } else if (line.startsWith("MQTT_SERVER=")) {
            strlcpy(mqtt_server, line.substring(12).c_str(), sizeof(mqtt_server));

        } else if (line.startsWith("MQTT_PORT=")) {
            mqtt_port = line.substring(10).toInt();

        } else if (line.startsWith("LOCAL_IP=")) {
            localIP.fromString(line.substring(9));

        } else if (line.startsWith("LOCAL_GATEWAY=")) {
            localGateway.fromString(line.substring(14));

        } else if (line.startsWith("LOCAL_SUBNET=")) {
            localSubnet.fromString(line.substring(13));

        } else if (line.startsWith("PRIMARY_DNS=")) {
            primaryDNS.fromString(line.substring(12));

        } else if (line.startsWith("SECONDARY_DNS=")) {
            secondaryDNS.fromString(line.substring(14));
        }
    }

    envFile.close();

    if (!WiFi.config(localIP, localGateway, localSubnet, primaryDNS, secondaryDNS))
    {
        Serial.println("STA Failed to configure");
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_wifi, password_wifi);

    // Attendre la connexion WiFi
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connexion WiFi en cours...");
    }

    Serial.println("WiFi connecté");
    Serial.print("Adresse IP: ");
    Serial.println(WiFi.localIP());

    Buzzer_Setup(); // Buzzer initialization
    WiFi_Init();    // WiFi paramters initialization
    WiFi_Setup(0);  // Start AP Mode. If you want to connect to a router, change 1 to 0.
    // server_Cmd.begin(4000);    // Start the command server
    server_Camera.begin(7000); // Turn on the camera server

    cameraSetup(); // Camera initialization
    camera_vflip(true);
    camera_hmirror(true);
    Emotion_Setup();    // Emotion initialization
    WS2812_Setup();     // WS2812 initialization
    PCA9685_Setup();    // PCA9685 initialization
    Light_Setup();      // Light initialization
    Track_Setup();      // Track initialization
    Ultrasonic_Setup(); // Initialize the ultrasonic module

    // Cette section serait peut être à virer...
    disableCore0WDT(); // Turn off the watchdog function in kernel 0
    xTaskCreateUniversal(loopTask_Camera, "loopTask_Camera", 8192, NULL, 0, NULL, 0);
    xTaskCreateUniversal(loopTask_WTD, "loopTask_WTD", 8192, NULL, 0, NULL, 0);

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(handleMQTTMessage);

    initWebSocket();

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { 
                camera_fb_t *fb = NULL;
                fb = esp_camera_fb_get();
                        if (fb != NULL)
                        {
                            uint8_t slen[4];
                            slen[0] = fb->len >> 0;
                            slen[1] = fb->len >> 8;
                            slen[2] = fb->len >> 16;
                            slen[3] = fb->len >> 24;
                            AsyncResponseStream *response = request->beginResponseStream("image");
                            // response->write(slen, 4);
                            response->write(fb->buf, fb->len);
                            request->send(response);
                            // client.write(slen, 4);
                            // client.write(fb->buf, fb->len);
                            // request->send_P(200, "application/octet-stream", fb->buf, fb->len);
                            // request->send(fb->buf, "application/octet-stream", fb->len);
                            // Serial.println("Camera send");
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } });

    server.begin();

    // Init the state of the car
    Emotion_SetMode(1);
    WS2812_SetMode(1);

    testDNS();
    testInternetConnectivity();
}

// Fonction pour calculer la distance
void updateDistance(unsigned long duration)
{
    // Vérifier si toutes les valeurs des capteurs sont égales
    if (data_total_0 == data_total_1 && data_total_1 == data_total_2 && data_total_2 == data_total_3)
    {
        // Calculer la distance totale avec les valeurs des capteurs égales
        float circonference = 20.42; // Circonférence en cm (ajuster selon votre véhicule)
        float tour_seconde = (data_total_0 * 2.5) / 100;
        total_Distance = circonference * tour_seconde * (duration / 1000.0);
    }
    else
    {
        // Calculer la distance totale avec les valeurs moyennes des capteurs
        float average = (data_total_0 + data_total_1 + data_total_2 + data_total_3) / 4.0;
        float circonference = 20.42; // Circonférence en cm (ajuster selon votre véhicule)
        float tour_seconde = (average * 2.5) / 100;
        total_Distance = circonference * tour_seconde * (duration / 1000.0);
    }
}

void loop()
{
    // put your main code here, to run repeatedly:
    ws.cleanupClients();

    Emotion_Show(emotion_task_mode); // Led matrix display function
    WS2812_Show(ws2812_task_mode);   // Car color lights display function

    // The MQTT part
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

   long now = millis();
    if (now - last_message > mqtt_interval_ms)
    {
        last_message = now;

        if (race_change)
        {
            // Les led et la batteries sont branchés tous les deux sur le pin 32
            // du coup, lire la valeur de batterie fait freeze la batterie
            // Battery level
            dtostrf(Get_Battery_Voltage(), 5, 2, buff);
            client.publish("esp32bis/battery", buff);

            // Track Read
            Track_Read();
            sensor_v = static_cast<int>(sensorValue[3]);
            char const *n_char = std::to_string(sensor_v).c_str();
            client.publish("esp32bis/track", n_char);

            // Ultrasonic Data
            dtostrf(Get_Sonar(), 5, 2, ultrasonic_buff);
            client.publish("esp32bis/sonar", ultrasonic_buff);

            // Photosensitive Data
            dtostrf(Get_Photosensitive(), 5, 2, ultrasonic_buff);
            client.publish("esp32bis/light", ultrasonic_buff);

            // Send the total distance
            dtostrf(total_Distance, 5, 2, distance_buff);
            client.publish("esp32bis/distance", distance_buff);

            // Send timer data if active
            if (timerActive)
            {
                // send Time
                unsigned long duration = millis() - startTime;
                float duration_in_seconds = duration / 1000.0; // Convert duration to seconds
                char timer_buff[10];
                dtostrf(duration_in_seconds, 5, 2, timer_buff);
                client.publish("esp32bis/timer", timer_buff);

                //send distance 
                updateDistance(duration);

                // send distance total
                dtostrf(total_Distance, 5, 2, distance_buff);
                client.publish("esp32bis/distance", distance_buff);

                // send speed
                total_speed = total_Distance / duration_in_seconds; // Calculate speed
                dtostrf(total_speed, 5, 2, speed_buff);
                client.publish("esp32bis/speed", speed_buff);
            }

            // Envoie l'ID de la course uniquement si race_change est true
            dtostrf(race_id, 5, 2, race_id_buffer);
            client.publish("esp32bis/race", race_id_buffer);
        }
        else
        {
              // Si race_change est false, envoie seulement "false" dans le topic "esp32bis/race"
        client.publish("esp32bis/race", "false");

        // Réinitialiser toutes les variables à leurs valeurs par défaut
        total_Distance = 0.0;
        total_speed = 0.0;
        timerActive = false;
        startTime = 0;
        race_id = 0;
        race_change = false;

        // Réinitialisation des buffers pour ne pas garder les anciennes données
        memset(buff, 0, sizeof(buff));
        memset(ultrasonic_buff, 0, sizeof(ultrasonic_buff));
        memset(distance_buff, 0, sizeof(distance_buff));
        memset(speed_buff, 0, sizeof(speed_buff));
        memset(race_id_buffer, 0, sizeof(race_id_buffer));
        }
    }
}

// put function definitions here:
void notifyClients()
{
    ws.textAll("ok");
}


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;

    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;  // Assure la terminaison de la chaîne
        processCommand((char*)data);
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
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32ClientBis"))
        {
            Serial.println("connected");
            // Subscribe
            client.subscribe("esp32bis/ajustments");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void loopTask_Camera(void *pvParameters)
{
    while (1)
    {
        char size_buf[12];
        WiFiClient wf_client = server_Camera.available(); // listen for incoming clients
        if (wf_client)
        { // if you get a client
            Serial.println("Camera_Server connected to a client.");
            if (wf_client.connected())
            {
                camera_fb_t *fb = NULL;
                wf_client.write("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: multipart/x-mixed-replace; boundary=" STREAM_CONTENT_BOUNDARY "\r\n");
                while (wf_client.connected())
                { // loop while the client's connected
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
                // close the connection:
                wf_client.stop();
                Serial.println("Camera Client Disconnected.");
            }
        }
    }
}

void testDNS()
{
    Serial.print("Testing DNS resolution: ");
    IPAddress ip;
    if (WiFi.hostByName("google.com", ip))
    {
        Serial.print("DNS OK: ");
        Serial.println(ip);
    }
    else
    {
        Serial.println("DNS Failed");
    }
}

void testInternetConnectivity()
{
    Serial.print("Testing Internet connectivity: ");
    WiFiClient client;
    if (client.connect("93.184.216.34", 80)) 
    {
        Serial.println("Internet connection OK");
        client.stop();
    }
    else
    {
        Serial.println("Internet connection failed");
    }
}
