#include <SD.h>
#include <SD_MMC.h>
#include <FFat.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include "WifiModule.h"
#include "LV_EZ1.h"
#include "WebSocketClient.h"
#include "EepromControl.h"

static const int8_t lvEz1Pin = 34;

static const String host = "192.168.0.195";
static const int port = 6001;
static const bool ssl = false;

//static const String host = "13.125.102.123";
//static const int port = 11002;
//static const bool ssl = false;

//static const String host = "kira-api.wimcorp.dev";
//static const int port = 443;
//static const bool ssl = true;

String authenticationToken;

//Consts
const uint16_t maxdist = 450; //초음파센서 측정 최대거리(450cm 추천, 300cm 이상부터는 noise 영향 심해짐)
const uint16_t mindist = 0; //초음파센서 측정 최소거리(30cm 추천, 너무 가까운 거리에서는 노이즈 발생

uint16_t max_target_dist = 0;
uint16_t min_target_dist = 0;
//uint32_t tick = 0;

const int threshold_count = 5; // 커질수록 해당위치에 오래있어야 trigger on
int detection_count = 0; // 커질수록 해당위치에 오래있어야 trigger on

bool isDetected = false;
bool isConnectToWifiWithAPI = false;

int wifiConnectCount = 0;

WebSocketClient wsClient;
WebServer webServer(80);
LV_EZ1 ult(lvEz1Pin, LV_EZ1_MODE_PULSE);

bool connectWifi() {
    auto ssid = EepromControl::getInstance().getWifiSsid();
    auto psk = EepromControl::getInstance().getWifiPsk();

    if (ssid.length() != 0) {
        String status = WifiModule::getInstance().connectWifi(ssid, psk);

        Serial.print("ssid : ");
        Serial.println(ssid);
        Serial.print("password : ");
        Serial.println(psk);

        if (status != "WL_CONNECTED") {
            Serial.println("wifi connect fail");
            return false;
        }

        Serial.println("wifi connect");
        return true;
    }
    else {
        return false;
    }
}

void receiveWifi() {
    isConnectToWifiWithAPI = true;
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "text/plain", "no plainBody");
        isConnectToWifiWithAPI = false;
        return;
    }
    String plainBody = webServer.arg("plain");
    DynamicJsonDocument doc(plainBody.length() * 2);
    deserializeJson(doc, plainBody);

    if (doc.containsKey("ssid") && doc.containsKey("password"), doc.containsKey("token")) {
        String strPayload;

        serializeJson(doc, strPayload);

        authenticationToken = String(doc["token"]);
        EepromControl::getInstance().setWifiPsk(doc["ssid"], doc["password"]);

        Serial.println(EepromControl::getInstance().getWifiSsid());
        Serial.println(EepromControl::getInstance().getWifiPsk());

        bool status = false;

        for (int i = 0; i < 3; i++) {
            if (connectWifi()) {
                status = true;
                break;
            }
        }

        if (!status) {
            EepromControl::getInstance().setWifiPsk("", "");
            isConnectToWifiWithAPI = false;

            webServer.send(400, "text/plain", "network connect fail");
            return;
        }

        isConnectToWifiWithAPI = false;
        webServer.send(200);
        delay(5000);

        WifiModule::getInstance().stop();

        wsClient.connect();

        wifiConnectCount = 0;
    }
    else {
        isConnectToWifiWithAPI = false;

        webServer.send(400, "text/plain", "wrong json data");
    }
}

void receiveSerial() {

    if (!webServer.hasArg("plain")) {
        webServer.send(400, "text/plain", "no plainBody");
        return;
    }
    String plainBody = webServer.arg("plain");
    DynamicJsonDocument doc(plainBody.length() * 2);
    deserializeJson(doc, plainBody);

    if (doc.containsKey("serial")) {
        String strPayload;

        serializeJson(doc, strPayload);

        EepromControl::getInstance().setSerial(doc["serial"]);
        Serial.println("get serial : " + EepromControl::getInstance().getSerial());

        webServer.send(200);
    }
    else {
        webServer.send(400, "text/plain", "wrong json data");
    }
}

double kalman(double dist) {
    static const double R = 100;
    static const double H = 1.0;
    static double Q = 10;
    static double P = 0;
    static double dist_hat = 0;
    static double K = 0;

    K = P * H / (H * P * H + R);
    dist_hat = dist_hat + K * (dist - H * dist_hat);
    P = (1 - K * H) * P + Q;
    return dist_hat;
}

void sendIsDetected(bool detected) {
    DynamicJsonDocument doc(512);
    JsonObject json = doc.to<JsonObject>();
    json["event"] = "SendHumanDetection";
    json["data"]["isDetected"] = detected;

    String strJson;
    serializeJson(json, strJson);
    wsClient.sendText(strJson);
}

void processConfig(const JsonObject &data) {
    min_target_dist = 0;
    max_target_dist = data["sens"];
    isDetected = data["humanDetected"];
}

void setup() {
    Serial.begin(115200);
    EepromControl::getInstance().init();
    EepromControl::getInstance().setWifiPsk("Wim", "Wim12345!");
    EepromControl::getInstance().setSerial("Kira_Detector_00001");

    WiFiClass::mode(WIFI_MODE_STA);

    Serial.println("SERIAL : " + EepromControl::getInstance().getSerial());
    Serial.println("SSID : " + EepromControl::getInstance().getWifiSsid());
    Serial.println("PSK : " + EepromControl::getInstance().getWifiPsk());

    WifiModule::getInstance().setIp("192.168.0.1", "192.168.0.1", "255.255.255.0");
    WifiModule::getInstance().setApInfo(EepromControl::getInstance().getSerial());

    wsClient.setHost(host);
    wsClient.setPort(port);
    wsClient.setWithSsl(ssl);
    wsClient.onConnected([&](uint8_t *payload, size_t length) {
        Serial.println("websocket connected");

        DynamicJsonDocument doc(512);
        JsonObject json = doc.to<JsonObject>();
        json["event"] = "registerDeviceSession";
        json["name"] = EepromControl::getInstance().getSerial();
        Serial.println(String(json["name"]));
        json["type"] = "HumanDetection";
        json["token"] = authenticationToken;

        String strJson;
        serializeJson(json, strJson);

        wsClient.sendText(strJson);
    });
    wsClient.onDisconnected([&](uint8_t *payload, size_t length) {
        Serial.println("websocket disconnected");
    });
    wsClient.onTextMessageReceived([&](uint8_t *payload, size_t length) {
        DynamicJsonDocument doc(length * 2);
        deserializeJson(doc, payload, length);

        String strJson;
        serializeJson(doc, strJson);
        Serial.println("onTextMessageReceived : " + strJson);

        if (doc.containsKey("authenticationToken")) {
            String token = String(doc["authenticationToken"]);

            if (token.isEmpty()) {
                wsClient.disconnect();
                EepromControl::getInstance().setWifiPsk("", "");
                WifiModule::getInstance().start();

                return;
            }

            authenticationToken = token;
        }
        else if (doc["event"] == "SendHumanDetectionConfig") {
            processConfig(doc["data"]);
        }
    });
    wsClient.onPingMessageReceived([&](uint8_t *payload, size_t length) {
        wsClient.sendPong();
    });
    wsClient.onErrorReceived([&](uint8_t *payload, size_t length) {
        Serial.println("websocket errorReceived");
    });

    webServer.on("/wifi", HTTP_POST, &receiveWifi);
    webServer.on("/serial", HTTP_POST, &receiveSerial);

    webServer.begin();

    ult.activate();
}

//int delayTime = 1000;
//int lastTick = 0;

void loop() {
    webServer.handleClient();
    wsClient.loop();

    if (WifiModule::getInstance().isConnectedST() && !isConnectToWifiWithAPI) {
        if (max_target_dist == 0 && min_target_dist == 0) return;
        auto distanceRaw = ult.read() * 2.54; // original value unit = inch
        if (distanceRaw > mindist && distanceRaw < maxdist) {
            auto distanceKalman = kalman(distanceRaw);

            if (distanceKalman > min_target_dist && distanceKalman < max_target_dist) {
                if (detection_count < threshold_count) detection_count += 1;
            }
            else {
                if (detection_count <= threshold_count && detection_count > 0) detection_count -= 1;
            }

            if (!isDetected && detection_count == threshold_count) {
                isDetected = true;
                sendIsDetected(isDetected);
            }
            else if (isDetected && detection_count == 0) {
                isDetected = false;
                sendIsDetected(isDetected);
            }

//            TickType_t xLastWakeTime = xTaskGetTickCount();
//            if (lastTick + pdMS_TO_TICKS(delayTime) < xLastWakeTime) {
//                lastTick = xLastWakeTime;
//                sendIsDetected(isDetected);
//            }

            Serial.println("kalman          : " + String(distanceKalman));
            Serial.println("raw             : " + String(distanceRaw));
            Serial.println("detection_count : " + String(detection_count));
        }
    }
    else {
        if (wifiConnectCount < 5) {
            if (connectWifi()) {
                delay(1000);
                WifiModule::getInstance().stop();
                wsClient.connect();
            }
            else {
                wifiConnectCount += 1;
                Serial.println("wifiConnectCount : " + String(wifiConnectCount));

                if (wifiConnectCount == 5) {
                    WifiModule::getInstance().start();
                }
            }
        }
    }
    delay(25);

}
