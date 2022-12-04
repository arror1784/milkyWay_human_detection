#include <SD.h>
#include <SD_MMC.h>
#include <FFat.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include "WifiModule.h"
#include "SEN0153.h"
#include "WebSocketClient.h"
#include "EepromControl.h"

static const String mirrorSSID = "Kira";
static const String ssid = "Kira humanDetection";

static const uint8_t send0153Address = SEN0153_ADDRESS_DEFAULT;
static const int8_t sen0153RX = 16;
static const int8_t sen0153TX = 17;

static const String host = "192.168.219.102";
static const int port = 6001;
static const bool ssl = false;

//static const String host = "kira-api.wimcorp.dev";
//static const int port = 443;
//static const bool ssl = true;


//Consts
const float maxdist = 450.0; //초음파센서 측정 최대거리(450cm 추천, 300cm 이상부터는 noise 영향 심해짐)
const float mindist = 30.0; //초음파센서 측정 최소거리(30cm 추천, 너무 가까운 거리에서는 노이즈 발생

float max_target_dist = 150.0;
float min_target_dist = 100.0;

const int threshold_count = 5; // 커질수록 해당위치에 오래있어야 trigger on
int detaction_count = 0; // 커질수록 해당위치에 오래있어야 trigger on
bool isDetected = false;

//


WebSocketClient wsClient;
WebServer webServer(80);
SEN0153 ult(sen0153RX, sen0153TX, SEN0153_BAUD_RATE_DEFAULT);

bool connectWifiByFlash() {
    // TODO: 플래시에서 와이파이 데이터 가져오기
    String status = WifiModule::getInstance().connectWifi("", "");

    if (status != "WL_CONNECTED") {
        Serial.println("wifi connect fail");
        WifiModule::getInstance().start();
        return false;
    }

    Serial.println("wifi connect");
    WifiModule::getInstance().stop();
    wsClient.connect();
    return true;
}

bool connectWifi() {

    auto ssid = EepromControl::getInstance().getWifiSsid();
    auto psk = EepromControl::getInstance().getWifiPsk();

    Serial.print("ssid : ");
    Serial.println(ssid);
    Serial.print("password : ");
    Serial.println(psk);

    if(ssid.length() != 0){

        String status = WifiModule::getInstance().connectWifi(ssid,psk);

        if (status != "WL_CONNECTED") {
            Serial.println("wifi connect fail");
            WifiModule::getInstance().start();
            return false;
        }

        Serial.println("wifi connect");
        WifiModule::getInstance().stop();
        wsClient.connect();
        return true;
    }else{
        WifiModule::getInstance().start();
        return false;
    }
}

void receiveWifi() {
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "text/plain", "no plainBody");
        return;
    }
    String plainBody = webServer.arg("plain");
    DynamicJsonDocument doc(plainBody.length() * 2);
    deserializeJson(doc, plainBody);

    if (doc.containsKey("ssid") && doc.containsKey("password")) {
        String strPayload;

        serializeJson(doc, strPayload);

        EepromControl::getInstance().setWifiPsk(doc["ssid"],doc["password"]);

        Serial.println(EepromControl::getInstance().getWifiSsid());
        Serial.println(EepromControl::getInstance().getWifiPsk());
        
        bool status = connectWifi();

        if (!status) {
            webServer.send(400, "text/plain", "network connect fail");
            return;
        }
        webServer.send(200);
    }
    else {
        webServer.send(400, "text/plain", "wrong json data");
    }
}

void receiveSerial(){

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

void setup() {
    Serial.begin(115200);
    EepromControl::getInstance().init();

    WiFiClass::mode(WIFI_MODE_STA);

    Serial.println("SERIAL : " + EepromControl::getInstance().getSerial());
    Serial.println("SSID : " + EepromControl::getInstance().getWifiSsid());
    Serial.println("PSK : " + EepromControl::getInstance().getWifiPsk());

    WifiModule::getInstance().setIp("192.168.0.1", "192.168.0.1", "255.255.255.0");
    // TODO : 플래시에서 시리얼 데이터 가져오기
    WifiModule::getInstance().setApInfo(EepromControl::getInstance().getSerial());

    wsClient.setHost(host);
    wsClient.setPort(port);
    wsClient.setWithSsl(ssl);
    wsClient.onConnected([&](uint8_t *payload, size_t length) {
        Serial.println("websocket connected");

        DynamicJsonDocument doc(512);
        JsonObject json = doc.to<JsonObject>();
        json["event"] = "registerDeviceSession";
        // TODO : 플래시에서 시리얼 데이터 가져오기
        json["name"] = EepromControl::getInstance().getSerial();
        Serial.println(String(json["name"]));
        json["type"] = "HumanDetection";

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

        if (doc.containsKey("sens")) {
            min_target_dist = 0;
            max_target_dist = doc["sens"];
        }
        else if (doc["event"] == "Ping") {
            DynamicJsonDocument json(512);
            json["event"] = "Pong";

            String strJson;
            serializeJson(json, strJson);

            wsClient.sendText(strJson);
        }
    });
    wsClient.onErrorReceived([&](uint8_t *payload, size_t length) {
        Serial.println(String(payload, length));
        Serial.println("websocket errorReceived");
    });

    webServer.on("/wifi", HTTP_POST, &receiveWifi);
    webServer.on("/serial", HTTP_POST, &receiveSerial);

    webServer.begin();

    ult.begin();
}

double kalman(double dist){
    static const double R = 100;
    static const double H = 1.0;
    static double Q = 10;
    static double P = 0;
    static double dist_hat = 0;
    static double K = 0;
    
    K = P*H/(H*P*H+R);
    dist_hat = dist_hat + K*(dist-H*dist_hat);
    P = (1-K*H)*P+Q;
    return dist_hat;
}

void sendIsDetected(bool detected){
    DynamicJsonDocument doc(512);
    JsonObject json = doc.to<JsonObject>();
    json["event"] = "SendHumanDetection";
    json["data"]["isDetected"] = detected;

    String strJson;
    serializeJson(json, strJson);
    Serial.println(strJson);
    wsClient.sendText(strJson);
}

void loop() {
    webServer.handleClient();
    wsClient.loop();

    // TODO: 큐를 임베디드 기기가 아닌 서버에서 구현하고, 이를 패치해 오는 방식을 바꾸는 것도 고려

    if (WifiModule::getInstance().isConnectedST()) {
        auto distanceRaw =  ult.readDistance(send0153Address);
        if ( distanceRaw > mindist && distanceRaw < maxdist ){

            auto distanceKalman = kalman(distanceRaw);
            Serial.println("kalman" + String(distanceKalman));
            // Serial.println("raw   " + String(distanceRaw));   


            if (distanceKalman > min_target_dist && distanceKalman < max_target_dist) {
                detaction_count++;
            }else{
                detaction_count = 0;
            }

            if(detaction_count > threshold_count){
                if(!isDetected){
                    isDetected = true;
                    sendIsDetected(isDetected);
                }
            }else{
                if(isDetected){
                    isDetected = false;
                    sendIsDetected(isDetected);
                }
            }

        }else{
            return;
        }
    }
    else {
        connectWifi();
    }

    delay(50);
}
