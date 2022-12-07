#ifndef MILKYWAY_WEB_SOCKET_CLIENT_H
#define MILKYWAY_WEB_SOCKET_CLIENT_H

#include "Singleton.h"
#include "WebSocketsClient.h"

#include <ArduinoJson.h>
#include <WString.h>
#include <functional>
#include <optional>

class WebSocketClient {
public:
    WebSocketClient();

    void connect();

    void disconnect();

    void loop();

    bool isConnected();

    void setHost(const String &host);

    void setPort(int port);

    void setWithSsl(bool withSsl);

    void sendText(String txt);

    void sendPong();

    typedef std::function<void(uint8_t *, size_t)> webSocketReceiveCB;

    void onTextMessageReceived(webSocketReceiveCB cb) { _webSocketReceiveText = cb; };

    void onPingMessageReceived(webSocketReceiveCB cb) { _webSocketReceivePing = cb; };

    void onConnected(webSocketReceiveCB cb) { _webSocketReceiveConnected = cb; };

    void onDisconnected(webSocketReceiveCB cb) { _webSocketReceiveDisconnected = cb; };

    void onErrorReceived(webSocketReceiveCB cb) { _webSocketReceiveError = cb; };

private:
    String _host = "0.0.0.0";
    int _port = 80;
    bool _withSSL = false;

    WebSocketsClient _client;

    std::optional<webSocketReceiveCB> _webSocketReceiveText;
    std::optional<webSocketReceiveCB> _webSocketReceivePing;
    std::optional<webSocketReceiveCB> _webSocketReceiveConnected;
    std::optional<webSocketReceiveCB> _webSocketReceiveDisconnected;
    std::optional<webSocketReceiveCB> _webSocketReceiveError;
};


#endif //MILKYWAY_WEBSOCKETCLIENT_H
