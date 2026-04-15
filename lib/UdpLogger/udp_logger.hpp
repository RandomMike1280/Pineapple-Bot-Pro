#ifndef UDP_LOGGER_HPP
#define UDP_LOGGER_HPP

#include <Arduino.h>
#include <WiFiUdp.h>
#include <udp_protocol.hpp>

/**
 * UdpLogger - Handles throttled serial-over-UDP logging.
 * 
 * Features:
 *  - Standard logs (Sent immediately)
 *  - Updating logs (Throttled to 10Hz to save bandwidth)
 *  - Important logs (Flagged for red highlighting in app)
 *  - Dual-output: Always mirrors to physical Serial.
 */
class UdpLogger {
public:
    UdpLogger(WiFiUDP& udp, IPAddress& targetIp, int port);

    void setEnabled(bool enabled);
    bool isEnabled() const { return _enabled; }

    // Standard log: mirrored to Serial, sent as L:S:<msg>
    void log(const char* fmt, ...);

    // Important log (Red in app): mirrored to Serial, sent as L:I:<msg>
    void important(const char* fmt, ...);

    // Updating log: mirrored to Serial, sent as L:U:<id>:<msg> (Throttled to 10Hz)
    void update(const char* id, const char* fmt, ...);

private:
    WiFiUDP& _udp;
    IPAddress& _targetIp;
    int _port;
    bool _enabled;

    // Throttle tracking for updating logs
    struct ThrottleEntry {
        char id[16];
        uint32_t lastSentMs;
    };
    static const int MAX_THROTTLE_ENTRIES = 8;
    ThrottleEntry _throttleEntries[MAX_THROTTLE_ENTRIES];
    int _entryCount;

    void sendUdp(char type, const char* id, const char* msg);
};

#endif // UDP_LOGGER_HPP
