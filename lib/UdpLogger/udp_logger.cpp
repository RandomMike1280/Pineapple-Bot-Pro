#include "udp_logger.hpp"
#include <stdarg.h>

UdpLogger::UdpLogger(WiFiUDP& udp, IPAddress& targetIp, int port)
    : _udp(udp), _targetIp(targetIp), _port(port), _enabled(false), _entryCount(0) {
    for (int i = 0; i < MAX_THROTTLE_ENTRIES; i++) {
        _throttleEntries[i].id[0] = '\0';
        _throttleEntries[i].lastSentMs = 0;
    }
}

void UdpLogger::setEnabled(bool enabled) {
    _enabled = enabled;
}

void UdpLogger::log(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // Always mirror to Serial (physical line)
    Serial.println(buf);

    if (_enabled) {
        sendUdp('S', NULL, buf);
    }
}

void UdpLogger::important(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.printf("[IMPORTANT] %s\n", buf);

    if (_enabled) {
        sendUdp('I', NULL, buf);
    }
}

void UdpLogger::update(const char* id, const char* fmt, ...) {
    char content[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(content, sizeof(content), fmt, args);
    va_end(args);

    // Mirror to Serial
    Serial.printf("[%s] %s\n", id, content);

    if (!_enabled) return;

    // Throttle to 10Hz (100ms)
    uint32_t now = millis();
    int entryIdx = -1;
    for (int i = 0; i < _entryCount; i++) {
        if (strcmp(_throttleEntries[i].id, id) == 0) {
            entryIdx = i;
            break;
        }
    }

    if (entryIdx == -1 && _entryCount < MAX_THROTTLE_ENTRIES) {
        entryIdx = _entryCount++;
        strncpy(_throttleEntries[entryIdx].id, id, sizeof(_throttleEntries[entryIdx].id) - 1);
    }

    if (entryIdx != -1) {
        if (now - _throttleEntries[entryIdx].lastSentMs >= 100) {
            sendUdp('U', id, content);
            _throttleEntries[entryIdx].lastSentMs = now;
        }
    }
}

void UdpLogger::sendUdp(char type, const char* id, const char* msg) {
    char packet[256];
    int len = buildLogMessage(packet, sizeof(packet), type, id, msg);
    
    if (_targetIp != INADDR_NONE) {
        _udp.beginPacket(_targetIp, _port);
        _udp.write((uint8_t*)packet, len);
        _udp.endPacket();
    }
}
