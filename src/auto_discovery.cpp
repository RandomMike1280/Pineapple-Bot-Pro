#include "auto_discovery.h"
#include <Arduino.h>

static uint32_t lastDiscoveryTime = 0;
const uint32_t DISCOVERY_INTERVAL_MS = 2000;

void setupAutoDiscovery(WiFiUDP& udp, int port, const char* botId) {
    // Perform an immediate broadcast on setup
    tickAutoDiscovery(udp, port, botId);
}

void tickAutoDiscovery(WiFiUDP& udp, int port, const char* botId) {
    uint32_t now = millis();
    
    // Broadcast periodically or immediately if lastDiscoveryTime is 0
    if (lastDiscoveryTime == 0 || now - lastDiscoveryTime >= DISCOVERY_INTERVAL_MS) {
        lastDiscoveryTime = now;
        
        char msg[64];
        // Format strictly adheres to what the phone expects
        snprintf(msg, sizeof(msg), "hello, i am robot id %s", botId);
        
        // 255.255.255.255 ensures broadcast across the subnet irrespective of gateway
        IPAddress broadcastIp(255, 255, 255, 255);
        
        udp.beginPacket(broadcastIp, port);
        udp.write((const uint8_t*)msg, strlen(msg));
        udp.endPacket();
    }
}
