#ifndef AUTO_DISCOVERY_H
#define AUTO_DISCOVERY_H

#include <WiFiUdp.h>

// Initialize auto-discovery mechanism
void setupAutoDiscovery(WiFiUDP& udp, int port, const char* botId);

// Handle periodic auto-discovery broadcast
void tickAutoDiscovery(WiFiUDP& udp, int port, const char* botId);

#endif // AUTO_DISCOVERY_H
