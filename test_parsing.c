#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main() {
    const char* ts_str = "1713000000000";
    uint32_t val32 = (uint32_t)strtoul(ts_str, NULL, 10);
    uint32_t val64 = (uint32_t)strtoull(ts_str, NULL, 10);
    
    printf("String: %s\n", ts_str);
    printf("strtoul (clamped): %u\n", val32);
    printf("strtoull (truncated): %u\n", val64);
    
    if (val32 == 4294967295U && val64 != 4294967295U) {
        printf("VERIFIED: strtoull correctly bypasses 32-bit clamping.\n");
    } else {
        printf("DEBUG: val32=%u, val64=%u\n", val32, val64);
    }
    
    return 0;
}
