#pragma once

#include <rom/cache.h>
#include <esp_types.h>  

#if CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE == 64
    #define DATA_CACHE_START_MASK 0xFFFFFFC0
    #define DATA_CACHE_STOP_MASK 0x3F
#elif CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE == 32
    #define DATA_CACHE_START_MASK 0xFFFFFFE0
    #define DATA_CACHE_STOP_MASK 0x1F
#else // 16
    #define DATA_CACHE_START_MASK 0xFFFFFFF0
    #define DATA_CACHE_STOP_MASK 0x0F
#endif

void My_Cache_Start_DCache_Preload(uint32_t start, uint32_t len);
