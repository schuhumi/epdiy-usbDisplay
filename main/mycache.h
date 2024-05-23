#pragma once

#include <rom/cache.h>
#include <esp_types.h>  

// The CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE is either 64, 32 or 16.
// The bit mask that indicates the addresses inside a block can be calculated like this
#define DATA_CACHE_OVERHANG_BITS ((uint32_t)CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE-1)
// The other bits denote the block number:
#define DATA_CACHE_BLOCK_BITS (~DATA_CACHE_OVERHANG_BITS)

void My_Cache_Start_DCache_Preload(uint32_t start, uint32_t len, bool finish_previous_preload);
