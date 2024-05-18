#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "mycache.h"

bool dcache_preload_active = false;

void My_Cache_Start_DCache_Preload(uint32_t start, uint32_t len, bool finish_previous_preload) {
    // Start manual preload of a piece of external ram. If a previous preload is still ongoing, it is being
    // stopped.

    if(dcache_preload_active) {
        if(Cache_DCache_Preload_Done()) {
            dcache_preload_active = false;
        } else {
            if(finish_previous_preload) {
                while(!Cache_DCache_Preload_Done()) {
                    taskYIELD();
                }
            } else {
                //ESP_LOGI("MyCache", "DCache preload is still active, stopping it first.");
                Cache_End_DCache_Preload(0); // Do not start auto-preload
            }
            dcache_preload_active = false;
        }
    }

    uint32_t block_start = start & DATA_CACHE_START_MASK;
    uint32_t block_end = ((start + len) | DATA_CACHE_STOP_MASK) + 1;
    Cache_Start_DCache_Preload(
        block_start,  // start address of the preload region
        block_end - block_start,  // size of the preload region, should not exceed the size of DCache
        0  // the preload order, 0 for positive, other for negative
    );
    dcache_preload_active = true;
}