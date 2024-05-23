#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "mycache.h"

bool dcache_preload_active = false;

void My_Cache_Start_DCache_Preload(uint32_t start, uint32_t len, bool finish_previous_preload) {
    // Start a manual preload of a piece of external ram. The requested region is padded such that
    // the preloading always works (expanded into blocks and one block at a minimum).
    // If a previous preload is still ongoing, it depends on the finish_previous_preload variable
    // what happens:
    //  - true: the call blocks until the previous preload is finished. The task yields while waiting
    //          to be able to get other independent things done.
    //  - false: the previous preload is stopped prematurely, and the new one starts instead.

    if(dcache_preload_active) {
        if(Cache_DCache_Preload_Done()) {
            dcache_preload_active = false;
        } else {
            if(finish_previous_preload) {
                while(!Cache_DCache_Preload_Done()) {
                    taskYIELD();
                }
            } else {
                Cache_End_DCache_Preload(0); // Do not start auto-preload
            }
            dcache_preload_active = false;
        }
    }

    uint32_t block_start = start & DATA_CACHE_BLOCK_BITS;
    uint32_t block_end = start + len;  // This is exclusive, i.e. the first address we do not require
    if((block_end & DATA_CACHE_OVERHANG_BITS) != 0) {
        // the end of our desired area happens to not fit the block grid
        // -> we need to include the block that our end overhangs into
        block_end = (block_end | DATA_CACHE_OVERHANG_BITS) + 1;
    }
    if( ((block_start^block_end) & DATA_CACHE_BLOCK_BITS) != 0 ) {
        // Our start and stop block are the same one, which makes a length of 0 blocks.
        // We need to request 1 block at a minimum though.
        block_end += CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE;
    }
    // With a CONFIG_ESP32S3_DATA_CACHE_LINE==16 case as example, at this point we should have:
    // block_start = 0b...aaaaa0000
    // block_stop  = 0b...bbbbb0000
    // where the block number indicated by the b bits is at least 1 larger than the one indicated by the a bits
    Cache_Start_DCache_Preload(
        block_start,                // start address of the preload region
        block_end - block_start,    // size of the preload region, should not exceed the size of DCache
        0                           // the preload order, 0 for positive, other for negative
    );
    dcache_preload_active = true;
}