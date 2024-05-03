#include "mycache.h"

void My_Cache_Start_DCache_Preload(uint32_t start, uint32_t len) {
    // Does not work reliably yet
    /*Cache_Start_DCache_Preload(
        start & DATA_CACHE_START_MASK,  // start address of the preload region
        (len | DATA_CACHE_STOP_MASK) + 1,  // size of the preload region, should not exceed the size of DCache
        0  // the preload order, 0 for positive, other for negative
    );*/
}