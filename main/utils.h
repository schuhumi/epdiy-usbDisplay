#pragma once

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _b : _a; })

#define with_semaphore(sem, code) { \
    if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE) { \
        code; \
        xSemaphoreGive(sem); \
    } else { \
        ESP_LOGE("SEMAPHORE", "Failed to take semaphore"); \
    } \
}
