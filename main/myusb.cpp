#include <esp_log.h>
#include "myusb.h"

static const char *TAG = "MyUSB";

struct usb_data_chunk_t usb_data_chunk[NUM_USB_DATA_CHUNKS];
StreamBufferHandle_t usb_data_chunks_empty;
StreamBufferHandle_t usb_data_chunks_full;

void init_usb_data_chunks(void) {
    usb_data_chunks_empty = xStreamBufferCreate(
        NUM_USB_DATA_CHUNKS, // size in bytes
        1 // trigger level
    );
    usb_data_chunks_full = xStreamBufferCreate(
        NUM_USB_DATA_CHUNKS, // size in bytes
        1 // trigger level
    );

    for(uint8_t i=0; i<NUM_USB_DATA_CHUNKS; i++) {
        usb_data_chunk[i].len = 0;
        usb_data_chunk[i].data = (uint8_t*)heap_caps_malloc(CONFIG_TINYUSB_CDC_RX_BUFSIZE+1, MALLOC_CAP_INTERNAL);

        xStreamBufferSend(
            usb_data_chunks_empty,
            &i, // data to send is the indices
            1, // just one byte,
            0  // do not wait for space to become free, because there's enough space
        );
    }
}

void init_usb_connection(tusb_cdcacm_callback_t callback_rx) {
    ESP_LOGI(TAG, "USB initialization");
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB ACM initialization");
    
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = callback_rx, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    ESP_LOGI(TAG, "CONFIG_TINYUSB_CDC_RX_BUFSIZE=%d\n", CONFIG_TINYUSB_CDC_RX_BUFSIZE);
    ESP_LOGI(TAG, "USB initialization DONE");
}