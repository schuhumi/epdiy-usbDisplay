#pragma once


#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#define NUM_USB_DATA_CHUNKS 128
struct usb_data_chunk_t {
    size_t len;
    uint8_t* data;
};
extern struct usb_data_chunk_t usb_data_chunk[NUM_USB_DATA_CHUNKS];
extern StreamBufferHandle_t usb_data_chunks_empty;
extern StreamBufferHandle_t usb_data_chunks_full;

static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_VENDOR_SPECIFIC, //TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A, // This is Espressif VID. This needs to be changed according to Users / Customers
    .idProduct = 0xA299,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

const tinyusb_config_t tusb_cfg = {
    .device_descriptor = &descriptor_config, //NULL,
    .string_descriptor = NULL,
    .string_descriptor_count = 0,
    .external_phy = false,
    .configuration_descriptor = NULL,
    .self_powered = false,
    .vbus_monitor_io = 0,  // ignored because .self_powered = false
};

void init_usb_data_chunks(void);
void init_usb_connection(tusb_cdcacm_callback_t callback_rx);
