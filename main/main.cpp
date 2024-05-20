#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <esp_types.h>
#include <esp_async_memcpy.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/stream_buffer.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <epdiy.h>

#include "mycache.h"
#include "utils.h"
#include "asm_templates.h"
#include "myusb.h"

#include "sdkconfig.h"

using namespace std;

static const char *TAG = "EPDiyGraphics";
extern "C" void app_main();

// choose the default demo board depending on the architecture
#ifdef CONFIG_IDF_TARGET_ESP32
#define DEMO_BOARD epd_board_v6
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define DEMO_BOARD epd_board_v7
#endif

#ifndef ARDUINO_ARCH_ESP32
void delay(uint32_t millis) {
    vTaskDelay(millis / portTICK_PERIOD_MS);
}
#endif

struct draw_pixels_args {
    uint16_t x;
    uint16_t y;
    uint32_t repeat;
    uint8_t color;
};
struct draw_img_ctx {
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
};

#define DRAW_PIXELS_QUEUE_LEN 64
struct draw_pixels_args draw_pixels_queue[DRAW_PIXELS_QUEUE_LEN];
StreamBufferHandle_t draw_pixels_args_empty;
StreamBufferHandle_t draw_pixels_args_full;
uint32_t delegate_pixel_drawing_threshold = 16<<8;

uint8_t *fb;
uint8_t** fb_by_row;
uint32_t fb_size;
int temperature = 17;
int _epd_width;
int _epd_height;


enum command {
    COMMAND_FOLLOWS = 128,
    SEND_128_BYTE   = 0,
    UNKNOWN_COMMAND = 254,

    SCREEN_SIZE     = 10,

    CLEAR_DISPLAY   = 100,
    REFRESH_DISPLAY = 101,
    DRAW_IMAGE_2BIT = 150
};


enum command current_command = UNKNOWN_COMMAND;
enum command previous_command = UNKNOWN_COMMAND; // Needed when SEND_128_BYTE Command comes
uint32_t draw_image_last_cached_line = 0;
EpdRect refresh_area_coords;
struct draw_pixels_args current_pixelpack;
struct draw_img_ctx current_image = {
    .x = 0,
    .y = 0,
    .width = 0,
    .height = 0
};



bool* drawn_lines;
uint16_t drawn_columns_start = 0;
uint16_t drawn_columns_end = 0;
bool drawn_lines_dirty = false;
bool* refreshed_lines;
volatile bool refreshcompute_active = false;
uint16_t refreshed_columns_start = 0;
uint16_t refreshed_columns_end = 0;
TaskHandle_t refresh_compute_task = NULL;

bool is_powered_on;


// indexing: [source][target]
const uint8_t DRAM_ATTR transitions[4][4] = {
    {0, 1, 2, 3},
    {3, 1, 2, 3},
    {3, 1, 2, 3},
    {0, 0, 0, 3}
};

async_memcpy_t async_memcpy_driver;
static bool IRAM_ATTR dmacpy_cb(async_memcpy_t hdl, async_memcpy_event_t*, void* args) {
    BaseType_t r = pdFALSE;
    xTaskNotifyFromISR((TaskHandle_t)args,0,eNoAction,&r);
    return (r!=pdFALSE);
}

void IRAM_ATTR write_flush(void) {
    esp_err_t ret;
    for(;;) {
        ret = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 100);
        if(ret == ESP_OK) {
            break;
        }
        taskYIELD();
    }
}

void IRAM_ATTR write_payload_bytes(
	unsigned char *bytes,
	unsigned int len
) {
    unsigned char cmd128[] = {COMMAND_FOLLOWS, SEND_128_BYTE};
	unsigned int copy_start = 0;
    for(unsigned int i=0; i<len; i++) {
        if( bytes[i]==0x80 ) {
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, &bytes[copy_start], i-copy_start);
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, cmd128, sizeof(cmd128));
			copy_start = i+1;
        }
    }
	tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, &bytes[copy_start], len-copy_start);
}

void IRAM_ATTR write_command(
	unsigned char cmdno,
	unsigned char *payload,
	unsigned int plen
) {
    unsigned char cmd[] = {
        COMMAND_FOLLOWS,
        0x00
    };
    cmd[1] = cmdno;
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, cmd, 2);
    if( payload )
        write_payload_bytes(payload, plen);
}


void IRAM_ATTR do_refresh(void) {
    //uint16_t lines_dirty_ctr;


    if(drawn_columns_start > drawn_columns_end) {
        ESP_LOGW(TAG, "drawn_columns_start > drawn_columns_end");
        drawn_columns_start = 0;
        drawn_columns_end = _epd_width;
    }

    // wait for the parallel drawing task to finish drawing pixels
    while(xStreamBufferBytesAvailable(draw_pixels_args_empty)<DRAW_PIXELS_QUEUE_LEN) {
        taskYIELD();
    }

    while(refreshcompute_active) {
        taskYIELD();
    }
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    esp_async_memcpy(async_memcpy_driver, refreshed_lines, drawn_lines, _epd_height, &dmacpy_cb, (void*)task);
    //memcpy(refreshed_lines, drawn_lines, _epd_height);  // dest, source, size
    refreshed_columns_end = drawn_columns_end;
    refreshed_columns_start = drawn_columns_start;

    //ESP_LOGI(TAG, "starting drawing (epd_draw_base)");
    //uint32_t t1 = esp_timer_get_time() / 1000;
    epd_draw_base(
        epd_full_screen(),
        fb,
        epd_full_screen(),
        (EpdDrawMode)(MODE_DU | MODE_PACKING_1PPB_DIFFERENCE),
        temperature,
        drawn_lines,
        NULL,
        epd_get_display()->default_waveform
    );
    //uint32_t t2 = esp_timer_get_time() / 1000;
    //ESP_LOGI(TAG, "[With vector extensions] epd_draw_base took %ldms.", t2 - t1);

    drawn_columns_start = _epd_width;
    drawn_columns_end = 0;
    drawn_lines_dirty = false;
    xTaskNotifyWait(0,0,NULL,portMAX_DELAY); // Wait for async copy to finish
    xTaskNotify( refresh_compute_task, 0, eNoAction );

}

void IRAM_ATTR refresh_compute( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
       pvParameters value in the call to xTaskCreate() below. */ 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
    while(1) {
        xTaskNotifyWait(0,0,NULL,portMAX_DELAY); // Wait for computation of new is-state of pixels to be needed
        refreshcompute_active = true;

        //lines_dirty_ctr = 0;
        uint32_t rpt_chunk_start_offset = refreshed_columns_start & 0xFFFFFF00;
        uint32_t rpt_cnt = (refreshed_columns_end>>2) - (rpt_chunk_start_offset>>2);
        if(refreshed_columns_end & 0b11)
            rpt_cnt++;
        uint8_t *fb_pixel = NULL;
        int32_t yy = 0;
        int32_t next_yy;
        while(!refreshed_lines[yy]) {
            yy++;
            if(yy>=_epd_height)
                break;
        }
        if(yy<_epd_height) {
            My_Cache_Start_DCache_Preload(
                (uint32_t)fb_by_row[yy] + rpt_chunk_start_offset,
                rpt_cnt<<2,
                true
            );
        }
        while(yy<_epd_height) {
            taskYIELD();  // Give the usb task a chance to run, otherwise we might lose data
            next_yy = yy+1;
            while(next_yy<_epd_height && (!refreshed_lines[next_yy])) {
                next_yy++;
            }
            if(next_yy<_epd_height) {
                My_Cache_Start_DCache_Preload(
                    ((uint32_t)fb_by_row[next_yy]) + rpt_chunk_start_offset,
                    rpt_cnt<<2,
                    true
                );
            }
            if(refreshed_lines[yy]) {
                drawn_lines[yy] = false;
                fb_pixel = fb_by_row[yy] + rpt_chunk_start_offset;
                
                // Handle 4 pixels at a time
                rpt(rpt_cnt, [&yy,&fb_pixel]() {
                    uint32_t sources =*((uint32_t*)fb_pixel);
                    uint32_t targets = (sources>>4)&0x0C0C0C0C;
                    sources &= 0x0C0C0C0C;
                    if(targets!=sources) {
                        uint32_t news = transitions[(sources>>26)&0b11][(targets>>26)&0b11]<<24;
                        news |= transitions[(sources>>18)&0b11][(targets>>18)&0b11]<<16;
                        news |= transitions[(sources>>10)&0b11][(targets>>10)&0b11]<<8;
                        news |= transitions[(sources>>2)&0b11][(targets>>2)&0b11]<<0;
                        *((uint32_t*)fb_pixel) = (targets<<4) | (news<<2);
                        if(news!=targets) {
                            drawn_lines[yy] = true;
                        }
                    }
                    fb_pixel+=4;
                });
                //lines_dirty_ctr += drawn_lines[yy];
                if(drawn_lines[yy]) {
                    drawn_lines_dirty = true;
                }
                refreshed_lines[yy] = false;
            }
            yy = next_yy;
        }
        if(drawn_lines_dirty) {
            drawn_columns_start = min(drawn_columns_start, refreshed_columns_start);
            drawn_columns_end = max(drawn_columns_end, refreshed_columns_end);
        }
        
        refreshcompute_active = false;
    }

}

void IRAM_ATTR draw_pixels(
    struct draw_pixels_args *args
) {
    uint32_t yy = current_image.y + args->y;
    while(refreshed_lines[yy]) taskYIELD();
    uint8_t *fb_yy = fb_by_row[yy];
    drawn_lines[yy] = true;
    drawn_lines_dirty = true;
    uint8_t color_shifted = args->color<<6;
    uint32_t color_shifted32 = 0;
    if((args->color!=0b101) && (args->repeat>=3)) {
        color_shifted32 = color_shifted;
        color_shifted32 |= (color_shifted32<<8)
            | (color_shifted32<<16)
            | (color_shifted32<<24);
    }
    for(uint32_t px_ctr=0; px_ctr<args->repeat+1;) {
        uint32_t xx_count = min(
            args->repeat+1-px_ctr,
            current_image.width-args->x
        );
        uint8_t *buf_ptr = fb_yy + current_image.x + args->x;
        // We do differentiate between putting a color and inverting outside
        // the loop, so we do not do this branch for every loop iteration again.
        if(args->color==0b101) {  // invert
            rpt(xx_count>>2, [&buf_ptr]() {
                uint32_t A = *((uint32_t*)buf_ptr);
                *((uint32_t*)buf_ptr) = (A & 0x0F0F0F0F) | (~A & 0xF0F0F0F0);
                buf_ptr += 4;
            });
            rpt(xx_count&0b11, [&buf_ptr]() {
                *buf_ptr = (*buf_ptr & 0x0F) | (~(*buf_ptr) & 0xF0);
                buf_ptr++;
            });
        } else {  // paint color
            rpt(xx_count>>2, [&buf_ptr, &color_shifted32]() {
                uint32_t A = *((uint32_t*)buf_ptr);
                *((uint32_t*)buf_ptr) = (A & 0x0F0F0F0F) | color_shifted32;
                buf_ptr += 4;
            });
            rpt(xx_count&0b11, [&buf_ptr, &color_shifted]() {
                *buf_ptr = (*buf_ptr & 0x0F) | color_shifted;
                buf_ptr++;
            });
        }

        // Since we only advanced in the row, we only need to update these
        // variables
        args->x += xx_count;
        px_ctr += xx_count;

        // All the logic of wrapping into the next row etc follows here
        if( args->x + 1 > current_image.width ) {
            args->x = 0;
            args->y++;
            yy++;
            fb_yy = fb_by_row[yy];
            
            if( 
                (args->y + 1 > current_image.height) ||
                (yy + 1 > _epd_height)
            ) {
                return;
            }
            // We will draw the next line in the follwing round of the for-loop
            while(refreshed_lines[yy]) taskYIELD();
            drawn_lines[yy] = true;
            drawn_lines_dirty = true;
        }
    }
}

void IRAM_ATTR vParallelDrawPixelsTask( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
       pvParameters value in the call to xTaskCreate() below. */ 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );

    for( ;; )
    {
        taskYIELD();  // Give the usb task a chance to run, otherwise we might lose data
        uint8_t args_index;
        xStreamBufferReceive(
            draw_pixels_args_full,
            &args_index,
            1, // size
            portMAX_DELAY
        );
        draw_pixels(
            &draw_pixels_queue[args_index]
        );
        xStreamBufferSend(
            draw_pixels_args_empty,
            &args_index, // data to send is the index where we wrote
            1, // just one byte,
            portMAX_DELAY
        );
    }
}


// This macro provides a shortcut to be used in digest_stream(). It can jump to processing the next data byte,
// without going through all the branches.
#define if_theres_more_data_goto_otherwise_continue(where) \
buf++; \
if(buf < buf_end) { \
    if(*buf != COMMAND_FOLLOWS) { \
        byte = *buf; \
        goto where; \
    } else { \
        if((buf+1<buf_end) && (*(buf+1)==SEND_128_BYTE)) { \
            buf++; \
            byte = 128; \
            goto where; \
        } \
        goto PARSING_LOOP_FIRST_ENTRY; \
    } \
} else return;

void IRAM_ATTR digest_stream(uint8_t* buf, size_t size)
{
    static uint32_t payload_bytes_counter = 0;
    uint8_t *buf_end = buf + size;
    
    goto PARSING_LOOP_FIRST_ENTRY;
    for(;;) {
        buf++;
        if(buf >= buf_end) break;
        PARSING_LOOP_FIRST_ENTRY:
        uint8_t byte = *buf;

        // Listen for unique "command follows" byte
        if( byte == COMMAND_FOLLOWS ) {
            previous_command = current_command;
            current_command = COMMAND_FOLLOWS;
            continue;
        }

        // If we are in "command follows"-state, the byte will tell which command
        if( current_command == COMMAND_FOLLOWS ) {
            //if(byte != SEND_128_BYTE)
            //    ESP_LOGI(TAG, "Parsing command: %hhu", byte);
            switch(byte) {
                case SEND_128_BYTE:
                    current_command = previous_command;
                    previous_command = UNKNOWN_COMMAND;
                    byte = 128;  // Will be processed below
                    break;
                case SCREEN_SIZE:
                    {
                        uint8_t screen_size[4] = {
                            (uint8_t)(_epd_width & 0xFF),
                            (uint8_t)(_epd_width >> 8),
                            (uint8_t)(_epd_height & 0xFF),
                            (uint8_t)(_epd_height >> 8)
                        };
                        write_command(SCREEN_SIZE, screen_size, 4);
                        write_flush();
                        previous_command = SCREEN_SIZE;
                        current_command = UNKNOWN_COMMAND;
                        continue;
                    }
                case CLEAR_DISPLAY:
                    ESP_LOGI(TAG, "Entering Command CLEAR_DISPLAY");
                    //epd_poweron();
                    //epd_fullclear(&hl, temperature);
                    //epd_hl_set_all_white(&hl);
                    memset(fb, 0xFF, fb_size);
                    epd_clear();
                    //epd_poweroff();
                    previous_command = CLEAR_DISPLAY;
                    current_command = UNKNOWN_COMMAND;
                    continue;
                case DRAW_IMAGE_2BIT:
                    //ESP_LOGI(TAG, "Entering Command DRAW_IMAGE_2BIT");
                    previous_command = current_command;
                    current_command = DRAW_IMAGE_2BIT;
                    payload_bytes_counter = 0;
                    current_pixelpack.x = 0;
                    current_pixelpack.y = 0;
                    continue;
                case REFRESH_DISPLAY:
                    previous_command = current_command;
                    current_command = REFRESH_DISPLAY;
                    continue;
                default:
                    ESP_LOGI(TAG, "do not know how to interpret this byte after 'command follows' byte: %hhu", byte);
                    continue;
            };
        }
            
        // process the payload depending on what command we are doing
        switch(current_command) { 
            case DRAW_IMAGE_2BIT:
                if( payload_bytes_counter >= sizeof(struct draw_img_ctx)) {

                    if( payload_bytes_counter == sizeof(struct draw_img_ctx) ) {
                        // In the first payload byte we find the color and the
                        // first three bits of the repeat-information
                        DRAW_IMAGE_2BIT_INNER_LOOP_GET_COLOR: 
                        current_pixelpack.color = byte & 0b111;  // two color bits and one bit for special operations
                        // we have only 4 bits left of the first byte for repeat information, because the left
                        // most bit tells if more repeat-bytes follow
                        current_pixelpack.repeat = (byte >> 3) & 0b1111;
                    } else {
                        // In every payload byte after that, we find seven more
                        // lower bits of the repeat information
                        DRAW_IMAGE_2BIT_INNER_LOOP_GET_SHIFT_BITS:
                        current_pixelpack.repeat <<= 7;
                        current_pixelpack.repeat |= byte & 0x7F;
                    }
                    //ESP_LOGI(TAG, "DRAW_IMAGE_2BIT color=%d repeats=%"PRIu32, current_pixelpack.color, current_pixelpack.repeat);

                    if( (byte) & 0x80 ) {
                        // There are more shift-bits to come
                        payload_bytes_counter += 1;
                        if_theres_more_data_goto_otherwise_continue(DRAW_IMAGE_2BIT_INNER_LOOP_GET_SHIFT_BITS);
                    }

                    // We have gathered all the repeat information
                    if(current_pixelpack.color==0b100) {  // draw transparent pixels
                        // Shortcut to jump over transparent pixels (=no change on fb)
                        current_pixelpack.repeat += current_pixelpack.x + 1;
                        current_pixelpack.y += current_pixelpack.repeat/current_image.width;
                        current_pixelpack.x = current_pixelpack.repeat%current_image.width;
                    } else {  // draw color or invert

                        // Take care of caching
                        if(draw_image_last_cached_line < current_pixelpack.y) {
                            // Cache the current line we're working on
                            My_Cache_Start_DCache_Preload(
                                ((uint32_t)fb_by_row[current_pixelpack.y]) + current_image.x,
                                current_image.width,
                                true
                            );
                            draw_image_last_cached_line = current_pixelpack.y;
                        } else {
                            if ((draw_image_last_cached_line == current_pixelpack.y) && Cache_DCache_Preload_Done()) {
                                // Cache the next line we will probably work on, if we're done caching the current one
                                if(current_pixelpack.y+1<_epd_height) {
                                    My_Cache_Start_DCache_Preload(
                                        ((uint32_t)fb_by_row[current_pixelpack.y+1]) + current_image.x,
                                        current_image.width,
                                        true
                                    );
                                }
                                draw_image_last_cached_line = current_pixelpack.y+1;
                            }
                        } 

                        bool delegated_drawing = false;
                        if(refreshcompute_active) {
                            uint8_t draw_queue_space = xStreamBufferBytesAvailable(draw_pixels_args_empty);
                            if(current_pixelpack.repeat >= (delegate_pixel_drawing_threshold>>8)) {
                                // Want to delegate drawing to the other core
                                if(draw_queue_space==0) {
                                    // But there's no space in the queue
                                    // -> We delegated too much before -> increase threshold to do some drawing ourselves
                                    delegate_pixel_drawing_threshold++;
                                } else {
                                    if(draw_queue_space<DRAW_PIXELS_QUEUE_LEN/2) {
                                        // We have some space in the queue, but not enough to be pretty sure we get
                                        // a spot most of the time we want to delegate
                                        delegate_pixel_drawing_threshold++;
                                    }
                                    if((current_image.height-current_pixelpack.y)>4) {
                                        // We only delegate the drawing if we're not almost done. Because then we need
                                        // some time left for the delegated tasks in the queue to finish
                                        // Put instructions into the queue for the other core to draw
                                        uint8_t args_index;
                                        xStreamBufferReceive(
                                            draw_pixels_args_empty,
                                            &args_index,
                                            1, // size
                                            portMAX_DELAY
                                        );

                                        memcpy(
                                            &draw_pixels_queue[args_index],
                                            &current_pixelpack,
                                            sizeof(struct draw_pixels_args)
                                        );

                                        xStreamBufferSend(
                                            draw_pixels_args_full,
                                            &args_index, // data to send is the index where we wrote
                                            1, // just one byte,
                                            portMAX_DELAY
                                        );

                                        // We have to compute the next position to draw ourselves, because the other core
                                        // has probably not done it yet, and also it only does that in its local copy
                                        // of the position information
                                        current_pixelpack.repeat += current_pixelpack.x + 1;
                                        current_pixelpack.y += current_pixelpack.repeat/current_image.width;
                                        current_pixelpack.x = current_pixelpack.repeat%current_image.width;

                                        delegated_drawing = true;
                                    }
                                }
                            } else {
                                // Want to draw ourselves
                                if((draw_queue_space==DRAW_PIXELS_QUEUE_LEN) && (delegate_pixel_drawing_threshold>0)) {
                                    // But the queue for delegation is empty
                                    // -> We drew too much ourselves before
                                    if(current_pixelpack.y > 4) {
                                        // We do not apply this for the first couple of lines to draw, because when
                                        // starting to draw the queue of course is empty
                                        delegate_pixel_drawing_threshold--;
                                    }
                                }
                            }
                        }
                        
                        if(!delegated_drawing){
                            // draw it directly ourselves
                            draw_pixels(&current_pixelpack);
                        }
                        
                    }

                    if( 
                        current_pixelpack.y + 1 > current_image.height
                    ) {
                        goto DRAW_IMAGE_2BIT_END;
                    }
                    // Next, there will be no more repeat-bits, but
                    // a byte that denotes the next color to set
                    payload_bytes_counter = sizeof(struct draw_img_ctx);
                    
                    if_theres_more_data_goto_otherwise_continue(DRAW_IMAGE_2BIT_INNER_LOOP_GET_COLOR);
                } else {
                    DRAW_IMAGE_2BIT_PARSE_IMAGE_HEADER:
                    //ESP_LOGI(TAG, "payload_bytes_counter: %lu, byte: %hhu", payload_bytes_counter, byte);
                    switch (payload_bytes_counter) {
                        case 0: current_image.x = ( (uint16_t)byte ); break;
                        case 1: current_image.x |= ( (uint16_t)byte ) << 8; break;
                        case 2: current_image.y = ( (uint16_t)byte ); break;
                        case 3: current_image.y |= ( (uint16_t)byte )<< 8; break;
                        case 4: current_image.width = (uint16_t)byte; break;
                        case 5: current_image.width |= ( (uint16_t)byte ) << 8; break;
                        case 6: current_image.height = (uint16_t)byte; break;
                        case 7:
                            current_image.height |= ( (uint16_t)byte ) << 8;
                            if(
                                (current_image.x+current_image.width > _epd_width) ||
                                (current_image.y+current_image.height > _epd_height)
                            ) {
                                ESP_LOGE(TAG, "Image overlaps screen edges, skipping! pos: %u,%u size: %u,%u", current_image.x, current_image.y, current_image.width, current_image.height);
                                goto DRAW_IMAGE_2BIT_END;
                            }
                            current_pixelpack.y = 0;
                            current_pixelpack.x = 0;
                            drawn_columns_start = min(drawn_columns_start, current_image.x);
                            drawn_columns_end = max(drawn_columns_end, current_image.x+current_image.width);
                            My_Cache_Start_DCache_Preload(
                                ((uint32_t)fb_by_row[current_image.y]) + current_image.x,
                                current_image.width,
                                true
                            );
                            draw_image_last_cached_line = current_image.y;
                            //ESP_LOGI(TAG, "Drawing image 2bit; 
                            payload_bytes_counter = sizeof(struct draw_img_ctx);
                            //continue;
                            if_theres_more_data_goto_otherwise_continue(DRAW_IMAGE_2BIT_INNER_LOOP_GET_COLOR);
                    };
                    payload_bytes_counter++;
                    //continue;
                    if_theres_more_data_goto_otherwise_continue(DRAW_IMAGE_2BIT_PARSE_IMAGE_HEADER);
                }
                DRAW_IMAGE_2BIT_END:
                previous_command = current_command;
                current_command = UNKNOWN_COMMAND;
                continue;

            case REFRESH_DISPLAY:
                do_refresh();
                write_command(REFRESH_DISPLAY, &byte, 1);
                write_flush();
                previous_command = REFRESH_DISPLAY;
                current_command = UNKNOWN_COMMAND;
                continue;

            default:
                if( current_command != previous_command ) {
                    ESP_LOGI(TAG, "Current command is unknown: %hhu", current_command);
                    previous_command = current_command;
                }
                continue;
        };
    }
}

void IRAM_ATTR tinyusb_cdc_rx_callback(tinyusb_cdcacm_itf_t itf, cdcacm_event_t *event)
{
    uint8_t this_chunk;
    if(xStreamBufferBytesAvailable(usb_data_chunks_empty)==0) {
        ESP_LOGW(TAG, "SLOW: USB data available, but need to wait for empty chunk.");
    }
    xStreamBufferReceive(
        usb_data_chunks_empty,
        &this_chunk,
        1, // size
        portMAX_DELAY  // Ticks to wait
    );
    esp_err_t ret = tinyusb_cdcacm_read(
        itf,
        usb_data_chunk[this_chunk].data,
        CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        &usb_data_chunk[this_chunk].len
    );
    //ESP_LOGI("TAG", "Received %u bytes", usb_data_chunk[this_chunk].len);
    if (ret == ESP_OK) {
        xStreamBufferSend(
            usb_data_chunks_full,
            &this_chunk, // data to send is the index where we wrote
            1, // just one byte,
            portMAX_DELAY
        );
        //ESP_LOGI(TAG, "Enqued buffer %hhu", this_chunk);
    } else {
        ESP_LOGE(TAG, "USB Read error");
        xStreamBufferSend(
            usb_data_chunks_empty,
            &this_chunk, // data to send is the index where we wrote
            1, // just one byte,
            portMAX_DELAY
        );
    }
}



void idf_setup() {
    async_memcpy_config_t async_memcpy_config = ASYNC_MEMCPY_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_async_memcpy_install(&async_memcpy_config, &async_memcpy_driver)); // install driver with default DMA engine


    epd_init(&DEMO_BOARD, &ED133UT2, EPD_LUT_1K); // EPD_LUT_64K);
    // Set VCOM for boards that allow to set this in software (in mV).
    // This will print an error if unsupported. In this case,
    // set VCOM using the hardware potentiometer and delete this line.
    epd_set_vcom(2200);
    //hl = epd_hl_init(WAVEFORM);

    _epd_width = epd_width();
    _epd_height = epd_height();
    fb_size = _epd_width * _epd_height;
    //fb = heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    fb = (uint8_t*)heap_caps_aligned_alloc(16, fb_size, MALLOC_CAP_SPIRAM);
    memset(fb, 0xFF, fb_size);
    fb_by_row = (uint8_t**)heap_caps_malloc(sizeof(fb)*_epd_height, MALLOC_CAP_INTERNAL);
    for(uint16_t y=0; y<_epd_height; y++) {
        fb_by_row[y] = fb + y*_epd_width;
    }
    drawn_lines = (bool*)heap_caps_malloc(_epd_height, MALLOC_CAP_INTERNAL);
    memset(drawn_lines, 0xff, _epd_height);
    drawn_lines_dirty = true;
    refreshed_lines = (bool*)heap_caps_malloc(_epd_height, MALLOC_CAP_INTERNAL);
    memset(refreshed_lines, 0x00, _epd_height);
    


    // Default orientation is EPD_ROT_LANDSCAPE
    epd_set_rotation(EPD_ROT_LANDSCAPE);

    printf(
        "Dimensions after rotation, width: %d height: %d\n\n", epd_rotated_display_width(),
        epd_rotated_display_height()
    );


    /* USB CDC Init */
    init_usb_data_chunks();
    init_usb_connection((tusb_cdcacm_callback_t)&tinyusb_cdc_rx_callback);

    draw_pixels_args_empty = xStreamBufferCreate(
        DRAW_PIXELS_QUEUE_LEN, // size in bytes
        1 // trigger level
    );
    draw_pixels_args_full = xStreamBufferCreate(
        DRAW_PIXELS_QUEUE_LEN, // size in bytes
        1 // trigger level
    );
    for(uint8_t i=0; i<DRAW_PIXELS_QUEUE_LEN; i++) {
        xStreamBufferSend(
            draw_pixels_args_empty,
            &i, // data to send is the indices
            1, // just one byte,
            0  // do not wait for space to become free, because there's enough space
        );
    }

    xTaskCreatePinnedToCore(
        vParallelDrawPixelsTask,       /* Function that implements the task. */
        "vParallelDrawPixelsTask",          /* Text name for the task. */
        2000,      /* Stack size in words, not bytes. */
        ( void * ) 1,    /* Parameter passed into the task. */
        3,/* Priority at which the task is created. */
        NULL,             /* Used to pass out the created task's handle. */
        0                /* Core where the task should run. */
    );
    xTaskCreatePinnedToCore(
        refresh_compute,       /* Function that implements the task. */
        "refresh_compute",          /* Text name for the task. */
        2000,      /* Stack size in words, not bytes. */
        ( void * ) 1,    /* Parameter passed into the task. */
        4,/* Priority at which the task is created. */
        &refresh_compute_task,             /* Used to pass out the created task's handle. */
        0                /* Core where the task should run. */
    );

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
}


void IRAM_ATTR idf_loop() {

    temperature = epd_ambient_temperature();
    printf("current temperature: %d\n", temperature);

    epd_poweron();
    is_powered_on = true;
    //for(uint8_t j=0; j<50; j++)
        epd_clear();

    for(uint8_t j=0; j<=2; j++) {
        for(uint16_t yy=0; yy<_epd_height; yy++) {
            while(refreshed_lines[yy]) {
                taskYIELD();
            }
            for(uint16_t xx=0; xx<_epd_width; xx++) {
                uint32_t i = yy*_epd_width+xx;
                fb[i] = (j&1?0xF0:0x00) | (fb[i]&0x0F);
            }
            drawn_lines[yy] = true;
        }
        drawn_lines_dirty = true;
        drawn_columns_start = 0;
        drawn_columns_end = _epd_width;
        do_refresh();
    }

    ESP_LOGI(TAG, "All refreshes done");

    size_t bytes_written;
    uint32_t idle_ctr = 0;
    uint8_t this_chunk;
    while (1) {
        bytes_written = xStreamBufferReceive(
            usb_data_chunks_full,
            &this_chunk,
            1, // size
            pdMS_TO_TICKS(10)
        );
        if(bytes_written == 0) {
            if(idle_ctr < ((uint32_t)2<<30))
                idle_ctr += 1;
            if(idle_ctr == 30) {
                while(drawn_lines_dirty) {
                    do_refresh();
                }
            }
            if(idle_ctr == 500) {
                if(is_powered_on) {
                    printf("Saving power...\n");
                    epd_poweroff();
                    is_powered_on = false;
                }
            }
            continue;
        }
        idle_ctr = 0;
        if(!is_powered_on) {
            epd_poweron();
            is_powered_on = true;
        }
        digest_stream(
            usb_data_chunk[this_chunk].data,
            usb_data_chunk[this_chunk].len
        );
        //ESP_LOGI(TAG, "Processed buffer %hhu (len %d bytes)", this_chunk, usb_data_chunk[this_chunk].len);
        xStreamBufferSend(
            usb_data_chunks_empty,
            &this_chunk, // data to send is the indices
            1, // just one byte,
            portMAX_DELAY
        );
        //printf("Digested %d bytes\n", bytes_written);
    }

    printf("going to sleep...\n");
    epd_deinit();
}

void app_main(void) {
    idf_setup();

    while (1) {
        idf_loop();
    };
}
