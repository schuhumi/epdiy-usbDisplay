#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <epdiy.h>

#include "sdkconfig.h"

//#include "firasans_12.h"
//#include "firasans_20.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _b : _a; })

#define WAVEFORM EPD_BUILTIN_WAVEFORM

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


uint8_t *fb;
uint8_t** fb_by_row;
uint32_t fb_size;
int temperature = 17;
uint32_t refresh_ctr = 0;
int _epd_width;
int _epd_height;

uint16_t cursor_pos_x = 500;
uint16_t cursor_pos_y = 500;
uint16_t cursor_pos_x_previous = 500;
uint16_t cursor_pos_y_previous = 500;
const uint8_t cursor_pointer[] = {
    0xF, 0xF, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
    0xF, 0x0, 0xF, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
    0xF, 0x0, 0x0, 0xF, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
    0xF, 0x0, 0x0, 0x0, 0xF, 0x1, 0x1, 0x1, 0x1, 0x1,
    0xF, 0x0, 0x0, 0x0, 0x0, 0xF, 0x1, 0x1, 0x1, 0x1,
    0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0x1, 0x1, 0x1,
    0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0x1, 0x1,
    0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0x1,
    0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF,
    0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0xF, 0xF, 0xF,
    0xF, 0x0, 0x0, 0xF, 0x0, 0x0, 0xF, 0x1, 0x1, 0x1,
    0xF, 0x0, 0xF, 0x1, 0xF, 0x0, 0x0, 0xF, 0x1, 0x1,
    0xF, 0xF, 0x1, 0x1, 0xF, 0x0, 0x0, 0xF, 0x1, 0x1,
    0x1, 0x1, 0x1, 0x1, 0x1, 0xF, 0x0, 0x0, 0xF, 0x1,
    0x1, 0x1, 0x1, 0x1, 0x1, 0xF, 0x0, 0x0, 0xF, 0x1,
    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0xF, 0xF, 0x1, 0x1
};
const uint8_t cursor_pointer_width = 10;
const uint8_t cursor_pointer_height = 16;

/*#define ringlen 8192
uint8_t ringbuf[ringlen];
uint16_t ringwriter = 1;
uint16_t ringreader = 0;*/

uint8_t buf[8][CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
volatile size_t buf_size[8] = {0};
volatile uint8_t buf_write_selector_current=0;
volatile uint8_t buf_read_selector_current=0;
#define WRITE_SELECTOR_INACTIVE 0
#define WRITE_SELECTOR_USED_BY_WRITER 1
#define WRITE_SELECTOR_USED_BY_READER 2
volatile uint8_t write_selector_user = WRITE_SELECTOR_INACTIVE;


static const char *TAG = "EPDiyGraphics";

static inline void checkError(enum EpdDrawError err) {
    if (err != EPD_DRAW_SUCCESS) {
        ESP_LOGE("demo", "draw error: %X", err);
    }
}

enum command {
    SEND_128_BYTE = 0,
    CLEAR_DISPLAY = 1,
    SET_ORIGIN_POS = 2,
    DRAW_IMAGE = 3,
    REFRESH_DISPLAY = 4,
    REFRESH_AREA = 5,
    CURSOR_MOVE = 6,
    ECHO_SYNC = 252,
    UNKNOWN_COMMAND = 254,
    COMMAND_FOLLOWS = 128
};


enum command current_command = UNKNOWN_COMMAND;
enum command previous_command = UNKNOWN_COMMAND; // Needed when SEND_128_BYTE Command comes
uint32_t payload_bytes_counter = 0;
uint16_t origin_x_pos = 0;
uint16_t origin_y_pos = 0;
uint16_t draw_image_width = 0;
uint16_t draw_image_height = 0;
uint16_t draw_image_current_x = 0;
uint16_t draw_image_current_y = 0;
uint32_t draw_image_repeat = -1;
uint8_t draw_image_color = 0;
EpdRect refresh_area_coords;
bool* drawn_lines;

void IRAM_ATTR digest_stream(uint8_t* buf, size_t size)
{
    uint8_t *buf_end = buf + size;
    
    while(buf < buf_end) {
        uint8_t byte = *buf;
        buf++;

        // Listen for unique "command follows" byte
        if( byte == COMMAND_FOLLOWS ) {
            previous_command = current_command;
            current_command = COMMAND_FOLLOWS;
            continue;
        }

        // If we are in "command follows"-state, the byte will tell which command
        if( current_command == COMMAND_FOLLOWS ) {
            switch(byte) {
                case SEND_128_BYTE:
                    current_command = previous_command;
                    previous_command = UNKNOWN_COMMAND;
                    byte = 128;  // Will be processed below
                    break;
                case CLEAR_DISPLAY:
                    //ESP_LOGI(TAG, "Entering Command CLEAR_DISPLAY");
                    //epd_poweron();
                    //epd_fullclear(&hl, temperature);
                    //epd_hl_set_all_white(&hl);
                    memset(fb, 0xFF, fb_size);
                    epd_clear();
                    //epd_poweroff();
                    previous_command = CLEAR_DISPLAY;
                    current_command = UNKNOWN_COMMAND;
                    continue;
                case SET_ORIGIN_POS:
                    //ESP_LOGI(TAG, "Entering Command SET_ORIGIN_POS");
                    previous_command = current_command;
                    current_command = SET_ORIGIN_POS;
                    payload_bytes_counter = 0;
                    continue;
                case DRAW_IMAGE:
                    //ESP_LOGI(TAG, "Entering Command DRAW_IMAGE");
                    previous_command = current_command;
                    current_command = DRAW_IMAGE;
                    payload_bytes_counter = 0;
                    draw_image_current_x = 0;
                    draw_image_current_y = 0;
                    continue;
                case REFRESH_DISPLAY:
                    //ESP_LOGI(TAG, "Entering Command REFRESH_DISPLAY");
                    //epd_poweron();
                    //checkError(epd_hl_update_screen(&hl, MODE_DU, temperature));
                    //checkError(epd_hl_update_screen(&hl, MODE_GL16, temperature));
                    /*enum EpdDrawMode mode = (enum EpdDrawMode)(MODE_GL16 | MODE_PACKING_1PPB_DIFFERENCE);
                    enum EpdDrawError _err = epd_draw_base(
                        epd_full_screen(),
                        fb,
                        epd_full_screen(),
                        mode,
                        temperature,
                        NULL,
                        epd_get_display()->default_waveform
                    );*/
                    //epd_poweroff();
                    previous_command = REFRESH_DISPLAY;
                    current_command = UNKNOWN_COMMAND;
                    continue;
                case REFRESH_AREA:
                    //ESP_LOGI(TAG, "Entering Command SET_ORIGIN_POS");
                    previous_command = current_command;
                    current_command = REFRESH_AREA;
                    payload_bytes_counter = 0;
                    continue;
                case CURSOR_MOVE:
                    previous_command = current_command;
                    current_command = CURSOR_MOVE;
                    payload_bytes_counter = 0;
                    continue;
                case ECHO_SYNC:
                    previous_command = current_command;
                    current_command = UNKNOWN_COMMAND;
                    tinyusb_cdcacm_write_queue_char(TINYUSB_CDC_ACM_0, ECHO_SYNC);
                    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
                    payload_bytes_counter = 0;
                default:
                    continue;
            };
        }
            
        // process the payload depending on what command we are doing
        switch(current_command) {
            case SET_ORIGIN_POS:
                switch (payload_bytes_counter) {
                    case 0: origin_x_pos = (origin_x_pos & 0x00FF) | ( (uint16_t)byte << 8 ); break;
                    case 1: origin_x_pos = (origin_x_pos & 0xFF00) | ( (uint16_t)byte ); break;
                    case 2: origin_y_pos = (origin_y_pos & 0x00FF) | ( (uint16_t)byte << 8 ); break;
                    case 3: origin_y_pos = (origin_y_pos & 0xFF00) | ( (uint16_t)byte ); break;
                };
                payload_bytes_counter += 1;
                continue;
            case DRAW_IMAGE:
                if( payload_bytes_counter >= 4) {
                                    

                    if( payload_bytes_counter == 4 ) {
                        // In the first payload byte we find the color and the
                        // first three bits of the repeat-information
                        DRAW_IMAGE_INNER_LOOP_GET_COLOR: 
                        draw_image_color = byte & 0b11;
                        draw_image_repeat = (byte >> 2) & 0b11111;
                    } else {
                        // In every payload byte after that, we find seven more
                        // lower bits of the repeat information
                        DRAW_IMAGE_INNER_LOOP_GET_SHIFT_BITS:
                        draw_image_repeat <<= 7;
                        draw_image_repeat |= byte & 0x7F;
                    }
                    //ESP_LOGI(TAG, "DRAW_IMAGE color=%d repeats=%"PRIu32, draw_image_color, draw_image_repeat);

                    if( (byte) & 0x80 ) {
                        // There are more shift-bits to come
                        payload_bytes_counter += 1;
                        goto DRAW_IMAGE_CHECK_NEXT_BYTE_FOR_SHIFT_BITS;
                    }

                    // We have gathered all the repeat information
                    if(draw_image_color==0b10) {  // draw transparent pixels
                        // Shortcut to jump over transparent pixels (=no change on fb)
                        draw_image_repeat += draw_image_current_x + 1;
                        draw_image_current_y += draw_image_repeat/draw_image_width;
                        draw_image_current_x = draw_image_repeat%draw_image_width;
                    } else {  // draw color or invert
                        uint16_t yy = origin_y_pos + draw_image_current_y;
                        uint8_t *fb_yy = fb_by_row[yy];
                        for(uint32_t px_ctr=0; px_ctr<draw_image_repeat+1;) {
                            uint16_t xx_start = origin_x_pos + draw_image_current_x;
                            // xx_stop denotes where we stop drawing the current row of the picture
                            // (due to different reasons)
                            uint16_t xx_count = min(
                                draw_image_repeat+1-px_ctr,
                                draw_image_width-draw_image_current_x
                            );
                            uint8_t *buf_ptr = fb_yy + xx_start;
                            if(xx_count >= 2) {
                                // If we have at least 2 repeating pixels left in this row, we take a shortcut,
                                // as we can skip calculating jumps into the next row, overflowing
                                // the screen and other checks
                                uint8_t *buf_ptr_stop = fb_yy + xx_start + xx_count;
                                // We do differentiate between putting a color and inverting outside
                                // the loop, so we do not do this branch for every loop iteration again.
                                if(draw_image_color==0b11) {  // invert
                                    for(; buf_ptr<buf_ptr_stop; buf_ptr++){
                                        *buf_ptr = (*buf_ptr & 0x0F) | (~(*buf_ptr) & 0xF0);
                                    }
                                } else {
                                    for(; buf_ptr<buf_ptr_stop; buf_ptr++){
                                        *buf_ptr = (*buf_ptr & 0x0F) | (draw_image_color&1?0xF0:0x00);
                                    }
                                }
                                // Since we only advanced in the row, we only need to update these
                                // variables
                                draw_image_current_x += xx_count;
                                px_ctr += xx_count;
                            }
                            else {
                                // If we have just 1 or zero repeating pixels left in the row, we take
                                // care of screen dimensions and draw it
                                if(draw_image_color==0b11) {
                                    *buf_ptr = (*buf_ptr & 0x0F) | (~(*buf_ptr) & 0xF0);
                                } else {
                                    *buf_ptr = (*buf_ptr & 0x0F) | (draw_image_color&1?0xF0:0x00);
                                }
                                draw_image_current_x++;
                                px_ctr++;
                            }
                            // All the logic of wrapping into the next row etc follows here
                            if( draw_image_current_x + 1 > draw_image_width ) {
                                draw_image_current_x = 0;
                                draw_image_current_y++;
                                yy++;
                                fb_yy += _epd_width;
                                if( 
                                    (draw_image_current_y + 1 > draw_image_height) ||
                                    (yy + 1 > _epd_height)
                                )
                                    goto DRAW_IMAGE_END;
                            }
                        }
                    }
                    // Next, there will be no more repeat-bits, but
                    // a byte that denotes the next color to set
                    payload_bytes_counter = 4;
                    
                    if(buf < buf_end) {
                        if(*buf != COMMAND_FOLLOWS) {
                            byte = *buf;
                            buf++;
                            goto DRAW_IMAGE_INNER_LOOP_GET_COLOR;
                        } else {
                            if(buf + 1 < buf_end) {
                                if(*(buf+1)==SEND_128_BYTE) {
                                    byte = 128;
                                    buf += 2;
                                    goto DRAW_IMAGE_INNER_LOOP_GET_COLOR;
                                }
                            }
                        }
                    }
                    continue;
                    DRAW_IMAGE_CHECK_NEXT_BYTE_FOR_SHIFT_BITS:
                    if(buf < buf_end) {
                        if(*buf != COMMAND_FOLLOWS) {
                            byte = *buf;
                            buf++;
                            goto DRAW_IMAGE_INNER_LOOP_GET_SHIFT_BITS;
                        } else {
                            if(buf + 1 < buf_end) {
                                if(*(buf+1)==SEND_128_BYTE) {
                                    byte = 128;
                                    buf += 2;
                                    goto DRAW_IMAGE_INNER_LOOP_GET_SHIFT_BITS;
                                }
                            }
                        }
                    }
                    continue;
                }
                switch (payload_bytes_counter) {
                    case 0: draw_image_width = (uint16_t)byte << 8; break;
                    case 1: draw_image_width |= (uint16_t)byte; break;
                    case 2: draw_image_height = (uint16_t)byte << 8; break;
                    case 3:
                        draw_image_height |= (uint16_t)byte;
                        if(
                            (origin_x_pos+draw_image_width > _epd_width) ||
                            (origin_y_pos+draw_image_height > _epd_height)
                        ) {
                            printf("Image overlaps screen edges, skipping!");
                            goto DRAW_IMAGE_END;
                        }
                        break;
                };
                payload_bytes_counter += 1;
                continue;
                DRAW_IMAGE_END:
                previous_command = current_command;
                current_command = UNKNOWN_COMMAND;
                continue;
            case REFRESH_AREA:
                switch (payload_bytes_counter) {
                    case 0: refresh_area_coords.x = ( (uint16_t)byte << 8 ); break;
                    case 1: refresh_area_coords.x = (refresh_area_coords.x & 0xFF00) | ( (uint16_t)byte ); break;
                    case 2: refresh_area_coords.y = ( (uint16_t)byte << 8 ); break;
                    case 3: refresh_area_coords.y = (refresh_area_coords.y & 0xFF00) | ( (uint16_t)byte ); break;
                    case 4: refresh_area_coords.width = ( (uint16_t)byte << 8 ); break;
                    case 5: refresh_area_coords.width = (refresh_area_coords.width & 0xFF00) | ( (uint16_t)byte ); break;
                    case 6: refresh_area_coords.height = ( (uint16_t)byte << 8 ); break;
                    case 7:
                        refresh_area_coords.height = (refresh_area_coords.height & 0xFF00) | ( (uint16_t)byte );

                        tinyusb_cdcacm_write_queue_char(TINYUSB_CDC_ACM_0, REFRESH_AREA);
                        tinyusb_cdcacm_write_queue_char(
                            TINYUSB_CDC_ACM_0, 
                            (refresh_area_coords.x^refresh_area_coords.y^refresh_area_coords.width^refresh_area_coords.height)&0xFF
                        );
                        tinyusb_cdcacm_write_queue_char(TINYUSB_CDC_ACM_0, '\n');
                        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);

                        uint8_t pixels_behind_cursor[sizeof(cursor_pointer)] = {0};
                        for(uint16_t y=0; y<cursor_pointer_height; y++) {
                            for (uint16_t x=0; x<cursor_pointer_width; x++)
                            {
                                int fb_x = cursor_pos_x + x;
                                int fb_y = cursor_pos_y + y;
                                if(
                                    (fb_x<0) ||
                                    (fb_y<0) ||
                                    (fb_x>_epd_width) ||
                                    (fb_y>_epd_height)
                                )
                                    continue;
                                uint8_t* fb_px = fb_by_row[fb_y]+fb_x;
                                uint16_t cursor_idx = (y*cursor_pointer_width)+x;
                                pixels_behind_cursor[cursor_idx] = *fb_px;
                                if(cursor_pointer[cursor_idx]==0)
                                    *fb_px &= 0x0F;
                                else if(cursor_pointer[cursor_idx]==0xF)
                                    *fb_px |= 0xF0 ;
                            }
                        }
                        if((cursor_pos_x!=cursor_pos_x_previous) || (cursor_pos_y!=cursor_pos_y_previous))
                            refresh_ctr += 1;
                        refresh_ctr += (refresh_area_coords.width>>9)*(refresh_area_coords.height>>9);
                        enum EpdDrawMode mode;
                        mode = (enum EpdDrawMode)(MODE_DU | MODE_PACKING_1PPB_DIFFERENCE);
                        if(false) { //if(refresh_ctr>=100) {
                            refresh_ctr = 0;
                            for(size_t i=0; i<fb_size; i++)
                                fb[i] = (fb[i] & 0x0F) | (~fb[i] & 0xF0);
                            epd_draw_base(
                                epd_full_screen(),
                                fb,
                                epd_full_screen(),
                                mode,
                                temperature,
                                NULL,
                                epd_get_display()->default_waveform
                            );
                            for(size_t i=0; i<fb_size; i++)
                                fb[i] = ((fb[i]>>4)&0x0F) | (~fb[i] & 0xF0);
                            epd_draw_base(
                                epd_full_screen(),
                                fb,
                                epd_full_screen(),
                                mode,
                                temperature,
                                NULL,
                                epd_get_display()->default_waveform
                            );
                            for(size_t i=0; i<fb_size; i++)
                                fb[i] = (fb[i]&0xF0) | ((fb[i]>>4)&0x0F);
                        } else {
                            memset(drawn_lines, 0x00, _epd_height);
                            for(uint16_t y=cursor_pos_y; y<cursor_pos_y+cursor_pointer_height; y++)
                                drawn_lines[y] = true;
                            for(uint16_t y=cursor_pos_y_previous; y<cursor_pos_y_previous+cursor_pointer_height; y++)
                                drawn_lines[y] = true;
                            for(uint16_t y=refresh_area_coords.y; y<refresh_area_coords.y+refresh_area_coords.height; y++)
                                drawn_lines[y] = true;
                        
                            epd_draw_base(
                                epd_full_screen(),
                                fb,
                                //refresh_area_coords,
                                epd_full_screen(),
                                mode,
                                temperature,
                                drawn_lines,
                                epd_get_display()->default_waveform
                            );

                            for(uint16_t y=0; y<cursor_pointer_height; y++) {
                                for (uint16_t x=0; x<cursor_pointer_width; x++)
                                {
                                    int fb_x = cursor_pos_x_previous + x;
                                    int fb_y = cursor_pos_y_previous + y;
                                    if(
                                        (fb_x<0) ||
                                        (fb_y<0) ||
                                        (fb_x>_epd_width) ||
                                        (fb_y>_epd_height)
                                    )
                                        continue;
                                    uint8_t* fb_px = fb_by_row[fb_y]+fb_x;
                                    uint16_t cursor_idx = (y*cursor_pointer_width)+x;
                                    *fb_px = (*fb_px&0xF0) | ((*fb_px>>4)&0x0F);
                                }
                            }

                            for(uint16_t yy=refresh_area_coords.y; yy<refresh_area_coords.y+refresh_area_coords.height; yy++) {
                                for(uint16_t xx=refresh_area_coords.x; xx<refresh_area_coords.x+refresh_area_coords.width; xx++) {
                                    uint32_t i = yy*_epd_width+xx;
                                    fb[i] = (fb[i]&0xF0) | ((fb[i]>>4)&0x0F);
                                }
                            }
                        }

                        for(uint16_t y=0; y<cursor_pointer_height; y++) {
                            for (uint16_t x=0; x<cursor_pointer_width; x++)
                            {
                                int fb_x = cursor_pos_x + x;
                                int fb_y = cursor_pos_y + y;
                                if(
                                    (fb_x<0) ||
                                    (fb_y<0) ||
                                    (fb_x>_epd_width) ||
                                    (fb_y>_epd_height)
                                )
                                    continue;
                                uint8_t* fb_px = fb_by_row[fb_y]+fb_x;
                                uint16_t cursor_idx = (y*cursor_pointer_width)+x;
                                *fb_px = (pixels_behind_cursor[cursor_idx]&0xF0) | ((*fb_px>>4)&0x0F);
                            }
                        }
                        cursor_pos_x_previous = cursor_pos_x;
                        cursor_pos_y_previous = cursor_pos_y;
                        break;
                };
                payload_bytes_counter += 1;
                continue;
            case CURSOR_MOVE:
                switch (payload_bytes_counter) {
                    case 0: cursor_pos_x = ( (uint16_t)byte << 8 ); break;
                    case 1: cursor_pos_x |= ( (uint16_t)byte ); break;
                    case 2: cursor_pos_y = ( (uint16_t)byte << 8 ); break;
                    case 3: cursor_pos_y |= ( (uint16_t)byte ); break;
                };
                payload_bytes_counter++;
                continue;
            default:
                break;
        };
    }
}




void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    CDC_RX_AQUIRE_LOCK:
    while(write_selector_user!=WRITE_SELECTOR_INACTIVE){
        vTaskDelay(1);
    }
    write_selector_user = WRITE_SELECTOR_USED_BY_WRITER;
    asm volatile ("nop");
    if(write_selector_user!=WRITE_SELECTOR_USED_BY_WRITER)
        goto CDC_RX_AQUIRE_LOCK;

    uint8_t buf_write_selector_next = (buf_write_selector_current+1)&0b111;
    if(buf_size[buf_write_selector_current]>=CONFIG_TINYUSB_CDC_RX_BUFSIZE) {
        while( buf_write_selector_next == buf_read_selector_current ) {
            // wait
            printf("Waiting to write buf %d\n", buf_write_selector_next);
            vTaskDelay(10);
        }
        buf_write_selector_current = buf_write_selector_next;
        buf_write_selector_next = (buf_write_selector_next+1)&0b111;
        buf_size[buf_write_selector_current] = 0;
    }
    size_t bytes_written;
    esp_err_t ret = tinyusb_cdcacm_read(
        itf,
        buf[buf_write_selector_current]+buf_size[buf_write_selector_current],
        CONFIG_TINYUSB_CDC_RX_BUFSIZE-buf_size[buf_write_selector_current],
        &bytes_written
    );
    buf_size[buf_write_selector_current] += bytes_written;
    /*if (ret == ESP_OK) {
        printf("Appended %d bytes to buf %d\n", bytes_written, buf_write_selector_current);
    } else {
        ESP_LOGE(TAG, "Read error");
    }*/
    write_selector_user = WRITE_SELECTOR_INACTIVE;
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

void idf_setup() {
    epd_init(&DEMO_BOARD, &ED133UT2, EPD_LUT_64K);
    // Set VCOM for boards that allow to set this in software (in mV).
    // This will print an error if unsupported. In this case,
    // set VCOM using the hardware potentiometer and delete this line.
    epd_set_vcom(2200);
    //hl = epd_hl_init(WAVEFORM);

    _epd_width = epd_width();
    _epd_height = epd_height();
    fb_size = epd_width() * epd_height();
    fb = heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    memset(fb, 0xFF, fb_size);
    fb_by_row = heap_caps_malloc(sizeof(fb)*_epd_height, MALLOC_CAP_SPIRAM);
    for(uint16_t y=0; y<_epd_height; y++) {
        fb_by_row[y] = fb + y*_epd_width;
    }
    drawn_lines = heap_caps_malloc(epd_height(), MALLOC_CAP_SPIRAM);

    // Default orientation is EPD_ROT_LANDSCAPE
    epd_set_rotation(EPD_ROT_LANDSCAPE);

    printf(
        "Dimensions after rotation, width: %d height: %d\n\n", epd_rotated_display_width(),
        epd_rotated_display_height()
    );

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);

    /* USB CDC Init */

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 4096,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

    printf("CONFIG_TINYUSB_CDC_RX_BUFSIZE=%d\n", CONFIG_TINYUSB_CDC_RX_BUFSIZE);
    ESP_LOGI(TAG, "USB initialization DONE");
}


void idf_loop() {
    // select the font based on display width
    /*const EpdFont* font;
    if (epd_width() < 1000) {
        font = &FiraSans_12;
    } else {
        font = &FiraSans_20;
    }*/

    //uint8_t* fb = epd_hl_get_framebuffer(&hl);

    epd_poweron();
    epd_clear();
    temperature = epd_ambient_temperature();
    //epd_poweroff();

    printf("current temperature: %d\n", temperature);

    while (1) {
        while(buf_write_selector_current == buf_read_selector_current) {
            
            if((write_selector_user==WRITE_SELECTOR_INACTIVE) && (buf_size[buf_write_selector_current]>0)) {
                write_selector_user = WRITE_SELECTOR_USED_BY_READER;
                asm volatile ("nop");
                if(write_selector_user==WRITE_SELECTOR_USED_BY_READER) {
                    if(buf_write_selector_current == buf_read_selector_current) {
                        printf(
                            "Caught up to non-empty buffer-to-write at %d, kicking over read selector to next buf\n",
                            buf_read_selector_current
                        );
                        buf_write_selector_current = (buf_write_selector_current+1)&0b111;
                        buf_size[buf_write_selector_current] = 0;
                    }
                    write_selector_user = WRITE_SELECTOR_INACTIVE;
                }
            } else {
                //printf("Caught up to empty buffer-to-write at %d, waiting...\n", buf_read_selector_current);
                vTaskDelay(1);
            }
        }
        
        digest_stream(
            buf[buf_read_selector_current],
            buf_size[buf_read_selector_current]
        );
        printf("Read %d bytes from buf %d\n", buf_size[buf_read_selector_current], buf_read_selector_current);
        buf_read_selector_current = (buf_read_selector_current+1)&0b111;
    }

    printf("going to sleep...\n");
    epd_deinit();
    //esp_deep_sleep_start();
}

#ifndef ARDUINO_ARCH_ESP32
void app_main() {
    idf_setup();

    while (1) {
        idf_loop();
    };
}
#endif
