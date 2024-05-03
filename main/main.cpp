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

#include "sdkconfig.h"


#include "tinyusb.h"
#include "tusb_cdc_acm.h"

using namespace std;

extern "C" void app_main();




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

static const char *TAG = "EPDiyGraphics";

async_memcpy_t async_memcpy_driver;

uint8_t *fb;
uint8_t** fb_by_row;
uint32_t fb_size;
int temperature = 17;
int _epd_width;
int _epd_height;




#define NUM_USB_DATA_CHUNKS 32
struct usb_data_chunk_t {
    size_t len;
    uint8_t* data;
};
struct usb_data_chunk_t usb_data_chunk[NUM_USB_DATA_CHUNKS];
StreamBufferHandle_t usb_data_chunks_empty;
StreamBufferHandle_t usb_data_chunks_full;


/*size_t xCDCStreamBufferSize = CONFIG_TINYUSB_CDC_RX_BUFSIZE*2;
static StreamBufferHandle_t xCDCStreamBuffer = NULL;
uint8_t *xCDCStreamBufferBuf = NULL;
StaticStreamBuffer_t xCDCStreamBufferStruct;*/

static inline void checkError(enum EpdDrawError err) {
    if (err != EPD_DRAW_SUCCESS) {
        ESP_LOGE("demo", "draw error: %X", err);
    }
}

enum command {
    SEND_128_BYTE = 0,
    CLEAR_DISPLAY = 1,
    SET_ORIGIN_POS = 2,
    DRAW_IMAGE_1BIT = 3,
    REFRESH_DISPLAY = 4,
    REFRESH_AREA = 5,
    DRAW_IMAGE_4BIT = 7,
    DRAW_IMAGE_2BIT = 8,
    ECHO_SYNC = 252,
    UNKNOWN_COMMAND = 254,
    COMMAND_FOLLOWS = 128
};


enum command current_command = UNKNOWN_COMMAND;
enum command previous_command = UNKNOWN_COMMAND; // Needed when SEND_128_BYTE Command comes
uint32_t payload_bytes_counter = 0;
uint32_t origin_x_pos = 0;
uint32_t origin_y_pos = 0;
uint32_t draw_image_width = 0;
uint32_t draw_image_height = 0;
uint32_t draw_image_current_x = 0;
uint32_t draw_image_current_y = 0;
uint32_t draw_image_repeat = -1;
uint8_t draw_image_color = 0;
EpdRect refresh_area_coords;

bool* drawn_lines;
uint16_t drawn_columns_start = 0;
uint16_t drawn_columns_end = 0;
uint8_t* drawn_columns;
uint8_t* drawn_columns_cpy;
bool drawn_lines_dirty = false;

bool is_powered_on;


SemaphoreHandle_t fb_xSemaphore = NULL;
StaticSemaphore_t fb_xSemaphoreBuffer;

// indexing: [source][target]
const uint8_t DRAM_ATTR transitions[4][4] = {
    {0, 1, 2, 3},
    {3, 1, 2, 3},
    {3, 1, 2, 3},
    {0, 0, 0, 3}
};

/*uint8_t transitions[4][4] = {
    {0, 1, 2, 3},
    {0, 1, 2, 3},
    {0, 1, 2, 3},
    {0, 1, 2, 3}
};*/

void IRAM_ATTR do_refresh(void) {
    uint16_t lines_dirty_ctr;


    if(drawn_columns_start > drawn_columns_end) {
        ESP_LOGW(TAG, "drawn_columns_start > drawn_columns_end");
        drawn_columns_start = 0;
        drawn_columns_end = _epd_width;
    }

    xSemaphoreTake( fb_xSemaphore, ( TickType_t ) portMAX_DELAY );
    do {
        //memcpy(drawn_columns_cpy, drawn_columns, _epd_width / 2);
        //ESP_LOGI(TAG, "starting drawing (epd_draw_base)");
        uint32_t t1 = esp_timer_get_time() / 1000;
        epd_draw_base(
            epd_full_screen(),
            fb,
            epd_full_screen(),
            (EpdDrawMode)(MODE_DU | MODE_PACKING_1PPB_DIFFERENCE),
            temperature,
            drawn_lines,
            NULL, //drawn_columns_cpy, // 
            epd_get_display()->default_waveform
        );
        uint32_t t2 = esp_timer_get_time() / 1000;
        //ESP_LOGI(TAG, "[With vector extensions] epd_draw_base took %ldms.", t2 - t1);

        lines_dirty_ctr = 0;
        drawn_lines_dirty = false;
        uint32_t rpt_chunk_start_offset = drawn_columns_start & 0xFFFFFF00;
        uint32_t rpt_cnt = (drawn_columns_end>>2) - (rpt_chunk_start_offset>>2);
        if(drawn_columns_end & 0b11)
            rpt_cnt++;
        uint8_t *fb_pixel = NULL;
        int32_t yy = 0;
        int32_t next_yy;
        while(!drawn_lines[yy]) {
            yy++;
            if(yy>=_epd_height)
                break;
        }
        if(yy<_epd_height) {
            My_Cache_Start_DCache_Preload(
                (uint32_t)fb_by_row[yy] + rpt_chunk_start_offset,
                rpt_cnt<<2
            );
        }
        while(yy<_epd_height) {
            next_yy = yy+1;
            while(!drawn_lines[next_yy] && next_yy<_epd_height) {
                next_yy++;
            }
            if(next_yy<_epd_height) {
                My_Cache_Start_DCache_Preload(
                    ((uint32_t)fb_by_row[next_yy]) + rpt_chunk_start_offset,
                    rpt_cnt<<2
                );
            }
            //ensure_current_line_is(yy, 0, _epd_width, next_yy);
            if(drawn_lines[yy]) {
                //memcpy(current_line, fb_by_row[yy], _epd_width);
                //fb_pixel = current_line;
                
                fb_pixel = fb_by_row[yy] + rpt_chunk_start_offset;
                drawn_lines[yy] = false;
                
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
                            drawn_lines_dirty = true;
                            drawn_lines[yy] = true;
                        }
                    }
                    fb_pixel+=4;
                });
                lines_dirty_ctr += drawn_lines[yy];
                //memcpy(fb_by_row[yy], current_line, _epd_width);
            }
            yy = next_yy;
        }
        //ensure_lines_writeback(0, _epd_width);
        //ESP_LOGI(TAG, "lines_dirty_ctr=%d", lines_dirty_ctr);
    } while(lines_dirty_ctr>100);
    if(!drawn_lines_dirty) {
        drawn_columns_start = _epd_width;
        drawn_columns_end = 0;
    }
    xSemaphoreGive( fb_xSemaphore );
}


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
            //if(byte != SEND_128_BYTE)
            //    ESP_LOGI(TAG, "Parsing command: %hhu", byte);
            switch(byte) {
                case SEND_128_BYTE:
                    current_command = previous_command;
                    previous_command = UNKNOWN_COMMAND;
                    byte = 128;  // Will be processed below
                    break;
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
                case SET_ORIGIN_POS:
                    //ESP_LOGI(TAG, "Entering Command SET_ORIGIN_POS");
                    previous_command = current_command;
                    current_command = SET_ORIGIN_POS;
                    payload_bytes_counter = 0;
                    continue;
                case DRAW_IMAGE_1BIT:
                    //ESP_LOGI(TAG, "Entering Command DRAW_IMAGE_1BIT");
                    previous_command = current_command;
                    current_command = DRAW_IMAGE_1BIT;
                    payload_bytes_counter = 0;
                    draw_image_current_x = 0;
                    draw_image_current_y = 0;
                    continue;
                case DRAW_IMAGE_2BIT:
                    //ESP_LOGI(TAG, "Entering Command DRAW_IMAGE_2BIT");
                    previous_command = current_command;
                    current_command = DRAW_IMAGE_2BIT;
                    payload_bytes_counter = 0;
                    draw_image_current_x = 0;
                    draw_image_current_y = 0;
                    continue;
                case DRAW_IMAGE_4BIT:
                    //ESP_LOGI(TAG, "Entering Command DRAW_IMAGE_4BIT");
                    previous_command = current_command;
                    current_command = DRAW_IMAGE_4BIT;
                    payload_bytes_counter = 0;
                    draw_image_current_x = 0;
                    draw_image_current_y = 0;
                    continue;
                case REFRESH_DISPLAY:
                    //ESP_LOGI(TAG, "Entering Command REFRESH_DISPLAY");
                    //epd_poweron();
                    
                    previous_command = REFRESH_DISPLAY;
                    current_command = UNKNOWN_COMMAND;
                    continue;
                case REFRESH_AREA:
                    //ESP_LOGI(TAG, "Entering Command SET_ORIGIN_POS");
                    previous_command = current_command;
                    current_command = REFRESH_AREA;
                    payload_bytes_counter = 0;
                    continue;
                case ECHO_SYNC:
                    previous_command = current_command;
                    current_command = UNKNOWN_COMMAND;
                    tinyusb_cdcacm_write_queue_char(TINYUSB_CDC_ACM_0, ECHO_SYNC);
                    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
                    payload_bytes_counter = 0;
                    continue;
                default:
                    ESP_LOGI(TAG, "do not know how to interpret this byte after 'command follows' byte: %hhu", byte);
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
 
            case DRAW_IMAGE_2BIT:
                if( payload_bytes_counter >= 4) {

                    //for(uint16_t xx=origin_x_pos; xx<origin_x_pos+draw_image_width; xx++)
                    //    drawn_columns[xx>>1] |= xx&1? 0x0F : 0xF0;
                    //memset(drawn_columns, 0xff, _epd_width / 2);
                    //for(uint16_t xx=refresh_area_coords.x; xx<refresh_area_coords.x+refresh_area_coords.width; xx++)
                    //    drawn_columns[xx>>1] |= xx&1? 0xF0 : 0x0F;

                    if( payload_bytes_counter == 4 ) {
                        // In the first payload byte we find the color and the
                        // first three bits of the repeat-information
                        DRAW_IMAGE_2BIT_INNER_LOOP_GET_COLOR: 
                        draw_image_color = byte & 0b111;  // two color bits and one bit for special operations
                        // we have only 4 bits left of the first byte for repeat information, because the left
                        // most bit tells if more repeat-bytes follow
                        draw_image_repeat = (byte >> 3) & 0b1111;
                    } else {
                        // In every payload byte after that, we find seven more
                        // lower bits of the repeat information
                        DRAW_IMAGE_2BIT_INNER_LOOP_GET_SHIFT_BITS:
                        draw_image_repeat <<= 7;
                        draw_image_repeat |= byte & 0x7F;
                    }
                    //ESP_LOGI(TAG, "DRAW_IMAGE_2BIT color=%d repeats=%"PRIu32, draw_image_color, draw_image_repeat);

                    if( (byte) & 0x80 ) {
                        // There are more shift-bits to come
                        payload_bytes_counter += 1;
                        goto DRAW_IMAGE_2BIT_CHECK_NEXT_BYTE_FOR_SHIFT_BITS;
                    }

                    // We have gathered all the repeat information
                    if(draw_image_color==0b100) {  // draw transparent pixels
                        // Shortcut to jump over transparent pixels (=no change on fb)
                        draw_image_repeat += draw_image_current_x + 1;
                        uint32_t line_jump = draw_image_repeat/draw_image_width;
                        draw_image_current_y += line_jump;
                        draw_image_current_x = draw_image_repeat%draw_image_width;
                        if(line_jump) {
                            My_Cache_Start_DCache_Preload(
                                ((uint32_t)fb_by_row[draw_image_current_y]) + origin_x_pos,  // start address of the preload region
                                draw_image_width
                            );
                            if(draw_image_current_y<_epd_height) {
                                My_Cache_Start_DCache_Preload(
                                    ((uint32_t)fb_by_row[draw_image_current_y+1]) + origin_x_pos,  // start address of the preload region
                                    draw_image_width
                                );
                            }
                        }
                    } else {  // draw color or invert
                        uint32_t yy = origin_y_pos + draw_image_current_y;
                        uint8_t *fb_yy = fb_by_row[yy];
                        //ensure_current_line_is(yy, origin_x_pos, draw_image_width, yy+1);
                        /*Cache_Start_DCache_Preload(
                            ((uint32_t)fb_by_row[yy+1]) + (origin_x_pos & 0xFFFFFF00),  // start address of the preload region
                            max(CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE+1, draw_image_width),  // size of the preload region, should not exceed the size of DCache
                            0  // the preload order, 0 for positive, other for negative
                        );*/
                        drawn_lines[yy] = true;
                        drawn_lines_dirty = true;
                        uint8_t draw_image_color_shifted = draw_image_color<<6;
                        uint32_t draw_image_color_shifted32 = 0;
                        if(draw_image_repeat>=4) {
                            draw_image_color_shifted32 = draw_image_color_shifted;
                            draw_image_color_shifted32 |= (draw_image_color_shifted32<<8)
                                | (draw_image_color_shifted32<<16)
                                | (draw_image_color_shifted32<<24);
                        }
                        for(uint32_t px_ctr=0; px_ctr<draw_image_repeat+1;) {
                            uint32_t xx_start = origin_x_pos + draw_image_current_x;
                            // xx_stop denotes where we stop drawing the current row of the picture
                            // (due to different reasons)
                            uint32_t xx_count = min(
                                draw_image_repeat+1-px_ctr,
                                draw_image_width-draw_image_current_x
                            );
                            uint8_t *buf_ptr = fb_yy + xx_start;
                            if(0) { //xx_count >= 2) {
                                // If we have at least 2 repeating pixels left in this row, we take a shortcut,
                                // as we can skip calculating jumps into the next row, overflowing
                                // the screen and other checks
                                uint8_t *buf_ptr_stop = fb_yy + xx_start + xx_count;
                                // We do differentiate between putting a color and inverting outside
                                // the loop, so we do not do this branch for every loop iteration again.
                                if(draw_image_color==0b101) {  // invert
                                    rpt((buf_ptr_stop - buf_ptr)>>3, [&buf_ptr]() {
                                        uint32_t A = *((uint32_t*)buf_ptr);
                                        uint32_t B = *((uint32_t*)(buf_ptr+4));
                                        *((uint32_t*)buf_ptr) = (A & 0x0F0F0F0F) | (~A & 0xF0F0F0F0);
                                        *((uint32_t*)(buf_ptr+4)) = (B & 0x0F0F0F0F) | (~B & 0xF0F0F0F0);
                                        buf_ptr += 8;
                                    });
                                    rpt(buf_ptr_stop-buf_ptr, [&buf_ptr]() {
                                        *buf_ptr = (*buf_ptr & 0x0F) | (~(*buf_ptr) & 0xF0);
                                        buf_ptr++;
                                    });
                                } else {
                                    rpt((buf_ptr_stop - buf_ptr)>>3, [&buf_ptr, &draw_image_color_shifted32]() {
                                        uint32_t A = *((uint32_t*)buf_ptr);
                                        uint32_t B = *((uint32_t*)(buf_ptr+4));
                                        *((uint32_t*)buf_ptr) = (A & 0x0F0F0F0F) | draw_image_color_shifted32;
                                        *((uint32_t*)(buf_ptr+4)) = (B & 0x0F0F0F0F) | draw_image_color_shifted32;
                                        buf_ptr += 8;
                                    });
                                    rpt(buf_ptr_stop-buf_ptr, [&buf_ptr, &draw_image_color_shifted]() {
                                        *buf_ptr = (*buf_ptr & 0x0F) | draw_image_color_shifted;
                                        buf_ptr++;
                                    });
                                }

                                // Since we only advanced in the row, we only need to update these
                                // variables
                                draw_image_current_x += xx_count;
                                px_ctr += xx_count;
                            }
                            else {
                                // If we have just 1 or zero repeating pixels left in the row, we take
                                // care of screen dimensions and draw it
                                if(draw_image_color==0b101) {
                                    *buf_ptr = (*buf_ptr & 0x0F) | (~(*buf_ptr) & 0xF0);
                                } else {
                                    *buf_ptr = (*buf_ptr & 0x0F) | draw_image_color_shifted;
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
                                //ESP_LOGI(TAG, "draw_image_current_y=%ld   draw_image_height=%ld", draw_image_current_y, draw_image_height);
                                
                                if( 
                                    (draw_image_current_y + 1 > draw_image_height) ||
                                    (yy + 1 > _epd_height)
                                )
                                    goto DRAW_IMAGE_2BIT_END;
                                
                                My_Cache_Start_DCache_Preload(
                                    ((uint32_t)fb_by_row[yy]) + origin_x_pos,  // start address of the preload region
                                    draw_image_width
                                );
                                if((yy+1) < _epd_height ) {
                                    My_Cache_Start_DCache_Preload(
                                        ((uint32_t)fb_by_row[yy+1]) + origin_x_pos,  // start address of the preload region
                                        draw_image_width
                                    );
                                }
                                //ensure_current_line_is(yy, origin_x_pos, draw_image_width, yy+1);
                                drawn_lines[yy] = true;
                                drawn_lines_dirty = true;
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
                            goto DRAW_IMAGE_2BIT_INNER_LOOP_GET_COLOR;
                        } else {
                            if(buf + 1 < buf_end) {
                                if(*(buf+1)==SEND_128_BYTE) {
                                    byte = 128;
                                    buf += 2;
                                    goto DRAW_IMAGE_2BIT_INNER_LOOP_GET_COLOR;
                                }
                            }
                        }
                    }
                    continue;
                    DRAW_IMAGE_2BIT_CHECK_NEXT_BYTE_FOR_SHIFT_BITS:
                    if(buf < buf_end) {
                        if(*buf != COMMAND_FOLLOWS) {
                            byte = *buf;
                            buf++;
                            goto DRAW_IMAGE_2BIT_INNER_LOOP_GET_SHIFT_BITS;
                        } else {
                            if(buf + 1 < buf_end) {
                                if(*(buf+1)==SEND_128_BYTE) {
                                    byte = 128;
                                    buf += 2;
                                    goto DRAW_IMAGE_2BIT_INNER_LOOP_GET_SHIFT_BITS;
                                }
                            }
                        }
                    }
                    continue;
                }
                //ESP_LOGI(TAG, "Draw 2bit payload bytes counter: %ld", payload_bytes_counter);
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
                            goto DRAW_IMAGE_2BIT_END;
                        }
                        draw_image_current_y = 0;
                        draw_image_current_x = 0;
                        drawn_columns_start = min(drawn_columns_start, origin_x_pos);
                        drawn_columns_end = max(drawn_columns_end, origin_x_pos+draw_image_width);
                        My_Cache_Start_DCache_Preload(
                            ((uint32_t)fb_by_row[origin_y_pos]) + origin_x_pos,
                            draw_image_width
                        );
                        //ESP_LOGI(TAG, "Damage 2BIT: %ld,%ld - %ld,%ld\n", origin_x_pos, origin_y_pos, draw_image_width, draw_image_height);
                        break;
                };
                payload_bytes_counter += 1;
                continue;
                DRAW_IMAGE_2BIT_END:
                //printf("DRAW_IMAGE_2BIT_END before ensure_lines_writeback\n");
                //ensure_lines_writeback(origin_x_pos, draw_image_width);
                //printf("DRAW_IMAGE_2BIT_END\n");
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

                        do_refresh();

                        tinyusb_cdcacm_write_queue_char(TINYUSB_CDC_ACM_0, 'r'); //REFRESH_AREA);
                        /*tinyusb_cdcacm_write_queue_char(
                            TINYUSB_CDC_ACM_0, 
                            (refresh_area_coords.x^refresh_area_coords.y^refresh_area_coords.width^refresh_area_coords.height)&0xFF
                        );*/
                        tinyusb_cdcacm_write_queue_char(TINYUSB_CDC_ACM_0, '\n');
                        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
                        
                        break;
                };
                payload_bytes_counter += 1;
                continue;
            default:
                ESP_LOGI(TAG, "Current command is unknown: %hhu", current_command);
                break;
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

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

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
    fb_size = epd_width() * epd_height();
    //fb = heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    fb = (uint8_t*)heap_caps_aligned_alloc(16, fb_size, MALLOC_CAP_SPIRAM);
    memset(fb, 0xFF, fb_size);
    fb_by_row = (uint8_t**)heap_caps_malloc(sizeof(fb)*_epd_height, MALLOC_CAP_INTERNAL);
    for(uint16_t y=0; y<_epd_height; y++) {
        fb_by_row[y] = fb + y*_epd_width;
    }
    drawn_lines = (bool*)heap_caps_malloc(epd_height(), MALLOC_CAP_INTERNAL);
    memset(drawn_lines, 0xff, _epd_height);
    //drawn_columns = (uint8_t*)heap_caps_aligned_alloc(16, _epd_width / 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    /*memset(drawn_columns, 0xff, _epd_width / 2);
    drawn_columns_cpy = heap_caps_aligned_alloc(16, _epd_width / 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    memcpy(drawn_columns_cpy, drawn_columns, _epd_width / 2);*/
    drawn_lines_dirty = true;
    


    // Default orientation is EPD_ROT_LANDSCAPE
    epd_set_rotation(EPD_ROT_LANDSCAPE);

    printf(
        "Dimensions after rotation, width: %d height: %d\n\n", epd_rotated_display_width(),
        epd_rotated_display_height()
    );

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


    fb_xSemaphore = xSemaphoreCreateBinaryStatic( &fb_xSemaphoreBuffer );
    xSemaphoreGive( fb_xSemaphore );

    /* USB CDC Init */

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_config, //NULL,
        .string_descriptor = NULL,
        .string_descriptor_count = 0,
        .external_phy = false,
        .configuration_descriptor = NULL,
        .self_powered = false,
        .vbus_monitor_io = 0,  // ignored because .self_powered = false
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = (tusb_cdcacm_callback_t)&tinyusb_cdc_rx_callback, // the first way to register a callback
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
                /*if(is_powered_on) {
                    printf("Saving power...\n");
                    epd_poweroff();
                    is_powered_on = false;
                }*/
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
