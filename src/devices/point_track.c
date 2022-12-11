#include <stdio.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "string.h"

#include "st7735.h"
#include "fonts.h"
#include "pico/time.h"
#include "arducam.h"

#include "point_track.h"


uint8_t image_buf[324*324];
uint8_t displayBuf[80*160*2];
uint8_t header[2] = {0x55,0xAA};



//index into array representing an X-by-Y matrix, where X increments first
static inline int xy_to_index(int x, int y, int x_len, int counts_per_position) {
    return (x + (y * x_len)) * counts_per_position;
}

static inline int get_max(int a, int b) {
    return (a >= b) ? a : b;
}
static inline int get_min(int a, int b) {
    return (a <= b) ? a : b;
}

//convolution, get value of this position plus every space within ? pixels (in x and y dirs)
// think: surrounding pixel values -> this pixel
#define BLOCK_RANGE 2
static uint16_t block_sum(int x, int y, int x_max, int y_max, uint8_t * evalbuf, int counts_per_position) {
    uint16_t ret = 0;
    for (int i_y = get_max(0, y - BLOCK_RANGE); i_y < get_min(y_max, y + BLOCK_RANGE + 1); i_y++) {
        for (int i_x = get_max(0, x - BLOCK_RANGE); i_x < get_min(x_max, x + BLOCK_RANGE + 1); i_x++) {
            ret += evalbuf[xy_to_index(i_x, i_y, x_max, counts_per_position)];
        }
    }
    return ret;
}

//also convolution, adds the value of pixel specified by x.y to surrounding pixels in scorebuf
// think: this pixel value -> surrounding pixels
static void add_pixel(int x, int y, int x_max, int y_max, uint16_t * scorebuf, int counts_per_position, uint8_t score) {
    for (int i_y = get_max(0, y - BLOCK_RANGE); i_y < get_min(y_max, y + BLOCK_RANGE + 1); i_y++) {
        for (int i_x = get_max(0, x - BLOCK_RANGE); i_x < get_min(x_max, x + BLOCK_RANGE + 1); i_x++) {
            scorebuf[xy_to_index(i_x, i_y, x_max, counts_per_position)] += score;
        }
    }
}

void core1_entry() {
    multicore_fifo_push_blocking(MULTICORE_FLAG_VALUE);

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != MULTICORE_FLAG_VALUE)
        printf("unexpected value from core0\n");
    else
        printf("core1 starting setup\n");

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, false);


    ST7735_Init();
    ST7735_DrawImage(0, 0, 80, 160, arducam_logo);

    struct arducam_config config;
    config.sccb = i2c0;
    config.sccb_mode = I2C_MODE_16_8;
    config.sensor_address = 0x24;
    config.pin_sioc = PIN_CAM_SIOC;
    config.pin_siod = PIN_CAM_SIOD;
    config.pin_resetb = PIN_CAM_RESETB;
    config.pin_xclk = PIN_CAM_XCLK;
    config.pin_vsync = PIN_CAM_VSYNC;
    config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;

    config.pio = pio0;
    config.pio_sm = 0;

    config.dma_channel = 0;
    config.image_buf = image_buf;
    config.image_buf_size = sizeof(image_buf);

    arducam_init(&config);

    struct {
        int x;
        int y;
        uint16_t val;
        uint16_t index;
    } max_point;
    uint16_t max_RGB = ST7735_COLOR565(0xff, 0x0, 0x0);

    uint16_t score_buf[160*80];
    absolute_time_t start;


    ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, ST7735_GREEN);
    ST7735_WriteString(2, 2, "CONN.", Font_16x26, ST7735_MAGENTA, ST7735_GREEN);
    ST7735_WriteString(2, 30, "TO", Font_16x26, ST7735_MAGENTA, ST7735_GREEN);
    ST7735_WriteString(2, 58, "USB", Font_16x26, ST7735_MAGENTA, ST7735_GREEN);
    ST7735_WriteString(2, 86, "CON-", Font_16x26, ST7735_MAGENTA, ST7735_GREEN);
    ST7735_WriteString(2, 112, "SOLE", Font_16x26, ST7735_MAGENTA, ST7735_GREEN);

    g = multicore_fifo_pop_blocking();
    if (g != MULTICORE_FLAG_VALUE)
        printf("unexpected value from core0\n");
    else
        printf("core1 starting image processing\n");

    while (true) {
        start = get_absolute_time();
        max_point.x = 0;
        max_point.y = 0;
        max_point.val = 0;
        arducam_capture_frame(&config);

        uint16_t index = 0;
        //memset(score_buf, 0, sizeof(score_buf));
        for(int i =0; i < 160*80; i++) {
            score_buf[i] = 0;
        }

        for (int y = 0; y < 160; y++) {
            for (int x = 0; x < 80; x++) {
                uint8_t c = image_buf[(2+320-2*y)*324+(2+40+2*x)];
                uint16_t imageRGB   = ST7735_COLOR565(c, c, c);
                add_pixel(x,y,80,160,score_buf,1,c);

                displayBuf[index++] = (uint8_t)(imageRGB >> 8) & 0xFF;
                displayBuf[index++] = (uint8_t)(imageRGB)&0xFF;
                }
        }
        for (int y = 0; y < 160; y++) {
            for (int x = 0; x < 80; x++) {
                if (score_buf[xy_to_index(x,y,80,1)] > max_point.val) {
                    max_point.val = score_buf[xy_to_index(x,y,80,1)];
                    max_point.x = x;
                    max_point.y = y;
                    max_point.index = xy_to_index(x,y,80,2);
                }
            }
        }
       
        //center
        uint16_t draw_index = max_point.index - (4*2);
        //max_point.index -= 2*2;
        for (int i = 0; i<9;i++){
            displayBuf[draw_index++] = (uint8_t)(max_RGB >> 8) & 0xFF;
            displayBuf[draw_index++] = (uint8_t)(max_RGB)&0xFF;
            if (draw_index >= (160 * 80)*2){break;}
        }
        draw_index = max_point.index - (80*2) * 4;
        for (int i = 0; i<9;i++){
            displayBuf[draw_index++] = (uint8_t)(max_RGB >> 8) & 0xFF;
            displayBuf[draw_index] = (uint8_t)(max_RGB)&0xFF;
            draw_index += (80*2)-1;
            if (draw_index >= (160 * 80)*2){break;}
        }
        //sleep_ms(32);
        
        ST7735_DrawImage(0, 0, 80, 160, displayBuf);
        char array[10];
        sprintf(array, "%d", absolute_time_diff_us(start,get_absolute_time()));
        ST7735_FillRectangle(0, 134, ST7735_WIDTH, 26, ST7735_BLACK);
        ST7735_WriteString(2, 134, array, Font_16x26, ST7735_GREEN, ST7735_BLACK);

        //printf("%s%s", header, displayBuf);
        //printf("x:%2d, y:%3d, t:%s\n", max_point.x, max_point.y,array);
    }
}