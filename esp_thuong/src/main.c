// ESP_THUONG - ESP-IDF / PlatformIO. Application source: src/main.c
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_rom_crc.h"
#include "esp_err.h"

enum { SLOT=32760, HEAD=20, CAP=SLOT-HEAD, DIM=512,
       START=1, POLL, END_SESSION, IDLE=16, AIM, SAMPLING, MATCH, DENIED,
       NO_FACE, BAD_REF, CAM_FAIL, AI_FAIL, LINK_FAIL, ENROLLED, ENROLL=4 };
// Versioned template: MFN_S8_V1, ESP-DL 3.3.15, RGB888 preprocessing.
#define TEMPLATE_VERSION 0x03331501u
typedef struct {
    uint32_t version;
    float vector[DIM];
} face_template_t;
typedef char template_layout_check[(sizeof(face_template_t) == 2052) ? 1 : -1];
typedef struct {
    uint32_t magic, session, kind, size, crc;
    uint8_t data[CAP];
} packet_t;
typedef char slot_layout_check[(sizeof(packet_t)==SLOT)?1:-1];
static inline uint32_t checksum(const packet_t *p) {
    uint32_t v=esp_rom_crc32_le(0,(const uint8_t*)p,16);
    return esp_rom_crc32_le(v,p->data,p->size);
}
static inline void seal(packet_t *p,uint32_t session,uint32_t kind,uint32_t bytes) {
    p->magic=0x34454346;
    p->session=session; p->kind=kind; p->size=bytes;
    p->crc=checksum(p);
}
static inline int valid(const packet_t *p) {
    return p->magic==0x34454346 && p->size<=CAP && p->crc==checksum(p);
}
static inline packet_t *new_slot(void) {
    packet_t *p=(packet_t*)heap_caps_calloc(1,SLOT,MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);
    ESP_ERROR_CHECK(p ? ESP_OK : ESP_ERR_NO_MEM);
    return p;
}

// SPI chỉ ghi hình. SDO/GPIO13 có thể vẫn nối vật lý theo sơ đồ.
#define TFT_CS_GPIO   21
#define TFT_RST_GPIO  19
#define TFT_DC_GPIO   18
#define TFT_MOSI_GPIO 17
#define TFT_SCK_GPIO  16
#define TFT_BL_GPIO    4    // -1 nếu đèn nền được cấp qua mạch riêng
#define TFT_BL_ON      1

#define TFT_SPI_MODE   3    // giá trị từ esp_thuong.ino
#define TFT_SPI_HZ     10000000
#define TFT_BGR        0    // đổi thành 1 nếu đỏ và xanh dương hoán đổi
#define TFT_INVERT     0    // Tắt đảo màu trên ST7789
#define TFT_MIRROR_X   1
#define TFT_MIRROR_Y   1
#define TFT_X_GAP      0
#define TFT_Y_GAP      0

#define CAM_WIDTH 320
#define CAM_HEIGHT 240
#define PANEL_W 240
#define PANEL_H 320
#define CROP_W 180


#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_jpeg_dec.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

enum { STRIPE_ROWS = 16, ROW_BYTES = CAM_WIDTH * 2, STRIPE_BYTES = ROW_BYTES * STRIPE_ROWS };
static const char *LCD_TAG = "U2_LCD";
static esp_lcd_panel_io_handle_t lcd_io;
static esp_lcd_panel_handle_t lcd;
static uint8_t *stripe;
static uint8_t *portrait;
enum { MAX_OUT_ROWS=(STRIPE_ROWS*4+2)/3, OUT_BYTES=MAX_OUT_ROWS*PANEL_W*2 };

void display_init(void)
{
    if (TFT_BL_GPIO >= 0) {
        ESP_ERROR_CHECK(gpio_set_direction(TFT_BL_GPIO, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_level(TFT_BL_GPIO, !TFT_BL_ON));
    }
    stripe = heap_caps_aligned_alloc(16, STRIPE_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_ERROR_CHECK(stripe ? ESP_OK : ESP_ERR_NO_MEM);
    portrait = heap_caps_aligned_alloc(16, OUT_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_ERROR_CHECK(portrait ? ESP_OK : ESP_ERR_NO_MEM);
    const spi_bus_config_t bus = {
        .mosi_io_num = TFT_MOSI_GPIO, .miso_io_num = -1, .sclk_io_num = TFT_SCK_GPIO,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = OUT_BYTES,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = TFT_CS_GPIO, .dc_gpio_num = TFT_DC_GPIO,
        .spi_mode = TFT_SPI_MODE, .pclk_hz = TFT_SPI_HZ,
        .lcd_cmd_bits = 8, .lcd_param_bits = 8, .trans_queue_depth = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                             &io_cfg, &lcd_io));
    const esp_lcd_panel_dev_config_t panel = {
        .reset_gpio_num = TFT_RST_GPIO, .bits_per_pixel = 16,
        .rgb_ele_order = TFT_BGR ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io, &panel, &lcd));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd, TFT_MIRROR_X, TFT_MIRROR_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd, TFT_X_GAP, TFT_Y_GAP));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd, TFT_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd, false));
}

static void overlay(int y,int rows);
static void draw_stripe(int y, int rows)
{
    overlay(y,rows);
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(lcd, 0, y, PANEL_W, y + rows, portrait));
    // tx_param đợi toàn bộ DMA trước nó. Lệnh NOP của ST7789 cho phép dùng lại stripe.
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(lcd_io, 0x00, NULL, 0));
}

bool display_jpeg(uint8_t *data, size_t length)
{
    jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
    cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_BE;
    cfg.block_enable = true;
    jpeg_dec_handle_t dec = NULL;
    jpeg_error_t rc = jpeg_dec_open(&cfg, &dec);
    if (rc != JPEG_ERR_OK) {
        ESP_LOGE(LCD_TAG, "JPEG open: %d", (int)rc);
        return false;
    }
    bool ok = false;
    jpeg_dec_io_t io = {.inbuf = data, .inbuf_len = (int)length};
    jpeg_dec_header_info_t info = {0};
    rc = jpeg_dec_parse_header(dec, &io, &info);
    if (rc != JPEG_ERR_OK || info.width != CAM_WIDTH || info.height != CAM_HEIGHT) {
        ESP_LOGW(LCD_TAG, "JPEG header: code=%d, %ux%u", (int)rc,
                 (unsigned)info.width, (unsigned)info.height);
        goto done;
    }
    int block_bytes = 0, blocks = 0;
    rc = jpeg_dec_get_outbuf_len(dec, &block_bytes);
    if (rc != JPEG_ERR_OK || block_bytes <= 0 || block_bytes > STRIPE_BYTES) {
        ESP_LOGW(LCD_TAG, "JPEG block size: code=%d, bytes=%d", (int)rc, block_bytes);
        goto done;
    }
    rc = jpeg_dec_get_process_count(dec, &blocks);
    if (rc != JPEG_ERR_OK || blocks <= 0 || blocks > CAM_HEIGHT) {
        ESP_LOGW(LCD_TAG, "JPEG block count: code=%d, blocks=%d", (int)rc, blocks);
        goto done;
    }
    io.outbuf = stripe;
    int y = 0;
    for (int i = 0; i < blocks; ++i) {
        rc = jpeg_dec_process(dec, &io);
        if (rc != JPEG_ERR_OK || io.out_size <= 0 || io.out_size > block_bytes ||
            io.out_size % ROW_BYTES != 0) {
            ESP_LOGW(LCD_TAG, "JPEG block %d: code=%d, bytes=%d", i, (int)rc, io.out_size);
            goto done;
        }
        int rows = io.out_size / ROW_BYTES;
        if (y + rows > CAM_HEIGHT) {
            ESP_LOGW(LCD_TAG, "JPEG block exceeds display height");
            goto done;
        }
        // Center crop, uniform 4/3 scale. Absolute row mapping avoids seams.
        int first=(4*y+2)/3;
        int end=(4*(y+rows)+2)/3;
        for (int dy=first; dy<end; ++dy) {
            const uint8_t *src=stripe+((dy*3/4-y)*CAM_WIDTH+(CAM_WIDTH-CROP_W)/2)*2;
            uint8_t *dst=portrait+(dy-first)*PANEL_W*2;
            for (int dx=0; dx<PANEL_W; ++dx) {
                int sx=(dx*3/4)*2;
                dst[2*dx]=src[sx];
                dst[2*dx+1]=src[sx+1];
            }
        }
        draw_stripe(first,end-first);
        y += rows;
    }
    ok = (y == CAM_HEIGHT);
    if (!ok) {
        ESP_LOGW(LCD_TAG, "JPEG ended after %d rows", y);
    }
done:
    jpeg_dec_close(dec);
    return ok;
}
#include <math.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CS GPIO_NUM_26
#define READY GPIO_NUM_33
#define BOOT GPIO_NUM_0
static spi_device_handle_t spi_link;
static packet_t *tx,*rx;
static const char *label;
static uint16_t color;
static nvs_handle_t face_store;
static bool storage_ready;
static bool template_valid(const face_template_t *value) {
    if (value->version != TEMPLATE_VERSION) return false;
    double norm = 0;
    for (int i = 0; i < DIM; ++i) norm += (double)value->vector[i] * value->vector[i];
    return isfinite(norm) && norm > 0.99 && norm < 1.01;
}
static bool load_template(face_template_t *value) {
    size_t size = sizeof(*value);
    return storage_ready && nvs_get_blob(face_store, "template", value, &size) == ESP_OK &&
           size == sizeof(*value) && template_valid(value);
}
static bool save_template(const packet_t *packet) {
    if (!storage_ready || packet->size != sizeof(face_template_t)) return false;
    face_template_t value;
    memcpy(&value, packet->data, sizeof(value));
    if (!template_valid(&value)) return false;
    esp_err_t rc = nvs_set_blob(face_store, "template", &value, sizeof(value));
    if (rc == ESP_OK) rc = nvs_commit(face_store);
    ESP_LOGI("FACE", "Save enrollment: %s", esp_err_to_name(rc));
    return rc == ESP_OK;
}
// Uppercase 5x7 column font, least significant bit at the top.
static const uint8_t font[][5]={
 {126,17,17,17,126},{127,73,73,73,54},{62,65,65,65,34},{127,65,65,34,28},
 {127,73,73,73,65},{127,9,9,9,1},{62,65,73,73,122},{127,8,8,8,127},
 {0,65,127,65,0},{32,64,65,63,1},{127,8,20,34,65},{127,64,64,64,64},
 {127,2,12,2,127},{127,4,8,16,127},{62,65,65,65,62},{127,9,9,9,6},
 {62,65,81,33,94},{127,9,25,41,70},{70,73,73,73,49},{1,1,127,1,1},
 {63,64,64,64,63},{31,32,64,32,31},{63,64,56,64,63},{99,20,8,20,99},
 {3,4,120,4,3},{97,81,73,69,67}
};
static void overlay(int y0,int rows) {
    int n=strlen(label),left=(PANEL_W-(n*12-2))/2;
    for(int row=0;row<rows;row++) {
        int y=y0+row;
        for(int x=0;x<PANEL_W;x++) {
            uint16_t pen;
            if(y>=282 && y<310) {
                pen=0;
                int col=x-left;
                if(col>=0 && col/12<n && y>=289 && y<303 && col%12<10) {
                    int ch=label[col/12]-'A';
                    if(ch>=0 && ch<26 && (font[ch][(col%12)/2]&(1<<((y-289)/2)))) pen=color;
                }
            } else {
                bool edge_x=(x>=32 && x<=34)||(x>=205 && x<=207);
                bool edge_y=(y>=45 && y<=47)||(y>=272 && y<=274);
                bool near_x=(x>=32 && x<58)||(x>181 && x<=207);
                bool near_y=(y>=45 && y<71)||(y>248 && y<=274);
                if(!(edge_x && near_y) && !(edge_y && near_x)) continue;
                pen=color;
            }
            int i=(row*PANEL_W+x)*2;
            portrait[i]=pen>>8; portrait[i+1]=pen;
        }
    }
}
static void blank(void) {
    for(int y=0;y<PANEL_H;y+=STRIPE_ROWS) {
        memset(portrait,0,STRIPE_ROWS*PANEL_W*2);
        draw_stripe(y,STRIPE_ROWS);
    }
}
static void message(const char *s,uint16_t c) { label=s; color=c; blank(); }
static void sleep_ms(int ms) { vTaskDelay(pdMS_TO_TICKS(ms)+1); }
static bool exchange(void) {
    if(!gpio_get_level(READY)) return false;
    spi_transaction_t t={.length=SLOT*8,.tx_buffer=tx,.rx_buffer=rx};
    gpio_set_level(CS,0);
    esp_err_t rc=spi_device_transmit(spi_link,&t);
    gpio_set_level(CS,1);
    sleep_ms(10);
    return rc==ESP_OK && valid(rx);
}
static void session(bool enroll) {
    uint32_t id;
    do { id=esp_random(); } while(!id);
    esp_lcd_panel_disp_on_off(lcd,true);
    message("DUA MAT VAO KHUNG",0xffff); gpio_set_level(TFT_BL_GPIO,TFT_BL_ON);
    if (!storage_ready) { message("LOI BO NHO NVS", 0xf800); return; }
    face_template_t value;
    if (enroll) {
        message("DANG KY MAT", 0xffe0);
        seal(tx, id, ENROLL, 0);
    } else {
        if (!load_template(&value)) { message("GIU BOOT BA GIAY", 0xffe0); return; }
        memcpy(tx->data, &value, sizeof(value));
        seal(tx, id, START, sizeof(value));
    }
    int64_t limit=esp_timer_get_time()+40000000, wake_again=0;
    bool ack=false, completed=false;
    while(esp_timer_get_time()<limit) {
        int64_t now=esp_timer_get_time();
        if(!gpio_get_level(READY)) {
            if(!ack && now>=wake_again) {
                // CS pulse with no SPI clocks wakes U1 from GPIO light sleep.
                gpio_set_level(CS,0); sleep_ms(45); gpio_set_level(CS,1);
                wake_again=now+1500000;
            }
            sleep_ms(20); continue;
        }
        if(!exchange() || rx->session!=id) continue;
        if(rx->kind<AIM || rx->kind>ENROLLED) continue;
        if(!ack) { ack=true; seal(tx,id,POLL,0); }
        if(rx->kind==AIM || rx->kind==SAMPLING) {
            label=rx->kind==AIM?"DUA MAT VAO KHUNG":"GIU YEN";
            color=rx->kind==AIM?0xffff:0xffe0;
            if(rx->size) display_jpeg(rx->data,rx->size);
            continue;
        }
        if((rx->kind==MATCH || rx->kind==DENIED) && rx->size!=sizeof(float)) continue;
        if(rx->kind==MATCH || rx->kind==DENIED) {
            float score; memcpy(&score,rx->data,sizeof(score));
            ESP_LOGI("FACE","cosine %.3f",score);
        }
        if (rx->kind == ENROLLED && !enroll) continue;
        if (enroll && (rx->kind == MATCH || rx->kind == DENIED)) continue;
        completed=true;
        switch(rx->kind) {
        case ENROLLED:
            if (save_template(rx)) message("DA LUU MAU", 0x07e0);
            else message("LOI LUU MAU", 0xf800);
            break;
        case MATCH: message("KHOP",0x07e0); break;
        case DENIED: message("KHONG KHOP",0xf800); break;
        case NO_FACE: message("CHUA THAY MAT",0xffe0); break;
        case BAD_REF: message("LOI EMBEDDING",0xf800); break;
        case CAM_FAIL: message("LOI CAMERA",0xf800); break;
        default: message("LOI AI",0xf800); break;
        }
        break;
    }
    if(!completed) message("MAT KET NOI SPI",0xf800);
    if(ack) {
        seal(tx,id,END_SESSION,0);
        for(int i=0;i<100 && !gpio_get_level(READY);i++) sleep_ms(10);
        exchange();
    }
    memset(tx->data,0,sizeof(face_template_t));
}
void app_main(void) {
    esp_err_t storage_error = nvs_flash_init();
    if (storage_error == ESP_OK) storage_error = nvs_open("face_v3", NVS_READWRITE, &face_store);
    storage_ready = storage_error == ESP_OK;
    if (!storage_ready) ESP_LOGE("FACE", "NVS init: %s", esp_err_to_name(storage_error));
    tx=new_slot(); rx=new_slot();
    gpio_set_level(CS,1); gpio_set_direction(CS,GPIO_MODE_OUTPUT);
    gpio_set_direction(BOOT,GPIO_MODE_INPUT); gpio_pullup_en(BOOT);
    gpio_set_direction(READY,GPIO_MODE_INPUT); gpio_pulldown_en(READY);
    display_init();
    spi_bus_config_t b={.sclk_io_num=14,.mosi_io_num=27,.miso_io_num=25,
        .quadhd_io_num=-1,.quadwp_io_num=-1,.max_transfer_sz=SLOT};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST,&b,SPI_DMA_CH_AUTO));
    spi_device_interface_config_t d={.clock_speed_hz=4000000,.mode=3,.spics_io_num=-1,.queue_size=1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST,&d,&spi_link));
    ESP_LOGI("FACE", "U2 ready: press BOOT to verify, hold 3s to enroll");
    while(true) {
        while(gpio_get_level(BOOT)) sleep_ms(20);
        sleep_ms(35);
        if(gpio_get_level(BOOT)) continue;
        // Short press verifies. Hold 3 seconds enrolls/replaces the stored face.
        int64_t pressed_at = esp_timer_get_time();
        while (!gpio_get_level(BOOT) && esp_timer_get_time() - pressed_at < 3000000) sleep_ms(10);
        bool enroll = !gpio_get_level(BOOT);
        session(enroll);
        sleep_ms(2500);
        gpio_set_level(TFT_BL_GPIO,!TFT_BL_ON); esp_lcd_panel_disp_on_off(lcd,false);
        while(!gpio_get_level(BOOT)) sleep_ms(20);
    }
}
