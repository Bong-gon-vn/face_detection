#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_rom_crc.h"
#include "esp_err.h"
// SPI wire protocol: change BOTH copies together. No pointers in wire structs.
enum { SLOT=32760, HEAD=20, CAP=SLOT-HEAD, DIM=512,
       START=1, POLL, END_SESSION, IDLE=16, AIM, SAMPLING, MATCH, DENIED,
       NO_FACE, BAD_REF, CAM_FAIL, AI_FAIL, LINK_FAIL, ENROLLED, ENROLL=4, STREAM=7, IMAGE_GET=8, STREAM_FRAME=36, IMAGE_DATA=37 };
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

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_jpeg_dec.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "mbedtls/base64.h"
#include <math.h>
#include <stdlib.h>

#define PN532_SDA_GPIO 22
#define PN532_SCL_GPIO 23
#define WAIT_SECONDS 180
#define NFC_SLEEP_POLL_MS 400
#define ENROLL_CARD_TIMEOUT_SECONDS 30
#define FACE_TIMEOUT_SECONDS 45
#define FACE_THRESHOLD 0.60f
#define HTTP_RESPONSE_CAP 6144
#define IMAGE_CHUNK_SIZE 4096
#define MAX_ENROLL_JPEG_SIZE 131072

typedef enum { MODE_SLEEP=0, MODE_WAIT=1, MODE_WORK=2 } app_mode_t;
typedef void (*idle_callback_t)(void);

esp_err_t services_init(void);
void wifi_suspend(void);
bool server_request(const char *path, const void *body, size_t length,
                    const char *content_type, char *response, size_t capacity,
                    int *status, idle_callback_t idle);
// -1 = communication error, 0 = no card, 1 = card. Never report an I/O error as removal.
bool pn532_init(void);
int pn532_read_uid(char uid[21], idle_callback_t idle);
