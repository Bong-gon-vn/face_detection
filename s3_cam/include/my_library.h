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

#include <algorithm>
#include <cmath>
#include <vector>
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_camera.h"
#include "dl_image_jpeg.hpp"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "human_face_detect.hpp"
#include "human_face_recognition.hpp"
#define FACE_THRESHOLD 0.60f
#define FACE_WARMUP_FRAMES 12
#define MAX_ENROLL_JPEG_SIZE 131072
#define IMAGE_CHUNK_SIZE 4096

// Current hardware: OV2640. esp32-camera still detects the actual sensor PID.
#define CAMERA_EXPECTED_NAME "OV2640"
#define CAMERA_EXPECTED_PID OV2640_PID
#define CAMERA_XCLK_HZ 20000000
#define CAMERA_JPEG_QUALITY 16
