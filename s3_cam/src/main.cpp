// S3_CAM - ESP-IDF / PlatformIO. Application source: src/main.cpp
#include <algorithm>
#include <cmath>
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

#include "human_face_detect.hpp"
#include "human_face_recognition.hpp"
#include <vector>

static constexpr gpio_num_t CS=GPIO_NUM_48, READY=GPIO_NUM_42;
static constexpr int W=320,H=240,N=W*H*3;
static constexpr float THRESHOLD=0.60f;
static uint8_t *rgb,*best;
static float reference[DIM], best_quality;
static bool enrolling;
static std::vector<int> best_landmarks;
static bool camera_on;
static int64_t start_us,first_face_us;
static bool rotate_ai;
static unsigned samples;
static uint32_t sid, state;
static packet_t *tx,*rx;

static void stop_camera() {
    if (camera_on) {
        esp_camera_deinit();
        camera_on=false;
    }
}
static bool open_camera() {
    camera_config_t c={};
    c.pin_pwdn=-1; c.pin_reset=-1; c.pin_xclk=15; c.pin_pclk=13;
    c.pin_sccb_sda=4; c.pin_sccb_scl=5;
    c.pin_vsync=6; c.pin_href=7;
    c.pin_d0=11; c.pin_d1=9; c.pin_d2=8; c.pin_d3=10;
    c.pin_d4=12; c.pin_d5=18; c.pin_d6=17; c.pin_d7=16;
    c.xclk_freq_hz=20000000; c.ledc_timer=LEDC_TIMER_0; c.ledc_channel=LEDC_CHANNEL_0;
    c.pixel_format=PIXFORMAT_JPEG; c.frame_size=FRAMESIZE_QVGA; c.jpeg_quality=16;
    c.fb_count=1; c.fb_location=CAMERA_FB_IN_PSRAM; c.grab_mode=CAMERA_GRAB_WHEN_EMPTY;
    esp_err_t result = esp_camera_init(&c);
    camera_on = result == ESP_OK;
    ESP_LOGI("FACE", "Camera init: %s", esp_err_to_name(result));
    return camera_on;
}
static bool load_refs() {
    if (rx->size != sizeof(face_template_t)) return false;
    face_template_t incoming;
    memcpy(&incoming, rx->data, sizeof(incoming));
    if (incoming.version != TEMPLATE_VERSION) return false;
    double sum = 0;
    for (float value : incoming.vector) sum += double(value) * value;
    if (!std::isfinite(sum) || sum < 1e-12) return false;
    float inv = 1.0 / std::sqrt(sum);
    for (int i = 0; i < DIM; ++i) reference[i] = incoming.vector[i] * inv;
    return true;
}
static int gray(const uint8_t *pixel) {
    return (pixel[0] + 2 * pixel[1] + pixel[2]) / 4;
}
// Measure each face on a fixed 64x64 grid, independent of bounding-box size.
static float sharpness(const std::vector<int> &box) {
    uint8_t patch[64 * 64];
    for (int y = 0; y < 64; ++y) {
        int sy = box[1] + y * (box[3] - box[1]) / 64;
        for (int x = 0; x < 64; ++x) {
            int sx = box[0] + x * (box[2] - box[0]) / 64;
            patch[y * 64 + x] = gray(rgb + 3 * (sy * W + sx));
        }
    }
    double sum = 0, squares = 0;
    int count = 0;
    for (int y = 8; y < 56; ++y) {
        for (int x = 8; x < 56; ++x) {
            int i = y * 64 + x;
            int lap = patch[i-1] + patch[i+1] + patch[i-64] + patch[i+64] - 4*patch[i];
            sum += lap;
            squares += lap * lap;
            ++count;
        }
    }
    return float(squares / count - (sum / count) * (sum / count));
}
static bool sample_face() {
    static HumanFaceDetect detector(HumanFaceDetect::MSRMNP_S8_V1);
    dl::image::img_t image = {rgb, W, H, dl::image::DL_IMAGE_PIX_TYPE_RGB888};
    auto &faces = detector.run(image);
    if (faces.size() != 1) return false;
    const auto &face = faces.front();
    if (face.box.size() != 4 || face.keypoint.size() != 10) return false;
    int cx = (face.box[0] + face.box[2]) / 2;
    int cy = (face.box[1] + face.box[3]) / 2;
    if (cx < 121 || cx > 199 || cy < 79 || cy > 161 ||
        face.box[2] - face.box[0] < 48 || face.box[0] < 70 ||
        face.box[2] >= 250 || face.box[1] < 0 || face.box[3] >= H ||
        face.box[3] <= face.box[1]) return false;
    for (int i = 0; i < 10; i += 2) {
        if (face.keypoint[i] < 0 || face.keypoint[i] >= W ||
            face.keypoint[i+1] < 0 || face.keypoint[i+1] >= H) return false;
    }
    float quality = sharpness(face.box);
    if (quality > best_quality) {
        memcpy(best, rgb, N);
        best_landmarks = face.keypoint;
        best_quality = quality;
    }
    return true;
}
static bool extract_feature(face_template_t &result) {
    static HumanFaceFeat model(HumanFaceFeat::MFN_S8_V1);
    dl::image::img_t image = {best, W, H, dl::image::DL_IMAGE_PIX_TYPE_RGB888};
    dl::TensorBase *feature = model.run(image, best_landmarks);
    if (!feature || !feature->data || feature->get_size() != DIM ||
        feature->dtype != dl::DATA_TYPE_FLOAT) return false;
    const float *values = static_cast<const float *>(feature->data);
    double sum = 0;
    for (int i = 0; i < DIM; ++i) sum += double(values[i]) * values[i];
    if (!std::isfinite(sum) || sum < 1e-12) return false;
    result.version = TEMPLATE_VERSION;
    for (int i = 0; i < DIM; ++i) result.vector[i] = values[i] / std::sqrt(sum);
    return true;
}
static float compare(const face_template_t &feature) {
    double score = 0;
    for (int i = 0; i < DIM; ++i) score += double(feature.vector[i]) * reference[i];
    return float(score);
}
static void finish(uint32_t outcome) {
    stop_camera();
    state=outcome;
    seal(tx,sid,state,0);
}
static void sample_frame() {
    camera_fb_t *fb=esp_camera_fb_get();
    if(!fb) { finish(CAM_FAIL); return; }
    dl::image::jpeg_img_t jpeg = {fb->buf, fb->len};
    auto decoded = dl::image::sw_decode_jpeg(jpeg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    bool ok = decoded.data && decoded.width == W && decoded.height == H;
    rgb = static_cast<uint8_t *>(decoded.data);
    size_t bytes=fb->len<=CAP ? fb->len : 0;
    if(bytes) memcpy(tx->data,fb->buf,bytes);
    esp_camera_fb_return(fb);
    if(!ok) { heap_caps_free(rgb); rgb=nullptr; finish(CAM_FAIL); return; }
    // Preview uses the original JPEG. Try the opposite sensor orientation on the
    // next miss so a physically inverted module can still detect upright faces.
    if(rotate_ai) {
        for(int i=0;i<W*H/2;i++) for(int ch=0;ch<3;ch++)
            std::swap(rgb[3*i+ch],rgb[3*(W*H-1-i)+ch]);
    }
    if(sample_face()) {
        ++samples;
        if(!first_face_us) first_face_us=esp_timer_get_time();
    } else if(!samples) rotate_ai=!rotate_ai;
    heap_caps_free(rgb); rgb = nullptr;
    int64_t now=esp_timer_get_time();
    if(first_face_us && now-first_face_us>=2000000) {
        if(samples<3) { finish(NO_FACE); return; }
        face_template_t feature;
        if (!extract_feature(feature)) { finish(AI_FAIL); return; }
        if (enrolling) {
            finish(ENROLLED);
            memcpy(tx->data, &feature, sizeof(feature));
            seal(tx, sid, ENROLLED, sizeof(feature));
            return;
        }
        float score = compare(feature);
        finish(score>=THRESHOLD ? MATCH : DENIED);
        memcpy(tx->data,&score,sizeof(score)); seal(tx,sid,state,sizeof(score));
        ESP_LOGI("FACE","cosine %.3f; sharpness %.1f",score,best_quality);
    } else if(!first_face_us && now-start_us>=10000000) finish(NO_FACE);
    else { state=first_face_us ? SAMPLING : AIM; seal(tx,sid,state,bytes); }
}
static void prepared(spi_slave_transaction_t*) { gpio_set_level(READY,1); }
static void finished(spi_slave_transaction_t*) { gpio_set_level(READY,0); }
static void wait_for_boot_button() {
    stop_camera();
    memset(reference,0,sizeof(reference));
    sid=0; state=IDLE;
    seal(tx,0,IDLE,0);
    ESP_ERROR_CHECK(gpio_wakeup_enable(CS,GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    if(gpio_get_level(CS)) ESP_ERROR_CHECK(esp_light_sleep_start());
    ESP_ERROR_CHECK(gpio_wakeup_disable(CS));
    while(!gpio_get_level(CS)) vTaskDelay(1);
}
extern "C" void app_main() {
    tx=new_slot(); rx=new_slot();
    best=(uint8_t*)heap_caps_malloc(N,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    gpio_set_direction(READY,GPIO_MODE_OUTPUT); gpio_set_level(READY,0);
    spi_bus_config_t b={}; b.sclk_io_num=21; b.mosi_io_num=47; b.miso_io_num=41;
    b.quadhd_io_num=-1; b.quadwp_io_num=-1; b.max_transfer_sz=SLOT;
    spi_slave_interface_config_t s={}; s.spics_io_num=CS; s.mode=3; s.queue_size=1;
    s.post_setup_cb=prepared; s.post_trans_cb=finished;
    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST,&b,&s,SPI_DMA_CH_AUTO));
    gpio_pullup_en(CS);
    ESP_LOGI("FACE", "S3 SPI face v4 ready; waiting for U2 BOOT");
    wait_for_boot_button();
    while(true) {
        spi_slave_transaction_t t={}; t.length=SLOT*8; t.tx_buffer=tx; t.rx_buffer=rx;
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST,&t,portMAX_DELAY));
        spi_slave_transaction_t *done=nullptr;
        // A reset U2 may never clock the queued DMA buffer. Reset U1 only after
        // stopping capture. Never reuse a transaction still owned by the driver.
        // Keep the queued transaction and DMA buffers alive while waiting.
        // Do not queue another transaction until this one completes.
        while (true) {
            esp_err_t err = spi_slave_get_trans_result(
                SPI2_HOST,
                &done,
                pdMS_TO_TICKS(5000)
            );

            if (err == ESP_OK) {
                break;
            }

            if (err == ESP_ERR_TIMEOUT) {
                ESP_LOGW(
                    "FACE",
                    "Waiting SPI clocks: CS=%d READY=%d",
                    gpio_get_level(CS),
                    gpio_get_level(READY)
                );
                continue;
            }

            ESP_LOGE("FACE", "SPI receive failed: %s", esp_err_to_name(err));
            ESP_ERROR_CHECK(err);
        }
        if(camera_on && esp_timer_get_time()-start_us>20000000) finish(NO_FACE);
        if (t.trans_len != SLOT * 8) {
            ESP_LOGW(
                "FACE",
                "SPI length mismatch: received=%u expected=%u bits",
                (unsigned)t.trans_len,
                (unsigned)(SLOT * 8)
            );
            continue;
        }
        
        if (!valid(rx)) {
            ESP_LOGW(
                "FACE",
                "SPI invalid packet: magic=0x%08lx size=%lu session=%lu",
                (unsigned long)rx->magic,
                (unsigned long)rx->size,
                (unsigned long)rx->session
            );
            continue;
        }
        
        if (!rx->session) {
            ESP_LOGW("FACE", "SPI packet has session=0");
            continue;
        }
        if((rx->kind==START || rx->kind==ENROLL) && rx->session!=sid) {
            stop_camera(); sid=rx->session;
            best_quality=-1; first_face_us=0; samples=0; rotate_ai=false;
            enrolling = rx->kind == ENROLL;
            ESP_LOGI("FACE", "Session %lu: %s", (unsigned long)sid, enrolling ? "enroll" : "verify");
            if(!enrolling && !load_refs()) finish(BAD_REF);
            else if(!best) finish(AI_FAIL);
            else if(!open_camera()) finish(CAM_FAIL);
            else { start_us=esp_timer_get_time(); state=AIM; seal(tx,sid,state,0); }
        } else if(rx->session==sid && rx->kind==END_SESSION) {
            while(!gpio_get_level(CS)) vTaskDelay(1);
            wait_for_boot_button();
        }
        else if(rx->session==sid && rx->kind==POLL && camera_on) sample_frame();
    }
}
