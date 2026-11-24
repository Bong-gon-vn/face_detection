// S3: camera, preview, embedding and cosine verification. No network credentials.
#include "my_library.h"

static constexpr gpio_num_t CS=GPIO_NUM_48, READY=GPIO_NUM_42;
static constexpr int W=320,H=240,N=W*H*3;
static constexpr float THRESHOLD=FACE_THRESHOLD;
static uint8_t *rgb,*best;
static float reference[DIM], best_quality;
static bool enrolling;
static bool streaming;
static unsigned warmup_left;
static uint8_t *enroll_jpeg;
static uint32_t enroll_jpeg_size;
static uint32_t enroll_capture_id;
static std::vector<int> best_landmarks;
static bool camera_on;
// Retain identification after deinit so capture.json describes the captured sensor.
static unsigned detected_sensor_pid;
static const char *detected_sensor_name = "unknown";
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
static bool open_camera(framesize_t size = FRAMESIZE_QVGA) {
    camera_config_t c={};
    c.pin_pwdn=-1; c.pin_reset=-1; c.pin_xclk=15; c.pin_pclk=13;
    c.pin_sccb_sda=4; c.pin_sccb_scl=5;
    c.pin_vsync=6; c.pin_href=7;
    c.pin_d0=11; c.pin_d1=9; c.pin_d2=8; c.pin_d3=10;
    c.pin_d4=12; c.pin_d5=18; c.pin_d6=17; c.pin_d7=16;
    c.xclk_freq_hz=CAMERA_XCLK_HZ; c.ledc_timer=LEDC_TIMER_0; c.ledc_channel=LEDC_CHANNEL_0;
    c.pixel_format=PIXFORMAT_JPEG; c.frame_size=size; c.jpeg_quality=CAMERA_JPEG_QUALITY;
    c.fb_count=1; c.fb_location=CAMERA_FB_IN_PSRAM; c.grab_mode=CAMERA_GRAB_WHEN_EMPTY;
    detected_sensor_pid = 0;
    detected_sensor_name = "unknown";
    esp_err_t result = esp_camera_init(&c);
    camera_on = result == ESP_OK;
    ESP_LOGI("FACE", "Camera init: %s", esp_err_to_name(result));
    if (!camera_on) return false;
    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGE("FACE", "Camera initialized without a sensor handle");
        stop_camera();
        return false;
    }
    detected_sensor_pid = sensor->id.PID;
    const camera_sensor_info_t *info = esp_camera_sensor_get_info(&sensor->id);
    if (info && info->name) detected_sensor_name = info->name;
    ESP_LOGI("FACE", "Sensor=%s PID=0x%04X expected=%s; XCLK=%d JPEG quality=%d",
             detected_sensor_name, detected_sensor_pid, CAMERA_EXPECTED_NAME,
             CAMERA_XCLK_HZ, CAMERA_JPEG_QUALITY);
    if (sensor->id.PID != CAMERA_EXPECTED_PID) {
        ESP_LOGW("FACE", "Sensor differs from configured hardware; using detected driver");
    }
    // Keep the tested driver's OV2640 defaults. Do not force exposure/gain/mirror:
    // changing them here would change recognition images and existing enrollment.
    ESP_LOGI("FACE", "Sensor state: AEC=%u AGC=%u AWB=%u hmirror=%u vflip=%u",
             unsigned(sensor->status.aec), unsigned(sensor->status.agc),
             unsigned(sensor->status.awb), unsigned(sensor->status.hmirror),
             unsigned(sensor->status.vflip));
    return true;
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
static HumanFaceDetect &face_detector() {
    static HumanFaceDetect detector(HumanFaceDetect::MSRMNP_S8_V1);
    return detector;
}
static bool sample_face() {
    dl::image::img_t image = {rgb, W, H, dl::image::DL_IMAGE_PIX_TYPE_RGB888};
    auto &faces = face_detector().run(image);
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
    state=outcome;
    seal(tx,sid,state,0);
}
// Preserve the ORIGINAL JPEG belonging to the RGB sample selected for embedding.
// It is retained in PSRAM during the subsequent stream and chunked upload.
static void stream_frame() {
    if (!camera_on && !open_camera()) { finish(CAM_FAIL); return; }
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) { finish(CAM_FAIL); return; }
    size_t bytes = frame->len <= CAP ? frame->len : 0;
    if (bytes) memcpy(tx->data, frame->buf, bytes);
    esp_camera_fb_return(frame);
    seal(tx, sid, STREAM_FRAME, bytes);
}

static void sample_frame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { finish(CAM_FAIL); return; }
    size_t bytes = fb->len <= CAP ? fb->len : 0;
    if (bytes) memcpy(tx->data, fb->buf, bytes);
    if (warmup_left) {
        --warmup_left;
        esp_camera_fb_return(fb);
        state = AIM;
        seal(tx, sid, AIM, bytes);
        vTaskDelay(pdMS_TO_TICKS(80));
        return;
    }
    dl::image::jpeg_img_t jpeg = {fb->buf, fb->len};
    auto decoded = dl::image::sw_decode_jpeg(jpeg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    rgb = static_cast<uint8_t *>(decoded.data);
    if (!rgb || decoded.width != W || decoded.height != H) {
        esp_camera_fb_return(fb);
        heap_caps_free(rgb); rgb = nullptr;
        finish(CAM_FAIL); return;
    }
    if (rotate_ai) {
        for (int i=0; i<W*H/2; ++i) for (int ch=0; ch<3; ++ch)
            std::swap(rgb[3*i+ch], rgb[3*(W*H-1-i)+ch]);
    }
    float previous_quality = best_quality;
    if (sample_face()) {
        ++samples;
        if (!first_face_us) first_face_us = esp_timer_get_time();
        if (enrolling && best_quality > previous_quality) {
            enroll_jpeg_size = 0;
            if (enroll_jpeg && fb->len <= MAX_ENROLL_JPEG_SIZE) {
                memcpy(enroll_jpeg, fb->buf, fb->len);
                enroll_jpeg_size = fb->len;
                enroll_capture_id = sid;
            }
        }
    } else if (!samples) {
        rotate_ai = !rotate_ai;
    }
    esp_camera_fb_return(fb);
    heap_caps_free(rgb); rgb = nullptr;
    int64_t now = esp_timer_get_time();
    if (first_face_us && now-first_face_us >= 2000000) {
        if (samples < 3) { finish(NO_FACE); return; }
        face_template_t feature;
        if (!extract_feature(feature)) { finish(AI_FAIL); return; }
        if (enrolling) {
            if (!enroll_jpeg_size) { finish(CAM_FAIL); return; }
            finish(ENROLLED);
            memcpy(tx->data, &feature, sizeof(feature));
            seal(tx, sid, ENROLLED, sizeof(feature));
        } else {
            float score = compare(feature);
            // A normalized dot product may exceed one by a few rounding bits.
            score = std::max(-1.0f, std::min(1.0f, score));
            finish(score >= THRESHOLD ? MATCH : DENIED);
            memcpy(tx->data, &score, sizeof(score));
            seal(tx, sid, state, sizeof(score));
            ESP_LOGI("FACE", "cosine %.4f; sharpness %.1f", score, best_quality);
        }
    } else if (!first_face_us && now-start_us >= 15000000) {
        finish(NO_FACE);
    } else {
        state = first_face_us ? SAMPLING : AIM;
        seal(tx, sid, state, bytes);
    }
}

static void image_reply() {
    // Request: capture ID + byte offset. Reply: capture ID + total + offset + bytes.
    uint32_t capture=0, offset=0;
    if (rx->size == 8) {
        memcpy(&capture, rx->data, 4);
        memcpy(&offset, rx->data+4, 4);
    }
    if (!enroll_jpeg_size || capture != enroll_capture_id || offset >= enroll_jpeg_size) {
        seal(tx, sid, CAM_FAIL, 0); return;
    }
    uint32_t bytes = std::min(enroll_jpeg_size-offset, uint32_t(IMAGE_CHUNK_SIZE));
    memcpy(tx->data, &capture, 4);
    memcpy(tx->data+4, &enroll_jpeg_size, 4);
    memcpy(tx->data+8, &offset, 4);
    memcpy(tx->data+12, enroll_jpeg+offset, bytes);
    seal(tx, sid, IMAGE_DATA, bytes+12);
}
#include "diagnostics.hpp"

static void prepared(spi_slave_transaction_t*) { gpio_set_level(READY,1); }
static void finished(spi_slave_transaction_t*) { gpio_set_level(READY,0); }
static void wait_for_boot_button() {
    diag_clear();
    stop_camera();
    streaming=false; enroll_jpeg_size=0; enroll_capture_id=0;
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
    enroll_jpeg=(uint8_t*)heap_caps_malloc(MAX_ENROLL_JPEG_SIZE,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    gpio_set_direction(READY,GPIO_MODE_OUTPUT); gpio_set_level(READY,0);
    spi_bus_config_t b={}; b.sclk_io_num=21; b.mosi_io_num=47; b.miso_io_num=41;
    b.quadhd_io_num=-1; b.quadwp_io_num=-1; b.max_transfer_sz=SLOT;
    spi_slave_interface_config_t s={}; s.spics_io_num=CS; s.mode=3; s.queue_size=1;
    s.post_setup_cb=prepared; s.post_trans_cb=finished;
    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST,&b,&s,SPI_DMA_CH_AUTO));
    gpio_pullup_en(CS);
    ESP_LOGI("FACE", "S3 LAN face ready; waiting for regular ESP32");
    wait_for_boot_button();
    while(true) {
        spi_slave_transaction_t t={}; t.length=SLOT*8; t.tx_buffer=tx; t.rx_buffer=rx;
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST,&t,portMAX_DELAY));
        spi_slave_transaction_t *done=nullptr;
        // A reset U2 may never clock the queued DMA buffer. Reset U1 only after
        // stopping capture. Never reuse a transaction still owned by the driver.
        // Keep the queued transaction and DMA buffers alive while waiting.
        // Do not queue another transaction until this one completes.
        int64_t clocks_deadline = esp_timer_get_time() + 60000000;
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
                if (esp_timer_get_time() > clocks_deadline) {
                    // Never free/reuse DMA buffers owned by a queued SPI transaction.
                    // Reset recovers the driver, then startup returns to light sleep.
                    stop_camera();
                    ESP_LOGE("FACE", "No SPI clocks for 60 seconds; restart into sleep");
                    esp_restart();
                }
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
        if (rx->kind == END_SESSION) {
            // Accept even after the regular ESP32 rebooted and forgot the old session.
            while (!gpio_get_level(CS)) vTaskDelay(1);
            wait_for_boot_button();
        } else if (rx->kind == DEBUG_CAPTURE) {
            if (rx->session != sid || !diag_active) {
                stop_camera(); diag_clear(); sid=rx->session;
                streaming=false; enroll_jpeg_size=0;
                diag_active=true;
                diag_capture();
            }
            diag_ready();
        } else if (rx->kind == DEBUG_GET && rx->session == sid && diag_active) {
            diag_reply();
        } else if (rx->kind == STREAM) {
            if (diag_active) { diag_clear(); stop_camera(); }
            sid=rx->session; streaming=true;
            stream_frame();
        } else if ((rx->kind == START || rx->kind == ENROLL) && rx->session != sid) {
            if (diag_active) { diag_clear(); stop_camera(); }
            sid=rx->session; streaming=false;
            best_quality=-1; first_face_us=0; samples=0; rotate_ai=false;
            enroll_jpeg_size=0; enroll_capture_id=0;
            enrolling=rx->kind == ENROLL;
            ESP_LOGI("FACE", "Session %lu: %s", (unsigned long)sid, enrolling ? "enroll" : "verify");
            if (!enrolling && !load_refs()) finish(BAD_REF);
            else if (!best || (enrolling && !enroll_jpeg)) finish(AI_FAIL);
            else if (!camera_on && !open_camera()) finish(CAM_FAIL);
            else {
                warmup_left=FACE_WARMUP_FRAMES;
                start_us=esp_timer_get_time(); state=AIM; seal(tx,sid,state,0);
            }
        } else if (rx->kind == IMAGE_GET && rx->session == sid && !diag_active) {
            image_reply();
        } else if (rx->session == sid && rx->kind == POLL && !diag_active) {
            if (streaming) stream_frame();
            else if (camera_on && (state == AIM || state == SAMPLING)) sample_frame();
            // Terminal result remains in tx until explicit STREAM / START / ENROLL.
        }
    }
}
