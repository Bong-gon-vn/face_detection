// Regular ESP32: LCD, PN532, LAN requests and application modes.
#include "my_library.h"
#include "secrets.h"
#include "diagnostic_wifi.h"

// Write-only SPI. SDO/GPIO13 can remain physically connected per schematic.
#define TFT_CS_GPIO   21
#define TFT_RST_GPIO  19
#define TFT_DC_GPIO   18
#define TFT_MOSI_GPIO 17
#define TFT_SCK_GPIO  16
#define TFT_BL_GPIO    4    // -1 if backlight is powered via a separate circuit
#define TFT_BL_ON      1

#define TFT_SPI_MODE   3    // value from esp_thuong.ino
#define TFT_SPI_HZ     10000000
#define TFT_BGR        0    // set to 1 if red and blue are swapped
#define TFT_INVERT     0    // Disable color inversion on ST7789
#define TFT_MIRROR_X   1
#define TFT_MIRROR_Y   1
#define TFT_X_GAP      0
#define TFT_Y_GAP      0

#define CAM_WIDTH 320
#define CAM_HEIGHT 240
#define PANEL_W 240
#define PANEL_H 320
#define CROP_W 180


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
    preview_unmirror(portrait, PANEL_W, rows);
    overlay(y, rows);
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(lcd, 0, y, PANEL_W, y + rows, portrait));
    // tx_param waits for all prior DMA transfers. ST7789 NOP allows reusing stripe.
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(lcd_io, 0x00, NULL, 0));
    if (dump_lcd) {
        size_t bytes=(size_t)rows*PANEL_W*2;
        dump_lcd_hash=dump_hash(dump_lcd_hash,portrait,bytes);
        dump_data(1000,(unsigned)y*PANEL_W*2,portrait,bytes);
    }
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
#define CS GPIO_NUM_26
#define READY GPIO_NUM_33
#define BOOT GPIO_NUM_0
static spi_device_handle_t spi_link;
static packet_t *tx,*rx;
static const char *label;
static uint16_t color;
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
#include "diagnostics.h"

static app_mode_t mode = MODE_SLEEP;
static int64_t wait_deadline;
static bool preview_enabled;
static bool preview_ack;
static uint32_t preview_id;
static int64_t preview_wake_at;
static int64_t preview_last_frame;
static int64_t preview_last_poll;
static char response_body[HTTP_RESPONSE_CAP];
static uint8_t image_chunk[IMAGE_CHUNK_SIZE];
static face_template_t active_template;
static char held_uid[21];
static unsigned absent_reads;
static bool nfc_ok;
static unsigned nfc_error_count;
static portMUX_TYPE button_lock = portMUX_INITIALIZER_UNLOCKED;
static bool button_pending;
static int64_t button_down_us;
static int64_t button_up_us;

static void boot_interrupt(void *argument)
{
    (void)argument;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&button_lock);
    if (!gpio_get_level(BOOT)) {
        // Ignore contact bounce on an already pending press.
        if (!button_pending) {
            button_pending = true;
            button_down_us = now;
            button_up_us = 0;
        }
    } else if (button_pending) {
        button_up_us = now;
    }
    portEXIT_CRITICAL_ISR(&button_lock);
}

static bool boot_requested(void)
{
    portENTER_CRITICAL(&button_lock);
    bool pending = button_pending;
    portEXIT_CRITICAL(&button_lock);
    return pending || !gpio_get_level(BOOT);
}

static void clear_boot_event(void)
{
    portENTER_CRITICAL(&button_lock);
    button_pending = false;
    button_down_us = 0;
    button_up_us = 0;
    portEXIT_CRITICAL(&button_lock);
}

static uint32_t new_session_id(void)
{
    uint32_t id;
    do { id = esp_random(); } while (!id);
    return id;
}

static void new_event_id(char output[80])
{
    snprintf(output, 80, "%s-%08lx%08lx", DEVICE_ID,
             (unsigned long)esp_random(), (unsigned long)esp_random());
}

static void preview_begin(void)
{
    preview_id = new_session_id();
    preview_ack = false;
    preview_enabled = true;
    preview_wake_at = 0;
    preview_last_poll = 0;
    preview_last_frame = esp_timer_get_time();
}

static void preview_pump(void)
{
    if (!preview_enabled) return;
    int64_t now = esp_timer_get_time();
    if (now - preview_last_poll < 50000) return; // Cap requests at about 20 FPS.
    preview_last_poll = now;
    if (!gpio_get_level(READY)) {
        if (!preview_ack && now >= preview_wake_at) {
            gpio_set_level(CS, 0); sleep_ms(45); gpio_set_level(CS, 1);
            preview_wake_at = now + 1500000;
        }
    } else {
        seal(tx, preview_id, preview_ack ? POLL : STREAM, 0);
        if (exchange() && rx->session == preview_id && rx->kind == STREAM_FRAME) {
            preview_ack = true;
            preview_last_frame = now;
            if (rx->size) display_jpeg(rx->data, rx->size);
        }
    }
    if (now - preview_last_frame > 15000000) {
        ESP_LOGW("FACE", "Preview timeout; reopen SPI stream");
        preview_begin();
    }
}

static void enter_work(void)
{
    mode = MODE_WORK;
    ESP_LOGI("APP", "MODE=2 WORK");
    esp_lcd_panel_disp_on_off(lcd, true);
    gpio_set_level(TFT_BL_GPIO, TFT_BL_ON);
    if (!preview_enabled) preview_begin();
}

static void enter_wait(void)
{
    mode = MODE_WAIT;
    wait_deadline = esp_timer_get_time() + (int64_t)WAIT_SECONDS * 1000000;
    if (!preview_enabled) preview_begin();
    ESP_LOGI("APP", "MODE=1 WAIT, %d seconds", WAIT_SECONDS);
}

static void enter_sleep(void)
{
    preview_enabled = false;
    uint32_t id = preview_id ? preview_id : new_session_id();
    seal(tx, id, END_SESSION, 0);
    // S3 accepts END_SESSION regardless of its previous session ID.
    int64_t deadline = esp_timer_get_time() + 3000000;
    while (!gpio_get_level(READY) && esp_timer_get_time() < deadline) sleep_ms(10);
    if (gpio_get_level(READY)) exchange();
    gpio_set_level(TFT_BL_GPIO, !TFT_BL_ON);
    esp_lcd_panel_disp_on_off(lcd, false);
    wifi_suspend();
    memset(&active_template, 0, sizeof(active_template));
    memset(tx->data, 0, sizeof(face_template_t));
    mode = MODE_SLEEP;
    ESP_LOGI("APP", "MODE=0 SLEEP; PN532 remains powered, timer polls every %d ms", NFC_SLEEP_POLL_MS);
}

static void idle_sleep_once(void)
{
    // UART output must drain before clock suspension. BOOT low wakes immediately.
    fflush(stdout);
    gpio_wakeup_enable(BOOT, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup((uint64_t)NFC_SLEEP_POLL_MS * 1000);
    if (gpio_get_level(BOOT)) {
        esp_err_t rc = esp_light_sleep_start();
        if (rc != ESP_OK) sleep_ms(20);
    }
    gpio_wakeup_disable(BOOT);
    gpio_set_intr_type(BOOT, GPIO_INTR_ANYEDGE);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
}

static bool json_string(cJSON *root, const char *key, char *out, size_t capacity)
{
    cJSON *value = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsString(value) || !value->valuestring || strlen(value->valuestring) >= capacity) return false;
    strcpy(out, value->valuestring);
    return true;
}

static cJSON *json_request(const char *path, cJSON *data)
{
    if (!data) return NULL;
    char *body = cJSON_PrintUnformatted(data);
    cJSON_Delete(data);
    if (!body) return NULL;
    int status = 0;
    bool ok = server_request(path, body, strlen(body), "application/json",
                             response_body, sizeof(response_body), &status, preview_pump);
    cJSON_free(body);
    if (!ok) {
        message(status == 401 ? "INVALID API KEY" : "WIFI SERVER ERROR", 0xf800);
        return NULL;
    }
    cJSON *reply = cJSON_Parse(response_body);
    if (!cJSON_IsObject(reply)) {
        cJSON_Delete(reply);
        message("SERVER DATA ERROR", 0xf800);
        return NULL;
    }
    return reply;
}

static bool template_from_json(cJSON *root)
{
    cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "model_version");
    cJSON *encoded = cJSON_GetObjectItemCaseSensitive(root, "embedding_b64");
    cJSON *threshold = cJSON_GetObjectItemCaseSensitive(root, "threshold");
    if (!cJSON_IsNumber(version) || version->valuedouble != TEMPLATE_VERSION ||
        !cJSON_IsString(encoded) || !encoded->valuestring || !cJSON_IsNumber(threshold) ||
        fabs(threshold->valuedouble - FACE_THRESHOLD) > 0.00001) return false;
    size_t written = 0;
    int rc = mbedtls_base64_decode((unsigned char *)active_template.vector,
        sizeof(active_template.vector), &written, (const unsigned char *)encoded->valuestring,
        strlen(encoded->valuestring));
    if (rc || written != sizeof(active_template.vector)) return false;
    double norm = 0;
    for (int i = 0; i < DIM; ++i) norm += (double)active_template.vector[i] * active_template.vector[i];
    if (!isfinite(norm) || norm < 0.99 || norm > 1.01) return false;
    active_template.version = TEMPLATE_VERSION;
    return true;
}

static uint32_t run_face(bool enroll, float *score, uint32_t *capture_id)
{
    preview_enabled = false;
    uint32_t id = new_session_id();
    *capture_id = id;
    *score = NAN;
    if (enroll) seal(tx, id, ENROLL, 0);
    else {
        memcpy(tx->data, &active_template, sizeof(active_template));
        seal(tx, id, START, sizeof(active_template));
    }
    label = enroll ? "ENROLLING FACE" : "ALIGN FACE IN FRAME";
    color = 0xffe0;
    int64_t deadline = esp_timer_get_time() + (int64_t)FACE_TIMEOUT_SECONDS * 1000000;
    int64_t wake_at = 0;
    bool ack = false;
    uint32_t result = LINK_FAIL;
    while (esp_timer_get_time() < deadline) {
        if (!gpio_get_level(READY)) {
            if (!ack && esp_timer_get_time() >= wake_at) {
                gpio_set_level(CS, 0); sleep_ms(45); gpio_set_level(CS, 1);
                wake_at = esp_timer_get_time() + 1500000;
            }
            sleep_ms(10); continue;
        }
        if (!exchange() || rx->session != id) continue;
        if (rx->kind < AIM || rx->kind > ENROLLED) continue;
        ack = true;
        seal(tx, id, POLL, 0);
        if (rx->kind == AIM || rx->kind == SAMPLING) {
            label = rx->kind == AIM ? "ALIGN FACE IN FRAME" : "HOLD STILL";
            if (rx->size) display_jpeg(rx->data, rx->size);
            continue;
        }
        if (enroll && rx->kind == ENROLLED && rx->size == sizeof(active_template)) {
            memcpy(&active_template, rx->data, sizeof(active_template));
            result = ENROLLED;
        } else if (!enroll && (rx->kind == MATCH || rx->kind == DENIED) && rx->size == sizeof(float)) {
            memcpy(score, rx->data, sizeof(float));
            if (!isfinite(*score) || *score < -1.001f || *score > 1.001f) result = AI_FAIL;
            else result = rx->kind;
        } else if (rx->kind >= NO_FACE && rx->kind <= LINK_FAIL) {
            result = rx->kind;
        } else {
            result = AI_FAIL;
        }
        break;
    }
    preview_begin();
    preview_pump();
    return result;
}

static void show_face_error(uint32_t result)
{
    switch (result) {
        case NO_FACE: message("NO FACE DETECTED", 0xffe0); break;
        case CAM_FAIL: message("CAMERA ERROR", 0xf800); break;
        case BAD_REF: message("EMBEDDING ERROR", 0xf800); break;
        case LINK_FAIL: message("SPI LINK LOST", 0xf800); break;
        default: message("AI ERROR", 0xf800); break;
    }
}

static void verify_card(const char *uid)
{
    enter_work();
    message("QUERYING SERVER", 0xffe0);
    char event_id[80], ticket[768], status[32];
    new_event_id(event_id);
    cJSON *request = cJSON_CreateObject();
    cJSON_AddStringToObject(request, "uid", uid);
    cJSON_AddStringToObject(request, "event_id", event_id);
    cJSON *reply = json_request("/api/lookup", request);
    if (!reply) goto finished;
    if (!json_string(reply, "status", status, sizeof(status))) {
        cJSON_Delete(reply); message("SERVER DATA ERROR", 0xf800); goto finished;
    }
    if (strcmp(status, "approved") != 0) {
        message(strcmp(status, "unregistered") == 0 ? "UNREGISTERED" :
                strcmp(status, "pending") == 0 ? "PENDING APPROVAL" : "ACCESS DENIED", 0xffe0);
        ESP_LOGI("APP", "UID=%s denied: %s", uid, status);
        cJSON_Delete(reply); goto finished;
    }
    cJSON *server_time = cJSON_GetObjectItemCaseSensitive(reply, "server_time");
    int64_t epoch = cJSON_IsNumber(server_time) ? (int64_t)server_time->valuedouble : 0;
    int64_t synchronized_at = esp_timer_get_time();
    if (!template_from_json(reply) || !json_string(reply, "ticket", ticket, sizeof(ticket)) || !epoch) {
        cJSON_Delete(reply); message("SERVER MODEL ERROR", 0xf800); goto finished;
    }
    cJSON_Delete(reply);
    float score;
    uint32_t capture_id;
    uint32_t result = run_face(false, &score, &capture_id);
    bool face_result = result == MATCH || result == DENIED;
    const char *reason = face_result ? "face_result" : result == NO_FACE ? "no_face" :
                         result == CAM_FAIL ? "camera_error" : result == LINK_FAIL ? "spi_timeout" : "ai_error";
    request = cJSON_CreateObject();
    cJSON_AddStringToObject(request, "uid", uid);
    cJSON_AddStringToObject(request, "event_id", event_id);
    cJSON_AddStringToObject(request, "ticket", ticket);
    cJSON_AddStringToObject(request, "reason", reason);
    cJSON_AddNumberToObject(request, "device_time", epoch + (esp_timer_get_time()-synchronized_at)/1000000);
    cJSON_AddNumberToObject(request, "matched", result == MATCH ? 1 : 0);
    if (face_result) cJSON_AddNumberToObject(request, "score", score);
    else cJSON_AddNullToObject(request, "score");
    message("SENDING RESULT", 0xffe0);
    reply = json_request("/api/attendance", request);
    if (reply) {
        cJSON *saved = cJSON_GetObjectItemCaseSensitive(reply, "saved");
        cJSON *matched = cJSON_GetObjectItemCaseSensitive(reply, "matched");
        if (!cJSON_IsTrue(saved) || !cJSON_IsNumber(matched)) message("SAVE FAILED", 0xf800);
        else if (matched->valueint == 1) message("CHECK IN OK", 0x07e0);
        else if (face_result) message("CHECK IN DENIED", 0xf800);
        else show_face_error(result);
        ESP_LOGI("APP", "UID=%s score=%.4f reply=%.160s", uid, score, response_body);
        cJSON_Delete(reply);
    }
finished:
    memset(&active_template, 0, sizeof(active_template));
    enter_wait();
}

static bool get_image_chunk(uint32_t capture_id, uint32_t offset, uint32_t *total, uint32_t *length)
{
    // A fresh STREAM must be acknowledged before addressing its retained image.
    int64_t deadline = esp_timer_get_time() + 10000000;
    while (!preview_ack && esp_timer_get_time() < deadline) {
        preview_pump(); sleep_ms(10);
    }
    if (!preview_ack) return false;
    while (esp_timer_get_time() < deadline) {
        if (!gpio_get_level(READY)) { sleep_ms(10); continue; }
        memcpy(tx->data, &capture_id, 4);
        memcpy(tx->data+4, &offset, 4);
        seal(tx, preview_id, IMAGE_GET, 8);
        if (!exchange() || rx->session != preview_id) continue;
        if (rx->kind == STREAM_FRAME && rx->size) {
            display_jpeg(rx->data, rx->size); continue;
        }
        if (rx->kind != IMAGE_DATA || rx->size <= 12 || rx->size > IMAGE_CHUNK_SIZE+12) continue;
        uint32_t got_id, got_offset, got_total;
        memcpy(&got_id, rx->data, 4); memcpy(&got_total, rx->data+4, 4);
        memcpy(&got_offset, rx->data+8, 4);
        if (got_id != capture_id || got_offset != offset) continue;
        uint32_t bytes = rx->size - 12;
        if (!got_total || got_total > MAX_ENROLL_JPEG_SIZE || offset > got_total || bytes > got_total-offset) return false;
        memcpy(image_chunk, rx->data+12, bytes);
        *total = got_total; *length = bytes;
        return true;
    }
    return false;
}

static bool upload_enrollment(const char *enrollment_id, const char *uid, uint32_t capture_id)
{
    uint32_t total, length;
    if (!get_image_chunk(capture_id, 0, &total, &length)) {
        message("S3 IMAGE ERROR", 0xf800); return false;
    }
    unsigned char encoded[DIM*4*4/3+8];
    size_t encoded_length = 0;
    if (mbedtls_base64_encode(encoded, sizeof(encoded), &encoded_length,
        (unsigned char *)active_template.vector, sizeof(active_template.vector))) return false;
    encoded[encoded_length] = 0;
    cJSON *request = cJSON_CreateObject();
    cJSON_AddStringToObject(request, "enrollment_id", enrollment_id);
    cJSON_AddStringToObject(request, "uid", uid);
    cJSON_AddStringToObject(request, "embedding_b64", (const char *)encoded);
    cJSON_AddNumberToObject(request, "model_version", active_template.version);
    cJSON_AddNumberToObject(request, "image_size", total);
    cJSON *reply = json_request("/api/enrollment/metadata", request);
    if (!reply) return false;
    cJSON_Delete(reply);
    uint32_t offset = 0;
    bool completed = false;
    while (offset < total) {
        uint32_t got_total;
        if (!get_image_chunk(capture_id, offset, &got_total, &length) || got_total != total) {
            message("S3 IMAGE ERROR", 0xf800); return false;
        }
        char path[160];
        snprintf(path, sizeof(path), "/api/enrollment/image?id=%s&offset=%lu", enrollment_id, (unsigned long)offset);
        int status = 0;
        if (!server_request(path, image_chunk, length, "image/jpeg", response_body,
                            sizeof(response_body), &status, preview_pump)) {
            message("IMAGE UPLOAD ERROR", 0xf800); return false;
        }
        reply = cJSON_Parse(response_body);
        cJSON *ack = cJSON_GetObjectItemCaseSensitive(reply, "offset");
        cJSON *done = cJSON_GetObjectItemCaseSensitive(reply, "complete");
        bool accepted = cJSON_IsNumber(ack) && ack->valuedouble == offset + length;
        completed = cJSON_IsTrue(done);
        cJSON_Delete(reply);
        if (!accepted) { message("IMAGE OFFSET ERROR", 0xf800); return false; }
        offset += length;
        ESP_LOGI("ENROLL", "JPEG %lu/%lu", (unsigned long)offset, (unsigned long)total);
    }
    return completed;
}

static void register_user(void)
{
    message("STARTING ENROLL", 0xffe0);
    char event_id[80], enrollment_id[64], uid[21];
    new_event_id(event_id);
    cJSON *request = cJSON_CreateObject();
    cJSON_AddStringToObject(request, "event_id", event_id);
    cJSON *reply = json_request("/api/enrollment/start", request);
    if (!reply) goto finished;
    bool ready = json_string(reply, "enrollment_id", enrollment_id, sizeof(enrollment_id));
    cJSON_Delete(reply);
    if (!ready) { message("SERVER DATA ERROR", 0xf800); goto finished; }
    message("SCAN CARD TO ENROLL", 0xffe0);
    int64_t deadline = esp_timer_get_time() + (int64_t)ENROLL_CARD_TIMEOUT_SECONDS * 1000000;
    bool found = false;
    while (esp_timer_get_time() < deadline) {
        preview_pump();
        if (pn532_read_uid(uid, preview_pump) == 1) { found = true; break; }
        sleep_ms(20);
    }
    if (!found) { message("CARD SCAN TIMEOUT", 0xffe0); goto finished; }
    strcpy(held_uid, uid); absent_reads = 0;
    float ignored;
    uint32_t capture_id;
    uint32_t result = run_face(true, &ignored, &capture_id);
    if (result != ENROLLED) { show_face_error(result); goto finished; }
    message("UPLOADING ENROLL", 0xffe0);
    if (upload_enrollment(enrollment_id, uid, capture_id)) {
        message("PENDING APPROVAL", 0x07e0);
        ESP_LOGI("ENROLL", "UID=%s saved pending; approve on PC", uid);
    }
finished:
    memset(&active_template, 0, sizeof(active_template));
    enter_wait();
}

static void boot_action(void)
{
    portENTER_CRITICAL(&button_lock);
    int64_t pressed_at = button_down_us;
    int64_t released_at = button_up_us;
    portEXIT_CRITICAL(&button_lock);
    if (!pressed_at) pressed_at = esp_timer_get_time();
    if (released_at && released_at - pressed_at < 25000) {
        clear_boot_event(); return;
    }
    enter_work();
    message("RELEASE TO ENROLL", 0xffe0);
    bool held = released_at && released_at - pressed_at >= 2000000;
    while (!gpio_get_level(BOOT)) {
        preview_pump();
        if (esp_timer_get_time() - pressed_at >= 2000000) { held = true; break; }
        sleep_ms(10);
    }
    if (held) {
        preview_enabled = false;
        diagnostic_session();
        preview_begin();
        enter_wait();
    } else {
        register_user();
    }
    // Never interpret the release of a long press as a new short press.
    while (!gpio_get_level(BOOT)) { preview_pump(); sleep_ms(10); }
    sleep_ms(30);
    clear_boot_event();
}

void app_main(void)
{
    esp_err_t rc = nvs_flash_init();
    // Do not silently erase NVS on an initialization error.
    ESP_ERROR_CHECK(rc);
    tx = new_slot(); rx = new_slot();
    gpio_set_level(CS, 1); gpio_set_direction(CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(BOOT, GPIO_MODE_INPUT); gpio_pullup_en(BOOT);
    gpio_set_direction(READY, GPIO_MODE_INPUT); gpio_pulldown_en(READY);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_set_intr_type(BOOT, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BOOT, boot_interrupt, NULL));
    label = "BOOTING"; color = 0xffff;
    display_init();
    spi_bus_config_t bus = {.sclk_io_num=14, .mosi_io_num=27, .miso_io_num=25,
        .quadhd_io_num=-1, .quadwp_io_num=-1, .max_transfer_sz=SLOT};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t device = {.clock_speed_hz=4000000, .mode=3,
                                            .spics_io_num=-1, .queue_size=1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &device, &spi_link));
    ESP_ERROR_CHECK(services_init());
    nfc_ok = pn532_init();
    ESP_LOGI("APP", "Short BOOT: enrollment; hold 2s: USB capture. PN532 SDA22 SCL23");
    enter_sleep();
    int64_t nfc_retry_at = 0;
    for (;;) {
        if (boot_requested()) { boot_action(); continue; }
        if (mode != MODE_SLEEP) preview_pump();
        if (!nfc_ok && esp_timer_get_time() >= nfc_retry_at) {
            nfc_ok = pn532_init();
            nfc_retry_at = esp_timer_get_time() + 5000000;
        }
        char uid[21];
        int detected = nfc_ok ? pn532_read_uid(uid, mode == MODE_SLEEP ? NULL : preview_pump) : -1;
        if (boot_requested()) { boot_action(); continue; }
        if (detected < 0 && nfc_ok) {
            if (++nfc_error_count >= 3) {
                ESP_LOGW("NFC", "Repeated I2C error; reinitialize PN532");
                nfc_ok = false;
                nfc_retry_at = esp_timer_get_time() + 1000000;
                nfc_error_count = 0;
            }
        } else if (detected >= 0) nfc_error_count = 0;
        if (detected == 1) {
            absent_reads = 0;
            if (strcmp(uid, held_uid) != 0) {
                strcpy(held_uid, uid);
                verify_card(uid);
            }
        } else if (detected == 0) {
            if (++absent_reads >= 2) held_uid[0] = 0;
        }
        if (mode == MODE_WAIT && esp_timer_get_time() >= wait_deadline) enter_sleep();
        if (mode == MODE_SLEEP) idle_sleep_once();
        else sleep_ms(30);
    }
}