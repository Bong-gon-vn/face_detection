#include "my_library.h"
#include "secrets.h"
#include "driver/i2c_master.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "SERVICES";
static EventGroupHandle_t wifi_events;
static bool wifi_running;
static bool wifi_wanted;
static QueueHandle_t http_jobs;
static SemaphoreHandle_t http_done;
#define WIFI_READY BIT0

typedef struct {
    const char *path;
    const void *body;
    size_t length;
    const char *content_type;
    char *response;
    size_t capacity;
    size_t received;
    bool overflow;
    bool ok;
    int status;
} http_job_t;

static void delay_ms(int ms) { vTaskDelay(pdMS_TO_TICKS(ms) + 1); }

static void wifi_event(void *argument, esp_event_base_t base, int32_t id, void *data)
{
    (void)argument; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (wifi_wanted) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_events, WIFI_READY);
        if (wifi_wanted) esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_events, WIFI_READY);
    }
}

static bool wifi_connect_for_request(void)
{
    if (!wifi_running) {
        wifi_wanted = true;
        esp_err_t rc = esp_wifi_start();
        if (rc != ESP_OK) {
            wifi_wanted = false;
            ESP_LOGE(TAG, "WiFi start: %s", esp_err_to_name(rc));
            return false;
        }
        wifi_running = true;
    }
    EventBits_t bits = xEventGroupWaitBits(wifi_events, WIFI_READY, pdFALSE, pdTRUE, pdMS_TO_TICKS(12000));
    return (bits & WIFI_READY) != 0;
}

void wifi_suspend(void)
{
    wifi_wanted = false;
    if (wifi_running) {
        esp_wifi_stop();
        wifi_running = false;
    }
    xEventGroupClearBits(wifi_events, WIFI_READY);
}

static esp_err_t http_event(esp_http_client_event_t *event)
{
    http_job_t *job = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_DATA && event->data_len > 0) {
        size_t bytes = (size_t)event->data_len;
        if (job->received + bytes >= job->capacity) {
            job->overflow = true;
            return ESP_FAIL;
        }
        memcpy(job->response + job->received, event->data, bytes);
        job->received += bytes;
        job->response[job->received] = 0;
    }
    return ESP_OK;
}

static void http_worker(void *argument)
{
    (void)argument;
    http_job_t *job;
    for (;;) {
        if (xQueueReceive(http_jobs, &job, portMAX_DELAY) != pdTRUE) continue;
        job->ok = false;
        job->status = 0;
        if (wifi_connect_for_request()) {
            char url[320];
            int n = snprintf(url, sizeof(url), "%s%s", LAN_SERVER_URL, job->path);
            if (n > 0 && n < sizeof(url)) {
                esp_http_client_config_t config = {
                    .url = url, .method = HTTP_METHOD_POST, .timeout_ms = 10000,
                    .event_handler = http_event, .user_data = job,
                    .buffer_size = 1024, .buffer_size_tx = 1024,
                    .disable_auto_redirect = true,
                };
                esp_http_client_handle_t client = esp_http_client_init(&config);
                if (client) {
                    esp_http_client_set_header(client, "X-API-Key", DEVICE_API_KEY);
                    esp_http_client_set_header(client, "X-Device-ID", DEVICE_ID);
                    esp_http_client_set_header(client, "Content-Type", job->content_type);
                    esp_http_client_set_post_field(client, job->body, (int)job->length);
                    esp_err_t rc = esp_http_client_perform(client);
                    job->status = esp_http_client_get_status_code(client);
                    job->ok = rc == ESP_OK && !job->overflow;
                    if (!job->ok) ESP_LOGW(TAG, "HTTP transport: %s", esp_err_to_name(rc));
                    esp_http_client_cleanup(client);
                }
            }
        } else {
            ESP_LOGW(TAG, "WiFi: no IP after 12 seconds");
        }
        xSemaphoreGive(http_done);
    }
}

esp_err_t services_init(void)
{
    esp_err_t rc = esp_netif_init();
    if (rc != ESP_OK) return rc;
    rc = esp_event_loop_create_default();
    if (rc != ESP_OK && rc != ESP_ERR_INVALID_STATE) return rc;
    if (!esp_netif_create_default_wifi_sta()) return ESP_ERR_NO_MEM;
    wifi_events = xEventGroupCreate();
    http_jobs = xQueueCreate(1, sizeof(http_job_t *));
    http_done = xSemaphoreCreateBinary();
    if (!wifi_events || !http_jobs || !http_done) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    rc = esp_wifi_init(&init);
    if (rc != ESP_OK) return rc;
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t config = {0};
    if (strlen(WIFI_SSID) > sizeof(config.sta.ssid) || strlen(WIFI_PASSWORD) >= sizeof(config.sta.password)) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(config.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
    memcpy(config.sta.password, WIFI_PASSWORD, strlen(WIFI_PASSWORD));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    if (xTaskCreate(http_worker, "face_http", 6144, NULL, 4, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

bool server_request(const char *path, const void *body, size_t length,
                    const char *content_type, char *response, size_t capacity,
                    int *status, idle_callback_t idle)
{
    // Call from the UI task only. Worker never accesses LCD or SPI buffers.
    http_job_t job = {.path=path, .body=body, .length=length, .content_type=content_type,
                      .response=response, .capacity=capacity};
    http_job_t *pointer = &job;
    for (int attempt = 0; attempt < 2; ++attempt) {
        job.received = 0; job.overflow = false; response[0] = 0;
        if (xQueueSend(http_jobs, &pointer, portMAX_DELAY) != pdTRUE) return false;
        // Do not leave this scope until worker finishes: job/body lifetimes matter.
        while (xSemaphoreTake(http_done, pdMS_TO_TICKS(20)) != pdTRUE) {
            if (idle) idle();
        }
        *status = job.status;
        if (job.ok && job.status >= 200 && job.status < 300) return true;
        if (job.ok && job.status >= 400 && job.status < 500) break;
    }
    ESP_LOGW(TAG, "Request failed path=%s HTTP=%d body=%.160s", path, *status, response);
    return false;
}

// PN532 native I2C, no Arduino dependency. Board switches must select I2C.
// NXP 8-bit address 0x48 corresponds to ESP-IDF 7-bit address 0x24.
static i2c_master_bus_handle_t nfc_bus;
static i2c_master_dev_handle_t nfc_device;
static bool nfc_ready;

static esp_err_t nfc_last_status_error;
static uint8_t nfc_last_status;

static bool nfc_wait_ready(int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    nfc_last_status_error = ESP_OK;
    nfc_last_status = 0;
    while (esp_timer_get_time() < deadline) {
        uint8_t status = 0;
        esp_err_t rc = i2c_master_receive(nfc_device, &status, 1, 100);
        nfc_last_status_error = rc;
        nfc_last_status = status;
        if (rc == ESP_OK && (status & 1)) return true;
        // Finish PN532 ACK/response exchange before servicing the LCD.
        // A stream callback can spend much longer than this polling interval.
        delay_ms(5);
    }
    return false;
}

static void nfc_abort(void)
{
    // Host ACK aborts a command still waiting for an RF target.
    const uint8_t ack[] = {0, 0, 0xff, 0, 0xff, 0};
    i2c_master_transmit(nfc_device, ack, sizeof(ack), 100);
    delay_ms(10);
    uint8_t status = 0;
    if (i2c_master_receive(nfc_device, &status, 1, 50) == ESP_OK && (status & 1)) {
        uint8_t discard[96];
        i2c_master_receive(nfc_device, discard, sizeof(discard), 100);
    }
}

static int nfc_command(const uint8_t *command, size_t length, uint8_t *reply, size_t capacity, int timeout_ms)
{
    uint8_t frame[48] = {0};
    if (!length || length > 36) return -1;
    uint8_t cmd = command[0];
    uint8_t wire_length = (uint8_t)(length + 1);
    frame[2] = 0xff;
    frame[3] = wire_length;
    frame[4] = (uint8_t)(0 - wire_length);
    frame[5] = 0xd4;
    memcpy(frame + 6, command, length);
    uint8_t sum = 0xd4;
    for (size_t i = 0; i < length; ++i) sum += command[i];
    frame[6 + length] = (uint8_t)(0 - sum);

    esp_err_t rc = i2c_master_transmit(nfc_device, frame, length + 8, 200);
    if (rc != ESP_OK) {
        ESP_LOGW("PN532", "cmd=0x%02X phase=WRITE error=%s", cmd, esp_err_to_name(rc));
        nfc_abort(); return -1;
    }
    if (!nfc_wait_ready(500)) {
        ESP_LOGW("PN532", "cmd=0x%02X phase=ACK_WAIT status=0x%02X i2c=%s",
                 cmd, nfc_last_status, esp_err_to_name(nfc_last_status_error));
        nfc_abort(); return -1;
    }
    uint8_t ack[7] = {0};
    const uint8_t expected[] = {0, 0, 0xff, 0, 0xff, 0};
    rc = i2c_master_receive(nfc_device, ack, sizeof(ack), 200);
    if (rc != ESP_OK || !(ack[0] & 1) || memcmp(ack + 1, expected, sizeof(expected)) != 0) {
        ESP_LOGW("PN532", "cmd=0x%02X phase=ACK_READ error=%s", cmd, esp_err_to_name(rc));
        ESP_LOG_BUFFER_HEX_LEVEL("PN532", ack, sizeof(ack), ESP_LOG_WARN);
        nfc_abort(); return -1;
    }
    if (!nfc_wait_ready(timeout_ms)) {
        ESP_LOGW("PN532", "cmd=0x%02X phase=RESPONSE_WAIT status=0x%02X i2c=%s timeout=%dms",
                 cmd, nfc_last_status, esp_err_to_name(nfc_last_status_error), timeout_ms);
        nfc_abort(); return -1;
    }
    uint8_t response[96] = {0};
    rc = i2c_master_receive(nfc_device, response, sizeof(response), 200);
    if (rc != ESP_OK) {
        ESP_LOGW("PN532", "cmd=0x%02X phase=RESPONSE_READ error=%s", cmd, esp_err_to_name(rc));
        nfc_abort(); return -1;
    }
    const uint8_t *p = response + 1;
    if (!(response[0] & 1) || p[0] || p[1] || p[2] != 0xff ||
        (uint8_t)(p[3] + p[4]) != 0 || p[3] < 2 || p[3] > sizeof(response) - 8) {
        ESP_LOGW("PN532", "cmd=0x%02X phase=FRAME_HEADER", cmd);
        ESP_LOG_BUFFER_HEX_LEVEL("PN532", response, 24, ESP_LOG_WARN);
        nfc_abort(); return -1;
    }
    size_t data_length = p[3];
    sum = 0;
    for (size_t i = 0; i <= data_length; ++i) sum += p[5 + i];
    if (sum != 0 || p[6 + data_length] != 0 || p[5] != 0xd5 || p[6] != cmd + 1) {
        ESP_LOGW("PN532", "cmd=0x%02X phase=FRAME_DATA sum=0x%02X TFI=0x%02X reply=0x%02X",
                 cmd, sum, p[5], p[6]);
        ESP_LOG_BUFFER_HEX_LEVEL("PN532", response, data_length + 8, ESP_LOG_WARN);
        nfc_abort(); return -1;
    }
    size_t payload_length = data_length - 2;
    if (payload_length > capacity) {
        ESP_LOGW("PN532", "cmd=0x%02X response too large: %u", cmd, (unsigned)payload_length);
        return -1;
    }
    memcpy(reply, p + 7, payload_length);
    return (int)payload_length;
}

bool pn532_init(void)
{
    nfc_ready = false;
    if (!nfc_device) {
        i2c_master_bus_config_t bus = {
            .i2c_port=I2C_NUM_0, .sda_io_num=PN532_SDA_GPIO, .scl_io_num=PN532_SCL_GPIO,
            .clk_source=I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt=7,
            .flags.enable_internal_pullup=true,
        };
        esp_err_t rc = i2c_new_master_bus(&bus, &nfc_bus);
        if (rc != ESP_OK) return false;
        i2c_device_config_t device = {.dev_addr_length=I2C_ADDR_BIT_LEN_7, .device_address=0x24,
                                    .scl_speed_hz=100000, .scl_wait_us=10000};
        rc = i2c_master_bus_add_device(nfc_bus, &device, &nfc_device);
        if (rc != ESP_OK) return false;
    }
    delay_ms(100);
    nfc_abort();
    uint8_t reply[64];
    const uint8_t version[] = {0x02};
    int n = nfc_command(version, sizeof(version), reply, sizeof(reply), 500);
    if (n < 4 || reply[0] != 0x32) {
        ESP_LOGE(TAG, "PN532 not responding. SDA=22 SCL=23, I2C mode, power/pullups?");
        return false;
    }
    ESP_LOGI(TAG, "PN532 firmware %u.%u", reply[1], reply[2]);
    const uint8_t sam[] = {0x14, 0x01, 0x14, 0x00}; // Normal mode, IRQ unused.
    if (nfc_command(sam, sizeof(sam), reply, sizeof(reply), 500) < 0) return false;
    const uint8_t retries[] = {0x32, 0x05, 0xff, 0x01, 0x00}; // Passive target: one attempt.
    if (nfc_command(retries, sizeof(retries), reply, sizeof(reply), 500) < 0) return false;
    nfc_ready = true;
    ESP_LOGI("PN532", "Ready: SAM normal, finite passive retries, polling v2");
    return true;
}

int pn532_read_uid(char uid[21], idle_callback_t idle)
{
    if (!nfc_ready) return -1;
    uid[0] = 0;
    if (idle) idle();

    // Let the activation command manage RF. Do not force RF off before every scan.
    // Initialization sets finite passive retries; a valid zero-target response is
    // NO CARD, whereas transport/ACK/frame failures remain errors.
    const uint8_t detect[] = {0x4a, 0x01, 0x00};
    uint8_t reply[64] = {0};
    int n = nfc_command(detect, sizeof(detect), reply, sizeof(reply), 1000);
    if (idle) idle();
    if (n < 0) return -1;
    if (n < 1) {
        ESP_LOGW("PN532", "InListPassiveTarget: empty payload");
        return -1;
    }
    if (reply[0] == 0) return 0;
    if (reply[0] != 1 || n < 6) {
        ESP_LOGW("PN532", "InListPassiveTarget: targets=%u bytes=%d", reply[0], n);
        ESP_LOG_BUFFER_HEX_LEVEL("PN532", reply, n, ESP_LOG_WARN);
        return -1;
    }
    unsigned length = reply[5];
    if ((length != 4 && length != 7 && length != 10) || n < 6 + (int)length) {
        ESP_LOGW("PN532", "UID length invalid: length=%u payload=%d", length, n);
        ESP_LOG_BUFFER_HEX_LEVEL("PN532", reply, n, ESP_LOG_WARN);
        return -1;
    }
    for (unsigned i = 0; i < length; ++i) snprintf(uid + 2*i, 3, "%02X", reply[6+i]);
    uid[2*length] = 0;
    static char last_logged_uid[21];
    if (strcmp(uid, last_logged_uid) != 0) {
        strcpy(last_logged_uid, uid);
        ESP_LOGI("PN532", "Card UID=%s", uid);
    }
    return 1;
}
