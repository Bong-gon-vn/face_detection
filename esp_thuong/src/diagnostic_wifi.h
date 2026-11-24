#pragma once

// main.c phải include my_library.h trước file này.
static uint32_t dump_session;
static bool dump_lcd;
static uint32_t dump_lcd_hash;

static bool dump_wifi_ok = true;
static bool dump_wifi_partial;
static char dump_session_text[9];
static char dump_response[512];

static uint32_t dump_hash(uint32_t hash,
                          const uint8_t *data,
                          size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        hash ^= data[i];
        hash *= 16777619u;
    }

    return hash;
}

// Đảo riêng pixel ảnh trước khi vẽ chữ/khung.
// Không thay đổi ảnh gửi server hoặc dữ liệu đưa vào model.
static void preview_unmirror(uint8_t *pixels, int width, int rows)
{
    for (int y = 0; y < rows; ++y) {
        uint8_t *row = pixels + y * width * 2;

        for (int x = 0; x < width / 2; ++x) {
            int left = x * 2;
            int right = (width - 1 - x) * 2;

            uint8_t first = row[left];
            uint8_t second = row[left + 1];

            row[left] = row[right];
            row[left + 1] = row[right + 1];

            row[right] = first;
            row[right + 1] = second;
        }
    }
}

static bool dump_json(const char *path, cJSON *object)
{
    if (!object) {
        dump_wifi_ok = false;
        return false;
    }

    char *body = cJSON_PrintUnformatted(object);
    cJSON_Delete(object);

    if (!body) {
        dump_wifi_ok = false;
        return false;
    }

    int http_status = 0;

    bool success = server_request(
        path,
        body,
        strlen(body),
        "application/json",
        dump_response,
        sizeof(dump_response),
        &http_status,
        NULL
    );

    cJSON_free(body);

    if (!success) {
        dump_wifi_ok = false;
        ESP_LOGE("CAPTURE", "HTTP failed: %d", http_status);
    }

    return success;
}

static cJSON *dump_object(void)
{
    cJSON *object = cJSON_CreateObject();

    if (object) {
        cJSON_AddStringToObject(
            object, "session", dump_session_text
        );
    }

    return object;
}

static bool dump_begin_session(uint32_t session)
{
    dump_session = session;
    dump_wifi_ok = true;
    dump_wifi_partial = false;
    dump_lcd = false;

    snprintf(
        dump_session_text,
        sizeof(dump_session_text),
        "%08lx",
        (unsigned long)session
    );

    return dump_json("/api/capture/start", dump_object());
}

static void dump_file_begin(unsigned id,
                            const char *name,
                            unsigned size)
{
    if (!dump_wifi_ok) return;

    cJSON *object = dump_object();

    if (object) {
        cJSON_AddNumberToObject(object, "file_id", id);
        cJSON_AddStringToObject(object, "name", name);
        cJSON_AddNumberToObject(object, "size", size);
    }

    if (dump_json("/api/capture/file", object)) {
        ESP_LOGI(
            "CAPTURE",
            "Sending %s: %u bytes",
            name,
            size
        );
    }
}

static void dump_data(unsigned id,
                      unsigned offset,
                      const uint8_t *data,
                      size_t length)
{
    while (length && dump_wifi_ok) {
        size_t count = length > 4096 ? 4096 : length;

        char path[160];
        snprintf(
            path,
            sizeof(path),
            "/api/capture/data?session=%s&file_id=%u&offset=%u",
            dump_session_text,
            id,
            offset
        );

        int http_status = 0;

        bool success = server_request(
            path,
            data,
            count,
            "application/octet-stream",
            dump_response,
            sizeof(dump_response),
            &http_status,
            NULL
        );

        if (!success) {
            dump_wifi_ok = false;
            ESP_LOGE(
                "CAPTURE",
                "Upload failed: file=%u offset=%u HTTP=%d",
                id, offset, http_status
            );
            break;
        }

        data += count;
        length -= count;
        offset += count;
    }
}

static void dump_file_end(unsigned id, uint32_t hash)
{
    if (!dump_wifi_ok) return;

    cJSON *object = dump_object();

    if (object) {
        cJSON_AddNumberToObject(object, "file_id", id);
        cJSON_AddNumberToObject(object, "hash", hash);
    }

    dump_json("/api/capture/file_end", object);
}

static bool dump_finish(const char *status)
{
    cJSON *object = dump_object();

    if (object) {
        cJSON_AddStringToObject(object, "status", status);
    }

    if (!dump_json("/api/capture/finish", object)) {
        return false;
    }

    cJSON *reply = cJSON_Parse(dump_response);
    cJSON *result = cJSON_GetObjectItemCaseSensitive(
        reply, "status"
    );

    bool success = false;

    if (cJSON_IsString(result) && result->valuestring) {
        dump_wifi_partial =
            strcmp(result->valuestring, "PARTIAL") == 0;

        success =
            strcmp(result->valuestring, "OK") == 0 ||
            dump_wifi_partial;
    }

    cJSON_Delete(reply);
    return success;
}