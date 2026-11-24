#pragma once

enum {
    DEBUG_CAPTURE = 5,
    DEBUG_GET = 6,
    DEBUG_READY = 32,
    DEBUG_META = 33,
    DEBUG_DATA = 34,
    DEBUG_ERROR = 35
};

static bool diagnostic_get(uint32_t id,
                           uint32_t file,
                           uint32_t offset)
{
    memcpy(tx->data, &file, 4);
    memcpy(tx->data + 4, &offset, 4);
    seal(tx, id, DEBUG_GET, 8);

    int64_t deadline = esp_timer_get_time() + 15000000;

    while (esp_timer_get_time() < deadline) {
        if (!gpio_get_level(READY)) {
            sleep_ms(10);
            continue;
        }

        if (!exchange() || rx->session != id) continue;
        if (rx->kind == DEBUG_ERROR) return false;
        if (rx->size < 8) continue;

        uint32_t got_file;
        memcpy(&got_file, rx->data, 4);

        if (got_file != file) continue;

        if (offset == UINT32_MAX &&
            rx->kind == DEBUG_META) {
            return true;
        }

        if (offset != UINT32_MAX &&
            rx->kind == DEBUG_DATA &&
            rx->size > 12) {
            uint32_t got_offset;
            memcpy(&got_offset, rx->data + 8, 4);

            if (got_offset == offset) return true;
        }
    }

    return false;
}

static void diagnostic_session(void)
{
    uint32_t id;

    do {
        id = esp_random();
    } while (!id);

    esp_lcd_panel_disp_on_off(lcd, true);
    gpio_set_level(TFT_BL_GPIO, TFT_BL_ON);

    message("KET NOI SERVER", 0xffe0);

    if (!dump_begin_session(id)) {
        message("LOI SERVER WIFI", 0xf800);
        return;
    }

    message("GIU YEN CHUP ANH", 0xffe0);
    sleep_ms(1500);

    seal(tx, id, DEBUG_CAPTURE, 0);

    uint32_t files = 0;
    uint32_t partial = 0;

    bool ready = false;
    bool ok = true;

    int64_t deadline = esp_timer_get_time() + 60000000;
    int64_t wake_again = 0;

    while (esp_timer_get_time() < deadline) {
        int64_t now = esp_timer_get_time();

        if (!gpio_get_level(READY)) {
            if (now >= wake_again) {
                gpio_set_level(CS, 0);
                sleep_ms(45);
                gpio_set_level(CS, 1);

                wake_again = now + 1500000;
            }

            sleep_ms(20);
            continue;
        }

        if (!exchange() || rx->session != id) continue;

        if (rx->kind == DEBUG_READY && rx->size == 8) {
            memcpy(&files, rx->data, 4);
            memcpy(&partial, rx->data + 4, 4);

            ready = files > 0 && files <= 32;
            break;
        }
    }

    ok = ready;

    if (ready) {
        message("GUI ANH QUA WIFI", 0xffe0);
    }

    bool lcd_saved = false;

    for (uint32_t file = 0;
         ok && dump_wifi_ok && file < files;
         ++file) {

        if (!diagnostic_get(id, file, UINT32_MAX) ||
            rx->size <= 8 ||
            rx->size > 104 ||
            rx->data[rx->size - 1] != 0) {
            ok = false;
            break;
        }

        uint32_t total;
        memcpy(&total, rx->data + 4, 4);

        if (!total || total > 2 * 1024 * 1024) {
            ok = false;
            break;
        }

        char name[96];
        memcpy(name, rx->data + 8, rx->size - 8);
        name[sizeof(name) - 1] = 0;

        dump_file_begin(file, name, total);

        bool is_preview =
            strcmp(name, "01_qvga_original.jpg") == 0;

        uint8_t *preview = NULL;

        if (is_preview && total <= 96 * 1024) {
            preview = heap_caps_malloc(
                total, MALLOC_CAP_8BIT
            );
        }

        uint32_t hash = 2166136261u;
        uint32_t offset = 0;

        while (ok && dump_wifi_ok && offset < total) {
            if (!diagnostic_get(id, file, offset)) {
                ok = false;
                break;
            }

            uint32_t advertised;
            memcpy(&advertised, rx->data + 4, 4);

            uint32_t count = rx->size - 12;

            if (advertised != total ||
                count > total - offset) {
                ok = false;
                break;
            }

            hash = dump_hash(
                hash, rx->data + 12, count
            );

            if (preview) {
                memcpy(
                    preview + offset,
                    rx->data + 12,
                    count
                );
            }

            dump_data(
                file, offset, rx->data + 12, count
            );

            offset += count;
            sleep_ms(1);
        }

        ok = ok && dump_wifi_ok;

        if (ok) {
            dump_file_end(file, hash);
            ok = dump_wifi_ok;
        }

        if (ok && preview) {
            label = "GIU YEN";
            color = 0xffe0;

            dump_file_begin(
                1000,
                "04_lcd_framebuffer_240x320.rgb565",
                PANEL_W * PANEL_H * 2
            );

            dump_lcd_hash = 2166136261u;
            dump_lcd = true;

            bool shown = display_jpeg(preview, total);

            dump_lcd = false;

            if (shown && dump_wifi_ok) {
                dump_file_end(1000, dump_lcd_hash);
                lcd_saved = dump_wifi_ok;
            } else {
                partial = 1;
            }

            message("GUI ANH QUA WIFI", 0xffe0);
        }

        heap_caps_free(preview);

        if (is_preview && !lcd_saved) {
            partial = 1;
        }

        ok = ok && dump_wifi_ok;
    }

    dump_lcd = false;

    // Giải phóng dữ liệu chẩn đoán trên S3.
    seal(tx, id, END_SESSION, 0);

    deadline = esp_timer_get_time() + 3000000;

    while (!gpio_get_level(READY) &&
           esp_timer_get_time() < deadline) {
        sleep_ms(10);
    }

    if (gpio_get_level(READY)) {
        exchange();
    }

    const char *status =
        !ok ? "ERROR" : partial ? "PARTIAL" : "OK";

    if (!dump_finish(status)) {
        ok = false;
    }

    if (dump_wifi_partial) {
        partial = 1;
    }

    message(
        !ok ? "LOI GUI ANH WIFI" :
        partial ? "GUI ANH CHUA DU" :
        "DA LUU ANH TREN PC",
        ok ? 0x07e0 : 0xf800
    );

    ESP_LOGI(
        "CAPTURE",
        "Session=%08lx result=%s",
        (unsigned long)id,
        !ok ? "ERROR" : partial ? "PARTIAL" : "OK"
    );
}