#pragma once
// Included after the normal capture functions; all transfers are request-addressed
// so a lost SPI response can be retried without skipping file bytes.
#include <cstdio>
#include <string>
#include "dl_image_process.hpp"
#include "dl_math_matrix.hpp"

enum { DEBUG_CAPTURE=5, DEBUG_GET=6, DEBUG_READY=32,
       DEBUG_META=33, DEBUG_DATA=34, DEBUG_ERROR=35 };
struct diag_file_t { std::string name; uint8_t *data; uint32_t size; };
static std::vector<diag_file_t> diag_files;
static bool diag_active;
static std::string diag_error;

static uint8_t *diag_alloc(size_t size) {
    return static_cast<uint8_t *>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}
static void diag_clear() {
    for (auto &f : diag_files) heap_caps_free(f.data);
    diag_files.clear(); diag_error.clear(); diag_active=false;
}
static bool diag_add(const char *name, const void *data, size_t size) {
    uint8_t *copy=diag_alloc(size);
    if (!copy) { diag_error="Not enough PSRAM for diagnostics"; return false; }
    memcpy(copy,data,size);
    diag_files.push_back({name,copy,static_cast<uint32_t>(size)});
    return true;
}
static bool diag_ppm(const char *name, const uint8_t *data, int w, int h) {
    char header[48]; int n=snprintf(header,sizeof(header),"P6\n%d %d\n255\n",w,h);
    size_t size=n+static_cast<size_t>(w)*h*3;
    uint8_t *out=diag_alloc(size);
    if (!out) { diag_error="Not enough PSRAM for RGB image"; return false; }
    memcpy(out,header,n); memcpy(out+n,data,size-n);
    diag_files.push_back({name,out,static_cast<uint32_t>(size)});
    return true;
}
static void diag_rotate(uint8_t *data, int w, int h) {
    for (int i=0;i<w*h/2;++i) for (int c=0;c<3;++c)
        std::swap(data[3*i+c],data[3*(w*h-1-i)+c]);
}
static bool diag_landmarks(uint8_t *data, std::vector<int> &points, bool &rotated,
                           std::string &note) {
    dl::image::img_t image={data,W,H,dl::image::DL_IMAGE_PIX_TYPE_RGB888};
    rotated=false;
    for (int attempt=0;attempt<2;++attempt) {
        auto &faces=face_detector().run(image);
        if (faces.size()==1 && faces.front().keypoint.size()==10) {
            points=faces.front().keypoint;
            bool valid=true;
            for (int i=0;i<10;i+=2)
                valid &= points[i]>=0 && points[i]<W && points[i+1]>=0 && points[i+1]<H;
            // Reject degenerate alignment. ESP-DL order: left eye, left mouth,
            // nose, right eye, right mouth.
            int dx=points[0]-points[6],dy=points[1]-points[7];
            if (valid && dx*dx+dy*dy>=16) {
                rotated=attempt!=0;
                note="one_face";
                return true;
            }
        }
        if (faces.size()>1) { note="multiple_faces"; break; }
        note="no_valid_face";
        if (attempt==0) diag_rotate(data,W,H);
    }
    // Caller does not use this buffer again unless landmarks succeeded.
    points.clear(); return false;
}
static bool diag_aligned(const char *name, uint8_t *src, int w, int h,
                         const std::vector<int> &points) {
    // Exact target landmarks and inverse similarity transform used by ESP-DL
    // v3.3.15 FeatImagePreprocessor. Export visible RGB before normalization,
    // BGR channel swap and int8 quantization (not a dump of the model tensor).
    const float standard[10]={38.2946f,51.6963f,41.5493f,92.3655f,56.0252f,
                              71.7366f,73.5318f,51.5014f,70.7299f,92.2041f};
    dl::math::Matrix<float> source(5,2),dest(5,2);
    dest.set_value(points);
    for (int i=0;i<5;++i) for (int j=0;j<2;++j) source.array[i][j]=standard[i*2+j];
    auto inverse=dl::math::get_similarity_transform(source,dest);
    for (int i=0;i<inverse.h;++i) for (int j=0;j<inverse.w;++j)
        if (!std::isfinite(inverse.array[i][j])) { diag_error="Invalid alignment matrix"; return false; }
    uint8_t *out=diag_alloc(112*112*3);
    if (!out) { diag_error="Not enough PSRAM for aligned face"; return false; }
    dl::image::img_t input={src,static_cast<uint16_t>(w),static_cast<uint16_t>(h),dl::image::DL_IMAGE_PIX_TYPE_RGB888};
    dl::image::img_t output={out,112,112,dl::image::DL_IMAGE_PIX_TYPE_RGB888};
    dl::image::ImageTransformer transform;
    esp_err_t rc=transform.set_src_img(input).set_dst_img(output)
        .set_warp_affine_matrix(inverse,true).transform();
    bool ok=rc==ESP_OK && diag_ppm(name,out,112,112);
    heap_caps_free(out);
    if (rc!=ESP_OK) diag_error="ESP-DL face alignment failed";
    return ok;
}
static bool diag_warmup() {
    // Capture and discard real frames, rather than sleeping with a stale queued frame.
    for (int i=0;i<12;++i) {
        camera_fb_t *fb=esp_camera_fb_get();
        if (!fb) return false;
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    return true;
}
static std::string diag_points(const std::vector<int> &points) {
    std::string out="[";
    for (size_t i=0;i<points.size();++i) {
        if (i) out+=",";
        out+=std::to_string(points[i]);
    }
    return out+"]";
}
static bool diag_capture_mode(bool vga, std::string &metadata) {
    stop_camera();
    if (!open_camera(vga?FRAMESIZE_VGA:FRAMESIZE_QVGA)) {
        diag_error="Camera init failed"; return false;
    }
    if (!vga) {
        camera_fb_t *first=esp_camera_fb_get();
        if (!first) { diag_error="First camera frame failed"; return false; }
        bool saved=diag_add("00_qvga_first.jpg",first->buf,first->len);
        esp_camera_fb_return(first);
        if (!saved) return false;
    }
    if (!diag_warmup()) { diag_error="Camera warmup failed"; return false; }
    camera_fb_t *fb=esp_camera_fb_get();
    if (!fb) { diag_error="Camera capture failed"; return false; }
    const int w=vga?640:W,h=vga?480:H;
    if (fb->width!=w || fb->height!=h) {
        esp_camera_fb_return(fb); diag_error="Unexpected camera frame size"; return false;
    }
    int64_t captured=static_cast<int64_t>(fb->timestamp.tv_sec)*1000000+fb->timestamp.tv_usec;
    bool saved=diag_add(vga?"10_vga_original.jpg":"01_qvga_original.jpg",fb->buf,fb->len);
    dl::image::jpeg_img_t jpeg={fb->buf,fb->len};
    auto decoded=dl::image::sw_decode_jpeg(jpeg,dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    esp_camera_fb_return(fb);
    stop_camera();
    uint8_t *full=static_cast<uint8_t *>(decoded.data);
    if (!saved || !full || decoded.width!=w || decoded.height!=h) {
        heap_caps_free(full);
        if (diag_error.empty()) diag_error="ESP-DL JPEG decode failed";
        return false;
    }
    uint8_t *small=full;
    if (vga) {
        small=diag_alloc(N);
        if (!small) { heap_caps_free(full); diag_error="Not enough PSRAM for resize"; return false; }
        // 2x2 box average, RGB888, same VGA frame; no second JPEG encode.
        for (int y=0;y<H;++y) for (int x=0;x<W;++x) for (int c=0;c<3;++c) {
            int p=((y*2)*640+x*2)*3+c;
            small[(y*W+x)*3+c]=(full[p]+full[p+3]+full[p+640*3]+full[p+640*3+3]+2)/4;
        }
    }
    saved=diag_ppm(vga?"11_vga_downscaled_320x240.ppm":"02_qvga_esp_rgb_320x240.ppm",small,W,H);
    std::vector<int> points;
    bool rotated=false;
    std::string note;
    bool found=saved && diag_landmarks(small,points,rotated,note);
    if (found) {
        saved=diag_aligned(vga?"12_face_from_vga_downscaled_112x112.ppm":"03_face_from_qvga_112x112.ppm",
                           small,W,H,points);
        if (vga && saved) {
            if (rotated) diag_rotate(full,w,h);
            // Pixel centers of the 2x2 average are (2*x+0.5,2*y+0.5).
            // ESP-DL landmark API is integer: use nearest integer (2*x+1).
            auto high_points=points;
            for (int &p:high_points) p=p*2+1;
            saved=diag_aligned("13_face_from_vga_112x112.ppm",full,w,h,high_points);
        }
    }
    char info[320];
    snprintf(info,sizeof(info),"\"%s\":{\"capture_us\":%lld,\"width\":%d,\"height\":%d,"
        "\"face_status\":\"%s\",\"ai_rotation_degrees\":%d,\"landmarks_320\":",
        vga?"vga":"qvga",static_cast<long long>(captured),w,h,note.c_str(),rotated?180:0);
    if (!metadata.empty()) metadata+=",";
    metadata+=info; metadata+=diag_points(points); metadata+="}";
    if (vga) heap_caps_free(small);
    heap_caps_free(full);
    return saved;
}
static void diag_capture() {
    std::string metadata;
    // Both modes are captured before USB transfer. QVGA and VGA are sequential
    // captures; the two VGA-derived 112 images share exactly the same frame.
    if (diag_capture_mode(false,metadata)) diag_capture_mode(true,metadata);
    stop_camera();
    char sensor_metadata[256];
    snprintf(sensor_metadata, sizeof(sensor_metadata),
             "{\"schema\":2,\"sensor_expected\":\"%s\",\"sensor_detected\":\"%s\","
             "\"sensor_pid\":%u,\"xclk_hz\":%d,\"jpeg_quality\":%d,",
             CAMERA_EXPECTED_NAME, detected_sensor_name, detected_sensor_pid,
             CAMERA_XCLK_HZ, CAMERA_JPEG_QUALITY);
    std::string json = sensor_metadata;
    json += "\"warmup_discard_frames\":12,\"alignment\":\"ESP-DL v3.3.15 RGB before normalize/quantize\","
            "\"vga_resize\":\"2x2 box average\",\"diagnostic_only\":true";
    if (!metadata.empty()) json += "," + metadata;
    json += ",\"error\":\"" + diag_error + "\"}";
    diag_add("capture.json",json.data(),json.size());
}
static void diag_ready() {
    uint32_t count=diag_files.size();
    memcpy(tx->data,&count,4);
    uint32_t failed=!diag_error.empty();
    memcpy(tx->data+4,&failed,4);
    seal(tx,sid,DEBUG_READY,8);
}
static void diag_reply() {
    if (rx->size!=8) { seal(tx,sid,DEBUG_ERROR,0); return; }
    uint32_t file,offset;
    memcpy(&file,rx->data,4); memcpy(&offset,rx->data+4,4);
    if (file>=diag_files.size()) {
        const char *why=diag_error.empty()?"file_out_of_range":diag_error.c_str();
        size_t n=std::min(strlen(why),static_cast<size_t>(CAP));
        memcpy(tx->data,why,n); seal(tx,sid,DEBUG_ERROR,n); return;
    }
    auto &f=diag_files[file];
    memcpy(tx->data,&file,4); memcpy(tx->data+4,&f.size,4);
    if (offset==UINT32_MAX) {
        memcpy(tx->data+8,f.name.c_str(),f.name.size()+1);
        seal(tx,sid,DEBUG_META,8+f.name.size()+1);
    } else if (offset<f.size) {
        memcpy(tx->data+8,&offset,4);
        size_t n=std::min(static_cast<size_t>(f.size-offset),static_cast<size_t>(CAP-12));
        memcpy(tx->data+12,f.data+offset,n); seal(tx,sid,DEBUG_DATA,12+n);
    } else seal(tx,sid,DEBUG_ERROR,0);
}
