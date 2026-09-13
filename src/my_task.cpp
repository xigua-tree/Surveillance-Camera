#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <ft2build.h>
#include FT_FREETYPE_H

#include "my_task.h"
#include "capture.h"
#include "lcd.h"
#include "fps_counter.h"
#include "inference.h"
#include "encoder.h"

static const char* FONT_PATH = "/usr/share/fonts/DejaVuSansMono.ttf";
static const int   FONT_SIZE = 24;

static FT_Library g_ft = nullptr;
static FT_Face    g_face = nullptr;
static int        g_asc_px = 0;
static int        g_line_h_px = 0;

static int osd_init_font(const char* path, int px) {
    if (FT_Init_FreeType(&g_ft)) {
        printf("[OSD] FT_Init_FreeType failed\n");
        return -1;
    }
    if (FT_New_Face(g_ft, path, 0, &g_face)) {
        printf("[OSD] FT_New_Face failed: %s\n", path);
        FT_Done_FreeType(g_ft);
        g_ft = nullptr;
        return -1;
    }
    if (FT_Set_Pixel_Sizes(g_face, 0, px)) {
        printf("[OSD] FT_Set_Pixel_Sizes failed\n");
        FT_Done_Face(g_face);
        FT_Done_FreeType(g_ft);
        g_face = nullptr;
        g_ft = nullptr;
        return -1;
    }
    g_asc_px = g_face->size->metrics.ascender >> 6;
    g_line_h_px = (g_face->size->metrics.ascender - g_face->size->metrics.descender) >> 6;
    return 0;
}

static void osd_destroy() {
    if (g_face) { FT_Done_Face(g_face); g_face = nullptr; }
    if (g_ft)   { FT_Done_FreeType(g_ft); g_ft = nullptr; }
}

static int osd_text_width(const char* text) {
    int w = 0;
    for (const char* p = text; *p; p++) {
        if (FT_Load_Char(g_face, (FT_ULong)(unsigned char)*p, FT_LOAD_ADVANCE_ONLY))
            continue;
        w += g_face->glyph->advance.x >> 6;
    }
    return w;
}

//半透明黑底
static void osd_fill_rect(uint8_t* nv12, int w, int h, int stride,
                          int x, int y, int rw, int rh) {
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = (x + rw) > w ? w : (x + rw);
    int y1 = (y + rh) > h ? h : (y + rh);
    for (int py = y0; py < y1; py++) {
        uint8_t* yrow = nv12 + py * stride;
        for (int px = x0; px < x1; px++)
            yrow[px] = (uint8_t)((yrow[px] * 127 + 16 * 128) / 255);
    }
}

//把单个 FreeType 字形写进 NV12 的 Y 平面
static void osd_draw_glyph(uint8_t* nv12, int w, int h, int stride,
                           const FT_Bitmap* bmp, int x, int y) {
    uint8_t* uv = nv12 + stride * h;
    for (int row = 0; row < (int)bmp->rows; row++) {
        int py = y + row;
        if (py < 0 || py >= h) continue;
        uint8_t* yrow = nv12 + py * stride;
        uint8_t* urow = uv + (py / 2) * stride;
        for (int col = 0; col < (int)bmp->width; col++) {
            int px = x + col;
            if (px < 0 || px >= w) continue;
            uint8_t a = bmp->buffer[row * bmp->pitch + col];
            if (!a) continue;
            yrow[px] = (uint8_t)((yrow[px] * (255 - a) + 235 * a) / 255);
            int ux = (px / 2) * 2;
            urow[ux] = 128;
            urow[ux + 1] = 128;
        }
    }
}

static void osd_draw_text(uint8_t* nv12, int w, int h, int stride,
                          int x, int y, const char* text) {
    if (!g_face || !text) return;
    const int pad = 4;
    int tw = osd_text_width(text);
    int baseline = y + g_asc_px;
    osd_fill_rect(nv12, w, h, stride, x - pad, y - pad, tw + pad * 2, g_line_h_px + pad * 2);

    int pen_x = x;
    for (const char* p = text; *p; p++) {
        if (FT_Load_Char(g_face, (FT_ULong)(unsigned char)*p, FT_LOAD_RENDER))
            continue;
        FT_GlyphSlot slot = g_face->glyph;
        osd_draw_glyph(nv12, w, h, stride, &slot->bitmap,
                       pen_x + slot->bitmap_left, baseline - slot->bitmap_top);
        pen_x += slot->advance.x >> 6;
    }
}

static void osd_draw(uint8_t* nv12, int w, int h, int stride,
                     const std::vector<Detection>& dets, const char* text) {
    if (!dets.empty())
        RKNNInference::draw_boxes_nv12(nv12, w, h, stride, dets);
    osd_draw_text(nv12, w, h, stride, 8, 8, text);
}

My_Task::My_Task(const Task_config& cfg)
{
    running_.store(false);
    task_cfg_ = cfg;
}

My_Task::~My_Task()
{
    stop();
}

int My_Task::start()
{
    running_.store(true);

    std::thread cap_th(&My_Task::capture_thread_func, this);
    std::thread inf_th(&My_Task::inference_thread_func, this);
    std::thread enc_th(&My_Task::encoder_thread_func, this);
    cap_th.join();
    inf_th.join();
    enc_th.join();
    return 0;
}

int My_Task::stop()
{
    running_.store(false);
    raw_queue_.set_done();
    enc_queue_.set_done();
    return 0;
}

void My_Task::capture_thread_func()
{
    printf("Task | Capture thread started\n");

    Capture cap;
    if (cap.init(task_cfg_.device_path, task_cfg_.capture_width,
                 task_cfg_.capture_height, task_cfg_.capture_buf_count) < 0) {
        printf("Task | Capture init failed\n");
        return;
    }

    LcdDisplay lcd;
    if (lcd.init() < 0) {
        printf("Task | LCD init failed\n");
        return;
    }

    if (osd_init_font(FONT_PATH, FONT_SIZE) < 0)
        printf("Task | font load failed, no text overlay\n");

    FpsCounter cap_fps;//采集帧率
    FpsCounter disp_fps;//显示帧率

    size_t frame_bytes = (size_t)cap.get_stride() * cap.get_height() * 3 / 2;
    uint64_t pts_us = 0;
    const uint64_t frame_interval_us = 33333;
    int frame_idx = 0;

    while (running_.load()) {
        //先归还编码线程已用完的 V4L2 buffer
        int done_idx;
        while (recycle_pop(done_idx))
            cap.enqueue(done_idx);

        int idx, bytesused;
        if (cap.dequeue(idx, bytesused) < 0) {
            if (running_.load()) printf("[Task] dequeue error\n");
            break;
        }

        uint8_t* src = (uint8_t*)cap.get_buf_addr(idx);
        int fd = cap.get_buf_fd(idx);
        int w = cap.get_width();
        int h = cap.get_height();
        int stride = cap.get_stride();

        //给推理单独拷贝一份
        uint8_t* inf_copy = (uint8_t*)malloc(frame_bytes);
        memcpy(inf_copy, src, frame_bytes);

        //画 OSD：检测框 + 帧率文字，画到源 NV12
        std::vector<Detection> dets;
        {
            std::lock_guard<std::mutex> lock(det_mtx_);
            dets = latest_dets_;
        }
        char osd_text[64];
        snprintf(osd_text, sizeof(osd_text), "CAP:%.1f DISP:%.1f ENC:%.1f",
                 cap_fps.fps(), disp_fps.fps(), enc_fps_.load());
        osd_draw(src, w, h, stride, dets, osd_text);

        // LCD 显示
        if (lcd.show(src, w, h, stride) == 0)
            disp_fps.tick();

        //推入推理队列
        FrameData inf_fd;
        inf_fd.index = frame_idx;
        inf_fd.v4l2_buf_index = idx;
        inf_fd.nv12_fd = fd;
        inf_fd.nv12_virt = inf_copy;
        inf_fd.width = w;
        inf_fd.height = h;
        inf_fd.width_stride = stride;
        inf_fd.height_stride = h;
        inf_fd.data_size = frame_bytes;
        inf_fd.capture_ts_us = pts_us;
        raw_queue_.push(inf_fd);

        //推入编码队列
        FrameData enc_fd;
        enc_fd.index = frame_idx;
        enc_fd.v4l2_buf_index = idx;
        enc_fd.nv12_fd = fd;
        enc_fd.nv12_virt = nullptr;
        enc_fd.width = w;
        enc_fd.height = h;
        enc_fd.width_stride = stride;
        enc_fd.height_stride = h;
        enc_fd.data_size = frame_bytes;
        enc_fd.capture_ts_us = pts_us;
        enc_queue_.push(enc_fd);

        cap_fps.tick();
        pts_us += frame_interval_us;
        frame_idx++;
    }

    cap.stop();
    osd_destroy();
    printf("Task | Capture thread exited\n");
}

void My_Task::inference_thread_func()
{
    printf("Task | Inference thread started\n");

    RKNNInference infer;
    RKNNInference::Config inf_cfg;
    inf_cfg.model_path = task_cfg_.model_path;
    inf_cfg.conf_thresh = task_cfg_.conf_threshold;
    inf_cfg.nms_thresh = task_cfg_.nms_threshold;

    if (task_cfg_.enable_inference) {
        if (infer.init(inf_cfg) < 0) {
            printf("Task | RKNN init failed, inference disabled\n");
            task_cfg_.enable_inference = false;
        }
    }

    int rgb_size = infer.get_rgb_buf_size();
    uint8_t* rgb_buf = (uint8_t*)malloc(rgb_size);
    if (!rgb_buf) {
        printf("Task | malloc rgb buf failed\n");
        return;
    }

    while (running_.load()) {
        FrameData fd = raw_queue_.pop();
        if (fd.index < 0) break;

        if (task_cfg_.enable_inference && (fd.index % task_cfg_.infer_interval == 0)) {
            std::vector<Detection> dets;
            int ret = infer.infer(fd.nv12_virt, fd.width, fd.height,
                                  fd.width_stride, fd.height_stride,
                                  rgb_buf, dets);
            if (ret == 0) {
                std::lock_guard<std::mutex> lock(det_mtx_);
                latest_dets_ = std::move(dets);
            }
        }

        free(fd.nv12_virt);
    }

    free(rgb_buf);
    printf("Task | Inference thread exited\n");
}

void My_Task::encoder_thread_func()
{
    printf("Task | Encoder thread started\n");

    EncoderStream enc;
    EncoderStream::Config enc_cfg;
    enc_cfg.width         = task_cfg_.encode_width;
    enc_cfg.height        = task_cfg_.encode_height;
    enc_cfg.bitrate_kbps  = task_cfg_.encode_bitrate;
    enc_cfg.fps_num       = 30;
    enc_cfg.gop           = 30;
    enc_cfg.rotation      = task_cfg_.rotation;
    enc_cfg.http_port     = task_cfg_.http_port;

    if (enc.init(enc_cfg) < 0) {
        printf("Task | Encoder init failed\n");
        return;
    }

    FpsCounter enc_fps;//编码帧率

    while (running_.load()) {
        FrameData fd = enc_queue_.pop();
        if (fd.index < 0) break;

        uint64_t pts_ms = fd.capture_ts_us / 1000;

        if (enc.feed_frame(fd.nv12_fd, fd.width, fd.height,
                           fd.width_stride, fd.height_stride, pts_ms) == 0) {
            enc_fps.tick();
            enc_fps_.store(enc_fps.fps());
        }

        //用完归还 V4L2 buffer，采集线程才能重新 enqueue
        recycle_buffer(fd.v4l2_buf_index);
    }

    printf("Task | Encoder thread exited\n");
}
