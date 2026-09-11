#include "inference.h"
#include "im2d.h"
#include "rga.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

RKNNInference::RKNNInference()
    : ctx_(0), net_w_(640), net_h_(640),
      num_classes_(0), out_proposals_(0),
      conf_thresh_(0.7f), nms_thresh_(0.45f) {
    memset(&io_num_, 0, sizeof(io_num_));
}

RKNNInference::~RKNNInference() {
    if (ctx_) {
        rknn_destroy(ctx_);
        ctx_ = 0;
    }
}

static unsigned char* load_model_file(const char* filename, int* model_size) {
    FILE* fp = fopen(filename, "rb");
    if (!fp) {
        printf("[Inference] Cannot open model: %s\n", filename);
        return nullptr;
    }
    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    unsigned char* data = (unsigned char*)malloc(size);
    if (!data) { fclose(fp); return nullptr; }
    fread(data, 1, size, fp);
    fclose(fp);
    *model_size = size;
    return data;
}

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

static float calc_iou(int ax1, int ay1, int ax2, int ay2,
                      int bx1, int by1, int bx2, int by2) {
    int ix1 = std::max(ax1, bx1);
    int iy1 = std::max(ay1, by1);
    int ix2 = std::min(ax2, bx2);
    int iy2 = std::min(ay2, by2);
    int iw = std::max(0, ix2 - ix1);
    int ih = std::max(0, iy2 - iy1);
    float inter = (float)(iw * ih);
    float area_a = (float)((ax2 - ax1) * (ay2 - ay1));
    float area_b = (float)((bx2 - bx1) * (by2 - by1));
    return inter / (area_a + area_b - inter + 1e-6f);
}

static void apply_nms(std::vector<Detection>& dets, float thresh) {
    std::sort(dets.begin(), dets.end(),
              [](const Detection& a, const Detection& b) { return a.confidence > b.confidence; });
    std::vector<bool> keep(dets.size(), true);
    for (size_t i = 0; i < dets.size(); i++) {
        if (!keep[i]) continue;
        for (size_t j = i + 1; j < dets.size(); j++) {
            if (!keep[j]) continue;
            float iou = calc_iou(dets[i].x, dets[i].y,
                                 dets[i].x + dets[i].w, dets[i].y + dets[i].h,
                                 dets[j].x, dets[j].y,
                                 dets[j].x + dets[j].w, dets[j].y + dets[j].h);
            if (iou > thresh) keep[j] = false;
        }
    }
    std::vector<Detection> result;
    for (size_t i = 0; i < dets.size(); i++)
        if (keep[i]) result.push_back(dets[i]);
    dets = std::move(result);
}

int RKNNInference::init(const Config& cfg) {
    conf_thresh_ = cfg.conf_thresh;
    nms_thresh_  = cfg.nms_thresh;

    // 加载模型
    int model_size = 0;
    unsigned char* model_data = load_model_file(cfg.model_path.c_str(), &model_size);
    if (!model_data) return -1;

    int ret = rknn_init(&ctx_, model_data, model_size, 0, nullptr);
    free(model_data);
    if (ret < 0) {
        printf("[Inference] rknn_init failed ret=%d\n", ret);
        return -1;
    }

    // 查询 I/O
    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret < 0) {
        printf("[Inference] rknn_query IN_OUT_NUM failed\n");
        return -1;
    }

    input_attrs_.resize(io_num_.n_input);
    for (uint32_t i = 0; i < io_num_.n_input; i++) {
        memset(&input_attrs_[i], 0, sizeof(rknn_tensor_attr));
        input_attrs_[i].index = i;
        rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attrs_[i], sizeof(rknn_tensor_attr));
    }

    output_attrs_.resize(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; i++) {
        memset(&output_attrs_[i], 0, sizeof(rknn_tensor_attr));
        output_attrs_[i].index = i;
        rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(rknn_tensor_attr));
    }

    // 提取模型输入尺寸
    net_h_ = input_attrs_[0].dims[1];
    net_w_ = input_attrs_[0].dims[2];

    // 提取输出维度: YOLOv8 输出 [1, 4+nc, np]
    int out_dims = output_attrs_[0].n_dims;
    int out_features = output_attrs_[0].dims[out_dims - 2];
    out_proposals_   = output_attrs_[0].dims[out_dims - 1];
    num_classes_     = out_features - 4;

    printf("[Inference] Model: %dx%d, classes=%d, proposals=%d\n",
           net_w_, net_h_, num_classes_, out_proposals_);

    if (num_classes_ <= 0 || num_classes_ > 100) {
        printf("[Inference] Unexpected num_classes=%d\n", num_classes_);
        return -1;
    }
    return 0;
}

// ---- 纯 CPU NV12 resize + CSC → RGB ----
// 彻底绕开 RGA (虚拟地址 + DMA 有缓存一致性问题，会导致模型输入乱码)
static inline uint8_t clamp255(int v) {
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static void nv12_resize_to_rgb_cpu(const uint8_t* nv12, int src_w, int src_h, int src_stride,
                                   uint8_t* rgb, int dst_w, int dst_h) {
    const uint8_t* uv = nv12 + src_stride * src_h;
    for (int dy = 0; dy < dst_h; dy++) {
        int sy = dy * src_h / dst_h;
        for (int dx = 0; dx < dst_w; dx++) {
            int sx = dx * src_w / dst_w;
            int y_val = nv12[sy * src_stride + sx];
            int uv_x = (sx / 2) * 2;
            int uv_y = sy / 2;
            int u_val = uv[uv_y * src_stride + uv_x];
            int v_val = uv[uv_y * src_stride + uv_x + 1];
            int c = y_val - 16;
            int d = u_val - 128;
            int e = v_val - 128;
            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;
            rgb[(dy * dst_w + dx) * 3 + 0] = clamp255(r);
            rgb[(dy * dst_w + dx) * 3 + 1] = clamp255(g);
            rgb[(dy * dst_w + dx) * 3 + 2] = clamp255(b);
        }
    }
}

int RKNNInference::infer(void* nv12_data, int src_w, int src_h,
                         int src_wstride, int src_hstride,
                         uint8_t* rgb_buf,
                         std::vector<Detection>& detections) {
    detections.clear();

    // 诊断: 打印 raw NV12 数据 (Y/UV 前 8 字节) + stride 信息
    {
        static bool nv12_checked = false;
        if (!nv12_checked) {
            nv12_checked = true;
            const uint8_t* nv12 = (const uint8_t*)nv12_data;
            const uint8_t* uv = nv12 + src_wstride * src_h;
            printf("[Inference] src=%dx%d stride=%d, raw Y[0..7]: ",
                   src_w, src_h, src_wstride);
            for (int k = 0; k < 8; k++) printf("%d ", nv12[k]);
            printf("| UV[0..7]: ");
            for (int k = 0; k < 8; k++) printf("%d ", uv[k]);
            printf("\n");
        }
    }

    // 步骤 1: CPU NV12 resize (最近邻) + CSC → RGB 640x640
    // 直接用 CPU 避免 RGA 虚拟地址的缓存一致性问题
    nv12_resize_to_rgb_cpu((const uint8_t*)nv12_data, src_w, src_h, src_wstride,
                           rgb_buf, net_w_, net_h_);

    // 诊断: 首帧检查 RGB 值 + 打印前 8 个像素
    {
        static bool rgb_checked = false;
        if (!rgb_checked) {
            rgb_checked = true;
            int rgb_size = net_w_ * net_h_ * 3;
            uint8_t r_min = 255, r_max = 0, g_min = 255, g_max = 0, b_min = 255, b_max = 0;
            long r_sum = 0, g_sum = 0, b_sum = 0;
            for (int k = 0; k < rgb_size; k += 3) {
                if (rgb_buf[k] < r_min) r_min = rgb_buf[k];
                if (rgb_buf[k] > r_max) r_max = rgb_buf[k];
                if (rgb_buf[k+1] < g_min) g_min = rgb_buf[k+1];
                if (rgb_buf[k+1] > g_max) g_max = rgb_buf[k+1];
                if (rgb_buf[k+2] < b_min) b_min = rgb_buf[k+2];
                if (rgb_buf[k+2] > b_max) b_max = rgb_buf[k+2];
                r_sum += rgb_buf[k]; g_sum += rgb_buf[k+1]; b_sum += rgb_buf[k+2];
            }
            int n = net_w_ * net_h_;
            printf("[Inference] RGB stats: R[%d-%d avg=%ld] G[%d-%d avg=%ld] B[%d-%d avg=%ld]\n",
                   r_min, r_max, r_sum/n, g_min, g_max, g_sum/n, b_min, b_max, b_sum/n);
            printf("[Inference] First 8 RGB pixels: ");
            for (int k = 0; k < 8 && k < rgb_size; k += 3) {
                printf("(%d,%d,%d) ", rgb_buf[k], rgb_buf[k+1], rgb_buf[k+2]);
            }
            printf("\n");
        }
    }

    // 步骤 2: 归一化 uint8 RGB [0,255] → float [0,1]，YOLOv8 要求 0~1 输入
    int rgb_pixels = net_w_ * net_h_ * 3;
    float* rgb_float = (float*)malloc(rgb_pixels * sizeof(float));
    if (!rgb_float) { printf("[Inference] malloc rgb_float failed\n"); return -1; }
    for (int k = 0; k < rgb_pixels; k++)
        rgb_float[k] = rgb_buf[k] / 255.0f;

    // 设置 RKNN 输入 (float [0,1], NHWC)
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type  = RKNN_TENSOR_FLOAT32;
    inputs[0].size  = rgb_pixels * sizeof(float);
    inputs[0].fmt   = RKNN_TENSOR_NHWC;
    inputs[0].buf   = rgb_float;

    int ret = rknn_inputs_set(ctx_, io_num_.n_input, inputs);
    free(rgb_float);
    if (ret < 0) {
        printf("[Inference] rknn_inputs_set failed ret=%d\n", ret);
        return -1;
    }

    // 步骤 3: NPU 推理
    ret = rknn_run(ctx_, nullptr);
    if (ret < 0) {
        printf("[Inference] rknn_run failed ret=%d\n", ret);
        return -1;
    }

    // 步骤 4: 获取输出
    rknn_output outputs[io_num_.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (uint32_t i = 0; i < io_num_.n_output; i++)
        outputs[i].want_float = 1;

    ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs, nullptr);
    if (ret < 0) {
        printf("[Inference] rknn_outputs_get failed ret=%d\n", ret);
        return -1;
    }

    float* out_data = (float*)outputs[0].buf;

    // 步骤 5: 坐标映射
    // 模型输出已经是解码后的像素坐标 (0~640 范围)，无需再 decode
    // 预处理是 RGA imresize 直接压缩 (1920x1080 → 640x640)
    // 映射: src_x = cx / net_w * src_w, src_y = cy / net_h * src_h

    for (int i = 0; i < out_proposals_; i++) {
        float cx_px = out_data[0 * out_proposals_ + i];  // 640 空间像素坐标
        float cy_px = out_data[1 * out_proposals_ + i];
        float w_px  = out_data[2 * out_proposals_ + i];
        float h_px  = out_data[3 * out_proposals_ + i];

        // 找最大置信度的类别
        float max_conf = 0.0f;
        int   max_cls  = -1;
        for (int c = 0; c < num_classes_; c++) {
            float score = out_data[(4 + c) * out_proposals_ + i];
            score = sigmoid(score);
            if (score > max_conf) { max_conf = score; max_cls = c; }
        }
        if (max_conf < conf_thresh_) continue;
        // 只检测 person (COCO class_id=0)
        if (max_cls != 0) continue;

        // 像素坐标 → 归一化 → 源图像素 (压缩映射)
        float x1 = (cx_px - w_px * 0.5f) / net_w_ * src_w;
        float y1 = (cy_px - h_px * 0.5f) / net_h_ * src_h;
        float x2 = (cx_px + w_px * 0.5f) / net_w_ * src_w;
        float y2 = (cy_px + h_px * 0.5f) / net_h_ * src_h;

        x1 = std::max(0.0f, std::min(x1, (float)src_w));
        y1 = std::max(0.0f, std::min(y1, (float)src_h));
        x2 = std::max(0.0f, std::min(x2, (float)src_w));
        y2 = std::max(0.0f, std::min(y2, (float)src_h));

        if (x2 <= x1 || y2 <= y1) continue;

        Detection det;
        det.x = (int)x1; det.y = (int)y1;
        det.w = (int)(x2 - x1); det.h = (int)(y2 - y1);
        det.class_id   = max_cls;
        det.confidence = max_conf;
        detections.push_back(det);
    }

    size_t detections_before_nms = detections.size();

    // NMS
    apply_nms(detections, nms_thresh_);

    // 释放输出
    rknn_outputs_release(ctx_, io_num_.n_output, outputs);

    static int infer_cnt = 0;
    infer_cnt++;
    if (infer_cnt % 30 == 0 || infer_cnt == 1) {
        printf("[Inference] #%d: %zu detections before NMS, %zu after\n",
               infer_cnt, detections_before_nms, detections.size());
    }
    // 首帧打印前 3 个 proposal 的像素坐标用于诊断
    if (infer_cnt == 1) {
        printf("[Inference] Sample proposals (pixel coords in 640-space, first 3):\n");
        for (int k = 0; k < 3 && k < out_proposals_; k++) {
            float rcx = out_data[0 * out_proposals_ + k];
            float rcy = out_data[1 * out_proposals_ + k];
            float rw  = out_data[2 * out_proposals_ + k];
            float rh  = out_data[3 * out_proposals_ + k];
            printf("  [%d] cx=%.1f cy=%.1f w=%.1f h=%.1f -> src(x=%.0f y=%.0f w=%.0f h=%.0f)\n",
                   k, rcx, rcy, rw, rh,
                   rcx / net_w_ * 1920, rcy / net_h_ * 1080,
                   rw / net_w_ * 1920, rh / net_h_ * 1080);
        }
    }
    return 0;
}

// ---- NV12 画框 ----
static void draw_rect_nv12(uint8_t* nv12, int width, int height, int stride,
                           int x, int y, int w, int h,
                           uint8_t y_val, uint8_t u_val, uint8_t v_val,
                           int thickness) {
    int x1 = std::max(0, x);
    int y1 = std::max(0, y);
    int x2 = std::min(width  - 1, x + w);
    int y2 = std::min(height - 1, y + h);

    uint8_t* y_plane  = nv12;
    uint8_t* uv_plane = nv12 + stride * height;

    for (int t = 0; t < thickness; t++) {
        int cy1 = std::max(0, y1 + t);
        int cy2 = std::min(height - 1, y2 - t);
        int cx1 = std::max(0, x1 + t);
        int cx2 = std::min(width  - 1, x2 - t);

        // 上下水平线
        for (int px = cx1; px <= cx2; px++) {
            if (cy1 >= 0 && cy1 < height) y_plane[cy1 * stride + px] = y_val;
            if (cy2 >= 0 && cy2 < height) y_plane[cy2 * stride + px] = y_val;
        }
        // 左右垂直线
        for (int py = cy1; py <= cy2; py++) {
            if (cx1 >= 0 && cx1 < width) y_plane[py * stride + cx1] = y_val;
            if (cx2 >= 0 && cx2 < width) y_plane[py * stride + cx2] = y_val;
        }
    }

    // UV 平面: 每个 UV 采样对应 2x2 Y 块，2 像素对齐
    int uv_stride = stride;
    for (int t = 0; t < thickness; t++) {
        int uy1 = std::max(0, (y1 + t) / 2);
        int uy2 = std::min(height / 2 - 1, (y2 - t) / 2);
        int ux1 = std::max(0, (x1 + t) / 2);
        int ux2 = std::min(width / 2 - 1, (x2 - t) / 2);

        for (int px = ux1; px <= ux2; px++) {
            if (uy1 >= 0 && uy1 < height / 2) {
                uv_plane[uy1 * uv_stride + px * 2]     = u_val;
                uv_plane[uy1 * uv_stride + px * 2 + 1] = v_val;
            }
            if (uy2 >= 0 && uy2 < height / 2) {
                uv_plane[uy2 * uv_stride + px * 2]     = u_val;
                uv_plane[uy2 * uv_stride + px * 2 + 1] = v_val;
            }
        }
        for (int py = uy1; py <= uy2; py++) {
            if (ux1 >= 0 && ux1 < width / 2) {
                uv_plane[py * uv_stride + ux1 * 2]     = u_val;
                uv_plane[py * uv_stride + ux1 * 2 + 1] = v_val;
            }
            if (ux2 >= 0 && ux2 < width / 2) {
                uv_plane[py * uv_stride + ux2 * 2]     = u_val;
                uv_plane[py * uv_stride + ux2 * 2 + 1] = v_val;
            }
        }
    }
}

void RKNNInference::draw_boxes_nv12(uint8_t* nv12_data, int width, int height, int stride,
                                    const std::vector<Detection>& dets) {
    // 红色框: Y=76, U=90 (偏蓝绿), V=240 (偏红)
    const uint8_t RED_Y = 76;
    const uint8_t RED_U = 90;
    const uint8_t RED_V = 240;

    for (const auto& det : dets) {
        // 统一红色框, 线宽 2
        draw_rect_nv12(nv12_data, width, height, stride,
                       det.x, det.y, det.w, det.h,
                       RED_Y, RED_U, RED_V, 2);
    }
}
