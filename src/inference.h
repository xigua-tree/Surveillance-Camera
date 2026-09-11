#pragma once

#include <string>
#include <vector>
#include "rknn_api.h"
#include "my_task.h"

// YOLOv8 RKNN 推理 + 后处理 + NV12 画框
// 改编自 nfs/yolov8/rknn_infer_test.cpp
class RKNNInference {
public:
    struct Config {
        std::string model_path;
        int   net_w = 640;
        int   net_h = 640;
        float conf_thresh = 0.25f;
        float nms_thresh  = 0.45f;
        int   nc;  // 类别数 (会自动从模型推导)
    };

    RKNNInference();
    ~RKNNInference();

    int init(const Config& cfg);

    // 对一帧 NV12 数据做推理。返回检测结果。
    // nv12_data: 虚拟地址 (malloc'd buffer), rgb_buf: 预分配的 RGB 缓冲区 (net_w * net_h * 3)
    int infer(void* nv12_data, int src_w, int src_h,
              int src_wstride, int src_hstride,
              uint8_t* rgb_buf,
              std::vector<Detection>& detections);

    // 直接在 NV12 缓冲区上画框
    static void draw_boxes_nv12(uint8_t* nv12_data, int width, int height, int stride,
                                const std::vector<Detection>& dets);

    int  get_net_w() const { return net_w_; }
    int  get_net_h() const { return net_h_; }
    int  get_rgb_buf_size() const { return net_w_ * net_h_ * 3; }
    int  get_nv12_small_size() const { return net_w_ * net_h_ * 3 / 2; }

private:
    int load_model(const char* path);
    int parse_output(float* out_data, int out_proposals, int nc,
                     int orig_w, int orig_h,
                     std::vector<Detection>& dets);

    rknn_context ctx_;
    rknn_input_output_num io_num_;
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;

    int net_w_, net_h_;
    int num_classes_;
    int out_proposals_;

    float conf_thresh_;
    float nms_thresh_;
};
