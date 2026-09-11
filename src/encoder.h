#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <cstdint>
#include <cstddef>
#include <string>

#include "rockchip/mpp_buffer.h"
#include "rockchip/mpp_frame.h"
#include "rockchip/rk_mpi.h"

class EncoderStream {
public:
    // 编码器与推流相关配置
    struct Config {
        int width        = 1920; 
        int height       = 1080;
        int bitrate_kbps = 4096;//目标码率kbps
        int fps_num      = 30;//帧率分子（如 30）
        int fps_den      = 1;//帧率分母（通常为 1，表示 30/1 = 30fps）
        int gop          = 30;//GOP大小，即关键帧间隔
        int rotation     = 0;

        //ZLMediaKit推流配置
        const char* vhost  = "xigua_tree's_host"; //虚拟主机名
        const char* app    = "live";//应用名
        const char* stream = "camera";//流名，RTSP URL中的stream
        int http_port      = 80;// HTTP 服务端口

        //H.264本地录制配置
        bool enable_record = false;//录制标志位
        const char* record_path = "/mnt/sdcard/record/"; // 录制文件保存目录
        int record_max_sec = 300; //单个录制分片最大时长，单位秒
    };

    EncoderStream();

    // 析构函数：调用 cleanup 释放所有资源
    ~EncoderStream();

    // 初始化编码器：依次初始化 MPP、获取 SPS/PPS、初始化 ZLMediaKit
    int init(const Config& cfg);

    // 编码一帧：nv12_fd 是 V4L2 导出的 dma-buf fd，RGA 直接零拷贝读取
    // width/height/hor_stride/ver_stride 描述输入 NV12 图像布局
    // pts_ms 为帧时间戳，单位毫秒
    int feed_frame(int nv12_fd,
                   int width, int height,
                   int hor_stride, int ver_stride,
                   uint64_t pts_ms);

    // 开始录制：创建 .h264 文件并写入 SPS/PPS
    // path：保存目录；max_sec：单个分片最大时长
    int start_record(const char* path, int max_sec);

    // 停止录制：关闭当前文件，重置录制状态
    int stop_record();

    // 清理函数：释放所有 MPP 资源、关闭文件、释放 ZLMediaKit 对象
    void cleanup();

private:
    // 初始化 MPP 硬件编码器
    int init_mpp();

    // 初始化 ZLMediaKit 流媒体服务（RTSP/HTTP 推流）
    int init_zlmediakit();

    // 获取 SPS/PPS 数据，保存到 sps_pps_buf_
    int get_sps_pps();

    // MPP 上下文
    MppCtx mpp_ctx_;

    // MPP 接口指针，包含 control、encode 等函数
    MppApi* mpp_mpi_;

    // MPP 编码配置结构体
    MppEncCfg enc_cfg_;

    // MPP 缓冲区组，用于管理内存池
    MppBufferGroup buf_grp_;

    // 当前帧输入缓冲区，存放 NV12 原始数据
    MppBuffer frm_buf_;

    // 输出码流包缓冲区，存放 H.264 数据
    MppBuffer pkt_buf_;

    // 运动信息缓冲区，用于编码器运动估计
    MppBuffer md_info_;

    // 单帧数据大小（含对齐）
    size_t frame_size_;

    // 运动信息缓冲区大小
    size_t mdinfo_size_;

    // 水平 stride，行对齐后字节数
    int hor_stride_;

    // 垂直 stride，通常等于 height 对齐后
    int ver_stride_;

    // SPS/PPS 数据，每个 IDR 帧前需要发送
    uint8_t sps_pps_buf_[1024];

    // SPS/PPS 数据长度
    int sps_pps_len_;

    // ZLMediaKit 媒体对象指针
    void* mk_media_;

    // 流是否已初始化（ZLMediaKit 是否就绪）
    bool stream_inited_;

    // 录制文件指针
    FILE* record_fp_;

    // 当前录制文件名
    char record_file_[512];

    // 当前录制分片的起始时间戳（秒）
    time_t segment_start_sec_;

    // 编码器配置
    Config cfg_;

    // 是否正在录制中
    bool recording_;
};

#endif