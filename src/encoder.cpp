// 包含编码器头文件
#include "encoder.h"
#include "mk_mediakit.h"
#include "im2d.h"
#include "rga.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>

//将x向上对齐到a的倍数。MPP 硬件编码器对输入图像的行跨度有对齐要求，
//通常要求水平方向按 16 字节对齐，垂直方向按 16 像素对齐。这样硬件在读取内存时效率最高，也避免越界。
#define MPP_ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))

EncoderStream::EncoderStream()
    : mpp_ctx_(nullptr),
      mpp_mpi_(nullptr),//MPP接口指针（包含 control、encode 等函数）
      enc_cfg_(nullptr),//MPP编码配置结构体指针
      buf_grp_(nullptr),//MPP 缓冲区组（管理内存池）
      frm_buf_(nullptr),//当前帧输入缓冲区（存放 NV12 原始数据）
      pkt_buf_(nullptr),// 输出码流包缓冲区（存放 H.264 数据）
      md_info_(nullptr),// 运动信息缓冲区（用于编码器运动估计）
      frame_size_(0),// 单帧数据大小（含对齐）
      mdinfo_size_(0),// 运动信息缓冲区大小
      hor_stride_(0),// 水平 stride（行对齐后字节数）
      ver_stride_(0),             // 垂直 stride（通常等于 height 对齐后）
      sps_pps_len_(0),            // SPS/PPS 数据长度
      mk_media_(nullptr),         // ZLMediaKit 媒体对象指针
      stream_inited_(false),      // 流是否已初始化（ZLMediaKit 是否就绪）
      record_fp_(nullptr),        // 录制文件指针
      segment_start_sec_(0),      // 当前录制分片的起始时间戳（秒）
      recording_(false) {         // 是否正在录制中
    
    memset(sps_pps_buf_, 0, sizeof(sps_pps_buf_));//清零 SPS/PPS 缓冲区
    memset(record_file_, 0, sizeof(record_file_));//清零当前录制文件名
}

EncoderStream::~EncoderStream() { cleanup(); }

void EncoderStream::cleanup() {
    // 如果录制文件指针有效，关闭文件
    if (record_fp_) { fclose(record_fp_); record_fp_ = nullptr; }
    // 如果 ZLMediaKit 媒体对象存在，释放它
    if (mk_media_) { mk_media_release(mk_media_); mk_media_ = nullptr; }
    // 销毁 MPP 上下文
    if (mpp_ctx_) { mpp_destroy(mpp_ctx_); mpp_ctx_ = nullptr; }
    // 反初始化编码配置结构体
    if (enc_cfg_) { mpp_enc_cfg_deinit(enc_cfg_); enc_cfg_ = nullptr; }
    // 释放输入帧缓冲区
    if (frm_buf_)  { mpp_buffer_put(frm_buf_);  frm_buf_  = nullptr; }
    // 释放输出码流包缓冲区
    if (pkt_buf_)  { mpp_buffer_put(pkt_buf_);  pkt_buf_  = nullptr; }
    // 释放运动信息缓冲区
    if (md_info_)  { mpp_buffer_put(md_info_);  md_info_  = nullptr; }
    // 释放缓冲区组（内存池）
    if (buf_grp_)  { mpp_buffer_group_put(buf_grp_); buf_grp_ = nullptr; }
}

int EncoderStream::init_mpp() {
    int ret; 

    //将宽高对齐到 16 的倍数，主要是MPP硬件要求:
    hor_stride_ = MPP_ALIGN(cfg_.width, 16);
    ver_stride_ = MPP_ALIGN(cfg_.height, 16);

    frame_size_ = hor_stride_ * ver_stride_ * 3 / 2;//计算NV12数据大小
    
    mdinfo_size_ = (MPP_ALIGN(hor_stride_, 64) >> 6) * (MPP_ALIGN(ver_stride_, 16) >> 4) * 16;//计算运动信息缓冲区大小

    ret = mpp_buffer_group_get_internal(&buf_grp_, MPP_BUFFER_TYPE_DRM);//获取一个内部缓冲区组，使用DRM
    if (ret) { 
        printf("Encoder | mpp_buffer_group_get failed!\n"); 
        return -1; 
    }

    ret = mpp_buffer_get(buf_grp_, &pkt_buf_, frame_size_);//从缓冲区组中分配一个输出码流包缓冲区
    if (ret) { 
        printf("Encoder | mpp_buffer_get pkt_buf failed\n"); 
        return -1; 
    }

    ret = mpp_buffer_get(buf_grp_, &md_info_, mdinfo_size_);//分配运动信息缓冲区
    if (ret) { 
        printf("Encoder | mpp_buffer_get md_info failed\n"); 
        return -1; 
    }

    ret = mpp_create(&mpp_ctx_, &mpp_mpi_);//创建MPP上下文和接口对象
    if (ret) { 
        printf("Encoder | mpp_create failed\n"); 
        return -1; 
    }

    //设置MPP输出超时模式为阻塞
    MppPollType timeout = MPP_POLL_BLOCK;
    mpp_mpi_->control(mpp_ctx_, MPP_SET_OUTPUT_TIMEOUT, &timeout);

    ret = mpp_init(mpp_ctx_, MPP_CTX_ENC, MPP_VIDEO_CodingAVC);//初始化MPP上下文为编码器类型，编码格式为H.264
    if (ret) {
        printf("Encoder | mpp_init failed\n");
        return -1;
    }

    // 初始化编码配置结构体
    ret = mpp_enc_cfg_init(&enc_cfg_);
    if (ret) {
        printf("Encoder | mpp_enc_cfg_init failed\n");
        return -1;
    }

    // 设置输入图像宽度
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:width",       cfg_.width);
    // 设置输入图像高度
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:height",      cfg_.height);
    // 设置水平 stride（行对齐字节数）
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:hor_stride",  hor_stride_);
    // 设置垂直 stride
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:ver_stride",  ver_stride_);
    // 设置输入像素格式为 YUV420SP（即 NV12）
    mpp_enc_cfg_set_s32(enc_cfg_, "prep:format",      MPP_FMT_YUV420SP);

    // 设置码率控制模式为 CBR（恒定比特率）
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:mode",          MPP_ENC_RC_MODE_CBR);
    // 输入帧率分子（如 30）
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_in_num",    cfg_.fps_num);
    // 输入帧率分母（通常为 1）
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_in_denorm", cfg_.fps_den);
    // 输出帧率分子（与输入一致）
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_out_num",   cfg_.fps_num);
    // 输出帧率分母
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:fps_out_denorm",cfg_.fps_den);
    // GOP（关键帧间隔）：每多少帧插入一个 I 帧
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:gop",           cfg_.gop);
    // 目标码率（bps），由 kbps 转换
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:bps_target",    cfg_.bitrate_kbps * 1000);
    // 最大码率（目标码率的 1.062 倍，容限）
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:bps_max",       cfg_.bitrate_kbps * 1062);
    // 最小码率（目标码率的 0.937 倍）
    mpp_enc_cfg_set_s32(enc_cfg_, "rc:bps_min",       cfg_.bitrate_kbps * 937);
    // 丢弃帧模式：禁止丢弃（保证每帧都编码）
    mpp_enc_cfg_set_u32(enc_cfg_, "rc:drop_mode",     MPP_ENC_RC_DROP_FRM_DISABLED);

    // 设置编码器类型为 H.264
    mpp_enc_cfg_set_s32(enc_cfg_, "codec:type",       MPP_VIDEO_CodingAVC);
    // H.264 档次（100 表示 High Profile）
    mpp_enc_cfg_set_s32(enc_cfg_, "h264:profile",     100);
    // H.264 级别（40 表示 Level 4.0，支持 1080p）
    mpp_enc_cfg_set_s32(enc_cfg_, "h264:level",       40);
    // 启用 CABAC 熵编码（提高压缩率）
    mpp_enc_cfg_set_s32(enc_cfg_, "h264:cabac_en",    1);
    // CABAC IDC（初始变量，0 为默认）
    mpp_enc_cfg_set_s32(enc_cfg_, "h264:cabac_idc",   0);
    // 启用 8x8 变换（High Profile 特性）
    mpp_enc_cfg_set_s32(enc_cfg_, "h264:trans8x8",    1);

    if (cfg_.rotation != 0) {
        int rot = 0;
        switch (cfg_.rotation) {
            case 90:  rot = 1; break;  // MPP_ENC_ROT_90
            case 180: rot = 2; break;  // MPP_ENC_ROT_180
            case 270: rot = 3; break;  // MPP_ENC_ROT_270
            default:  rot = 0; break;
        }
        // 设置旋转参数
        mpp_enc_cfg_set_s32(enc_cfg_, "prep:rotation", rot);
        printf("Encoder | Rotation: %d deg -> MPP rot enum=%d\n", cfg_.rotation, rot);
    }

    ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_CFG, enc_cfg_);//将编码配置应用到MPP上下文
    if (ret) { printf("Encoder | MPP_ENC_SET_CFG failed ret=%d\n", ret); return -1; }

    RK_U32 header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;//设置头部模式：每个IDR帧都附带SPS/PPS
    mpp_mpi_->control(mpp_ctx_, MPP_ENC_SET_HEADER_MODE, &header_mode);

    printf("Encoder | MPP H.264 init ok: %dx%d, %d kbps\n",
           cfg_.width, cfg_.height, cfg_.bitrate_kbps);
    return 0;
}

// 获取SPS和PPS
int EncoderStream::get_sps_pps() {
    MppPacket packet = nullptr; // 用于接收头部数据的包

    mpp_packet_init_with_buffer(&packet, pkt_buf_);
    mpp_packet_set_length(packet, 0);

    int ret = mpp_mpi_->control(mpp_ctx_, MPP_ENC_GET_HDR_SYNC, packet);
    if (ret) {
        printf("Encoder | MPP_ENC_GET_HDR_SYNC failed ret=%d\n", ret);
        mpp_packet_deinit(&packet);
        return -1;
    }

    void* ptr = mpp_packet_get_pos(packet);
    size_t len = mpp_packet_get_length(packet);

    if (len > sizeof(sps_pps_buf_)) len = sizeof(sps_pps_buf_);

    memcpy(sps_pps_buf_, ptr, len);
    sps_pps_len_ = (int)len;

    mpp_packet_deinit(&packet);

    printf("Encoder | SPS/PPS len=%d\n", sps_pps_len_);
    return 0;
}

//初始化ZLMediaKit
int EncoderStream::init_zlmediakit() {
    mk_config zlm_cfg;  
    memset(&zlm_cfg, 0, sizeof(zlm_cfg));
    zlm_cfg.log_mask = LOG_CONSOLE;//日志输出到控制台
    zlm_cfg.log_level = 0;// 日志级别 0（调试）
    mk_env_init(&zlm_cfg);//初始化ZLMediaKit环境

    mk_http_server_start(cfg_.http_port, 0);//启动http
    mk_rtsp_server_start(554, 0);//启动rtsp

    mk_media_ = mk_media_create(cfg_.vhost, cfg_.app, cfg_.stream,0.0f, 0, 0);
    if (!mk_media_) {
        printf("Encoder | mk_media_create failed\n");
        return -1;
    }

    int out_w = cfg_.width;
    int out_h = cfg_.height;
    if (cfg_.rotation == 90 || cfg_.rotation == 270) {
        out_w = cfg_.height;
        out_h = cfg_.width;
    }

    //初始化视频轨道：轨道ID0，宽高，帧率，码率
    mk_media_init_video(mk_media_, 0,
                        out_w, out_h,
                        (float)cfg_.fps_num / cfg_.fps_den,
                        cfg_.bitrate_kbps * 1000);

    mk_media_init_complete(mk_media_);

    stream_inited_ = true; //标记流已初始化
    printf("[Encoder] ZLMediaKit RTSP+HTTP server started.\n");
    return 0;
}

//外部初始化调用接口
int EncoderStream::init(const Config& cfg) {
    cfg_ = cfg;                     // 保存配置
    if (init_mpp() != 0) return -1;
    if (get_sps_pps() != 0) return -1;
    if (init_zlmediakit() != 0) return -1;
    return 0;
}

// 核心函数：喂入一帧 NV12 数据，进行编码、画框、推流和录制
int EncoderStream::feed_frame(int nv12_fd,                          // 输入的 NV12 dma-buf fd
                              int width, int height,               // 输入图像宽高
                              int hor_stride, int ver_stride,     // 输入 stride（可能来自 V4L2）
                              uint64_t pts_ms) {                  // 时间戳（毫秒）
    int ret;

    //如果当前没有输入帧缓冲区，从缓冲区组分配一个
    if (!frm_buf_) {
        ret = mpp_buffer_get(buf_grp_, &frm_buf_, frame_size_);
        if (ret) { printf("Encoder | get input buffer failed\n"); return -1; }
    }

    //获取MPP缓冲区的dma-buf fd
    int mpp_fd = mpp_buffer_get_fd(frm_buf_);

    //使用RGA硬件加速，将输入dma-buf拷贝到MPP缓冲区
    rga_buffer_t src = wrapbuffer_fd_t(nv12_fd, width, height,
                                       hor_stride, ver_stride,
                                       RK_FORMAT_YCbCr_420_SP);
    //目标图像信息：MPP缓冲区，使用编码器配置的stride
    rga_buffer_t dst = wrapbuffer_fd_t(mpp_fd, cfg_.width, cfg_.height,
                                        hor_stride_, ver_stride_,
                                        RK_FORMAT_YCbCr_420_SP);
    
    IM_STATUS st = imcopy(src, dst);//执行硬件拷贝
    if (st != IM_STATUS_SUCCESS) {
        printf("Encoder | RGA imcopy from dma-buf failed (%d)\n", (int)st);
        mpp_buffer_put(frm_buf_);
        frm_buf_ = nullptr;
        return -1;
    }

    //创建 MPP 帧对象，用于包装输入数据
    MppFrame frame = nullptr;
    ret = mpp_frame_init(&frame);
    if (ret) {
        printf("Encoder | mpp_frame_init failed\n");
        return -1;
    }

    //设置帧属性：宽高、stride、格式、结束标志、绑定缓冲区
    mpp_frame_set_width(frame, cfg_.width);
    mpp_frame_set_height(frame, cfg_.height);
    mpp_frame_set_hor_stride(frame, hor_stride_);
    mpp_frame_set_ver_stride(frame, ver_stride_);
    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
    mpp_frame_set_eos(frame, 0);// 非结束帧
    mpp_frame_set_buffer(frame, frm_buf_);

    //获取帧的元数据
    MppMeta meta = mpp_frame_get_meta(frame);
    //初始化一个输出包，使用pkt_buf_作为缓冲区
    MppPacket packet = nullptr;
    mpp_packet_init_with_buffer(&packet, pkt_buf_);
    mpp_packet_set_length(packet, 0);    //清空长度，等待编码器填充

    // 将输出包和运动信息缓冲区挂到元数据上
    mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet);
    mpp_meta_set_buffer(meta, KEY_MOTION_INFO, md_info_);

    //将帧提交给编码器
    ret = mpp_mpi_->encode_put_frame(mpp_ctx_, frame);
    
    mpp_frame_deinit(&frame);//释放帧对象
    if (ret) {
        printf("Encoder | encode_put_frame failed\n");
        mpp_packet_deinit(&packet);
        return -1;
    }

    //循环取出编码后的码流包
    RK_U32 eoi = 1;//结束标志
    do {
        
        ret = mpp_mpi_->encode_get_packet(mpp_ctx_, &packet);//从编码器获取一个输出包
        if (ret) { 
            printf("Encoder | encode_get_packet failed\n");
            return -1;
        }

        if (packet) {
            //获取包中的数据和长度
            void*  ptr = mpp_packet_get_pos(packet);
            size_t len = mpp_packet_get_length(packet);

            if (len > 0) {
                
                static int enc_frame_idx = 0;//静态变量记录编码帧序号，用于判断IDR帧
                bool is_idr = (enc_frame_idx % cfg_.gop == 0);//判断当前帧是否为IDR帧

                //推流给ZLMediaKit：
                if (is_idr && sps_pps_len_ > 0) {
                    mk_media_input_h264(mk_media_, sps_pps_buf_, sps_pps_len_, pts_ms, pts_ms);
                }
                //发送当前帧的 H.264 数据
                mk_media_input_h264(mk_media_, ptr, (int)len, pts_ms, pts_ms);

                //如果正在录制，写入本地.h264 文件
                if (record_fp_) {
                    //同样，IDR 帧前写入SPS/PPS
                    if (is_idr && sps_pps_len_ > 0) {
                        fwrite(sps_pps_buf_, 1, sps_pps_len_, record_fp_);
                    }
                    //写入当前帧数据
                    fwrite(ptr, 1, len, record_fp_);

                    //检查是否达到分片时长限制
                    time_t now = time(nullptr);
                    if (now - segment_start_sec_ >= cfg_.record_max_sec) {
                        //关闭当前文件，开启新分片
                        fclose(record_fp_);
                        record_fp_ = nullptr;
                        segment_start_sec_ = now;
                        // 新文件名以时间戳命名
                        snprintf(record_file_, sizeof(record_file_),
                                 "%s/%ld.h264", cfg_.record_path, (long)now);
                        record_fp_ = fopen(record_file_, "wb");
                        if (record_fp_) {
                            //新文件开头写入SPS/PPS
                            fwrite(sps_pps_buf_, 1, sps_pps_len_, record_fp_);
                            printf("Encoder | New segment: %s\n", record_file_);
                        } else {
                            printf("Encoder | Failed to open new segment\n");
                            recording_ = false; //关闭录制标志
                        }
                    }
                }
                enc_frame_idx++;//增加编码帧计数
            }

            //释放当前包
            mpp_packet_deinit(&packet);
        }
    } while (!eoi); //因为 eoi=1，循环只执行一次

    //释放输入帧缓冲区
    mpp_buffer_put(frm_buf_);
    frm_buf_ = nullptr;

    return 0;
}

//开始录制：创建 .h264 文件并写入 SPS/PPS
int EncoderStream::start_record(const char* path, int max_sec) {
    // 如果已经在录制，直接返回
    if (recording_) return 0;

    // 更新配置中的分片时长
    cfg_.record_max_sec = max_sec;

    // 直接写入裸 H.264 文件，后续可通过 ffmpeg 转封装
    segment_start_sec_ = time(nullptr);
    snprintf(record_file_, sizeof(record_file_),
             "%s/%ld.h264", path, (long)segment_start_sec_);
    record_fp_ = fopen(record_file_, "wb");
    if (record_fp_) {
        // 文件头部写入 SPS/PPS，确保解码器可正确初始化解码
        fwrite(sps_pps_buf_, 1, sps_pps_len_, record_fp_);
        recording_ = true;
        printf("Encoder | H.264 recording: %s (segment every %ds)\n",
               record_file_, cfg_.record_max_sec);
        return 0;
    } else {
        printf("Encoder | Cannot open: %s\n", record_file_);
        return -1;
    }
}

// 停止录制：关闭当前文件，重置录制状态
int EncoderStream::stop_record() {
    if (!recording_) return 0;  // 未录制则直接返回

    if (record_fp_) {
        fclose(record_fp_);
        record_fp_ = nullptr;
        printf("Encoder | Recording stopped: %s\n", record_file_);
    }

    recording_ = false;
    return 0;
}