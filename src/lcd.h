#ifndef __LCD_H__
#define __LCD_H__

#include <cstdint>
#include <cstddef>

class LcdDisplay{
public:
    LcdDisplay();
    ~LcdDisplay();

    int init();
    int show(void* nv12_addr, int width, int height, int stride);
    void stop();

private:
    struct FbBuffer {
        uint32_t fb_id;   //Framebuffer ID
        uint32_t handle;  //Dumb Buffer句柄
        uint32_t pitch;   //每行字节数
        uint32_t size;
        int      dumb_fd; //dma-buf文件描述符，给RGA用
        void*    map;     //mmap映射后的用户空间虚拟地址
    };

    int  alloc_fb(FbBuffer& buf);
    void free_fb(FbBuffer& buf);
    int  wait_flip(volatile bool* done);

    int      fd_;
    uint32_t crtc_id_;//显示控制器ID
    uint32_t conn_id_;//连接器ID
    int      width_;
    int      height_;

    FbBuffer bufs_[2];//双缓冲
    int      front_;  //正在扫描输出的buffer下标
    int      back_;   //正在绘制(待翻页)的buffer下标
};

#endif
