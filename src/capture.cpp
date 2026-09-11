#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <errno.h>
#include <cstdint>
#include <cstddef>
#include <linux/videodev2.h>
#include "capture.h"

Capture::Capture()
    :fd_(-1), sub_fd_(-1), width_(0), height_(0), stride_(0),
      buf_count_(0), data_size_(0), buffers_(nullptr) {}

Capture::~Capture()
{
    stop();
}

int Capture::init(const char* device,int w,int h,int buf_count)
{
    this->width_ = w;
    this->height_ = h;
    this->buf_count_ = buf_count;
    data_size_ = width_ * height_ * 3 / 2; // NV12的数据大小

    fd_ = open(device, O_RDWR | O_CLOEXEC, 0);//允许读写和执行时可关闭
    if (fd_ < 0) {
        printf("Capture | Open device failed! (errno=%d)\n",errno);
        return -1;
    }

    //查询设备能力
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        printf("Capture | VIDIOC_QUERYCAP failed!\n");
        return -1;
    }
    printf("Capture | Driver: %s\n", cap.driver);

    //设置视频格式为NV12
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;//设置为多平面视频捕获类型，一般固定
    fmt.fmt.pix_mp.width = width_;
    fmt.fmt.pix_mp.height = height_;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;//设置像素格式
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;//设置为逐行扫描

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        printf("Capture | VIDIOC_S_FMT failed (errno=%d)\n", errno);
        return -1;
    }

    width_  = fmt.fmt.pix_mp.width;//读回驱动传回来的宽和高
    height_ = fmt.fmt.pix_mp.height;
    stride_ = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;//读回实际的stride
    int num_planes = fmt.fmt.pix_mp.num_planes;//读取平面数量
    printf("Capture | Format: %dx%d NV12, stride=%d, planes=%d\n",
           width_, height_, stride_, num_planes);
    data_size_ = (size_t)stride_ * height_ * 3 / 2;//计算一帧数据的大小

    //申请缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = buf_count;//申请内存块的数量
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;//设置为多平面视频捕获类型，一般固定
    req.memory = V4L2_MEMORY_MMAP;//通过MMAP映射到用户态

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        printf("Capture | VIDIOC_REQBUFS failed\n");
        return -1;
    }

    buffers_ = (Buffer*)calloc(buf_count, sizeof(Buffer));//在堆上创建Buffer结构体数组，用于管理内存块
    for (int i = 0; i < buf_count; ++i) {//获取该缓冲区的物理偏移量和实际长度
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;//设置为多平面视频捕获类型，一般固定
        buf.memory = V4L2_MEMORY_MMAP;//查询MMAP模式下的信息
        buf.index = i;//查询第i快内存
        buf.m.planes = planes;//指向平面信息数组
        buf.length = 1;//NV12只用了一个平面

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            printf("Capture | VIDIOC_QUERYBUF[%d] failed\n", i);
            return -1;
        }

        buffers_[i].length[0] = buf.m.planes[0].length;//保存该平面的实际大小
        //将内核物理内存映射到用户态虚拟地址空间
        buffers_[i].start[0] = mmap(NULL, buf.m.planes[0].length,
                                     PROT_READ | PROT_WRITE,//可读可写
                                     MAP_SHARED,//共享映射
                                     fd_, 
                                     buf.m.planes[0].m.mem_offset);//物理偏移量
        if (buffers_[i].start[0] == MAP_FAILED) {
            printf("Capture | mmap[%d] failed\n", i);
            return -1;
        }

        //导出dma-buf的文件描述符fd，需要直接通过DMA读取物理内存
        struct v4l2_exportbuffer expbuf;
        memset(&expbuf, 0, sizeof(expbuf));
        expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;//设置为多平面视频捕获类型，一般固定
        expbuf.index = i;//要导出哪个缓冲区
        expbuf.plane = 0;//导出第0个平面
        expbuf.flags = O_RDWR;//导出的fd具备读写权限
        if (ioctl(fd_, VIDIOC_EXPBUF, &expbuf) == 0) {//将dma-fd保存到expbuf中
            buffers_[i].dma_fd = expbuf.fd;
            printf("Capture | Buffer[%d] dma_fd = %d\n", i, expbuf.fd);
        } else {
            buffers_[i].dma_fd = -1;
            printf("Capture | EXPBUF[%d] failed!\n", i);
        }
    }

    //所有缓冲区入队
    for (int i = 0; i < buf_count; ++i) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;//设置为多平面视频捕获类型，一般固定
        buf.memory = V4L2_MEMORY_MMAP;//查询MMAP模式下的信息
        buf.index = i;//第i快内存入队
        buf.m.planes = planes;//指向平面信息数组
        buf.length = 1;//NV12只用了一个平面

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {//入队操作；内核会校验该缓冲区是否成功映射，校验后将该缓冲区的地址放入DMA环形队列的尾部
            printf("Capture | VIDIOC_QBUF[%d] failed\n", i);
            return -1;
        }
    }

    //开始采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;//设置为多平面视频捕获类型，一般固定
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {//启动视频流
        printf("Capture | VIDIOC_STREAMON failed\n");
        return -1;
    }
    printf("Capture | Stream started\n");

    return 0;
}

//封装出队操作
int Capture::dequeue(int& index, int& bytesused)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length = 1;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        printf("Capture | VIDIOC_DQBUF failed (errno=%d)\n", errno);
        return -1;
    }

    index = buf.index;
    bytesused = buf.m.planes[0].bytesused;
    return 0;
}

//入队操作
int Capture::enqueue(int index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    buf.m.planes = planes;
    buf.length = 1;

    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        printf("Capture | VIDIOC_QBUF[%d] failed (errno=%d)\n", index, errno);
        return -1;
    }
    return 0;
}

void Capture::stop()
{
    if (fd_ >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);//停止流
        for (int i = 0; i < buf_count_; i++) {
            if (buffers_[i].dma_fd > 0) {
                close(buffers_[i].dma_fd);//释放dma_fd句柄
                buffers_[i].dma_fd = -1;
            }
            if (buffers_[i].start[0]) {
                munmap(buffers_[i].start[0], buffers_[i].length[0]);//解除映射
                buffers_[i].start[0] = nullptr;
            }
        }
        close(fd_);
        fd_ = -1;
    }
    if (buffers_) {
        free(buffers_);
        buffers_ = nullptr;
    }
}