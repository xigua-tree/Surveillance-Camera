#ifndef __CAPTURE_H__
#define __CAPTURE_H__

class Capture{
public:
    struct Buffer {
        void*  start[4];
        size_t length[4];
        int    dma_fd;            // EXPBUF 导出的 dma-buf fd
    };

    Capture();
    ~Capture();

    int init(const char* device,int w,int h,int buf_count);
    int dequeue(int& index, int& bytesused);//出队一帧(阻塞)
    int enqueue(int index);//入队归还buffer
    void stop();

    int  get_width()  const { return width_; }
    int  get_height() const { return height_; }
    int  get_stride() const { return stride_; }
    int  get_buf_fd(int i) const { return buffers_[i].dma_fd; }
    void* get_buf_addr(int i) const { return buffers_[i].start[0]; }

private:
    int fd_;
    int sub_fd_;
    int width_;
    int height_;
    int stride_;
    int  buf_count_;
    size_t data_size_;
    Buffer* buffers_;
};

#endif