#ifndef __MY_TASK_H__
#define __MY_TASK_H__

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <cstdlib>

//检测结果（画框用）
struct Detection {
    int x, y, w, h;
    int class_id;
    float confidence;
};

//帧数据——在 capture / inference / encoder 线程之间传递
struct FrameData {
    int      index;            //帧序号 (<0 表示队列结束哨兵)
    int      v4l2_buf_index;   //V4L2 buffer index，编码线程用完后需归还
    int      nv12_fd;          //dma-buf fd (NV12)
    void*    nv12_virt;        //mmap 虚拟地址 (仅推理拷贝使用)
    int      width;
    int      height;
    int      width_stride;
    int      height_stride;
    size_t   data_size;        //NV12 总字节数
    uint64_t capture_ts_us;    //采集时间戳 (微秒)
};

//线程安全队列
template<typename T>
class SafeQueue {
public:
    SafeQueue(size_t max_size = 0) : max_size_(max_size) {}//限制队列最大容量

    //生产者入队
    bool push(T item) {
        {
            std::lock_guard<std::mutex> lock(mtx_);//加锁
            if (max_size_ > 0 && queue_.size() >= max_size_) {//如果设置了容量上限且队列已满
                T old = std::move(queue_.front());//取出首帧
                queue_.pop();//弹出首帧
                if (old.nv12_virt)
                    free(old.nv12_virt);//释放旧帧内存
                queue_.push(std::move(item));//将新帧放入队尾
                cv_.notify_one();//通知消费者
                return false;//表示丢了一帧旧帧
            }
            queue_.push(std::move(item));//将新帧放入队尾
        }
        cv_.notify_one();//通知消费者
        return true;
    }

    //消费者阻塞出队
    T pop() {
        std::unique_lock<std::mutex> lock(mtx_);//加锁
        cv_.wait(lock, [this] { return !queue_.empty() || done_; });//条件变量等待：队列非空或被通知结束
        if (done_ && queue_.empty()) {//如果被通知结束，且队列已经空了
            T item{};
            item.index = -1;  //表示结束哨兵
            return item;
        }
        T item = std::move(queue_.front());//取出首帧
        queue_.pop();//弹出首帧
        return item;
    }

    void set_done() {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            done_ = true;
        }
        cv_.notify_all();
    }

private:
    std::queue<T> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
    size_t max_size_;
    bool done_ = false;
};

struct Task_config {
    /*---视频采集配置---*/
    const char* device_path   = "/dev/video0";//主视频设备节点
    const char* subdev_path   = "/dev/v4l-subdev3";//子设备节点，用于调整摄像头参数
    int  capture_width        = 1920;//摄像头分辨率
    int  capture_height       = 1080;//摄像头分辨率
    int  capture_buf_count    = 4;//环形缓冲区数量；正常4-6
    /*------------------*/

    /*---编码推流配置---*/
    int  encode_width         = 1920;
    int  encode_height        = 1080;
    int  encode_bitrate       = 4096;       // kbps
    int  rotation             = 0;          // MPP 旋转: 0/90/180/270
    int  http_port            = 8080;       // HTTP 服务端口
    /*------------------*/

    /*---推理配置---*/
    const char* model_path    = "yolov8_person.rknn";//RKNN 模型路径
    float conf_threshold      = 0.55f;//置信度阈值
    float nms_threshold       = 0.45f;//NMS 阈值
    bool  enable_inference    = true;//是否启用推理
    int   infer_interval      = 3;//每 N 帧推理一次
    /*------------------*/
};

class My_Task{
public:
    My_Task(const Task_config& cfg);
    ~My_Task();

    int start();
    int stop();

private:
    void capture_thread_func();
    void inference_thread_func();
    void encoder_thread_func();

    //编码线程用完的 V4L2 buffer index 回收队列
    void recycle_buffer(int idx) {
        std::lock_guard<std::mutex> lock(recycle_mtx_);
        recycle_queue_.push(idx);
    }
    bool recycle_pop(int& idx) {
        std::lock_guard<std::mutex> lock(recycle_mtx_);
        if (recycle_queue_.empty()) return false;
        idx = recycle_queue_.front();
        recycle_queue_.pop();
        return true;
    }

    Task_config task_cfg_;

    SafeQueue<FrameData>  raw_queue_{4};//采集 -> 推理
    SafeQueue<FrameData>  enc_queue_{4};//采集 -> 编码

    std::mutex            recycle_mtx_;
    std::queue<int>       recycle_queue_;

    //共享的检测结果 (inference 写入, capture 读取画框)
    std::mutex            det_mtx_;
    std::vector<Detection> latest_dets_;

    //编码帧率 (encoder 写入, capture 读取显示)
    std::atomic<double>   enc_fps_{0.0};

    std::atomic<bool>     running_;
};

#endif
