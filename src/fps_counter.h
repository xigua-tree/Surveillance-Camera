#ifndef __FPS_COUNTER_H__
#define __FPS_COUNTER_H__

#include <ctime>

// 基于单调时钟的帧率计数器：固定 1 秒窗口，fps = 帧数 / 真实耗时
class FpsCounter {
public:
    FpsCounter() : count_(0), fps_(0.0), window_start_(now_mono()) {}

    void tick() {
        count_++;
        double now = now_mono();
        double dt = now - window_start_;
        if (dt >= 1.0) {
            fps_ = count_ / dt;
            count_ = 0;
            window_start_ = now;
        }
    }

    double fps() const { return fps_; }

private:
    static double now_mono() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + ts.tv_nsec * 1e-9;
    }

    int    count_;
    double fps_;
    double window_start_;
};

#endif
