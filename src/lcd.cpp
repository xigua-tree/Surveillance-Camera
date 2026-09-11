#include "lcd.h"
#include "RgaApi.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <sys/mman.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <libdrm/drm_mode.h>
#include <libdrm/drm_fourcc.h>

static void page_flip_handler(int fd, unsigned int frame, unsigned int sec,
                              unsigned int usec, void* data)
{
    (void)fd; (void)frame; (void)sec; (void)usec;
    *static_cast<volatile bool*>(data) = true;
}

LcdDisplay::LcdDisplay()
    : fd_(-1), crtc_id_(0), conn_id_(0), width_(0), height_(0),
      front_(0), back_(1)
{
    memset(bufs_, 0, sizeof(bufs_));
    for (int i = 0; i < 2; i++) bufs_[i].dumb_fd = -1;
}

LcdDisplay::~LcdDisplay()
{
    stop();
}

int LcdDisplay::init()
{
    //打开DRM设备
    fd_ = drmOpen("rockchip", NULL);
    if (fd_ < 0) {
        printf("LCD | open drm failed!\n");
        return -1;
    }

    drmModeResPtr res = drmModeGetResources(fd_);//指向显示硬件资源
    if (!res) {
        printf("LCD | GetResources failed\n");
        return -1;
    }

    drmModeConnectorPtr lcd = nullptr;
    for (int i = 0; i < res->count_connectors && !lcd; i++) {//遍历连接器，找到已经连接的屏幕
        drmModeConnectorPtr c = drmModeGetConnector(fd_, res->connectors[i]);//获取连接器信息
        if (!c) continue;//获取失败直接跳过
        if (c->connection != DRM_MODE_CONNECTED || c->count_modes <= 0) {//如果连接器没有实际连接屏幕或者没有可用的显示模式，跳过
            drmModeFreeConnector(c);//释放资源
            continue;
        }
        if (c->connector_type == DRM_MODE_CONNECTOR_DSI) {//我这里用的是DSI接口的屏幕
            lcd = c;
        } else {
            drmModeFreeConnector(c);
        }
    }

    if (!lcd) {//没有可用屏幕
        printf("LCD | no connected connector\n");
        drmModeFreeResources(res);//释放资源
        return -1;
    }

    width_  = lcd->modes[0].hdisplay;//屏幕支持的第一个显示模式的水平分辨率
    height_ = lcd->modes[0].vdisplay;//屏幕支持的第一个显示模式的垂直分辨率
    conn_id_ = lcd->connector_id;//保存连接器ID
    drmModeModeInfo mode = lcd->modes[0];//拷贝模式信息
    printf("LCD | w:%d h:%d, connector type %d\n", width_, height_, lcd->connector_type);

    //由连接器找CRTC
    drmModeEncoderPtr enc = drmModeGetEncoder(fd_, lcd->encoder_id);
    bool crtc_found = false;//CRTC标志位
    if (enc) {
        crtc_id_ = enc->crtc_id;
        crtc_found = true;
        drmModeFreeEncoder(enc);//释放资源
    }
    drmModeFreeConnector(lcd);//释放资源
    drmModeFreeResources(res);//释放资源
    if (!crtc_found) {
        printf("LCD | no crtc for connector\n");
        return -1;
    }

    //申请双缓冲
    if (alloc_fb(bufs_[0]) < 0 || alloc_fb(bufs_[1]) < 0) {
        stop();
        return -1;
    }

    //初始显示 front = buffer 0
    if (drmModeSetCrtc(fd_, crtc_id_, bufs_[0].fb_id, 0, 0, &conn_id_, 1, &mode)) {
        printf("LCD | SetCrtc failed\n");
        stop();
        return -1;
    }
    front_ = 0;
    back_ = 1;

    printf("LCD | init ok\n");
    return 0;
}

int LcdDisplay::alloc_fb(FbBuffer& buf)
{
    memset(&buf, 0, sizeof(buf));
    buf.dumb_fd = -1;

    //创建dumb buffer
    struct drm_mode_create_dumb create;
    memset(&create, 0, sizeof(create));
    create.bpp = 32;//使用XRGB8888格式，每像素32位
    create.width = width_;
    create.height = height_;
    if (drmIoctl(fd_, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) {
        printf("LCD | CREATE_DUMB failed!\n");
        return -1;
    }
    buf.handle = create.handle;
    buf.pitch  = create.pitch;
    buf.size   = create.size;

    //映射到用户态
    struct drm_mode_map_dumb map;
    memset(&map, 0, sizeof(map));
    map.handle = buf.handle;
    if (drmIoctl(fd_, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) {
        printf("LCD | MAP_DUMB failed!\n");
        free_fb(buf);
        return -1;
    }
    buf.map = mmap(0, buf.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, map.offset);
    if (buf.map == MAP_FAILED) {
        printf("LCD | mmap failed\n");
        buf.map = nullptr;
        free_fb(buf);
        return -1;
    }

    //导出dma-buf的fd给RGA用
    if (drmPrimeHandleToFD(fd_, buf.handle, 0, &buf.dumb_fd) < 0) {
        printf("LCD | PrimeHandleToFD failed\n");
        buf.dumb_fd = -1;
        free_fb(buf);
        return -1;
    }

    //把handle注册为framebuffer
    uint32_t handles[4] = { buf.handle, 0, 0, 0 };
    uint32_t pitches[4] = { buf.pitch, 0, 0, 0 };
    uint32_t offsets[4] = { 0, 0, 0, 0 };
    if (drmModeAddFB2(fd_, width_, height_, DRM_FORMAT_XRGB8888,
                      handles, pitches, offsets, &buf.fb_id, 0)) {
        printf("LCD | AddFB2 failed\n");
        free_fb(buf);
        return -1;
    }
    return 0;
}

void LcdDisplay::free_fb(FbBuffer& buf)
{
    if (buf.fb_id) {
        drmModeRmFB(fd_, buf.fb_id);
        buf.fb_id = 0;
    }
    if (buf.dumb_fd >= 0) {
        close(buf.dumb_fd);
        buf.dumb_fd = -1;
    }
    if (buf.map) {
        munmap(buf.map, buf.size);
        buf.map = nullptr;
    }
    if (buf.handle) {
        struct drm_mode_destroy_dumb d;
        memset(&d, 0, sizeof(d));
        d.handle = buf.handle;
        drmIoctl(fd_, DRM_IOCTL_MODE_DESTROY_DUMB, &d);
        buf.handle = 0;
    }
}

int LcdDisplay::show(void* nv12_addr, int width, int height, int stride)
{
    if (fd_ < 0 || bufs_[back_].map == nullptr) return -1;

    FbBuffer& back = bufs_[back_];

    //RGA:NV12->RGB转换，写入back buffer
    rga_info_t src, dst;
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));

    src.fd = -1;
    src.virAddr = nv12_addr;//源地址
    src.mmuFlag = 1;
    src.rotation = HAL_TRANSFORM_ROT_90;//这里旋转90°
    rga_set_rect(&src.rect, 0, 0, width, height, stride, height, RK_FORMAT_YCbCr_420_SP);

    dst.fd = -1;
    dst.virAddr = back.map;//back buffer地址
    dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, 0, 0, width_, height_, width_, height_, RK_FORMAT_BGRA_8888);

    if (c_RkRgaBlit(&src, &dst, NULL)) {//启动硬件加速
        printf("LCD | RGA blit failed\n");
        return -1;
    }

    //翻页到back buffer，带事件等待vblank完成
    volatile bool flip_done = false;
    if (drmModePageFlip(fd_, crtc_id_, back.fb_id, DRM_MODE_PAGE_FLIP_EVENT,
                        (void*)&flip_done) < 0) {
        printf("LCD | page flip failed\n");
        return -1;
    }
    if (wait_flip(&flip_done) < 0)
        return -1;

    //交换front/back
    front_ = back_;
    back_ = 1 - back_;
    return 0;
}

int LcdDisplay::wait_flip(volatile bool* done)
{
    drmEventContext ev;
    memset(&ev, 0, sizeof(ev));
    ev.version = DRM_EVENT_CONTEXT_VERSION;
    ev.page_flip_handler = page_flip_handler;

    while (!*done) {
        struct pollfd pfd;
        pfd.fd = fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;

        int r = poll(&pfd, 1, 1000);//1秒超时，防止卡死
        if (r < 0) {
            if (errno == EINTR) continue;
            printf("LCD | poll failed\n");
            return -1;
        }
        if (r == 0) {
            printf("LCD | page flip timeout\n");
            return -1;
        }
        if (pfd.revents & POLLIN)
            drmHandleEvent(fd_, &ev);
    }
    return 0;
}

void LcdDisplay::stop()
{
    free_fb(bufs_[0]);
    free_fb(bufs_[1]);

    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}
