# Surveillance-Camera

基于 RK3568 的智能监控摄像头工程：V4L2 采集 → RKNN 人体检测 → MPP 硬编码 H.264 → ZLMediaKit 推流，并在 LCD 上实时预览。

## 功能

- **视频采集**：V4L2，1920×1080 NV12，dma-buf 零拷贝
- **目标检测**：YOLOv8（RKNN，NPU 加速），人体检测 + NMS
- **硬件编码**：MPP H.264
- **推流**：内嵌 ZLMediaKit，支持 RTSP / HTTP-FLV
- **本地预览**：DRM + RGA 双缓冲显示，叠加检测框和实时帧率

## 架构（三线程生产者-消费者）

```
capture ──raw_queue──▶ inference ──latest_dets──┐
   │                                             │
   ├─ 画 OSD到源 NV12                            │
   ├─ LCD 显示                                   │
   └─enc_queue──▶ encoder(编码+推流) ──recycle──▶ 归还 V4L2 buffer
```

检测框和帧率文字只画一次到源 NV12，LCD 和推流共用，两边画面一致。

## 拉流地址

| 协议 | 地址 |
|------|------|
| RTSP | `rtsp://<板子IP>:554/live/camera` |
| HTTP-FLV | `http://<板子IP>:8080/live/camera.live.flv` |

用 VLC 打开上面的 RTSP 地址即可预览。

## 编译

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../rk3568.toolchain.cmake
make
```

依赖：MPP、RGA、libdrm、FreeType、librknnrt、ZLMediaKit。

```

## 说明

- 检测类别为 person（COCO class 0），换模型可通过 `Task_config::model_path`
- LCD 走 RGA 90° 旋转；推流默认不旋转
