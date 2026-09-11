cmake_minimum_required(VERSION 3.10)
project(surveillance)

set(CMAKE_CXX_STANDARD 17)#c++17标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)#不支持就报错
set(CMAKE_POSITION_INDEPENDENT_CODE ON)# 生成位置无关代码

#----路径配置----
set(SYSROOT /opt/atk-dlrk356x-toolchain/aarch64-buildroot-linux-gnu/sysroot)#rk3568根文件系统路径
set(RK3568_SDK /home/xigua-tree/rk3568/SDK)#SDK目录
set(RKNN_TOOL /home/xigua-tree/rk3568/rknn_tool/rknn-toolkit2-2.0.0-beta0)#RKNN神经网络加速工具包路径
#----------------------------

#----RGA头文件----
set(RGA_INC ${RK3568_SDK}/external/linux-rga/im2d_api)
set(RGA_INC2 ${RK3568_SDK}/external/linux-rga/include)
#----------------------------

#----MPP头文件----
set(MPP_INC ${SYSROOT}/usr/include/rockchip)
#----------------------------

#----ZLMediaKit推流库的头文件和动态库----
set(ZLM_INC ${RKNN_TOOL}/rknpu2/examples/3rdparty/zlmediakit/include)
set(ZLM_LIB ${RKNN_TOOL}/rknpu2/examples/3rdparty/zlmediakit/aarch64/libmk_api.so)
#----------------------------

#----RKNN推理库的头文件和动态库----
set(RKNN_INC ${RKNN_TOOL}/rknpu2/runtime/Linux/librknn_api/include)
set(RKNN_LIB ${RKNN_TOOL}/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so)
#----------------------------

#----包含目录----
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${SYSROOT}/usr/include#系统头文件
    ${SYSROOT}/usr/include/libdrm#DRM头文件
    ${SYSROOT}/usr/include/glib-2.0#glib库
    ${SYSROOT}/usr/lib/glib-2.0/include
    ${SYSROOT}/usr/include/freetype2#FreeType文字渲染库
    ${RGA_INC}
    ${RGA_INC2}
    ${MPP_INC}
    ${ZLM_INC}
    ${RKNN_INC}
)
#----------------------------

#----源文件列表----
set(SOURCES
    src/main.cpp
    src/my_task.cpp
    src/capture.cpp
    src/lcd.cpp
    src/inference.cpp
    src/encoder.cpp
)
#----------------------------

add_executable(surveillance ${SOURCES})

#----链接库----
target_link_libraries(surveillance
    librga.so#RGA硬件加速库
    librockchip_mpp.so#MPP硬件编码库
    libdrm.so#DRM显示库
    freetype#FreeType文字渲染库
    ${ZLM_LIB}
    ${RKNN_LIB}
    pthread#线程库
    dl#动态加载库
)
#----END----