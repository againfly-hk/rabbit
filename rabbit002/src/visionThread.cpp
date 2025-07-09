#include "visionThread.h"

#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cstdint>

#define GREEN_THRESHOLD 200
#define MIN_RB_DIFF     100
#define MIN_AREA        1

uint8_t image_flag = 0;
uint8_t image_ready_flag = 0;

cv::Mat frame0(240, 320, CV_8UC3);
cv::Mat frame1(240, 320, CV_8UC3);
cv::Mat mask(240, 320, CV_8UC1);
std::vector<std::vector<cv::Point>> contours;

void bindThreadToCPU(int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);

    pthread_t current_thread = pthread_self();
    int rc = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
    } else {
        std::cout << "Vision thread bound to CPU " << cpu_id << std::endl;
    }
}

void visionThreadFunc(std::atomic<bool>& keepRunning) {
    bindThreadToCPU(0);
    raspicam::RaspiCam_Cv camera;
    camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);           // 图像格式：彩色
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);          // 图像宽度
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);         // 图像高度
    camera.set(cv::CAP_PROP_FPS, 120);                  // 帧率
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);          // 自动曝光（部分驱动支持）
    camera.set(cv::CAP_PROP_EXPOSURE, 1);               // 曝光值
    camera.set(cv::CAP_PROP_AUTO_WB, 0);                // 自动白平衡（部分驱动支持）
    camera.set(cv::CAP_PROP_WB_TEMPERATURE, 4000);      // 白平衡色温（部分驱动支持）
    camera.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, 50);   // 红通道白平衡
    camera.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 50);  // 蓝通道白平衡
    camera.set(cv::CAP_PROP_BRIGHTNESS, 50);            // 亮度
    camera.set(cv::CAP_PROP_CONTRAST, 70);              // 对比度
    camera.set(cv::CAP_PROP_SATURATION, 80);            // 饱和度
    camera.set(cv::CAP_PROP_GAIN, 1);                   // 增益
    camera.set(cv::CAP_PROP_SHARPNESS, 20);             // 锐度（部分驱动支持）
    camera.set(cv::CAP_PROP_MODE, 0);                   // 模式
    camera.setRotation(2);                              // 180度旋转
    camera.setImageEffect(0);                           // 普通图像（无特效）
    camera.setVideoStabilization(false);                 // 开启视频防抖
    camera.setExposureCompensation(0);                  // 曝光补
    camera.setAWB(0);                                   // 自动白平衡
    camera.setMetering(0);                              // 测光模式
    camera.setHorizontalFlip(false);                     // 左右翻转
    camera.setVerticalFlip(false);                       // 上下翻转

    if (!camera.open()) {
        std::cerr << "无法打开摄像头!" << std::endl;
        return;
    }

    while (keepRunning.load()) {
        camera.grab();
        if (image_flag == 0) {
            camera.retrieve(frame0);
            if (frame0.empty()) {
                std::cerr << "获取帧失败!" << std::endl;
                break;
            }
            image_flag = 1;
            image_ready_flag = 1;
        } else if (image_flag == 1) {
            camera.retrieve(frame1);
            if (frame1.empty()) {
                std::cerr << "获取帧失败!" << std::endl;
                break;
            }
            image_flag = 0;
            image_ready_flag = 1;
        }
    }
    camera.release();
}

void imageProcessingThreadFunc(std::atomic<bool>& keepRunning) {
    bindThreadToCPU(1);

    int  frameCount = 0;
    int  fps = 0;
    auto lastTime = std::chrono::high_resolution_clock::now();
    while (keepRunning.load()) {
        if (image_ready_flag == 1 && image_flag == 1) {
            uchar* imgData = frame0.data;
            uchar* maskData = mask.data;
            int totalPixels = frame0.rows * frame0.cols;

            for (int i = 0; i < totalPixels; i++) {
                uchar b = imgData[i * 3];
                uchar g = imgData[i * 3 + 1];
                uchar r = imgData[i * 3 + 2];
                maskData[i] = (g > GREEN_THRESHOLD && g - r > MIN_RB_DIFF && g - b > MIN_RB_DIFF) ? 255 : 0;
            }

            contours.clear();
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& contour : contours) {
                if (cv::contourArea(contour) < MIN_AREA) continue;

                cv::Rect rect = cv::boundingRect(contour);
                int cx = rect.x + rect.width / 2;
                int cy = rect.y + rect.height / 2;

                std::cout << "中心: (" << cx << ", " << cy << ")" << std::endl;
            }

            frameCount++;
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() > 1000) {
                fps = frameCount;
                frameCount = 0;
                lastTime = now;
                std::cout << "FPS: " << fps << std::endl;
            }
        } else if (image_ready_flag == 1 && image_flag == 0) {
            uchar* imgData = frame1.data;
            uchar* maskData = mask.data;
            int totalPixels = frame1.rows * frame1.cols;

            for (int i = 0; i < totalPixels; i++) {
                uchar b = imgData[i * 3];
                uchar g = imgData[i * 3 + 1];
                uchar r = imgData[i * 3 + 2];
                maskData[i] = (g > GREEN_THRESHOLD && g - r > MIN_RB_DIFF && g - b > MIN_RB_DIFF) ? 255 : 0;
            }

            contours.clear();
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& contour : contours) {
                if (cv::contourArea(contour) < MIN_AREA) continue;

                cv::Rect rect = cv::boundingRect(contour);
                int cx = rect.x + rect.width / 2;
                int cy = rect.y + rect.height / 2;

                std::cout << "中心: (" << cx << ", " << cy << ")" << std::endl;
            }

            frameCount++;
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() > 1000) {
                fps = frameCount;
                frameCount = 0;
                lastTime = now;
                std::cout << "FPS: " << fps << std::endl;
            }
        }
    }
}
