#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {
    raspicam::RaspiCam_Cv camera;

    // 设置分辨率和帧率
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    camera.set(cv::CAP_PROP_FPS, 120);

    // 使用YUV422格式，减少ISP干预
    camera.set(cv::CAP_PROP_FORMAT, CV_8UC2);

    // 关闭自动曝光和自动白平衡，减少ISP自动处理
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
    camera.set(cv::CAP_PROP_EXPOSURE, 1);

    camera.set(cv::CAP_PROP_AUTO_WB, 0);
    camera.set(cv::CAP_PROP_WB_TEMPERATURE, 4000);

    if (!camera.open()) {
        std::cerr << "无法打开摄像头！" << std::endl;
        return -1;
    }

    std::cout << "摄像头打开成功，开始采集..." << std::endl;

    cv::Mat frame;
    auto start = std::chrono::steady_clock::now();
    int frameCount = 0;

    while (true) {
        camera.grab();
        camera.retrieve(frame);

        if (frame.empty()) {
            std::cerr << "捕获帧失败！" << std::endl;
            break;
        }

        frameCount++;

        auto now = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (diff >= 1) {
            std::cout << "FPS: " << frameCount / diff << std::endl;
            start = now;
            frameCount = 0;
        }

        // 按ESC退出
        if (cv::waitKey(1) == 27) break;
    }

    camera.release();
    return 0;
}
