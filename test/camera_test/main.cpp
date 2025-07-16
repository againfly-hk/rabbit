#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

const uchar GREEN_THRESHOLD = 200;
const uchar MIN_RB_DIFF = 100;
const int MIN_AREA = 1;

cv::Mat frame(240, 320, CV_8UC3);
cv::Mat mask(240, 320, CV_8UC1);

int main() {
    raspicam::RaspiCam_Cv camera;
    camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    camera.set(cv::CAP_PROP_FPS, 90);

    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
    camera.set(cv::CAP_PROP_EXPOSURE, 0);

    camera.set(cv::CAP_PROP_AUTO_WB, 0);
    camera.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, 100);
    camera.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 100);
    camera.set(cv::CAP_PROP_WB_TEMPERATURE, 4000);

    camera.set(cv::CAP_PROP_GAIN, 1);

    camera.set(cv::CAP_PROP_BRIGHTNESS, 50);
    camera.set(cv::CAP_PROP_CONTRAST, 0);
    camera.set(cv::CAP_PROP_SATURATION, 0);
    camera.set(cv::CAP_PROP_SHARPNESS, 0);

    camera.set(cv::CAP_PROP_MODE, 0);

    if (!camera.open()) {
        std::cerr << "无法打开摄像头!" << std::endl;
        return -1;
    }

    std::vector<std::vector<cv::Point>> contours;

    int frameCount = 0, fps = 0;
    auto lastTime = std::chrono::high_resolution_clock::now();

    while (true) {
        camera.grab();
        camera.retrieve(frame);
        if (frame.empty()) {
            std::cerr << "获取帧失败!" << std::endl;
            break;
        }

        uchar* imgData = frame.data;
        uchar* maskData = mask.data;
        int totalPixels = frame.rows * frame.cols;

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

            // 仅打印，不显示图像
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

        // 注释掉GUI显示以提升性能
        cv::imshow("Feed", frame);
        if (cv::waitKey(1) == 27) break;
    }

    camera.release();
    return 0;
}
