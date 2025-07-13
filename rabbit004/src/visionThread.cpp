#include "visionThread.h"
#include "threadAffinity.h"

#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <fstream>

#define GREEN_THRESHOLD 200
#define MIN_RB_DIFF     100
#define MIN_AREA        1

uint8_t image_flag = 0;
uint8_t image_ready_flag = 0;
int image_index = 0;

cv::Mat frame0(240, 320, CV_8UC3);
cv::Mat frame1(240, 320, CV_8UC3);
cv::Mat mask(240, 320, CV_8UC1);
std::vector<std::vector<cv::Point>> contours;

void visionThreadFunc(std::atomic<bool>& keepRunning) {
    setCurrentThreadAffinity(3, "visionThread");
    raspicam::RaspiCam_Cv camera;
    camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);           // Image format: color
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);          // Image width
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);         // Image height
    camera.set(cv::CAP_PROP_FPS, 90);                   // Frame rate
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);          // Auto exposure (depends on driver support)
    camera.set(cv::CAP_PROP_EXPOSURE, 1);               // Exposure value
    camera.set(cv::CAP_PROP_AUTO_WB, 0);                // Auto white balance (depends on driver support)
    camera.set(cv::CAP_PROP_WB_TEMPERATURE, 4000);      // White balance temperature (depends on driver support)
    camera.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, 50);   // White balance for red channel
    camera.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 50);  // White balance for blue channel
    camera.set(cv::CAP_PROP_BRIGHTNESS, 50);            // Brightness
    camera.set(cv::CAP_PROP_CONTRAST, 50);              // Contrast
    camera.set(cv::CAP_PROP_SATURATION, 50);            // Saturation
    camera.set(cv::CAP_PROP_GAIN, 1);                   // Gain
    camera.set(cv::CAP_PROP_SHARPNESS, 20);             // Sharpness (depends on driver support)
    camera.set(cv::CAP_PROP_MODE, 0);                   // Mode
    camera.setRotation(0);                              // 180-degree rotation
    camera.setImageEffect(0);                           // Normal image (no effects)
    camera.setVideoStabilization(false);                // Disable video stabilization
    camera.setExposureCompensation(0);                  // Exposure compensation
    camera.setAWB(0);                                   // Auto white balance
    camera.setMetering(0);                              // Metering mode
    camera.setHorizontalFlip(false);                    // Horizontal flip
    camera.setVerticalFlip(false);                      // Vertical flip

    if (!camera.open()) {
        std::cerr << "Failed to open camera!" << std::endl;
        return;
    }

    while (keepRunning.load()) {
        camera.grab();
        std::ostringstream filename;
        filename << "image_" << std::setw(6) << std::setfill('0') << image_index << ".raw";

        if (image_flag == 0) {
            camera.retrieve(frame0);
            if (frame0.empty()) {
                std::cerr << "Failed to retrieve frame!" << std::endl;
                break;
            }

            std::ofstream raw_file(filename.str(), std::ios::out | std::ios::binary);
            raw_file.write(reinterpret_cast<const char*>(frame0.data), frame0.total() * frame0.elemSize());
            raw_file.close();
            image_index++;

            image_flag = 1;
            image_ready_flag = 1;
        } else if (image_flag == 1) {
            camera.retrieve(frame1);
            if (frame1.empty()) {
                std::cerr << "Failed to retrieve frame!" << std::endl;
                break;
            }

            std::ofstream raw_file(filename.str(), std::ios::out | std::ios::binary);
            raw_file.write(reinterpret_cast<const char*>(frame1.data), frame1.total() * frame1.elemSize());
            raw_file.close();
            image_index++;

            image_flag = 0;
            image_ready_flag = 1;
        }
    }
    camera.release();
}

void imageProcessingThreadFunc(std::atomic<bool>& keepRunning) {
    setCurrentThreadAffinity(3, "imageProcessingThread");

    int frameCount = 0;
    int fps = 0;
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

                //std::cout << "Center: (" << cx << ", " << cy << ")" << std::endl;
            }

            frameCount++;
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() > 1000) {
                fps = frameCount;
                frameCount = 0;
                lastTime = now;
                //std::cout << "FPS: " << fps << std::endl;
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

                //std::cout << "Center: (" << cx << ", " << cy << ")" << std::endl;
            }

            frameCount++;
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() > 1000) {
                fps = frameCount;
                frameCount = 0;
                lastTime = now;
                //std::cout << "FPS: " << fps << std::endl;
            }
        }
    }
}
