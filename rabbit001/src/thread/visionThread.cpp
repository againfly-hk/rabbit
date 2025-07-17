#include "visionThread.h"
#include "threadAffinity.h"

#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <fstream>

#define GREEN_THRESHOLD 50
#define MIN_RB_DIFF     50
#define MIN_AREA        1

#define X_OFFSET 0
#define Z_OFFSET 0

/* log index */
int     image_index = 0;

/* controller */
uint8_t detectflag  = 0;
int     detecty     = 0;
int     detectz     = 0;

/* double buffer */
uint8_t image_flag = 0;
uint8_t image_ready_flag = 0;
cv::Mat frame0(240, 320, CV_8UC3);
cv::Mat frame1(240, 320, CV_8UC3);
cv::Mat mask(240, 320, CV_8UC1);
std::vector<std::vector<cv::Point>> contours;

/* fly control */
extern uint8_t flyFlag;

void saveImage(const cv::Mat& image) {
    std::ostringstream filename;
    filename << "image_" << std::setw(6) << std::setfill('0') << image_index << ".raw";
    std::ofstream raw_file(filename.str(), std::ios::out | std::ios::binary);
    raw_file.write(reinterpret_cast<const char*>(image.data), image.total() * image.elemSize());
    raw_file.close();
    image_index++;
}

void visionThreadFunc(std::atomic<bool>& keepRunning) {
    setCurrentThreadAffinity(3, "visionThread");
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
    camera.set(cv::CAP_PROP_CONTRAST, 50);
    camera.set(cv::CAP_PROP_SATURATION, 50);
    camera.set(cv::CAP_PROP_SHARPNESS, 50);

    if (!camera.open()) {
        std::cerr << "Failed to open camera!" << std::endl;
        return;
    }

    while (keepRunning.load()) {
        camera.grab();

        if (image_flag == 0) {
            camera.retrieve(frame0);
            if (frame0.empty()) {
                std::cerr << "Failed to retrieve frame!" << std::endl;
                break;
            }

            image_flag = 1;
            image_ready_flag = 1;

            // if (flyFlag) {
            //     saveImage(frame0);
            // }
        } else if (image_flag == 1) {
            camera.retrieve(frame1);
            if (frame1.empty()) {
                std::cerr << "Failed to retrieve frame!" << std::endl;
                break;
            }

            image_flag = 0;
            image_ready_flag = 1;
            // if (flyFlag) {
            //     saveImage(frame1);
            // }
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
            image_ready_flag = 0;
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
                int cz = rect.x + rect.width / 2;
                int cy = rect.y + rect.height / 2;

                detectflag = 0;
                detectz = cz - X_OFFSET;
                detecty = cy - Z_OFFSET;
                detectflag = 1;
            }
            frameCount++;
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() > 1000) {
                fps = frameCount;
                frameCount = 0;
                lastTime = now;
            }
        } else if (image_ready_flag == 1 && image_flag == 0) {
            image_ready_flag = 0;
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
                int cz = rect.x + rect.width / 2;
                int cy = rect.y + rect.height / 2;

                detectflag = 0;
                detectz = cz - X_OFFSET;
                detecty = cy - Z_OFFSET;
                detectflag = 1;
            }
            frameCount++;
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count() > 1000) {
                fps = frameCount;
                frameCount = 0;
                lastTime = now;
            }
        }
    }
}
