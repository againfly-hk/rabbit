#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

const int board_width = 12;   // 列方向角点数
const int board_height = 8;   // 行方向角点数
const float square_size = 0.02f;  // 每个方格边长为 20mm = 0.02m

const int num_images_to_capture = 15;

int main() {
    raspicam::RaspiCam_Cv camera;
    camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    camera.set(cv::CAP_PROP_FPS, 90);
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
    camera.set(cv::CAP_PROP_EXPOSURE, 5);
    camera.set(cv::CAP_PROP_AUTO_WB, 0);
    camera.set(cv::CAP_PROP_WB_TEMPERATURE, 4000);
    camera.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, 100);
    camera.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 100);
    camera.set(cv::CAP_PROP_BRIGHTNESS, 50);
    camera.set(cv::CAP_PROP_CONTRAST, 50);
    camera.set(cv::CAP_PROP_SATURATION, 50);
    camera.set(cv::CAP_PROP_GAIN, 1);
    camera.set(cv::CAP_PROP_SHARPNESS, 20);
    camera.set(cv::CAP_PROP_MODE, 0);

    if (!camera.open()) {
        std::cerr << "Failed to open camera!" << std::endl;
        return -1;
    }

    std::cout << "Camera opened. Capturing chessboard images..." << std::endl;

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<cv::Point3f> objp;

    for (int i = 0; i < board_height; ++i) {
        for (int j = 0; j < board_width; ++j) {
            objp.emplace_back(j * square_size, i * square_size, 0);
        }
    }

    int captured = 0;
    cv::Mat frame, gray;

    while (captured < num_images_to_capture) {
        camera.grab();
        camera.retrieve(frame);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, cv::Size(board_width, board_height), corners);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            image_points.push_back(corners);
            object_points.push_back(objp);
            captured++;
            std::cout << "Captured image " << captured << "/" << num_images_to_capture << std::endl;
            cv::drawChessboardCorners(frame, cv::Size(board_width, board_height), corners, found);
            cv::imshow("Captured", frame);
            cv::waitKey(500); // 等待 500ms 显示
        }

        cv::imshow("Live", frame);
        if (cv::waitKey(30) == 27) break; // 按下 ESC 退出
    }

    cv::destroyAllWindows();

    std::cout << "Calibrating..." << std::endl;

    cv::Mat camera_matrix, dist_coeffs, R, T;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(object_points, image_points, gray.size(),
                                     camera_matrix, dist_coeffs, rvecs, tvecs);

    std::cout << "Calibration RMS error: " << rms << std::endl;
    std::cout << "Camera Matrix:\n" << camera_matrix << std::endl;
    std::cout << "Distortion Coefficients:\n" << dist_coeffs << std::endl;

    // 保存参数
    cv::FileStorage fs("camera_calibration.yaml", cv::FileStorage::WRITE);
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << dist_coeffs;
    fs.release();

    std::cout << "Calibration saved to camera_calibration.yaml" << std::endl;

    camera.release();
    return 0;
}
