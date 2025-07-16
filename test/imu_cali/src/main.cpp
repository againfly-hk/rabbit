#include "bmi088.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>

std::atomic<int> current_file_num(0);
std::mutex file_mutex;
std::map<int, int> write_count;

void inputThread() {
    while (true) {
        int num;
        std::cin >> num;
        if (num >= 1 && num <= 6) {
            current_file_num.store(num);
        } else {
            std::cout << "Please enter a number between 1 and 6 to start recording data." << std::endl;
        }
    }
}

int main() {
    BMI088 imu;

    std::ofstream outfile;
    int last_file_num = -1;

    std::thread user_input_thread(inputThread);
    user_input_thread.detach();

    while (true) {
        imu.readAccel();
        imu.readGyro();
        imu.readTemperature();

        int file_num = current_file_num.load();

        if (file_num != last_file_num) {
            std::lock_guard<std::mutex> lock(file_mutex);
            if (outfile.is_open()) {
                outfile.close();
            }

            if (write_count[file_num] >= 100) {
                std::cout << "File " << file_num << ".txt has already recorded 100 entries. Writing stopped." << std::endl;
                last_file_num = file_num;
                continue;
            }

            std::string filename = std::to_string(file_num) + ".txt";
            outfile.open(filename, std::ios::app);
            if (!outfile.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
            } else {
                std::cout << "Start recording to file: " << filename << std::endl;
            }
            last_file_num = file_num;
        }

        if (outfile.is_open() && write_count[file_num] < 100) {
            std::lock_guard<std::mutex> lock(file_mutex);
            outfile << imu.real_data.accel_x << " "
                    << imu.real_data.accel_y << " "
                    << imu.real_data.accel_z << " "
                    << imu.real_data.gyro_x << " "
                    << imu.real_data.gyro_y << " "
                    << imu.real_data.gyro_z << " "
                    << imu.real_data.temperature << std::endl;

            write_count[file_num]++;

            if (write_count[file_num] == 100) {
                std::cout << "File " << file_num << ".txt has reached 100 entries." << std::endl;
                outfile.close();
            }
        }

        usleep(2000);
    }

    return 0;
}
