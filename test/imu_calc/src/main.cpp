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
std::map<int, int> write_count; // 每个文件已写入的条数

// 后台线程：监听用户输入数字（1, 2, 3）
void inputThread() {
    while (true) {
        int num;
        std::cin >> num;
        if (num >= 1 && num <= 3) {
            current_file_num.store(num);
        } else {
            std::cout << "请输入 1~3 的数字以记录数据" << std::endl;
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
        imu.readTempture();
        const auto& raw_data = imu.getRawData();

        int file_num = current_file_num.load();

        // 如果选择了新文件
        if (file_num != last_file_num) {
            std::lock_guard<std::mutex> lock(file_mutex);
            if (outfile.is_open()) {
                outfile.close();
            }

            // 如果该文件已写满 100 条，就不打开它
            if (write_count[file_num] >= 100) {
                std::cout << "文件 " << file_num << ".txt 已记录 100 条数据，停止写入。" << std::endl;
                last_file_num = file_num; // 避免重复尝试打开
                continue;
            }

            std::string filename = std::to_string(file_num) + ".txt";
            outfile.open(filename, std::ios::app);
            if (!outfile.is_open()) {
                std::cerr << "无法打开文件: " << filename << std::endl;
            } else {
                std::cout << "开始记录到文件: " << filename << std::endl;
            }
            last_file_num = file_num;
        }

        // 写入数据（仅当未满100条）
        if (outfile.is_open() && write_count[file_num] < 100) {
            std::lock_guard<std::mutex> lock(file_mutex);
            outfile << raw_data.accel_x << " "
                    << raw_data.accel_y << " "
                    << raw_data.accel_z << " "
                    << raw_data.gyro_x << " "
                    << raw_data.gyro_y << " "
                    << raw_data.gyro_z << " "
                    << raw_data.temperature << std::endl;

            write_count[file_num]++;

            // 若刚好写满100条，关闭文件
            if (write_count[file_num] == 100) {
                std::cout << "文件 " << file_num << ".txt 已写满 100 条数据。" << std::endl;
                outfile.close();
            }
        }

        usleep(2000); // 2ms
    }

    return 0;
}
