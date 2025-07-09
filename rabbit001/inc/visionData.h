#ifndef VISION_DATA_H
#define VISION_DATA_H

#include <mutex>
#include <condition_variable>
#include <queue>

struct Point {
    int x;
    int y;
};

class VisionQueue {
public:
    void push(const Point& p) {
        std::unique_lock<std::mutex> lock(mtx);
        queue.push(p);
        cv.notify_one();
    }

    Point wait_and_pop() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !queue.empty(); });
        Point p = queue.front();
        queue.pop();
        return p;
    }

    bool empty() {
        std::unique_lock<std::mutex> lock(mtx);
        return queue.empty();
    }

private:
    std::mutex mtx;
    std::condition_variable cv;
    std::queue<Point> queue;
};

#endif