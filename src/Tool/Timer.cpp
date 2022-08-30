//
// Created by mpl on 22-7-7.
//

#include "Tool/Timer.h"
#include <iostream>

Timer::Timer(std::string name):total_duration_(0), running_(false), time_(0), task_num_(0), name_(std::move(name)){}

Timer::Timer(std::string name, double init_duration):total_duration_(init_duration), running_(false), time_(0),
task_num_(0), name_(std::move(name)){}

void Timer::start()
{
    if (running_)
    {
        std::cout << "Error: Timer is running...\n";
        return;
    }
    running_ = true;
    start_time_ = std::chrono::high_resolution_clock::now();
}

void Timer::stop()
{
    if (!running_)
    {
        std::cout << "Error: Timer is not running...\n";
        return;
    }
    auto stop_time = std::chrono::high_resolution_clock::now();
    time_ = std::chrono::duration<double>(stop_time - start_time_).count();
    running_ = false;
    total_duration_ += time_;
    task_num_ += 1;
}

double Timer::time()
{
    return time_;
}

double Timer::duration()
{
    return total_duration_;
}
void Timer::reset()
{
    running_ = false;
    total_duration_ = 0;
    time_ = 0;
    task_num_ = 0;
}

void Timer::show()
{
    std::cout << "----------Timer " << name_ << " Info Start----------\n";
    std::cout << "Total duration: " << total_duration_ << "(s)\n";
    std::cout << "Last task time: " << time_ << "(s)\n";
    std::cout << "Task num: " << task_num_ << "\n";
    std::cout << "----------Timer " << name_ << " Info End----------\n";
}

