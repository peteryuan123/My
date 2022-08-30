#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <string>

class Timer
{
public:
    Timer(std::string name);
    Timer(std::string name, double init_duration);
    void start();
    void stop();
    double duration();
    double time();
    void reset();
    void show();

protected:
    std::string name_;
    bool running_;
    double total_duration_;
    std::chrono::high_resolution_clock::time_point start_time_;
    double time_;
    int task_num_;
};


#endif //TIMER_H
