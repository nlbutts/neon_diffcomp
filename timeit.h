#ifndef _TIMEIT_H_
#define _TIMEIT_H_

#include <chrono>
#include <string>

class Timeit
{
public:
    Timeit();
    Timeit(std::string description);
    ~Timeit();

    void start();
    void stop();
    void print(int iterations=0);

    double getTime();

    void accumTime();
    double getAccumTime();

private:
    std::string _description;
    bool _running;
    bool _printInfo;
    std::chrono::time_point<std::chrono::high_resolution_clock> _start;
    std::chrono::time_point<std::chrono::high_resolution_clock> _stop;
    double _accumTime;
};

#endif // _TIMEIT_H_
