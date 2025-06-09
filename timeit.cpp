#include "timeit.h"

Timeit::Timeit()
: _description("")
, _running(true)
, _printInfo(false)
, _accumTime(0)
{
    _start = std::chrono::high_resolution_clock::now();
}

Timeit::Timeit(std::string description)
: _description(description)
, _running(true)
, _printInfo(true)
{
    _start = std::chrono::high_resolution_clock::now();
}

Timeit::~Timeit()
{
    if (_running && _printInfo)
        print();
}

void Timeit::start()
{
    _start = std::chrono::high_resolution_clock::now();
    _running = true;
}

void Timeit::stop()
{
    _stop = std::chrono::high_resolution_clock::now();
    _running = false;
}

void Timeit::print(int iterations)
{
    stop();
    std::chrono::duration<float> diff = _stop - _start;
    if (iterations > 0) {
        double time_per_iteration = (diff.count() * 1000) / iterations;
        printf("%s took %f ms per iteration\n", _description.c_str(), time_per_iteration);
    }
    else {
        printf("%s took %f ms\n", _description.c_str(), diff.count() * 1000);
    }
}

double Timeit::getTime()
{
    stop();
    std::chrono::duration<double> diff = _stop - _start;
    return diff.count();
}

void Timeit::accumTime()
{
    stop();
    std::chrono::duration<double> diff = _stop - _start;
    _accumTime += diff.count();
}

double Timeit::getAccumTime()
{
    return _accumTime;
}

