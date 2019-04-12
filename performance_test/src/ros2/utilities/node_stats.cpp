#include "performance_test/ros2/node_stats.hpp"

#include <stdint.h>


NodeStats::IterativeStats::IterativeStats()
{
    mean = 0;
    variance = 0;
    count = 0;
    M2 = 0;

    max_v = INT64_MIN;
    min_v = INT64_MAX;
}


void NodeStats::IterativeStats::update(int64_t v)
{

    if (v > max_v){
        max_v = v;
    }
    else if (v < min_v){
        min_v = v;
    }

    int64_t delta = v - this->mean;
    this->M2 += (delta * delta);

    this->mean += delta/(this->count + 1.0);
    this->variance = this->M2 / (this->count + 1.0);

    this->count ++;

}
