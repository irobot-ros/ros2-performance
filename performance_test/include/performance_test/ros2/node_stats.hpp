#ifndef __PERFORMANCE_TESTS_NODE_STATS_HPP__
#define __PERFORMANCE_TESTS_NODE_STATS_HPP__

#include <stdint.h>
#include <map>
#include <string>


struct NodeStats{

    struct IterativeStats{
        int64_t mean;
        int64_t variance;
        int64_t max_v;
        int64_t min_v;
        int64_t count;
        int64_t M2;


        IterativeStats();


        /**
         * IterativeStats members update
         * running mean and variance computed using Welford's algorithm
         * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford%27s_Online_algorithm
         */
        void update(int64_t v);

    };

    uint64_t start_time;

    std::map<int, IterativeStats> durations_map;
    std::map<int, int> spin_frequency_map;
    std::map<int, int> send_frequency_map;
    std::map<int, int64_t> first_message_delta;


};


#endif // __PERFORMANCE_TESTS_NODE_STATS_HPP__