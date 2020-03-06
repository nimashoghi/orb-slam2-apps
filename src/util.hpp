#pragma once

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

std::string random_string(const size_t length)
{
    std::string str(length, 0);
    std::generate_n(str.begin(), length, []() -> char {
        const char charset[] =
            "0123456789"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[rand() % max_index];
    });
    return str;
}

inline void wait_for_times()
{
    while (true)
    {
        if (std::ifstream("/start").get() == '1')
        {
            std::cout << "Received start signal! Starting..." << '\n';
            break;
        }

        usleep(100000);
    }
}
