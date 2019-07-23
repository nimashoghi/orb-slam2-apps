#include <algorithm>
#include <cstdlib>
#include <string>

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
