# include "timing/clock_sync.hpp"

# include <cmath>
# include <iostream>

int main()
{
    kalfilter::ClockSync clock(0.002);
    uint64_t board_time = 0;
    double host_time = 0.0;

    if (!clock.observeAndMap(
            4294901760.0F, 1000.0, board_time, host_time) ||
        board_time != 0xFFFF0000ULL ||
        std::abs(host_time - 999.998) > 1e-6)
    {
        std::cerr << "initial clock mapping failed\n";
        return 1;
    }

    const double first_host_time = host_time;
    if (!clock.observeAndMap(
            256.0F, 1000.065792, board_time, host_time) ||
        board_time != (1ULL << 32) + 256ULL ||
        std::abs((host_time - first_host_time) - 0.065792) > 1e-6)
    {
        std::cerr << "clock wrap mapping failed\n";
        return 2;
    }

    if (clock.observeAndMap(
            240.0F, 1000.066, board_time, host_time))
    {
        std::cerr << "out-of-order timestamp accepted\n";
        return 3;
    }

    std::cout << "clock sync test passed\n";
    return 0;
}
