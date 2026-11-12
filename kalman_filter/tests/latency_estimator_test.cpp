# include "timing/latency_estimator.hpp"

# include <cmath>
# include <iostream>

int main()
{
    kalfilter::LatencyEstimator latency;
    kalfilter::FrameTiming timing;
    timing.exposure_time = 10.0;
    timing.command_ready_time = 10.02;

    latency.recordFrame(timing);
    latency.recordCommunication(0.002);
    latency.recordFlightTime(0.05);

    const auto snapshot = latency.snapshot();
    const bool valid =
        snapshot.valid &&
        std::abs(snapshot.process.median - 0.02) < 1e-9 &&
        std::abs(snapshot.total_delay - 0.072) < 1e-9;

    if (!valid)
    {
        std::cerr << "latency estimator test failed\n";
        return 1;
    }

    std::cout << "latency estimator test passed\n";
    return 0;
}
