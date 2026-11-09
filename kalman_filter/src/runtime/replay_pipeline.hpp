# ifndef AUTOAIM_RUNTIME_REPLAY_PIPELINE_HPP
# define AUTOAIM_RUNTIME_REPLAY_PIPELINE_HPP

# include <string>

# include "core/config.hpp"

namespace realtime
{

class ReplayPipeline
{
public:
    explicit ReplayPipeline(core::AppConfig config = {});

    bool run(std::string* error = nullptr);

    const core::AppConfig& config() const noexcept { return config_; }

private:
    core::AppConfig config_;
};

}   // namespace realtime

# endif
