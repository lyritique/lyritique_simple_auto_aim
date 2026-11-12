# include "runtime/realtime_pipeline.hpp"

# include "sensors/mvs_camera.hpp"

# include <algorithm>
# include <chrono>
# include <utility>
# include <filesystem>
# include <fstream>

namespace realtime
{
namespace
{

// 作用：拿当前 steady_clock 的秒数，全链路统一的时间尺。实现：取 time_since_epoch 再转成 double 秒。
double steadyNowSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}   // namespace

// 作用：组装整条实时链路，把配置分发到各组件。实现：移动配置，用配置里的延迟、历史时长、帧率参数逐个构造成员。
RealtimePipeline::RealtimePipeline(RealtimePipelineConfig config)
    : config_(std::move(config)),
      clock_(config_.serial.communication_latency),
      history_(
          config_.timing.max_history_seconds,
          config_.timing.max_attitude_gap_seconds,
          config_.timing.max_attitude_prediction_seconds),
      processor_(config_)
{
}

// 作用：对象销毁时别把设备晾着。实现：转调 stop() 收拾线程和串口。
RealtimePipeline::~RealtimePipeline()
{
    stop();
}

// 作用：把链路从冷启动拉到运行态。实现：先全部复位，再开串口、开相机，最后起反馈线程；中途失败原路回滚。
bool RealtimePipeline::start(std::string* error)
{
    // 作用：防止重复启动导致两套线程和设备打架。实现：先 stop 一次，再清错误信息。
    stop();
    last_error_.clear();

    // 作用：清掉上一轮的时钟偏移，重新标定。实现：reset 后重新灌入配置里的通信延迟。
    clock_.reset();
    clock_.setCommLatency(config_.serial.communication_latency);
    // 作用：清空云台历史和延迟统计，就当第一次开机。实现：两个 clear，再把通信延迟当第一条样本记进去。
    history_.clear();
    latency_.clear();
    latency_.recordCommunication(
        config_.serial.communication_latency);

    // 作用：重置滤波器、初始姿态和反馈时间。实现：processor_.reset() 加三个成员归位。
    processor_.reset();
    has_q_init_ = false;
    q_init_.setIdentity();
    last_feedback_receive_time_.store(0.0);

    // 作用：计数器全部归零，统计从新的一段开始。实现：锁内整体赋值一个空 PipelineStats。
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_ = PipelineStats {};
    }

    // 作用：串口路径都没配就别往下走了。实现：empty 检查后调 fail 返回。
    if (config_.serial.device.empty())
    {
        return fail("串口设备路径为空", error);
    }

    // 作用：打开串口，反馈线程就靠它吃饭。实现：SerialLink::open 带设备路径和波特率，失败就报错。
    if (!serial_.open(
            config_.serial.device.c_str(),
            config_.serial.baud))
    {
        return fail(
            "串口打开失败: " + config_.serial.device +
                " @" + std::to_string(config_.serial.baud),
            error);
    }

    // 作用：按配置造出具体相机（实时是 MVS）。实现：make_unique 后调 open，失败就把刚开的串口也关掉。
    camera_ = std::make_unique<camera::MvsCamera>(config_.camera);
    if (!camera_->open(error))
    {
        serial_.close();
        camera_->close();
        return false;
    }

    // 作用：设备都就绪了，才正式宣布运行，再起收包线程。实现：running_ 置真，jthread 里跑 feedbackLoop。
    running_.store(true);
    feedback_thread_ = std::jthread(
        [this](std::stop_token token)
        {
            feedbackLoop(token);
        });
    return true;
}

// 作用：主线程读帧循环，直到外部喊停。实现：相机轮询，超时就再读，出错计数并小睡，成功就交给 processFrame。
void RealtimePipeline::run(
    const std::atomic_bool& external_stop,
    FrameCallback callback)
{
    // 作用：没 start 就别空转。实现：running_ 为假直接返回。
    if (!running_.load()) return;

    std::ofstream csv; 
    std::error_code ec;
    std::filesystem::create_directories(config_.output_directory, ec);

    if (!ec)
    {
        csv.open(config_.output_directory + "/realtime.csv");
        if (csv)
        {
            csv << "time,valid,px,py,pz,yaw,vx,vy,vz,vyaw,r," << "latency_ms,detections,feedback\r\n";
        }
    }

    // 作用：两个停止条件都要看：自己不想跑了，或外部喊停。实现：while 同时检查 running_ 和 external_stop。
    while (running_.load() && !external_stop.load())
    {
        // 作用：准备一个空帧和错误槽，交给相机去填。实现：栈上默认构造。
        core::TimestampedFrame frame;
        std::string error;
        // 作用：向相机要一帧。实现：read 返回状态码，后续分支处理。
        const auto status = camera_->read(frame, &error);
        // 作用：暂时没图不算错误，下一轮再要。实现：Timeout 就 continue，不计数。
        if (status == core::FrameReadStatus::Timeout) continue;
        
        // 作用：真读坏了才计丢帧并留错误，睡 5ms 防止忙等刷屏。实现：锁内自增 dropped_frames，记错误，再 sleep。
        if (status != core::FrameReadStatus::Ok)
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.dropped_frames;
            last_error_ = std::move(error);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(5));
            continue;
        }

        // 作用：好帧就进入加工流水线。实现：直接调 processFrame，回调原样透传。
        const PipelineFrameResult result = processFrame(frame, callback);

        if (csv)
        {
            csv << result.frame_time << ','
                << (result.state_valid ? 1 : 0) << ',';
            for (int i = 0; i < 9; ++i)
            {
                csv << result.state.state(i) << ',';
            }
            csv << result.process_latency << ','
                << result.valid_detections << ','
                << (result.feedback_available ? 1 : 0) << '\r\n';
        }
    }

    // 作用：循环退出说明该收摊了。实现：调 stop() 统一关线程和设备。
    if (csv) csv.close();
    stop();
}

// 作用：不带停止标志的 run 便利重载。实现：造一个永远为假的 atomic，转调带标志的版本。
void RealtimePipeline::run(FrameCallback callback)
{
    std::atomic_bool never_stop {false};
    run(never_stop, std::move(callback));
}

// 作用：把整条链路安全关掉，可重复调用。实现：清运行标志，请求停止并 join 反馈线程，再依次关相机和串口。
void RealtimePipeline::stop()
{
    running_.store(false);

    // 作用：反馈线程还活着就礼貌地请它走。实现：request_stop 后 join，等它真正退出。
    if (feedback_thread_.joinable())
    {
        feedback_thread_.request_stop();
        feedback_thread_.join();
    }

    // 作用：有相机就关，串口也顺手关掉。实现：空指针检查和 close。
    if (camera_) camera_->close();
    serial_.close();
}

// 作用：给上层留的发送口，目前控制层还没用上。实现：串口 send，失败就统一记错误。
bool RealtimePipeline::send(
    const comm::VisionToGimbal& packet,
    std::string* error)
{
    // 作用：发不出去得让人知道。实现：send 返回假就调 fail 打包错误。
    if (!serial_.send(packet))
    {
        return fail("串口发送失败", error);
    }
    return true;
}

// 作用：线程安全地取最新滤波状态。实现：锁内拷贝一份，再返回 valid。
bool RealtimePipeline::latestState(kalfilter::KFResult& state) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    return state.valid;
}

// 作用：取延迟统计快照，内部已经自带锁。实现：转调 latency_.snapshot()。
kalfilter::LatencySnapshot RealtimePipeline::latencySnapshot() const
{
    return latency_.snapshot();
}

// 作用：取计数器快照。实现：锁内整体拷贝 stats_，避免读到改了一半。
PipelineStats RealtimePipeline::stats() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

// 作用：把一帧从像素加工到滤波状态，是整条链路的正餐。实现：等姿态、算相对旋转、跑处理器、发布状态、记延迟、触发回调。
PipelineFrameResult RealtimePipeline::processFrame(
    const core::TimestampedFrame& frame,
    const FrameCallback& callback)
{
    // 作用：先备好结果容器，填上帧时间。实现：栈上默认构造后赋 timestamp。
    PipelineFrameResult result;
    result.frame_time = frame.timestamp;

    // 作用：处理数先加一，成败都算处理过。实现：锁内自增 processed_frames。
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.processed_frames;
    }

    // 作用：去历史里找这一帧时刻的云台姿态。实现：调 waitForAttitude，带超时等待。
    kalfilter::GimbalSample attitude;
    // 作用：等不到就丢帧，别拿旧姿态凑合。实现：返回假就计数 dropped_frames 并原样返回空结果。
    if (!waitForAttitude(frame.timestamp, attitude))
    {
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.dropped_frames;
        }
        return result;
    }

    // 作用：姿态到手，标记本帧反馈可用。实现：置 true。
    result.feedback_available = true;

    // 作用：第一次成功拿到姿态就把它当世界系基准。实现：没初始化过就存 q_init_ 并置标志。
    if (!has_q_init_)
    {
        q_init_ = attitude.q;
        has_q_init_ = true;
    }

    // 作用：算从初始姿态到当前姿态的相对旋转，算法世界 C0 就建立在这上面。实现：共轭乘当前姿态再单位化。
    const Eigen::Quaterniond q_relative =
        (q_init_.conjugate() * attitude.q).normalized();
    // 作用：把四元数变成旋转矩阵，PnP 结果转世界系要用。实现：toRotationMatrix。
    const Eigen::Matrix3d R_C0C =
        q_relative.toRotationMatrix();

    // 作用：把帧和旋转交给下游处理器，走检测、PnP、EKF。实现：一次 process 调用全套。
    const FrameProcessingResult processing =
        processor_.process(frame, R_C0C);
    // 作用：把有效检测数、滤波状态、可信标志搬进对外结果。实现：逐字段赋值，检测数取 observations 大小。
    result.valid_detections = static_cast<int>(
        processing.observations.size());
    result.state = processing.state;
    result.state_valid = processing.state_valid;

    // 作用：好状态就发布出去，供 UI 或控制层取用。实现：valid 时锁内覆盖 latest_state_。
    if (result.state_valid)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = result.state;
    }

    // 作用：统计有效帧数，只在滤波器活着且状态可信时加。实现：先看 ekf_initialized，再看 state_valid，锁内自增。
    if (processing.ekf_initialized)
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (result.state_valid) ++stats_.valid_frames;
    }

    // 作用：记下命令可发的时刻，算出本帧总处理延迟。实现：取当前 steady 时间减帧时间戳。
    const double command_ready_time = steadyNowSeconds();
    result.process_latency =
        command_ready_time - frame.timestamp;

    // 作用：凑一份打点交给延迟统计，目前只填了首尾和到达时间。实现：构造 FrameTiming 后调 recordFrame。
    kalfilter::FrameTiming timing;
    timing.exposure_time = frame.timestamp;
    timing.frame_received_time = frame.timestamp;
    timing.command_ready_time = command_ready_time;
    latency_.recordFrame(timing);

    // 作用：有回调就通知一声，让外面画图或发指令。实现：std::function 非空才调用。
    if (callback)
    {
        callback(frame, result);
    }

    return result;
}

// 作用：等云台历史覆盖到目标时刻，最多等配置的毫秒数。实现：循环尝试插值，没成且没停就睡 1ms，到点或停机即认输。
bool RealtimePipeline::waitForAttitude(
    double host_time,
    kalfilter::GimbalSample& sample)
{
    // 作用：算出等待截止时刻，等待时长从配置来，负数按 0 处理。实现：now 加上转换成 steady duration 的毫秒数。
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(
                std::max(0.0, config_.timing.attitude_wait_ms)));

    // 作用：反复尝试，直到拿到姿态、停机或超时。实现：无限循环里三个出口。
    for (;;)
    {
        // 作用：历史已经覆盖目标时刻，直接成功。实现：interpolate 返回真就 return true。
        if (history_.interpolate(host_time, sample)) return true;
        // 作用：链路都停了就别等了。实现：running_ 为假返回 false。
        if (!running_.load()) return false;
        // 作用：等到头还没来，只能放弃这帧。实现：当前时间超过 deadline 返回 false。
        if (std::chrono::steady_clock::now() >= deadline) return false;
        // 作用：别死循环烧 CPU，小睡 1ms 再问。实现：sleep_for 一毫秒。
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }
}

// 作用：后台线程专收 C 板反馈，收一条处理一条。实现：poll 循环把串口缓冲抽干，没收到包就睡 1ms，直到停机。
void RealtimePipeline::feedbackLoop(std::stop_token stop_token)
{
    // 作用：复用同一个包对象，省得每轮构造。实现：循环外声明。
    comm::GimbalToVision packet;

    // 作用：只要还在跑且没被请求停止就继续收。实现：while 同时检查 running_ 和 stop_token。
    while (running_.load() && !stop_token.stop_requested())
    {
        // 作用：记录这一轮有没有收到东西，决定要不要睡。实现：bool 标志。
        bool received = false;
        // 作用：把串口里积压的包一口气全取出来。实现：while poll 成功就处理，直到没包可读。
        while (serial_.poll(packet))
        {
            received = true;
            handleFeedback(packet);
        }

        // 作用：空转时让出 CPU，1ms 后再说。实现：没收到包就 sleep_for。
        if (!received)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));
        }
    }
}

// 作用：把一条原始反馈包翻译成云台样本并入历史。实现：先做时钟同步，再逐字段拷贝，最后 push 成功才计数。
void RealtimePipeline::handleFeedback(
    const comm::GimbalToVision& packet)
{
    // 作用：记下包到达本机的时刻，时钟同步要用它倒推事件时间。实现：取当前 steady 秒数。
    const double receive_time = steadyNowSeconds();

    // 作用：准备接收同步结果：展开后的板卡微秒和对应的上位机时刻。实现：两个局部变量，由 observeAndMap 填。
    uint64_t board_time_us {};
    double event_host_time {};
    // 作用：同步失败说明戳有问题（乱序、越界），这条反馈不要也罢。实现：返回假直接 return。
    if (!clock_.observeAndMap(
            packet.DWT_stamp,
            receive_time,
            board_time_us,
            event_host_time))
    {
        return;
    }

    // 作用：把包里的字段拼成历史样本。实现：先建对象，再逐字段赋值，四元数按 wxyz 顺序构造。
    kalfilter::GimbalSample sample;
    sample.host_time = event_host_time;
    sample.board_time_us = board_time_us;
    sample.q = Eigen::Quaterniond(
        packet.q[0],
        packet.q[1],
        packet.q[2],
        packet.q[3]);
    sample.pitch = packet.pitch;
    sample.pitch_vel = packet.pitch_vel;
    sample.yaw = packet.yaw;
    sample.yaw_vel = packet.yaw_vel;
    sample.bullet_speed = packet.bullet_speed;
    sample.mode = packet.mode;

    // 作用：入库失败一般是时间没前进，直接丢弃，不入账。实现：push 返回假就 return。
    if (!history_.push(sample)) return;

    // 作用：更新“最近收到反馈”的时刻，顺便累计收包数。实现：atomic store 加锁内自增。
    last_feedback_receive_time_.store(receive_time);
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.received_feedback;
}

// 作用：统一处理失败路径，记录并上报错误。实现：写 last_error_，外部指针非空就一并拷过去。
bool RealtimePipeline::fail(
    const std::string& message,
    std::string* error)
{
    last_error_ = message;
    if (error) *error = message;
    return false;
}

}   // namespace realtime
