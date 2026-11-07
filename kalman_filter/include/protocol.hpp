# ifndef AUTOAIM_PROTOCOL_
# define AUTOAIM_PROTOCOL_


# include <stdint.h>
# include <array>
# include <cstddef>

namespace comm {

struct __attribute__((packed)) GimbalToVision
{
uint8_t head [2];   // 5A 01
float DWT_stamp;    // DWT 时间，源码标注单位 μS

uint8_t mode;
float q[4];   //四元数，w、x、y、2
float pitch;
float pitch_vel;
float yaw;
float yaw_vel;
float yaw_diff;
float bullet_speed;
uint16_t bullet_count; // 累计发弹次数

uint8_t reserved[12];
uint8_t tail;          //55
uint16_t check_sum;    // 64 字节  
} ; 

struct __attribute__((packed)) VisionToGimbal
{
uint8_t head[2] = {0x5A,0x01};
uint64_t time_stamp;

uint8_t mode;
float pitch;
float pitch_vel;
float pitch_acc;
float yaw;
float yaw_vel;
float yaw_acc;

uint8_t reserved [26];
uint16_t check_sum;
uint8_t tail = 0x55;
};                 // 64 字节

constexpr std::size_t kFrameSize = 64;

bool checkQtoV(const uint8_t* p);   // 累加和
void packVtoQ(uint8_t* p);  // 封口（4.3）
bool parse(const uint8_t* frame, GimbalToVision& out); // 验货（4.2）

class SerialLink {
public:
    bool open(const char* dev, int baud);
    bool send(const VisionToGimbal&);          // 内部 pack + 循环 write
    bool poll(GimbalToVision& out);            // 非阻塞取一帧
    void close();

private:
    int fd_ = -1;
    std::array<uint8_t, 128> rx_buffer_{};
    std::size_t rx_size_ = 0;
};

}

# endif