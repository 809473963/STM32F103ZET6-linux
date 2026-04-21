#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr float kPMin = -12.5f;
constexpr float kPMax = 12.5f;
// DM4310 (dmcan官方映射): P=12.5, V=30, T=10
constexpr float kVMin = -30.0f;
constexpr float kVMax = 30.0f;
constexpr float kTMin = -10.0f;
constexpr float kTMax = 10.0f;
constexpr float kKpMin = 0.0f;
constexpr float kKpMax = 500.0f;
constexpr float kKdMin = 0.0f;
constexpr float kKdMax = 5.0f;

constexpr uint8_t kEnableLastByte = 0xFC;
constexpr uint8_t kDisableLastByte = 0xFD;
constexpr uint8_t kSaveZeroLastByte = 0xFE;

struct MitFeedback {
    bool valid{false};
    int motor_id{0};
    int err{0};
    float pos{0.0f};
    float vel{0.0f};
    float tau{0.0f};
    int mos_temp{0};
    int rotor_temp{0};
};

struct RawCanFrame {
    bool valid{false};
    uint32_t can_id{0};
    uint8_t dlc{0};
    std::array<uint8_t, 8> data{};
    uint64_t seq{0};
};

std::string bytes_to_hex(const std::array<uint8_t, 8> &data, uint8_t dlc) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (uint8_t i = 0; i < dlc && i < 8; ++i) {
        oss << std::setw(2) << static_cast<int>(data[i]);
        if (i + 1 < dlc) {
            oss << ' ';
        }
    }
    return oss.str();
}

const char *err_to_text(int err) {
    switch (err) {
        case 0x0: return "disabled";
        case 0x1: return "enabled";
        case 0x8: return "over_voltage";
        case 0x9: return "under_voltage";
        case 0xA: return "over_current";
        case 0xB: return "mos_over_temp";
        case 0xC: return "coil_over_temp";
        case 0xD: return "comm_lost";
        case 0xE: return "overload";
        default: return "unknown";
    }
}

std::string win_error_text(DWORD err) {
    LPSTR buffer = nullptr;
    DWORD n = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        err,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        reinterpret_cast<LPSTR>(&buffer),
        0,
        nullptr
    );
    std::string out = (n && buffer) ? std::string(buffer, n) : "Unknown error";
    if (buffer) {
        LocalFree(buffer);
    }
    while (!out.empty() && (out.back() == '\r' || out.back() == '\n' || out.back() == ' ')) {
        out.pop_back();
    }
    return out;
}

std::vector<std::string> enum_serial_ports() {
    std::vector<std::string> ports;
    for (int i = 1; i <= 64; ++i) {
        std::string name = "COM" + std::to_string(i);
        char target[256] = {0};
        if (QueryDosDeviceA(name.c_str(), target, static_cast<DWORD>(sizeof(target))) != 0) {
            ports.push_back(name);
        }
    }
    return ports;
}

class SerialPort {
public:
    ~SerialPort() {
        close();
    }

    bool open(const std::string &port_name, int baudrate) {
        close();

        std::string device = "\\\\.\\" + port_name;
        handle_ = CreateFileA(
            device.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr
        );

        if (handle_ == INVALID_HANDLE_VALUE) {
            DWORD err = GetLastError();
            std::cerr << "[ERR] 打开串口失败: " << port_name
                      << " (WinErr=" << err << ", " << win_error_text(err) << ")\n";
            auto ports = enum_serial_ports();
            std::cerr << "[INFO] 当前可见串口:";
            if (ports.empty()) {
                std::cerr << " (无)\n";
            } else {
                std::cerr << " ";
                for (size_t i = 0; i < ports.size(); ++i) {
                    std::cerr << ports[i] << (i + 1 < ports.size() ? ", " : "\n");
                }
            }
            return false;
        }

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(handle_, &dcb)) {
            std::cerr << "[ERR] GetCommState 失败\n";
            close();
            return false;
        }

        dcb.BaudRate = static_cast<DWORD>(baudrate);
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;

        if (!SetCommState(handle_, &dcb)) {
            std::cerr << "[ERR] SetCommState 失败\n";
            close();
            return false;
        }

        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout = 1;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 2;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 50;
        if (!SetCommTimeouts(handle_, &timeouts)) {
            std::cerr << "[ERR] SetCommTimeouts 失败\n";
            close();
            return false;
        }

        PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
        return true;
    }

    bool is_open() const {
        return handle_ != INVALID_HANDLE_VALUE;
    }

    void close() {
        if (handle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
        }
    }

    bool write_bytes(const uint8_t *data, size_t len) {
        if (!is_open()) {
            return false;
        }
        DWORD written = 0;
        if (!WriteFile(handle_, data, static_cast<DWORD>(len), &written, nullptr)) {
            return false;
        }
        return written == len;
    }

    int read_bytes(uint8_t *data, size_t max_len) {
        if (!is_open()) {
            return -1;
        }
        DWORD read_len = 0;
        if (!ReadFile(handle_, data, static_cast<DWORD>(max_len), &read_len, nullptr)) {
            return -1;
        }
        return static_cast<int>(read_len);
    }

private:
    HANDLE handle_{INVALID_HANDLE_VALUE};
};

float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    x = clampf(x, x_min, x_max);
    const float span = x_max - x_min;
    const float max_int = static_cast<float>((1u << bits) - 1u);
    return static_cast<uint16_t>((x - x_min) * max_int / span);
}

float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    const float span = x_max - x_min;
    const float max_int = static_cast<float>((1u << bits) - 1u);
    return static_cast<float>(x) * span / max_int + x_min;
}

std::array<uint8_t, 8> pack_mit(float p_des, float v_des, float kp, float kd, float t_ff) {
    const uint16_t p_uint = float_to_uint(p_des, kPMin, kPMax, 16);
    const uint16_t v_uint = float_to_uint(v_des, kVMin, kVMax, 12);
    const uint16_t kp_uint = float_to_uint(kp, kKpMin, kKpMax, 12);
    const uint16_t kd_uint = float_to_uint(kd, kKdMin, kKdMax, 12);
    const uint16_t t_uint = float_to_uint(t_ff, kTMin, kTMax, 12);

    return {
        static_cast<uint8_t>((p_uint >> 8) & 0xFF),
        static_cast<uint8_t>(p_uint & 0xFF),
        static_cast<uint8_t>((v_uint >> 4) & 0xFF),
        static_cast<uint8_t>(((v_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)),
        static_cast<uint8_t>(kp_uint & 0xFF),
        static_cast<uint8_t>((kd_uint >> 4) & 0xFF),
        static_cast<uint8_t>(((kd_uint & 0x0F) << 4) | ((t_uint >> 8) & 0x0F)),
        static_cast<uint8_t>(t_uint & 0xFF)
    };
}

bool parse_feedback_8bytes(const std::array<uint8_t, 8> &d, MitFeedback &fb) {
    // DM4310反馈与dmcan一致: data[0]=ID|ERR<<4, q:[1..2], dq:[3..4高4], tau:[4低4..5]
    const int err = (d[0] >> 4) & 0x0F;
    const int motor_id_low4 = d[0] & 0x0F;
    const uint16_t p_uint = static_cast<uint16_t>((d[1] << 8) | d[2]);
    const uint16_t v_uint = static_cast<uint16_t>((d[3] << 4) | (d[4] >> 4));
    const uint16_t t_uint = static_cast<uint16_t>(((d[4] & 0x0F) << 8) | d[5]);

    fb.valid = true;
    fb.err = err;
    fb.motor_id = motor_id_low4;
    fb.pos = uint_to_float(p_uint, kPMin, kPMax, 16);
    fb.vel = uint_to_float(v_uint, kVMin, kVMax, 12);
    fb.tau = uint_to_float(t_uint, kTMin, kTMax, 12);
    fb.mos_temp = static_cast<int>(d[6]);
    fb.rotor_temp = static_cast<int>(d[7]);
    return true;
}

class PcanBridge {
public:
    ~PcanBridge() {
        close();
    }

    bool open(int channel_index) {
        close();

        dll_ = LoadLibraryA("PCANBasic.dll");
        if (!dll_) {
            std::cerr << "[ERR] 未找到 PCANBasic.dll，请安装 PEAK PCAN 驱动。\n";
            return false;
        }

        if (!load_symbols()) {
            std::cerr << "[ERR] PCANBasic.dll 符号加载失败。\n";
            close();
            return false;
        }

        // PCAN_USBBUS1=0x51, PCAN_USBBUS2=0x52 ...
        handle_ = static_cast<uint16_t>(0x51 + (channel_index - 1));
        constexpr uint16_t kPcanBaud1M = 0x0014;

        // 清理当前进程里可能残留的PCAN会话。
        can_uninitialize_(0x00);

        uint32_t st = can_initialize_(handle_, kPcanBaud1M, 0, 0, 0);
        if (st != 0) {
            std::cerr << "[ERR] PCAN 初始化失败, status=0x" << std::hex << st << std::dec << "\n";
            if (st == 0x04000000u) {
                std::cerr << "[HINT] PCAN 通道未就绪/被占用。请关闭 PCAN-View 等占用程序后重试，或尝试 USBBUS2。\n";
            }
            close();
            return false;
        }

        running_.store(true);
        rx_thread_ = std::thread([this]() { this->rx_loop(); });
        std::cout << "[INFO] PCAN 打开成功: USBBUS" << channel_index << " @1Mbps\n";
        return true;
    }

    void close() {
        running_.store(false);
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (can_uninitialize_ && handle_ != 0) {
            can_uninitialize_(handle_);
        }
        handle_ = 0;
        if (dll_) {
            FreeLibrary(dll_);
            dll_ = nullptr;
        }
        can_initialize_ = nullptr;
        can_uninitialize_ = nullptr;
        can_write_ = nullptr;
        can_read_ = nullptr;
    }

    bool send_can_frame(uint16_t can_id, const std::array<uint8_t, 8> &payload) {
        if (!can_write_ || handle_ == 0) {
            return false;
        }
        TPCANMsg msg{};
        msg.ID = can_id;
        msg.MSGTYPE = 0x00;  // standard frame
        msg.LEN = 8;
        for (int i = 0; i < 8; ++i) {
            msg.DATA[i] = payload[static_cast<size_t>(i)];
        }
        return can_write_(handle_, &msg) == 0;
    }

    MitFeedback latest_feedback() const {
        std::lock_guard<std::mutex> lock(fb_mu_);
        return latest_fb_;
    }

    RawCanFrame latest_raw_frame() const {
        std::lock_guard<std::mutex> lock(raw_mu_);
        return latest_raw_;
    }

private:
    struct TPCANMsg {
        uint32_t ID;
        uint8_t MSGTYPE;
        uint8_t LEN;
        uint8_t DATA[8];
    };

    struct TPCANTimestamp {
        uint32_t millis;
        uint16_t millis_overflow;
        uint16_t micros;
    };

    using CAN_Initialize_t = uint32_t(__stdcall *)(uint16_t, uint16_t, uint32_t, uint16_t, uint32_t);
    using CAN_Uninitialize_t = uint32_t(__stdcall *)(uint16_t);
    using CAN_Write_t = uint32_t(__stdcall *)(uint16_t, TPCANMsg *);
    using CAN_Read_t = uint32_t(__stdcall *)(uint16_t, TPCANMsg *, TPCANTimestamp *);

    bool load_symbols() {
        can_initialize_ = reinterpret_cast<CAN_Initialize_t>(GetProcAddress(dll_, "CAN_Initialize"));
        can_uninitialize_ = reinterpret_cast<CAN_Uninitialize_t>(GetProcAddress(dll_, "CAN_Uninitialize"));
        can_write_ = reinterpret_cast<CAN_Write_t>(GetProcAddress(dll_, "CAN_Write"));
        can_read_ = reinterpret_cast<CAN_Read_t>(GetProcAddress(dll_, "CAN_Read"));
        return can_initialize_ && can_uninitialize_ && can_write_ && can_read_;
    }

    void rx_loop() {
        while (running_.load()) {
            TPCANMsg msg{};
            TPCANTimestamp ts{};
            uint32_t st = can_read_(handle_, &msg, &ts);
            if (st == 0x00020u) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (st != 0 || msg.LEN < 8) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            std::array<uint8_t, 8> d{};
            for (int i = 0; i < 8; ++i) {
                d[static_cast<size_t>(i)] = msg.DATA[i];
            }

            {
                std::lock_guard<std::mutex> lock(raw_mu_);
                latest_raw_.valid = true;
                latest_raw_.can_id = msg.ID;
                latest_raw_.dlc = msg.LEN;
                latest_raw_.data = d;
                ++raw_seq_;
                latest_raw_.seq = raw_seq_;
            }

            MitFeedback fb;
            if (parse_feedback_8bytes(d, fb)) {
                std::lock_guard<std::mutex> lock(fb_mu_);
                latest_fb_ = fb;
            }
        }
    }

    HMODULE dll_{nullptr};
    uint16_t handle_{0};

    CAN_Initialize_t can_initialize_{nullptr};
    CAN_Uninitialize_t can_uninitialize_{nullptr};
    CAN_Write_t can_write_{nullptr};
    CAN_Read_t can_read_{nullptr};

    std::atomic<bool> running_{false};
    std::thread rx_thread_;

    mutable std::mutex fb_mu_;
    MitFeedback latest_fb_;

    mutable std::mutex raw_mu_;
    RawCanFrame latest_raw_;
    uint64_t raw_seq_{0};
};

class DmUsbCanBridge {
public:
    bool open(const std::string &com_port, int baudrate) {
        return serial_.open(com_port, baudrate);
    }

    void close() {
        running_.store(false);
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        serial_.close();
    }

    bool send_can_frame(uint16_t can_id, const std::array<uint8_t, 8> &payload) {
        // 该30字节格式来自当前仓库 debug_raw.py 的实测结构。
        std::array<uint8_t, 30> frame{};
        frame[0] = 0x55;
        frame[1] = 0xAA;
        frame[2] = 0x1E;
        frame[3] = 0x03;
        frame[4] = 0x01;
        frame[8] = 0x0A;
        frame[13] = static_cast<uint8_t>(can_id & 0xFF);
        frame[14] = static_cast<uint8_t>((can_id >> 8) & 0xFF);
        for (size_t i = 0; i < 8; ++i) {
            frame[21 + i] = payload[i];
        }
        return serial_.write_bytes(frame.data(), frame.size());
    }

    void start_rx_loop() {
        running_.store(true);
        rx_thread_ = std::thread([this]() { this->rx_loop(); });
    }

    MitFeedback latest_feedback() const {
        std::lock_guard<std::mutex> lock(fb_mu_);
        return latest_fb_;
    }

    RawCanFrame latest_raw_frame() const {
        std::lock_guard<std::mutex> lock(raw_mu_);
        return latest_raw_;
    }

private:
    void rx_loop() {
        std::vector<uint8_t> buffer;
        buffer.reserve(512);
        uint8_t tmp[128];

        while (running_.load()) {
            int n = serial_.read_bytes(tmp, sizeof(tmp));
            if (n <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            buffer.insert(buffer.end(), tmp, tmp + n);

            // 实测反馈帧常见格式: [0xAA ... 16字节 ... 0x55]
            while (buffer.size() >= 16) {
                auto it = std::find(buffer.begin(), buffer.end(), static_cast<uint8_t>(0xAA));
                if (it == buffer.end()) {
                    buffer.clear();
                    break;
                }

                size_t start = static_cast<size_t>(std::distance(buffer.begin(), it));
                if (buffer.size() < start + 16) {
                    if (start > 0) {
                        buffer.erase(buffer.begin(), buffer.begin() + static_cast<long long>(start));
                    }
                    break;
                }

                if (buffer[start + 15] != 0x55) {
                    buffer.erase(buffer.begin(), buffer.begin() + static_cast<long long>(start + 1));
                    continue;
                }

                std::array<uint8_t, 8> can_data{};
                for (size_t i = 0; i < 8; ++i) {
                    can_data[i] = buffer[start + 7 + i];
                }

                uint32_t can_id =
                    (static_cast<uint32_t>(buffer[start + 6]) << 24) |
                    (static_cast<uint32_t>(buffer[start + 5]) << 16) |
                    (static_cast<uint32_t>(buffer[start + 4]) << 8) |
                    static_cast<uint32_t>(buffer[start + 3]);

                {
                    std::lock_guard<std::mutex> lock(raw_mu_);
                    latest_raw_.valid = true;
                    latest_raw_.can_id = can_id;
                    latest_raw_.dlc = 8;
                    latest_raw_.data = can_data;
                    ++raw_seq_;
                    latest_raw_.seq = raw_seq_;
                }

                MitFeedback fb;
                if (parse_feedback_8bytes(can_data, fb)) {
                    std::lock_guard<std::mutex> lock(fb_mu_);
                    latest_fb_ = fb;
                }

                buffer.erase(buffer.begin(), buffer.begin() + static_cast<long long>(start + 16));
            }
        }
    }

    SerialPort serial_;
    std::atomic<bool> running_{false};
    std::thread rx_thread_;

    mutable std::mutex fb_mu_;
    MitFeedback latest_fb_;

    mutable std::mutex raw_mu_;
    RawCanFrame latest_raw_;
    uint64_t raw_seq_{0};
};

std::array<uint8_t, 8> make_special_cmd(uint8_t last_byte) {
    return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, last_byte};
}

std::array<uint8_t, 8> make_clear_error_cmd() {
    return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
}

std::array<uint8_t, 8> make_write_u32_cmd(uint16_t can_id, uint8_t rid, uint32_t value) {
    return {
        static_cast<uint8_t>(can_id & 0xFF),
        static_cast<uint8_t>((can_id >> 8) & 0xFF),
        0x55,
        rid,
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF)
    };
}

std::vector<uint16_t> candidate_ctrl_ids(uint16_t base_id) {
    std::vector<uint16_t> ids = {
        static_cast<uint16_t>(base_id),
        static_cast<uint16_t>(0x100 + base_id),
        static_cast<uint16_t>(0x200 + base_id),
        static_cast<uint16_t>(0x300 + base_id)
    };

    // 兼容部分DM旧固件: enable_id=((mode-1)<<2)+SlaveID
    for (uint16_t mode = 1; mode <= 4; ++mode) {
        ids.push_back(static_cast<uint16_t>(((mode - 1u) << 2u) + base_id));
    }

    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}

bool key_down(int vk) {
    return (GetAsyncKeyState(vk) & 0x8000) != 0;
}

}  // namespace

int main(int argc, char **argv) {
    std::string com = "COM3";
    int baud = 921600;
    uint16_t motor_id = 1;
    bool use_pcan = false;
    int pcan_bus_index = 1;
    bool diag_enable_only = false;
    int diag_seconds = 15;

    if (argc >= 2) {
        std::string arg1 = argv[1];
        if (!arg1.empty() && arg1[0] == '-' && arg1 != "--pcan" && arg1 != "--pan" && arg1 != "--diag-enable") {
            std::cerr << "[ERR] 未知参数: " << arg1 << "\n";
            std::cerr << "用法:\n";
            std::cerr << "  串口模式: dm4310_can_keyboard.exe [COM3] [921600] [motor_id]\n";
            std::cerr << "  PCAN模式: dm4310_can_keyboard.exe --pcan [bus_index] [motor_id]\n";
            std::cerr << "  使能诊断: dm4310_can_keyboard.exe --pcan [bus_index] [motor_id] --diag-enable [seconds]\n";
            std::cerr << "  (兼容别名: --pan)\n";
            return 2;
        }
    }

    if (argc >= 2 && (std::string(argv[1]) == "--pcan" || std::string(argv[1]) == "--pan")) {
        use_pcan = true;
        if (argc >= 3) {
            pcan_bus_index = std::stoi(argv[2]);
        }
        if (argc >= 4) {
            motor_id = static_cast<uint16_t>(std::stoi(argv[3]));
        }
    } else {
        if (argc >= 2) {
            com = argv[1];
        }
        if (argc >= 3) {
            baud = std::stoi(argv[2]);
        }
        if (argc >= 4) {
            motor_id = static_cast<uint16_t>(std::stoi(argv[3]));
        }
    }

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--diag-enable") {
            diag_enable_only = true;
            if (i + 1 < argc) {
                std::string s = argv[i + 1];
                if (!s.empty() && s[0] != '-') {
                    diag_seconds = std::max(1, std::stoi(s));
                }
            }
        }
    }

    std::cout << "=== DM4310 直连 USB-CAN 键盘调试 (MIT) ===\n";
    std::cout << "参数映射: P=+-12.5 rad, V=+-30 rad/s, T=+-10 Nm\n";
    if (use_pcan) {
        std::cout << "接口: PCAN-USB, 通道: USBBUS" << pcan_bus_index << ", 电机ID: " << motor_id << "\n";
    } else {
        std::cout << "接口: Serial-USB-CAN, 串口: " << com << ", 波特率: " << baud << ", 电机ID: " << motor_id << "\n";
    }
    std::cout << "按键说明:\n";
    std::cout << "  U/P : 正转/反转 (速度控制)\n";
    std::cout << "  I/K : 增加/减小前馈扭矩\n";
    std::cout << "  Z   : 保存当前位置为零点\n";
    std::cout << "  SPACE: 前馈扭矩清零\n";
    std::cout << "  Q 或 ESC: 退出\n\n";

    DmUsbCanBridge serial_bridge;
    PcanBridge pcan_bridge;

    auto send_frame = [&](uint16_t can_id, const std::array<uint8_t, 8> &payload) {
        return use_pcan ? pcan_bridge.send_can_frame(can_id, payload)
                        : serial_bridge.send_can_frame(can_id, payload);
    };

    auto latest_feedback = [&]() {
        return use_pcan ? pcan_bridge.latest_feedback() : serial_bridge.latest_feedback();
    };

    auto latest_raw_frame = [&]() {
        return use_pcan ? pcan_bridge.latest_raw_frame() : serial_bridge.latest_raw_frame();
    };

    auto close_bridge = [&]() {
        if (use_pcan) {
            pcan_bridge.close();
        } else {
            serial_bridge.close();
        }
    };

    bool opened = use_pcan ? pcan_bridge.open(pcan_bus_index) : serial_bridge.open(com, baud);
    if (!opened) {
        return 1;
    }
    if (!use_pcan) {
        serial_bridge.start_rx_loop();
    }

    // 按手册先将控制模式寄存器(0x0A)写为1(MIT)，再做使能。
    auto wr_mode = make_write_u32_cmd(motor_id, 0x0A, 1u);
    bool wr_ok = send_frame(0x7FF, wr_mode);
    std::cout << (wr_ok ? "[INIT] 已发送模式切换: CTRL_MODE=MIT(1)\n"
                       : "[INIT] 模式切换帧发送失败(继续尝试兜底使能)\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    auto ids = candidate_ctrl_ids(motor_id);
    auto clr = make_clear_error_cmd();
    auto en = make_special_cmd(kEnableLastByte);

    // 先清错并失能一次，避免故障锁定状态影响使能。
    for (uint16_t id : ids) {
        send_frame(id, clr);
        send_frame(id, make_special_cmd(kDisableLastByte));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    bool any_enable_sent = false;
    for (int k = 0; k < 3; ++k) {
        for (uint16_t id : ids) {
            any_enable_sent = send_frame(id, en) || any_enable_sent;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (!any_enable_sent) {
        std::cerr << "[ERR] 发送使能失败\n";
        close_bridge();
        return 1;
    }
    std::cout << "[INIT] 已发送兜底使能(含标准ID/偏移ID/旧固件ID)\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (diag_enable_only) {
        std::cout << "[DIAG] 进入使能诊断模式，仅发送使能命令，不发送MIT控制。时长: "
                  << diag_seconds << "s\n";
        uint64_t last_seq = 0;
        auto t0 = std::chrono::steady_clock::now();
        while (true) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - t0).count();
            if (elapsed >= diag_seconds) {
                break;
            }

            for (uint16_t id : ids) {
                send_frame(id, en);
            }

            RawCanFrame raw = latest_raw_frame();
            if (raw.valid && raw.seq != last_seq) {
                last_seq = raw.seq;
                std::cout << "[RX] can_id=0x" << std::hex << raw.can_id << std::dec
                          << " dlc=" << static_cast<int>(raw.dlc)
                          << " data=" << bytes_to_hex(raw.data, raw.dlc) << "\n";

                MitFeedback fb;
                if (raw.dlc >= 6 && parse_feedback_8bytes(raw.data, fb)) {
                    std::cout << std::fixed << std::setprecision(3)
                              << "[RX-DEC] id=" << fb.motor_id
                              << " err=" << fb.err
                              << "(" << err_to_text(fb.err) << ")"
                              << " p=" << fb.pos
                              << " v=" << fb.vel
                              << " t=" << fb.tau
                              << "\n";
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        send_frame(motor_id, make_special_cmd(kDisableLastByte));
        std::cout << "[DIAG] 诊断结束，已发送失能。\n";
        close_bridge();
        return 0;
    }

    float kp = 0.0f;
    float kd = 1.0f;
    float p_des = 0.0f;
    float v_des = 0.0f;
    float t_ff = 0.0f;

    const float vel_cmd = 6.0f;
    const float tau_step = 0.05f;
    const auto loop_dt = std::chrono::milliseconds(10);
    int print_div = 0;
    int no_fb_cycles = 0;
    bool printed_no_fb_hint = false;

    bool last_z = false;

    while (true) {
        if (key_down('Q') || key_down(VK_ESCAPE)) {
            break;
        }

        if (key_down('U')) {
            v_des = vel_cmd;
        } else if (key_down('P')) {
            v_des = -vel_cmd;
        } else {
            v_des = 0.0f;
        }

        if (key_down('I')) {
            t_ff += tau_step;
        }
        if (key_down('K')) {
            t_ff -= tau_step;
        }
        if (key_down(VK_SPACE)) {
            t_ff = 0.0f;
        }

        t_ff = clampf(t_ff, kTMin, kTMax);

        bool z_now = key_down('Z');
        if (z_now && !last_z) {
            send_frame(motor_id, make_special_cmd(kSaveZeroLastByte));
            std::cout << "[CMD] 保存零点\n";
        }
        last_z = z_now;

        auto mit = pack_mit(p_des, v_des, kp, kd, t_ff);
        if (!send_frame(motor_id, mit)) {
            std::cerr << "[WARN] 发送MIT帧失败\n";
        }

        ++print_div;
        if (print_div >= 10) {
            print_div = 0;
            MitFeedback fb = latest_feedback();
            if (fb.valid) {
                no_fb_cycles = 0;
                std::cout
                    << std::fixed << std::setprecision(3)
                    << "cmd[v=" << v_des << ", t=" << t_ff << "]  "
                    << "fb[id=" << fb.motor_id
                    << ", err=" << fb.err
                    << "(" << err_to_text(fb.err) << ")"
                    << ", p=" << fb.pos
                    << ", v=" << fb.vel
                    << ", t=" << fb.tau
                    << ", mos=" << fb.mos_temp
                    << ", rotor=" << fb.rotor_temp
                    << "]\n";
            } else {
                ++no_fb_cycles;
                std::cout
                    << std::fixed << std::setprecision(3)
                    << "cmd[v=" << v_des << ", t=" << t_ff << "]  fb[none]\n";

                if (!printed_no_fb_hint && no_fb_cycles >= 20) {
                    printed_no_fb_hint = true;
                    std::cout
                        << "[HINT] 已连续无反馈约2秒。按说明书检查:"
                        << " CAN波特率是否为1Mbps(寄存器0x23=4)，"
                        << " 若>1Mbps会进入CAN FD，CAN2.0主控通常收不到反馈。\n";
                }
            }
        }

        std::this_thread::sleep_for(loop_dt);
    }

    send_frame(motor_id, make_special_cmd(kDisableLastByte));
    std::cout << "已发送失能，程序退出。\n";
    close_bridge();
    return 0;
}
