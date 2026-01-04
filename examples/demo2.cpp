#include <algorithm>  // std::max
#include <chrono>     // 时间相关：microseconds、sleep_for
#include <cstdint>    // uint32_t
#include <exception>  // std::exception
#include <iostream>   // std::cout / std::cerr
#include <string>     // std::string
#include <thread>     // std::this_thread::sleep_for

// OpenArm CAN Socket 版本主入口：包含 OpenArm 类定义
#include <openarm/can/socket/openarm.hpp>

// 给类型起别名，避免写一长串命名空间
using OpenArmT = openarm::can::socket::OpenArm;

/**
 * @brief 在一段时间内“持续重复发送”夹爪 open/close 指令。
 *
 * 为什么要“持续发送”？
 * - 你的 GripperComponent::open()/close() 最终是一次 MIT 控制帧（设置目标位置 q，dq=0，tau=0，以及 kp/kd）。
 * - 很多电机的 MIT/伺服模式需要周期性刷新目标，否则可能因为超时、总线竞争、状态刷新等原因导致目标不持续生效。
 * - 所以这里采用“固定频率，持续 hold 一段时间”的方式，提高动作可靠性（你之前“打开能动，闭合不动”就很符合这个现象）。
 *
 * @param robot          OpenArm 实例（内部持有 CAN socket，以及 arm/gripper 组件）
 * @param open_not_close true 表示发送 open()；false 表示发送 close()
 * @param kp             位置环 P 增益（越大越“硬”，跟随更紧，但过大可能抖/冲击/发热）
 * @param kd             速度阻尼 D 增益（越大阻尼越强，更稳更不抖，但太大可能“粘滞/变慢”）
 * @param hold_seconds   持续发送命令的总时间（秒）
 * @param hz             发送频率（Hz）。例如 50Hz 表示每 20ms 发一次命令。
 */
static void hold_gripper_command(OpenArmT& robot,
                                 bool open_not_close,
                                 double kp,
                                 double kd,
                                 double hold_seconds,
                                 int hz) {
    // steps = 一共要循环多少次
    // 例如 hold_seconds=3.0，hz=50 => steps=150 次
    const int steps = std::max(1, static_cast<int>(hold_seconds * hz));

    // 每个周期的时间间隔（微秒）
    // 例如 50Hz => 1e6/50=20000us=20ms
    const auto period = std::chrono::microseconds(1'000'000 / std::max(1, hz));

    // 主循环：重复发送 open()/close()
    for (int i = 0; i < steps; ++i) {
        // 1) 下发夹爪目标（open 或 close）
        // open()/close() 内部会调用 MIT 控制（目标位置、kp/kd 等）
        if (open_not_close) {
            robot.get_gripper().open(kp, kd);
        } else {
            robot.get_gripper().close(kp, kd);
        }

        // 2) 处理 CAN 收包
        // recv_all(timeout_us) 会从 socket 读取回包并分发给内部 device，用于状态更新等。
        // 这里传入一个与 period 相近的超时，避免阻塞太久，同时也能及时“清回包”。
        robot.recv_all(static_cast<int>(period.count()));

        // 3) 控制循环频率：保持稳定的发送节拍
        std::this_thread::sleep_for(period);
    }
}

int main() {
    // ===== 1) CAN 接口参数 =====
    // 对应 Linux SocketCAN 设备名：比如 can0 / can1
    const std::string can_iface = "can0";

    // 是否启用 CAN-FD：要与 ip link 配置、电机固件/库设置一致
    const bool use_canfd = false;

    // ===== 2) 夹爪电机参数（达妙 DM4310 + 你的 CAN ID）=====
    // motor_type：电机型号（决定协议参数、编码范围等）
    const auto motor_type = openarm::damiao_motor::MotorType::DM4310;

    // send_id / recv_id：
    // - send_id：你向电机发送控制帧使用的 CAN ID
    // - recv_id：电机回传状态帧使用的 CAN ID
    // 这两个 ID 必须与你实际电机固件/配置一致，否则会“能发但收不到”或“收到了别的设备”
    const uint32_t send_id = 0x08;
    const uint32_t recv_id = 0x18;

    // ===== 3) 控制增益 =====
    // kp/kd 是 MIT 控制里的 PD 增益：
    // - kp 大：更硬、更快贴近目标；过大可能抖、冲击大、发热
    // - kd 大：阻尼更强、更稳；过大可能运动粘滞、响应慢
    //
    // 你之前设置 kp=4 kd=1：属于相对偏硬一点的设置（具体还要看机械负载/摩擦/减速比）
    const double kp = 4.0;
    const double kd = 1.0;

    try {
        // ===== 4) 创建 OpenArm 对象（打开 CAN socket，并准备组件）=====
        // OpenArm 是“主协调器”：内部包含 ArmComponent + GripperComponent 等
        OpenArmT robot(can_iface, use_canfd);

        // ===== 5) 初始化夹爪电机设备 =====
        // 这一步会在 GripperComponent 内创建 motor、DMCANDevice，并加入 device collection
        robot.init_gripper_motor(motor_type, send_id, recv_id);

        // ===== 6) 使能电机（上电/进入可控状态）=====
        // CallbackMode::IGNORE：
        // - 表示暂时不处理详细回调（比如参数/状态回调），先把 enable 流程走完
        robot.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        // enable_all 会对所有已注册的设备发 enable 指令（至少包括夹爪）
        robot.enable_all();

        // 读一波回包，让 enable 后状态稳定、清空 socket 缓冲
        robot.recv_all(50'000);  // 50ms

        // ===== 7) 切到 STATE 模式（开始解析状态回包）=====
        // STATE 模式通常用于持续接收位置/速度/电流等信息（具体取决于库实现）
        robot.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // ===== 8) 打开夹爪：持续 3 秒，每秒 50 次 =====
        std::cout << "[action] gripper open (hold)\n";
        hold_gripper_command(robot,
                             /*open_not_close=*/true,
                             kp,
                             kd,
                             /*hold_seconds=*/3.0,
                             /*hz=*/50);

        // 打开结束后稍等一会（机械和控制都更稳定，也方便观察）
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // ===== 9) 闭合夹爪：持续 3 秒，每秒 50 次 =====
        std::cout << "[action] gripper close (hold)\n";
        hold_gripper_command(robot,
                             /*open_not_close=*/false,
                             kp,
                             kd,
                             /*hold_seconds=*/3.0,
                             /*hz=*/50);

        // ===== 10) 退出前下电（推荐）=====
        // 先忽略回调，然后 disable_all 下电，再读一波回包，确保指令发送与状态收敛
        robot.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
        robot.disable_all();
        robot.recv_all(50'000);

        std::cout << "[done]\n";
        return 0;
    } catch (const std::exception& e) {
        // 捕获异常并输出，避免程序静默退出
        std::cerr << "[fatal] " << e.what() << "\n";
        return 1;
    }
}