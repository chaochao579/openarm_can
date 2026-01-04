#include <chrono>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <thread>
#include <vector>

/**
 * @brief 以“慢速分段”的方式驱动夹爪从当前位置缓慢移动到目标开合度。
 *
 * 说明：
 * - “真正的降速”需要底层支持“目标位置/开合度”或“MIT 参数控制”等接口。
 * - 这里给出一个可读性强的“分段控制框架”：把大动作拆成很多小动作，
 *   每次发一个小步进目标，然后等待电机执行并读取回包。
 *
 * 你需要把 set_gripper_target(...) 这类调用替换为你库里实际存在的接口：
 * - 例如某些库会提供：gripper.set_position(rad)、gripper.command_position(x)、motor.mit_control(...)
 * - 如果库没有提供任何可调目标的 API，就只能用 open()/close() + wait 的方式。
 */
static void slow_gripper_move_stepwise(
    openarm::can::socket::OpenArm& openarm,
    double start,          // 起始开合度（示意值）
    double target,         // 目标开合度（示意值）
    int steps,             // 分成多少步（越多越慢越平滑）
    int recv_timeout_ms,   // 每步 recv_all 等待回包时间
    int sleep_ms           // 每步之间额外 sleep（放慢节奏）
) {
    if (steps <= 0) steps = 1;

    const double delta = (target - start) / static_cast<double>(steps);

    for (int i = 1; i <= steps; i++) {
        const double intermediate = start + delta * static_cast<double>(i);

        // ====== 关键：在这里下发“夹爪目标开合度/位置” ======
        // TODO: 请把下面这一行替换成你库里控制夹爪“位置/开合度”的真实接口。
        //
        // 例子（伪代码）：
        // openarm.get_gripper().set_position(intermediate);
        // 或 openarm.get_gripper().command_position(intermediate);
        //
        // 如果你不知道接口名字：把 openarm.get_gripper() 可用方法（头文件/IDE提示）截图给我，
        // 我可以帮你把这一行改成可编译且可用的版本。
        (void)intermediate; // 暂时防止未使用警告；替换后可删掉

        // 让库有机会接收电机回包并更新状态缓存。
        // recv_all 的含义通常是：阻塞/轮询接收一定时间内的所有电机消息，
        // 以便更新 position/velocity 等 state。
        openarm.recv_all(recv_timeout_ms);

        // 主动 sleep：拉长每一步之间间隔，让动作更“慢”和更“分段明显”
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
}

int main() {
    try {
        std::cout << "=== OpenArm Gripper Slow Open/Close Demo ===\n";
        std::cout << "Goal: slow down gripper open/close with detailed comments.\n";

        // ========= 1) 打开 CAN 接口 =========
        // 第二个参数：是否启用 CAN-FD
        // - 如果你的电机/驱动器是经典 CAN(2.0)，这里必须是 false
        // - 如果硬件明确是 CAN-FD，才改 true（并确保系统 can0 也 fd on）
        openarm::can::socket::OpenArm openarm("can0", false);

        // ========= 2) 初始化电机（保持与你原程序一致）=========
        // 这部分包括：机械臂电机 + 夹爪电机
        // 注意：ID 必须与你实际设备配置一致，否则收不到回包（candump 会只有 TX 无 RX）
        std::vector<openarm::damiao_motor::MotorType> motor_types = {
            openarm::damiao_motor::MotorType::DM4310,
            openarm::damiao_motor::MotorType::DM4310};
        std::vector<uint32_t> send_can_ids = {0x01, 0x02};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12};
        openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        std::cout << "Initializing gripper...\n";
        openarm.init_gripper_motor(openarm::damiao_motor::MotorType::DM4310, 0x08, 0x18);

        // ========= 3) 使能所有电机 =========
        // CallbackMode::IGNORE 表示不解析回包（或忽略回调），常用于 enable/disable 等操作前后
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

        std::cout << "Enabling motors...\n";
        openarm.enable_all();

        // 给电机一定时间响应 enable（单位毫秒）
        // 如果你 CAN 总线有回包，这里能接收到状态/确认等数据
        openarm.recv_all(2000);

        // ========= 4) 切换到 STATE 模式 =========
        // STATE 模式通常用于周期接收位置/速度/扭矩等状态。
        openarm.set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

        // ========= 5) 夹爪慢速控制 =========
        //
        // 下面提供两种方案：
        // A) 优先方案：分段位置控制（真正降速，需要你把 TODO 替换成真实 API）
        // B) 退化方案：只用 open()/close()，通过更长等待让动作“看起来慢一些”
        //
        // 你现在想“速度降低”，建议尽量用 A。

        const bool try_stepwise_position_control = false; // TODO: 你确认有位置接口后改 true

        if (try_stepwise_position_control) {
            std::cout << "Slow opening (stepwise)...\n";
            // 下面的 start/target 是“示意开合度”，你需要按库接口的单位来设定：
            // - 可能是角度(rad)、可能是归一化(0~1)、也可能是电机编码器单位
            slow_gripper_move_stepwise(
                openarm,
                /*start=*/1.0,
                /*target=*/0.0,
                /*steps=*/30,
                /*recv_timeout_ms=*/50,
                /*sleep_ms=*/50);

            std::cout << "Pause...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(800));

            std::cout << "Slow closing (stepwise)...\n";
            slow_gripper_move_stepwise(
                openarm,
                /*start=*/0.0,
                /*target=*/1.0,
                /*steps=*/30,
                /*recv_timeout_ms=*/50,
                /*sleep_ms=*/50);
        } else {
            // ========== 退化方案：没有位置控制接口时 ==========
            //
            // 说明：
            // - open()/close() 通常是“一次性命令”，电机内部用固定速度跑到目标位置。
            // - 我们无法改变电机内部速度，但可以：
            //   1) 增加 recv_all 时间，保证状态更新/命令执行更充分
            //   2) 增加 sleep，让你观察到“动作完成后停留更久”，避免显得太“跳”
            //
            // 如果你想“真正变慢”，请把 try_stepwise_position_control 打开并补齐 TODO API。
            std::cout << "Opening gripper (high-level command)...\n";
            openarm.get_gripper().open();

            // 等待更久：让夹爪完成开合动作、并接收更多回包
            openarm.recv_all(2500);
            std::this_thread::sleep_for(std::chrono::milliseconds(800));

            std::cout << "Closing gripper (high-level command)...\n";
            openarm.get_gripper().close();

            openarm.recv_all(3000);
            std::this_thread::sleep_for(std::chrono::milliseconds(800));

            std::cout << "Opening gripper again (high-level command)...\n";
            openarm.get_gripper().open();

            openarm.recv_all(2500);
        }

        // ========= 6) 可选：打印夹爪状态 =========
        // 如果 CAN 回包正常，这里能看到 position 发生变化；
        // 如果 position 仍然一直 0，说明仍未收到状态回包（要回到 CAN 链路排查）。
        for (const auto& motor : openarm.get_gripper().get_motors()) {
            std::cout << "Gripper Motor: " << motor.get_send_can_id()
                      << " position: " << motor.get_position() << "\n";
        }

        // ========= 7) 下电/禁用 =========
        std::cout << "Disabling motors...\n";
        openarm.disable_all();
        openarm.recv_all(1000);

        std::cout << "Done.\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return -1;
    }
}