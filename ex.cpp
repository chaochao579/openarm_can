#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <cstdint>
#include <vector>

int main() {
    openarm::can::socket::OpenArm arm("can0", true);  // CAN-FD enabled
    std::vector<openarm::damiao_motor::MotorType> motor_types = {
        openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310};
    std::vector<uint32_t> send_can_ids = {0x01, 0x02};
    std::vector<uint32_t> recv_can_ids = {0x11, 0x12};

    arm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);
    arm.enable_all();

    return 0;
    }